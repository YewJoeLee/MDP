#!/usr/bin/env python3
"""
Send a sequence of commands to the STM32 over /dev/ttyACM0.

Standalone usage:
    python3 rpi_stm_conn.py                  # runs the COMMANDS list below
    python3 rpi_stm_conn.py FW50 RT90 FW30   # runs commands given on the command line
    python3 rpi_stm_conn.py -f path.txt      # runs commands from a file (one per line)

As a module (used by fake_rpi_client.py):
    import rpi_stm_conn
    cmds = rpi_stm_conn.read_cmds_file(path)
    rpi_stm_conn.run_commands(cmds, on_snap=my_camera_function)

SNAP<id> commands are never written to the UART. They are handed to the
on_snap callback instead, because the STM does not understand them.

Around every SNAP:
    APPROACH_CMD (e.g. AP25) is sent first, so the STM uses the ultrasonic
    to settle at the right distance for the camera.
    RETURN_CMD (RA) is sent after the photo, so the STM drives back by
    exactly the distance the approach moved, restoring the planned pose.
Set either to None to disable it.

If the first SNAP attempt at an obstacle fails to recognise anything and
ANGLE_CORRECTION_ENABLED is True, run_commands also tries a one-shot
heading correction (see correct_snap_angle): it backs up to the last turn,
nudges and redoes that turn based on which side the camera now sees the
obstacle on, recalibrates distance, and tries SNAP once more - then
undoes the correction so every later command in the plan is still valid.
This only ever runs around a SNAP command; every other command behaves
exactly as before.
"""
import sys
import time
import serial

PORT = "/dev/ttyACM0"
BAUD = 115200

# Default sequence if no arguments are given
COMMANDS = [
    "FW50",
    "RT90",
    "FW30",
    "LT90",
    "BW20",
]

REPLY_TIMEOUT_S = 20      # longest move is 15 s (MOVE_TIMEOUT_MS) + margin
GAP_BETWEEN_CMDS_S = 0.2  # settle time between commands
STOP_ON_ERROR = True      # stop the sequence on ERR or timeout

APPROACH_CMD = "AC25"     # sent before every SNAP; set to None to disable
RETURN_CMD = "RA"         # sent after every SNAP; set to None to disable

# ---- angle correction (runs only when the first SNAP attempt fails) ----
ANGLE_CORRECTION_ENABLED = True
ANGLE_NUDGE_CM = 10       # how far to creep forward/back before redoing the turn

# Undoing a turn = doing the opposite-direction version of the same turn
# style (forward turn undone by the matching reverse turn, and back).
REVERSE_TURN = {
    "LT90": "XL90",
    "XL90": "LT90",
    "RT90": "XR90",
    "XR90": "RT90",
}

# (last turn we made, side the obstacle now appears on) -> which way to
# creep before redoing that turn. See the long comment on
# correct_snap_angle() for how this table was derived.
TURN_CORRECTION_NUDGE = {
    ("LT90", "right"): "FW",  # undershot the left turn
    ("LT90", "left"):  "BW",  # overshot the left turn
    ("RT90", "left"):  "FW",  # undershot the right turn
    ("RT90", "right"): "BW",  # overshot the right turn
    ("XR90", "left"):  "BW",  # undershot the reverse-right turn
    ("XR90", "right"): "FW",  # overshot the reverse-right turn
    ("XL90", "right"): "BW",  # undershot the reverse-left turn
    ("XL90", "left"):  "FW",  # overshot the reverse-left turn
}

# Reply lines that mean "command finished"
DONE_PREFIXES = ("DONE", "ERR", "SPD", "SERVO", "BIAS", "YAW",
                 "US", "ZEROED", "HERR")


def wait_reply(ser, cmd):
    """Return True on success, False on ERR, None on timeout."""
    deadline = time.time() + REPLY_TIMEOUT_S
    is_gyro_dump = cmd[:2].upper() == "GY"
    gy_lines = 0

    while time.time() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        print(f"  <- {line}")

        if is_gyro_dump:
            # GY prints 200 lines of X:/Y:/Z: and no DONE
            if line.startswith("X:"):
                gy_lines += 1
                if gy_lines >= 200:
                    return True
            continue

        if line.startswith(DONE_PREFIXES):
            return not line.startswith("ERR")

    return None


def send_and_wait(ser, cmd):
    """Write one command, wait for its reply. Returns wait_reply's result."""
    ser.write((cmd + "\n").encode())
    ser.flush()
    return wait_reply(ser, cmd)


def send_helper(ser, cmd, label):
    """
    Send an approach / return command around a SNAP.
    Failures are reported but never stop the run: a slightly-off photo
    is better than no photo.
    """
    print(f"    -> {cmd} ({label})")
    result = send_and_wait(ser, cmd)
    if result is not True:
        print(f"    {label} failed, continuing")
    time.sleep(GAP_BETWEEN_CMDS_S)


def reverse_straight_cmd(cmd):
    """"FW50" -> "BW50", "BW20" -> "FW20". None if cmd isn't a straight move."""
    upper = cmd.upper()
    if upper.startswith("FW"):
        return "BW" + cmd[2:]
    if upper.startswith("BW"):
        return "FW" + cmd[2:]
    return None


def inverse_cmd(cmd):
    """The single command that undoes cmd, for turns and straights only."""
    upper = cmd.upper()
    if upper in REVERSE_TURN:
        return REVERSE_TURN[upper]
    return reverse_straight_cmd(cmd)


def correct_snap_angle(ser, obstacle_id, last_turn_cmd, last_straight_cmd, on_snap, on_check_position):
    """
    Called only after a first SNAP attempt has failed to recognise anything.

    The idea: the robot's final heading can be a bit off even though the
    ultrasonic calibration (AC) got the *distance* right, because AC only
    ever drives straight - it can't rotate the robot. So instead we:

      1. Ask the camera which side of the frame the obstacle sits on.
         If we can't tell, there's nothing to correct - give up cleanly.
      2. Walk back to the pose we were in just before the final turn
         (undo the last straight move, then undo the last turn).
      3. Work out whether that turn under- or over-shot from which side
         the obstacle was on, nudge a few cm forward/back to compensate,
         then redo the turn and the last straight move (if there was one)
      4. Re-run the same ultrasonic alignment (AC) command as before, now
         that the heading should be closer to correct. (NOT ATTEMPTING THIS IN THIS FUNCTION)
      5. Undo everything done in steps 2-3, so the robot ends up exactly
         back where the (uncorrected) plan expects it to be - every
         command A* sends after this one is still valid. (NOT ATTEMPTING THIS)

    Returns nothing; on_snap is called once or twice as a side effect.
    """
    if last_turn_cmd not in REVERSE_TURN:
        print("    [ANGLE] no turn history yet, can't correct angle - skipping")
        return

    side = on_check_position() if on_check_position else None
    if side not in ("left", "right"):
        print("    [ANGLE] camera couldn't tell left/right - skipping correction")
        return

    applied = []

    def do(cmd):
        applied.append(cmd)
        send_and_wait(ser, cmd)
        time.sleep(GAP_BETWEEN_CMDS_S)

    # --- step 2: get back to the pose just before the final turn ---
    if last_straight_cmd:
        undo_straight = reverse_straight_cmd(last_straight_cmd)
        if undo_straight:
            print(f"    [ANGLE] undo last straight: {undo_straight}")
            do(undo_straight)

    reverse_turn_cmd = REVERSE_TURN[last_turn_cmd]
    print(f"    [ANGLE] undo last turn: {reverse_turn_cmd}")
    do(reverse_turn_cmd)

    # --- step 2 (cont.): nudge to compensate, then redo the turn ---
    nudge_dir = TURN_CORRECTION_NUDGE.get((last_turn_cmd, side))
    if nudge_dir:
        nudge_cmd = f"{nudge_dir}{ANGLE_NUDGE_CM}"
        print(f"    [ANGLE] obstacle on {side}; last turn was {last_turn_cmd} "
              f"-> nudging {nudge_cmd}")
        do(nudge_cmd)

    print(f"    [ANGLE] redo turn: {last_turn_cmd}")
    do(last_turn_cmd)

    if last_straight_cmd:
        do(last_straight_cmd)
        
    # --- step 3: recalibrate distance on the corrected heading ---
    #if last_align_cmd:
        #print(f"    [ANGLE] re-running alignment: {last_align_cmd}")
        #send_and_wait(ser, last_align_cmd)
        #time.sleep(GAP_BETWEEN_CMDS_S)

    # --- step 4: undo everything from step 2, in reverse order ---
    #print("    [ANGLE] restoring original pose for the rest of the plan")
    #for cmd in reversed(applied):
        #inv = inverse_cmd(cmd)
        #if inv:
            #send_and_wait(ser, inv)
            #time.sleep(GAP_BETWEEN_CMDS_S)


def open_serial():
    """
    Open the port, let the STM finish booting / gyro bias calibration,
    print whatever it says on the way up, then clear the buffer.
    """
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    print(f"Opened {PORT} @ {BAUD}")

    time.sleep(2.0)
    while ser.in_waiting:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print(f"  [boot] {line}")
    ser.reset_input_buffer()

    return ser


def read_cmds_file(path):
    """One command per line. Blank lines and # comments are ignored."""
    with open(path) as f:
        return [ln.strip() for ln in f
                if ln.strip() and not ln.strip().startswith("#")]


def run_commands(cmds, on_snap=None, on_check_position=None):
    """
    Send each command and wait for its reply.

    SNAP<id> is not sent to the STM. Instead:
        1. on_snap(id) is called (if given) for the first attempt
        2. if that attempt fails and ANGLE_CORRECTION_ENABLED is True,
           correct_snap_angle() tries the turn-nudge-realign-retry dance,
           using on_check_position() to ask the camera which side the
            obstacle is on
        3. We always try and calibrate by sending APPROACH_CMD and then RETURN_CMD

    on_snap(obstacle_id) should return True/False (recognised or not).
    on_check_position() should return "left", "right", or None.

    Returns True if the whole sequence completed, False if it was cut
    short by an error or timeout while STOP_ON_ERROR is set.
    """
    ser = open_serial()
    completed = True

    # History of the last hardware commands actually sent, used only to
    # drive the angle-correction dance around SNAP.
    last_turn_cmd = None
    last_straight_cmd = None
    #last_align_cmd = None

    try:
        for i, cmd in enumerate(cmds, 1):
            print(f"[{i}/{len(cmds)}] -> {cmd}")
            upper = cmd.upper()

            if upper.startswith("SNAP"):
                obstacle_id = cmd[4:]
                
                #First attempt to recognise image
                if on_snap:
                    success = on_snap(obstacle_id)
                else:
                    print(f"    [CAMERA] no handler for {cmd}, skipping")
                    success = False

                print(f"1st attempt to recognise image - {'SUCCESS' if success else 'FAIL'}")

                if not success:
                    if ANGLE_CORRECTION_ENABLED:
                        correct_snap_angle(
                            ser, obstacle_id,
                            last_turn_cmd, last_straight_cmd,
                            on_snap, on_check_position,
                        )

                    if APPROACH_CMD:
                        send_helper(ser, APPROACH_CMD, "approach before photo")

                    #Second and final attempt to recognise image
                    if on_snap:
                        success = on_snap(obstacle_id)
                        print(f"    [ANGLE] second attempt {'succeeded' if success else 'still failed'}")
                    else:
                        print(f"    [ANGLE] no camera handler, skipping second attempt")

                    if APPROACH_CMD and RETURN_CMD:
                        send_helper(ser, RETURN_CMD, "return after photo")

                continue

            t0 = time.time()
            result = send_and_wait(ser, cmd)
            dt = time.time() - t0

            if result is True:
                print(f"    ok ({dt:.1f} s)")
                if upper in REVERSE_TURN:
                    last_turn_cmd = upper
                    last_straight_cmd = None
                elif upper.startswith("FW") or upper.startswith("BW"):
                    last_straight_cmd = cmd
                #elif upper.startswith("AC"):
                    #last_align_cmd = cmd
            elif result is False:
                print(f"    ERROR ({dt:.1f} s)")
                if STOP_ON_ERROR:
                    completed = False
                    break
            else:
                print(f"    TIMEOUT after {REPLY_TIMEOUT_S} s")
                if STOP_ON_ERROR:
                    completed = False
                    break

            time.sleep(GAP_BETWEEN_CMDS_S)

    finally:
        ser.close()

    print("Finished.")
    return completed


def load_commands():
    args = sys.argv[1:]
    if not args:
        return COMMANDS
    if args[0] == "-f":
        return read_cmds_file(args[1])
    return args


def main():
    run_commands(load_commands())


if __name__ == "__main__":
    main()

    