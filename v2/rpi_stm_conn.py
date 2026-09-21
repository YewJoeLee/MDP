#!/usr/bin/env python3
"""
Send a sequence of commands to the STM32 over /dev/ttyACM0.

Standalone usage:
    python3 rpi_stm_conn.py                  # runs the COMMANDS list below
    python3 rpi_stm_conn.py FW50 RT90 FW30   # runs commands given on the command line
    python3 rpi_stm_conn.py AC25             # test the ultrasonic align on its own
    python3 rpi_stm_conn.py -f path.txt      # runs commands from a file (one per line)

As a module (used by rpi_client.py):
    import rpi_stm_conn
    cmds = rpi_stm_conn.read_cmds_file(path)
    rpi_stm_conn.run_commands(cmds, on_snap=my_camera_function)

SNAP<id> commands are never written to the UART. They are handed to the
on_snap callback instead, because the STM does not understand them and
would simply time out.

AC<cm> (ultrasonic align, inserted by the planner just before each SNAP)
is sent to the STM like any other command, but:
  - its reply is decoded and printed as a correction,
  - a failed or timed-out align never stops the run; the photo is taken
    anyway, because a slightly-off photo beats aborting everything.

Expected STM reply to AC<cm> (agreed with the STM team):
    DONE AC25 D:<first reading, mm> MV:<net move, mm, + = forward>
Extra or missing fields are fine; the raw line is always printed too.
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
STOP_ON_ERROR = True      # stop the sequence on ERR or timeout (not for align)

ALIGN_PREFIX = "AC"       # must match config.ALIGN_PREFIX on the laptop

# Reply lines that mean "command finished"
DONE_PREFIXES = ("DONE", "ERR", "SPD", "SERVO", "BIAS", "YAW")

# One entry per align command in the last run, for the summary.
align_log = []


def is_align(cmd):
    """AC, AC25, ac30 ... (nothing else the STM understands starts with AC)."""
    return cmd.upper().startswith(ALIGN_PREFIX.upper())


def wait_reply(ser, cmd):
    """
    Return (result, line):
        result True on success, False on ERR, None on timeout
        line   the reply line that finished the command (or None)
    """
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
                    return True, line
            continue

        if line.startswith(DONE_PREFIXES):
            return (not line.startswith("ERR")), line

    return None, None


def parse_fields(line):
    """'DONE AC25 D:283 MV:30 OK' -> {'D': '283', 'MV': '30'}"""
    fields = {}
    for token in (line or "").split():
        if ":" in token:
            key, _, value = token.partition(":")
            fields[key.upper()] = value
    return fields


def mm_to_cm_text(value):
    try:
        return f"{int(value) / 10:.1f} cm"
    except (TypeError, ValueError):
        return "?"


def report_align(cmd, result, line, dt):
    """Print the align outcome in plain words and remember it."""
    target = cmd[len(ALIGN_PREFIX):] or "STM default"
    fields = parse_fields(line)

    measured = fields.get("D")
    moved = fields.get("MV")

    if result is True:
        status = "ok"
    elif result is False:
        status = "ERROR"
    else:
        status = "TIMEOUT"

    print(f"    [ALIGN] target {target} cm | status {status} | {dt:.1f} s")
    if measured is not None:
        print(f"    [ALIGN] sensor read   {mm_to_cm_text(measured)}")
    if moved is not None:
        try:
            mv = int(moved)
            direction = "forward" if mv > 0 else "backward" if mv < 0 else "no move"
            print(f"    [ALIGN] correction    {abs(mv) / 10:.1f} cm {direction}")
        except ValueError:
            print(f"    [ALIGN] correction    {moved}")
    if result is not True:
        print("    [ALIGN] carrying on and taking the photo anyway")

    align_log.append({
        "cmd": cmd,
        "status": status,
        "measured": measured,
        "moved": moved,
        "reply": line,
    })


def print_align_summary():
    if not align_log:
        return
    print()
    print("Align summary:")
    for n, entry in enumerate(align_log, 1):
        measured = mm_to_cm_text(entry["measured"]) if entry["measured"] else "-"
        moved = mm_to_cm_text(entry["moved"]) if entry["moved"] else "-"
        print(f"  {n}. {entry['cmd']:<6} {entry['status']:<8} "
              f"read {measured:<9} moved {moved}")


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


def run_commands(cmds, on_snap=None):
    """
    Send each command and wait for its reply.

    SNAP<id> is not sent to the STM. If on_snap is given it is called with
    the id string; otherwise the command is skipped with a warning.

    Returns True if the whole sequence completed, False if it was cut
    short by an error or timeout while STOP_ON_ERROR is set.
    """
    align_log.clear()
    ser = open_serial()
    completed = True

    try:
        for i, cmd in enumerate(cmds, 1):
            print(f"[{i}/{len(cmds)}] -> {cmd}")

            if cmd.upper().startswith("SNAP"):
                if on_snap:
                    on_snap(cmd[4:])
                else:
                    print(f"    [CAMERA] no handler for {cmd}, skipping")
                continue

            ser.write((cmd + "\n").encode())
            ser.flush()

            t0 = time.time()
            result, line = wait_reply(ser, cmd)
            dt = time.time() - t0

            if is_align(cmd):
                report_align(cmd, result, line, dt)

            elif result is True:
                print(f"    ok ({dt:.1f} s)")

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

    print_align_summary()
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
