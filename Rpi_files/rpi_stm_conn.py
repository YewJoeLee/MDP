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
on_snap callback instead, because the STM does not understand them and
would simply time out.
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

# Reply lines that mean "command finished"
DONE_PREFIXES = ("DONE", "ERR", "SPD", "SERVO", "BIAS", "YAW")


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
            result = wait_reply(ser, cmd)
            dt = time.time() - t0

            if result is True:
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