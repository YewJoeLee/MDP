#!/usr/bin/env python3
"""Interactive STM console for testing - type a command, see a clear answer.

Usage (on the RPi):
    python3 stm_console.py                 # interactive
    python3 stm_console.py FW30 RT45 US    # run these, print the results, exit
    python3 stm_console.py --log           # also append everything to stm_console_log.txt
    python3 stm_console.py --port /dev/ttyACM1

At the  stm>  prompt:
    FW30  BW20                 drive straight (cm)
    LT45  RT90  XL45  XR10     turn left / right, forward (LT/RT) or reverse (XL/XR), any angle 1-360
    AC30                       drive until the ultrasonic reads 30 cm, then stop
    IERN100  IELA100           FW up to 100 cm, stop at the edge the Right/Left IR sees
                               (N = must see the obstacle first, A = far counts at once)
    RA                         undo the last AC (drives back what AC moved)
    BI                         gyro bias calibration - keep the robot STILL
    us   (or u)                ultrasonic distance
    ir   (or i)                both IR sensors
    s                          ultrasonic + both IRs
    watch  (or w)              live US + IR readings, Ctrl+C to end
    stop                       stop the STM now
    FW30 RT45 FW10 s           several commands in one line run one after another
    3x FW10                    repeat a command (or a whole line) 3 times
    Enter on an empty line     repeat the last line
    help / q

Ctrl+C while the robot is moving sends STOP straight away.
Any other firmware command (YW, GY, TM1, VS2600, KQ, ...) is passed through as typed
and its reply is printed.

Protocol (main_merged.c): one ASCII command per line, wait for its reply before the
next. Moves reply  DONE <cmd> ... STATUS:<OK|TIMEOUT|STALL|STOPPED|...>.
US / IR asked during a move are answered only after the move, so this script never
does that.
"""
import argparse
import queue
import re
import sys
import threading
import time
from datetime import datetime

try:
    import serial
except ImportError:
    print("pyserial missing:  pip3 install pyserial")
    sys.exit(1)

PORT = "/dev/ttyACM0"
BAUD = 115200
LOG_FILE = "stm_console_log.txt"

# Which side each IR is on. Check once: put your hand by one side, send  ir , see
# which number drops, and swap these two if needed.
IR_NAMES = {"IR1": "IR1 (left)", "IR2": "IR2 (right)"}

MOVE_TIMEOUT_S = 20        # FW/BW/turns: firmware gives up after 15 s
AC_TIMEOUT_S = 65          # AC: firmware gives up after 60 s
YW_TIMEOUT_S = 15
OTHER_TIMEOUT_S = 3

MOVE_RE = re.compile(r"^(FW|BW|LT|RT|XL|XR|DL|DR)(\d+)$")
FINISH_PREFIXES = ("DONE", "ERR", "BIAS", "YAW", "US ", "IR1", "TURNMODE", "SPEED", "SERVO",
                   "SPD", "COAST", "INNER", "COMP")


# =============================================================================
# Serial link: a background reader puts every line from the STM into a queue
# =============================================================================

class Link:
    def __init__(self, port, baud, log):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.lines = queue.Queue()
        self.log = log
        self.alive = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        buf = b""
        while self.alive:
            try:
                chunk = self.ser.read(256)
            except Exception as e:
                self.lines.put(f"__SERIAL_ERROR__ {e}")
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line:
                    self.lines.put(line)

    def write(self, cmd):
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()
        self.log_line(f">> {cmd}")

    def drain(self):
        """Throw away (but log) anything left over from an earlier command."""
        while True:
            try:
                self.log_line(f"<< (late) {self.lines.get_nowait()}")
            except queue.Empty:
                return

    def log_line(self, text):
        if self.log:
            self.log.write(f"{datetime.now():%H:%M:%S.%f}"[:-3] + f"  {text}\n")
            self.log.flush()

    def close(self):
        self.alive = False
        try:
            self.ser.close()
        except Exception:
            pass


def send(link, cmd):
    """Send one command and wait for the line that finishes it.
    Returns (finishing line or None, all lines received, seconds taken)."""
    link.drain()
    if cmd.startswith("AC"):
        timeout = AC_TIMEOUT_S
    elif MOVE_RE.match(cmd) or cmd == "RA" or cmd.startswith("IE"):
        timeout = MOVE_TIMEOUT_S
    elif cmd == "YW":
        timeout = YW_TIMEOUT_S
    elif cmd == "GY":
        timeout = 15
    else:
        timeout = OTHER_TIMEOUT_S
    known = MOVE_RE.match(cmd) or cmd[:2] in ("AC", "RA", "US", "IR", "BI", "YW", "TM", "VS", "SV", "SP", "IE")
    t0 = time.time()
    link.write(cmd)
    got, gy = [], 0
    try:
        while time.time() - t0 < timeout:
            try:
                line = link.lines.get(timeout=0.1)
            except queue.Empty:
                if got and not known:          # unknown command: whatever arrived is its reply
                    return got[-1], got, time.time() - t0
                continue
            link.log_line(f"<< {line}")
            if line.startswith("__SERIAL_ERROR__"):
                print(f"  !! serial error: {line[17:]}")
                return None, got, time.time() - t0
            got.append(line)
            if cmd == "GY":
                print(f"  {line}")
                gy += 1
                if gy >= 200:
                    return line, got, time.time() - t0
                continue
            if line.startswith(FINISH_PREFIXES) or line.startswith("K"):
                return line, got, time.time() - t0
    except KeyboardInterrupt:
        stop(link)
        return "STOPPED BY YOU (Ctrl+C)", got, time.time() - t0
    return None, got, time.time() - t0


def stop(link):
    link.write("STOP")
    print("\n  >> STOP sent")
    end = time.time() + 2
    while time.time() < end:
        try:
            line = link.lines.get(timeout=0.1)
            link.log_line(f"<< {line}")
            print(f"     {line}")
            if line == "DONE STOP":           # firmware: motors braked, queue cleared, ready again
                break
        except queue.Empty:
            pass


# =============================================================================
# Pretty printing
# =============================================================================

def fields(line):
    """'DONE AC30 US:302 MOVED:12 STATUS:OK' -> {'US': '302', 'MOVED': '12', 'STATUS': 'OK'}"""
    return dict(m.groups() for m in re.finditer(r"([A-Z]+):(-?[\w./-]+)", line))


def cm(mm):
    try:
        return f"{int(mm) / 10:.1f} cm"
    except (TypeError, ValueError):
        return str(mm)


def show(cmd, line, lines, secs):
    if line is None:
        print(f"  {cmd}: no reply after {secs:.1f} s" + (f"  (got: {lines[-1]})" if lines else ""))
        return
    if line.startswith("STOPPED BY YOU"):
        print(f"  {cmd}: stopped by you after {secs:.1f} s")
        return
    if line.startswith("ERR"):
        print(f"  {cmd}: STM refused it -> {line}")
        return
    f = fields(line)
    status = f.get("STATUS", line.split()[-1] if line.startswith("DONE") else "")
    flag = "" if status in ("OK", "") else f"   <-- {status}"

    if cmd == "US" or line.startswith("US "):
        m = re.search(r"US (-?\d+)", line)
        mm = int(m.group(1)) if m else None
        if mm is None or mm <= 0 or mm > 4000:
            print(f"  Ultrasonic: no echo ({line})")
        else:
            print(f"  Ultrasonic: {mm / 10:.1f} cm")
    elif line.startswith("IR1"):
        m = re.findall(r"(IR\d) raw:(\d+) mm:(-?\d+)", line)
        print("  " + "    ".join(f"{IR_NAMES.get(n, n)}: {int(v) / 10:.1f} cm (raw {r})" for n, r, v in m))
    elif line.startswith("BIAS"):
        if "UNSTABLE" in line:
            print(f"  Gyro bias: UNSTABLE - keep the robot still and send BI again  ({line})")
        else:
            print(f"  Gyro bias: OK  ({line})")
    elif line.startswith("YAW"):
        m = re.search(r"YAW (-?\d+)", line)
        print(f"  Yaw over the 8 s window: {int(m.group(1)) / 10:.1f} deg" if m else f"  {line}")
    elif cmd.startswith("AC"):
        us = f.get("US")
        print(f"  {cmd}: stopped at {cm(us)}, moved {f.get('MOVED', '?')} cm, {secs:.1f} s{flag}")
    elif cmd.startswith("IE"):
        def mm(k):
            v = f.get(k)
            return "-" if v is None or int(v) < 0 else f"{int(v) / 10:.1f} cm"
        what = {"EDGE": "edge found", "NEVER": "nothing beside it", "MAX": "no edge before max",
                "WALL": "STOPPED BY WALL GUARD"}.get(f.get("END"), f.get("END"))
        print(f"  {cmd}: {what} · edge at {mm('EDGE')} · IR beside {mm('NEAR')} ({f.get('NN', 0)} reads)"
              f" · moved {mm('MOVED')} · {secs:.1f} s{flag}")
    elif cmd == "RA":
        print(f"  RA: drove back {f.get('UNDID', '?')} cm, {secs:.1f} s{flag}")
    elif MOVE_RE.match(cmd) and cmd[:2] in ("LT", "RT", "XL", "XR"):
        got = f.get("IMU")
        ang = f" gyro says {int(got) / 10:.1f} deg," if got and got.lstrip("-").isdigit() else ""
        print(f"  {cmd}: done,{ang} {secs:.1f} s{flag}")
    elif MOVE_RE.match(cmd):
        he = f.get("HE")
        head = f", heading error {int(he) / 10:.1f} deg" if he and he.lstrip("-").isdigit() else ""
        print(f"  {cmd}: done, {secs:.1f} s{head}{flag}")
    else:
        for l in lines:
            print(f"  {l}")


def sensors(link, quiet=False):
    """Ultrasonic + both IRs, printed on one line. Returns the text."""
    l1, _, _ = send(link, "US")
    l2, _, _ = send(link, "IR")
    parts = []
    m = re.search(r"US (-?\d+)", l1 or "")
    mm = int(m.group(1)) if m else -1
    parts.append(f"US {mm / 10:5.1f} cm" if 0 < mm <= 4000 else "US   ---   ")
    for n, r, v in re.findall(r"(IR\d) raw:(\d+) mm:(-?\d+)", l2 or ""):
        parts.append(f"{IR_NAMES.get(n, n)} {int(v) / 10:5.1f} cm")
    text = "   ".join(parts)
    if not quiet:
        print("  " + text)
    return text


def watch(link, period=0.3):
    print("  live readings - Ctrl+C to stop watching")
    try:
        while True:
            text = sensors(link, quiet=True)
            print("\r  " + text + "    ", end="", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print()


# =============================================================================
# Command line handling
# =============================================================================

ALIASES = {"U": "US", "I": "IR", "B": "BI", "Q": "QUIT", "EXIT": "QUIT", "ST": "STOP",
           "W": "WATCH", "S": "SENSORS", "H": "HELP", "?": "HELP"}


def expand(text):
    """'3x FW10 RT45'  -> ['FW10', 'RT45'] * 3 ;  'fw30 s' -> ['FW30', 'SENSORS']"""
    toks = text.upper().split()
    reps = 1
    if toks and re.fullmatch(r"\d+X", toks[0]):
        reps, toks = int(toks[0][:-1]), toks[1:]
    toks = [ALIASES.get(t, t) for t in toks]
    return toks * reps


def run_one(link, cmd):
    """Returns False to quit."""
    if cmd == "QUIT":
        return False
    if cmd == "HELP":
        print(__doc__.split("At the  stm>  prompt:")[1].split("Protocol")[0])
    elif cmd == "SENSORS":
        sensors(link)
    elif cmd == "WATCH":
        watch(link)
    elif cmd == "STOP":
        stop(link)
    else:
        if cmd[:2] in ("FW", "BW") and not re.fullmatch(r"(FW|BW)\d+", cmd):
            print(f"  {cmd}: needs a distance in cm, e.g. FW30")
            return True
        if cmd[:2] in ("LT", "RT", "XL", "XR") and not re.fullmatch(r"(LT|RT|XL|XR)\d+", cmd):
            print(f"  {cmd}: needs an angle, e.g. LT45")
            return True
        if cmd.startswith("AC") and not re.fullmatch(r"AC\d+", cmd):
            print(f"  {cmd}: needs a distance in cm, e.g. AC30")
            return True
        if cmd == "BI":
            print("  BI: calibrating gyro - keep the robot still ...")
        line, lines, secs = send(link, cmd)
        show(cmd, line, lines, secs)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("commands", nargs="*", help="run these and exit")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--log", action="store_true", help=f"append everything to {LOG_FILE}")
    args = ap.parse_args()

    log = open(LOG_FILE, "a", encoding="utf-8") if args.log else None
    try:
        link = Link(args.port, args.baud, log)
    except Exception as e:
        print(f"Cannot open {args.port}: {e}")
        print("Is the STM plugged in, and is rpi_client.py (or another script) still using it?")
        sys.exit(1)
    link.log_line(f"--- session start {datetime.now():%Y-%m-%d %H:%M:%S} on {args.port}")
    print(f"Connected to {args.port} @ {args.baud}.  'help' for commands, Ctrl+C while moving = STOP.")

    try:
        if args.commands:
            for cmd in expand(" ".join(args.commands)):
                if not run_one(link, cmd):
                    break
            return
        last = ""
        while True:
            try:
                text = input("stm> ").strip()
            except EOFError:
                break
            except KeyboardInterrupt:
                print()
                continue
            if not text:
                text = last
                if not text:
                    continue
                print(f"  (repeat: {text})")
            last = text
            keep = True
            for cmd in expand(text):
                keep = run_one(link, cmd)
                if not keep:
                    break
            if not keep:
                break
    finally:
        link.close()
        if log:
            log.close()
        print("Bye.")


if __name__ == "__main__":
    main()
