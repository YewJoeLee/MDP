#!/usr/bin/env python3
"""Task 2 (fastest car) on the RPi - no Android yet.

Runs task2.run_task2() (steps 0 -> 5) with the real STM and camera:
    send(cmd)      -> STM over serial, waits for DONE, stops the run on ERR / STALL / TIMEOUT
    read_us()      -> STM "US"   (cm)
    read_ir_*()    -> STM "IR"   (cm, left / right)
    snap()         -> Picamera2 + YOLO, best Left/Right Arrow box, photo saved in photos/
    read_heading() -> real heading: gyro angle after each turn, re-synced to the STM's
                      own HE (actual - commanded) after every FW / BW / IE

Usage:
    python3 rpi_client_task2.py                # BI, press Enter, run
    python3 rpi_client_task2.py --go           # no Enter prompt
    python3 rpi_client_task2.py --no-bi        # skip the gyro calibration
    python3 rpi_client_task2.py --no-gyro      # trust the turn models in config.py, not the gyro
    python3 rpi_client_task2.py --no-ie        # STM not yet flashed with IE: old FW3 + IR stepping
    python3 rpi_client_task2.py --sim          # laptop: simulated arena from the EDIT HERE block
                                               # in show_path.py (arrows, G1, G2, length)
    python3 rpi_client_task2.py --sim --a1 R --g1 60   # one-off override, --sim only

The real run is never given the arrows, gaps or obstacle 2 length - those
options are refused without --sim.

Screen: one block per step - what it is for, the commands sent (repeats
collapsed: FW3 x8 = 24 cm), photos, the decisions task2.py made, the result.
Everything (every command, every reply, every sensor reading) also goes to
photos/task2_run_<n>.log for debugging.

Ctrl+C at any time sends STOP to the STM.
"""
import argparse
import math
import os
import queue
import re
import sys
import threading
import time
from datetime import datetime

import config as C
import task2 as T

PORT = "/dev/ttyACM0"
BAUD = 115200
MODEL_PATH = "best_ncnn_model"
FRAME_SIZE = (416, 416)
PHOTO_DIR = "photos"

# Which STM IR is on which side - check with stm_console.py (hand beside one side, "ir").
IR_LEFT, IR_RIGHT = "IR1", "IR2"
IR_MAX_CM = 80.0            # Sharp GP2Y0A21 range is 10-80 cm; beyond = "nothing there"
US_MAX_CM = 400.0

ARROW_CLASSES = {"Left Arrow": "LEFT", "Right Arrow": "RIGHT"}
MOVE_RE = re.compile(r"^(FW|BW|LT|RT|XL|XR)(\d+)$")
TURN_SIGN = {"LT": +1, "RT": -1, "XL": -1, "XR": +1}   # + = heading to the left (task2 convention)

STEPS = [
    ("start_to_obs1", "0", "Start -> obstacle 1",
     "Drive out until the ultrasonic is at AC distance from obstacle 1, then read its arrow."),
    ("dodge_plan", "1", "Dodge obstacle 1",
     "Pass obstacle 1 on the arrow's side with one S-curve: 45 out, straight, 45 back."),
    ("pass_obs1_measure_ir", "1b", "Pass obstacle 1 (IR)",
     "Creep past obstacle 1 reading the side IR -> real sideways offset, not an estimate."),
    ("approach_and_snap_obs2", "2", "Approach obstacle 2",
     "Ultrasonic finds obstacle 2 (U1); if it is too short to echo, angled search (U2). Read its arrow."),
    ("loop_obs2", "3", "Loop obstacle 2",
     "Go round obstacle 2 on the arrow's side; the side IR finds both ends, so its length is measured."),
    ("return_to_carpark_edge", "4", "Back to carpark",
     "Shift into the home lane, drive back; ultrasonic locks on the carpark arm, IR re-checks at obstacle 1."),
    ("park_in_carpark", "5", "Park",
     "S-curve onto the carpark centre line, drive in (both IRs check centring), stop off the back wall."),
]


class MoveFailed(Exception):
    pass


# =============================================================================
# Output: clean screen + full log file
# =============================================================================

class Out:
    def __init__(self, log_path):
        self.log = open(log_path, "a", encoding="utf-8")
        self.pending = None            # [cmd, count] waiting to be printed collapsed
        self.t0 = time.time()

    def logf(self, text):
        self.log.write(f"{time.time() - self.t0:7.2f}  {text}\n")
        self.log.flush()

    def flush_cmds(self):
        if self.pending:
            cmd, n, ok_txt = self.pending
            m = MOVE_RE.match(cmd)
            if n > 1 and m and m.group(1) in ("FW", "BW"):
                txt = f"{cmd} x{n} = {int(m.group(2)) * n} cm"
            elif n > 1:
                txt = f"{cmd} x{n}"
            else:
                txt = cmd
            print(f"     > {txt}{ok_txt}")
            self.pending = None

    def cmd(self, cmd, note=""):
        """Commands are printed collapsed: FW3 FW3 FW3 -> 'FW3 x3 = 9 cm'."""
        if self.pending and self.pending[0] == cmd and not note:
            self.pending[1] += 1
            return
        self.flush_cmds()
        self.pending = [cmd, 1, note]

    def say(self, text=""):
        self.flush_cmds()
        print(text)
        self.logf(text)

    def close(self):
        self.flush_cmds()
        self.log.close()


OUT = None


# =============================================================================
# STM link
# =============================================================================

class Stm:
    def __init__(self, port, baud):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.q = queue.Queue()
        self.alive = True
        # Heading, deg from the start heading, + = left.
        #   nominal : sum of the COMMANDED turns (LT45 = +45 ...)
        #   heading : best estimate of the real heading. After a turn: + the gyro angle
        #             the STM reports. After FW/BW/IE: nominal + HE, the STM's own
        #             "actual minus commanded" - because that move may have steered out
        #             part of the error (heading hold), which the Pi cannot see otherwise.
        self.nominal = 0.0
        self.heading = 0.0
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        buf = b""
        while self.alive:
            try:
                chunk = self.ser.read(256)
            except Exception as e:
                self.q.put(f"__ERR__ {e}")
                return
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line:
                    self.q.put(line)

    def _drain(self):
        while True:
            try:
                OUT.logf(f"   << (late) {self.q.get_nowait()}")
            except queue.Empty:
                return

    def ask(self, cmd, done=("DONE", "ERR"), timeout=20.0):
        """Send one command, return the line that finishes it (None on timeout)."""
        self._drain()
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()
        OUT.logf(f">> {cmd}")
        end = time.time() + timeout
        while time.time() < end:
            try:
                line = self.q.get(timeout=0.1)
            except queue.Empty:
                continue
            OUT.logf(f"   << {line}")
            if line.startswith("__ERR__"):
                raise MoveFailed(f"serial error: {line[8:]}")
            if line.startswith(done):
                return line
        return None

    def stop(self):
        try:
            self.ser.write(b"STOP\n")
            self.ser.flush()
            OUT.logf(">> STOP")
            end = time.time() + 2
            while time.time() < end:
                try:
                    line = self.q.get(timeout=0.1)
                    OUT.logf(f"   << {line}")
                    if line == "DONE STOP":
                        break
                except queue.Empty:
                    pass
        except Exception:
            pass

    # ---- what task2.py calls ----
    def send(self, cmd):
        t0 = time.time()
        line = self.ask(cmd, timeout=20.0)
        dt = time.time() - t0
        if line is None:
            raise MoveFailed(f"{cmd}: no reply from the STM after {dt:.0f} s")
        if line.startswith("ERR"):
            raise MoveFailed(f"{cmd}: STM refused it ({line})")
        status = dict(re.findall(r"([A-Z]+):(-?\w+)", line)).get("STATUS", line.split()[-1])
        if status not in ("OK",):
            raise MoveFailed(f"{cmd}: {status}  ({line})")
        m = MOVE_RE.match(cmd)
        note = ""
        if m and m.group(1) in TURN_SIGN:
            imu = re.search(r"IMU:(-?\d+)", line)
            nominal = self._nominal(cmd)
            got = abs(int(imu.group(1))) / 10 if imu else None
            if got is None or abs(got - nominal) > 15:          # nonsense -> use the model
                if got is not None:
                    note = f"   (gyro {got:.1f} deg looks wrong, using {nominal:g})"
                got = nominal
            elif abs(got - nominal) >= 2:
                note = f"   (gyro {got:.1f} deg)"
            self.heading += TURN_SIGN[m.group(1)] * got
            self.nominal += TURN_SIGN[m.group(1)] * nominal
        elif m:                                                 # FW / BW
            self._sync_heading(cmd, line)
        OUT.cmd(cmd, note)
        return line

    def _sync_heading(self, cmd, line):
        """After a straight move: heading = commanded heading + the STM's HE."""
        he = re.search(r"HE:(-?\d+)", line)
        if he is None:
            return
        new = self.nominal + int(he.group(1)) / 10
        OUT.logf(f"   heading {self.heading:+.1f} -> {new:+.1f} deg (commanded {self.nominal:+.1f}, "
                 f"HE {int(he.group(1)) / 10:+.1f}) after {cmd}")
        self.heading = new

    @staticmethod
    def _nominal(cmd):
        return float(MOVE_RE.match(cmd).group(2))

    def drive_edge(self, side, need_seen, max_cm):
        """STM IE command: FW up to max_cm, stop at the edge the side IR sees (main_merged.c)."""
        cmd = f"IE{side[0]}{'N' if need_seen else 'A'}{max(1, min(1000, int(round(max_cm))))}"
        t0 = time.time()
        line = self.ask(cmd, timeout=20.0)
        if line is None:
            raise MoveFailed(f"{cmd}: no reply from the STM after {time.time() - t0:.0f} s")
        if line.startswith("ERR"):
            raise MoveFailed(f"{cmd}: STM refused it ({line}) - old firmware without IE? use --no-ie")
        f = dict(re.findall(r"([A-Z]+):(-?\w+)", line))
        if f.get("STATUS") != "OK":
            raise MoveFailed(f"{cmd}: {f.get('STATUS')}  ({line})")

        def cm(k):
            v = int(f.get(k, -1))
            return None if v < 0 else v / 10
        r = {"end": f.get("END", "MAX"), "edge_cm": cm("EDGE"), "near_cm": cm("NEAR"),
             "moved_cm": cm("MOVED") or 0.0, "cmd": cmd}
        self._sync_heading(cmd, line)
        OUT.cmd(cmd, "   -> " + edge_note(r))
        return r

    def read_us(self):
        line = self.ask("US", done=("US ", "ERR"), timeout=3)
        m = re.search(r"US (-?\d+)", line or "")
        cm = int(m.group(1)) / 10 if m else -1
        return cm if 2 < cm <= US_MAX_CM else None

    def read_ir(self):
        line = self.ask("IR", done=("IR1", "ERR"), timeout=3) or ""
        vals = {n: int(v) / 10 for n, v in re.findall(r"(IR\d) raw:\d+ mm:(-?\d+)", line)}
        clean = {n: (v if 0 < v <= IR_MAX_CM else None) for n, v in vals.items()}
        return clean.get(IR_LEFT), clean.get(IR_RIGHT)

    def read_ir_left(self):
        return self.read_ir()[0]

    def read_ir_right(self):
        return self.read_ir()[1]

    def read_heading(self):
        return self.heading

    def bias(self):
        return self.ask("BI", done=("BIAS", "ERR"), timeout=10)

    def close(self):
        self.alive = False
        try:
            self.ser.close()
        except Exception:
            pass


# =============================================================================
# Camera + YOLO
# =============================================================================

class Camera:
    def __init__(self, run):
        from picamera2 import Picamera2
        from ultralytics import YOLO
        import cv2
        self.cv2, self.run, self.n = cv2, run, 0
        self.cam = Picamera2()
        self.cam.preview_configuration.main.size = FRAME_SIZE
        self.cam.preview_configuration.main.format = "BGR888"
        self.cam.preview_configuration.align()
        self.cam.configure("preview")
        self.cam.start()
        self.model = YOLO(MODEL_PATH)
        self.model(self.cam.capture_array(), conf=C.MODEL_MIN_CONF, imgsz=416, verbose=False)  # warm-up

    def snap(self, step):
        self.n += 1
        frame = self.cam.capture_array()
        res = self.model(frame, conf=C.MODEL_MIN_CONF, iou=0.45, imgsz=416, verbose=False)[0]
        names, best, img = self.model.names, None, frame.copy()
        for i in range(len(res.boxes or [])):
            name = names[int(res.boxes.cls[i])]
            conf = float(res.boxes.conf[i])
            x1, y1, x2, y2 = (int(v) for v in res.boxes.xyxy[i].tolist())
            arrow = ARROW_CLASSES.get(name)
            colour = (0, 255, 0) if arrow else (160, 160, 160)
            self.cv2.rectangle(img, (x1, y1), (x2, y2), colour, 2)
            self.cv2.putText(img, f"{name} {conf:.2f}", (x1, max(12, y1 - 5)),
                             self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
            if arrow and (best is None or conf > best["conf"]):
                best = {"label": arrow, "conf": conf, "box_cx": (x1 + x2) / 2,
                        "img_w": frame.shape[1], "body_edge_px": None}
        tag = f"{best['label']}_{best['conf']:.2f}" if best else "none"
        path = os.path.join(PHOTO_DIR, f"task2_run_{self.run}_snap{self.n}_step{step}_{tag}.jpg")
        self.cv2.imwrite(path, img)
        return best, path

    def close(self):
        try:
            self.cam.stop()
        except Exception:
            pass


# =============================================================================
# Simulated robot (--sim): same output, no hardware
# =============================================================================

class SimRobot:
    def __init__(self, a):
        import sim
        self.w = sim.World(a.g1, a.g2, a.length, a.a1, a.a2, wall=a.wall,
                           ir_fwd=getattr(C, "T2_IR_FWD_CM", 0.0), s=getattr(C, "T2_IR_INSET_CM", 0.0))
        self.n = 0

    def send(self, cmd):
        OUT.logf(f">> {cmd}")
        self.w.send(cmd)
        OUT.cmd(cmd)

    def read_us(self):
        v = self.w.read_us()
        OUT.logf(f"   US {v}")
        return v

    def read_ir_left(self):
        return self.w.read_ir_left()

    def read_ir_right(self):
        return self.w.read_ir_right()

    def snap(self, step):
        self.n += 1
        return self.w.snap(), f"(sim snap {self.n})"

    def drive_edge(self, side, need_seen, max_cm):
        OUT.logf(f">> IE {side} {need_seen} {max_cm}")
        r = self.w.drive_edge(side, need_seen, max_cm)
        OUT.cmd(r["cmd"], "   -> " + edge_note(r))
        return r

    read_heading = None

    def stop(self):
        pass

    def close(self):
        pass


# =============================================================================
# Printing each step
# =============================================================================

def edge_note(r):
    """Short text for an IE result: what stopped it and where."""
    what = {"EDGE": "edge", "NEVER": "nothing beside", "MAX": "no edge before max",
            "WALL": "WALL GUARD"}.get(r["end"], r["end"])
    txt = f"{what}"
    if r.get("edge_cm") is not None:
        txt += f" at {r['edge_cm']:.1f} cm"
    if r.get("near_cm") is not None:
        txt += f", IR beside {r['near_cm']:.1f} cm"
    return txt + f", moved {r['moved_cm']:.1f} cm"


def f1(v):
    return "-" if v is None else (f"{v:.1f}" if isinstance(v, float) else str(v))


def summary(fname, r):
    """One or two lines that say what the step found / decided."""
    g = r.get
    if fname == "start_to_obs1":
        return f"arrow 1 = {g('arrow')} (conf {f1(g('conf'))}) · drove {f1(g('D1'))} cm"
    if fname == "dodge_plan":
        side = "left" if g("cmds", [""])[0].startswith("LT") else "right"
        return (f"pass on the {side}: {' '.join(g('cmds', []))} · IR gap {f1(g('gap_cm'))} cm · "
                f"{'safe' if g('safe') else 'NOT SAFE - not executed'}")
    if fname == "pass_obs1_measure_ir":
        if g("ir_reading") is None:
            return f"IR never saw obstacle 1 -> assumed offset {f1(g('Y0'))} cm"
        return f"IR read {f1(g('ir_reading'))} cm beside obstacle 1 -> robot {f1(g('Y0'))} cm off centre line"
    if fname == "approach_and_snap_obs2":
        why = ("ultrasonic saw obstacle 2" if g("case") == "U1"
               else "no echo -> obstacle 2 is short, angled search")
        return (f"{g('case')}: {why}\n         arrow 2 = {g('arrow')} (conf {f1(g('conf'))}) · "
                f"face {f1(g('d_front'))} cm ahead ({g('face_source')}) · offset {f1(g('Y'))} cm")
    if fname == "loop_obs2":
        why = {"A": "short obstacle seen in the photo -> one S-curve past its end",
               "B": "length unknown -> follow its face with the IR to find the end"}.get(g("play"), "")
        return (f"play {g('play')}: {why}\n         ends {f1(g('near_end'))} / {f1(g('far_end'))} cm · "
                f"length {f1(g('length'))} cm · centre line corrected {f1(g('centre_shift'))} cm")
    if fname == "return_to_carpark_edge":
        src = {"US": "ultrasonic on the arm", "US_ANGLED": "ultrasonic, angled",
               "ODOM": "tracked distance only - no ultrasonic lock"}.get(g("source"), g("source"))
        return (f"stopped {f1(g('ac_cm'))} cm from the carpark arm ({src}) · offset {f1(g('y'))} cm\n"
                f"         IR at obstacle 1: {f1(g('ir_obs1'))} cm (expected {f1(g('expected_ir'))})")
    if fname == "park_in_carpark":
        lr = g("ir_lr")
        irs = f" · IR L/R {lr[0]:.1f}/{lr[1]:.1f}" if lr else ""
        return (f"back wall {f1(g('back_cm'))} cm · off centre {f1(g('y'))} cm{irs} · "
                f"re-entries {g('fixes')} · fully inside: {g('inside')}")
    return ""


def decisions(r):
    """The step's own log minus the plain commands = the reasons."""
    out = []
    for line in r.get("log", []) if isinstance(r, dict) else []:
        s = str(line)
        if MOVE_RE.match(s) or re.match(r"^IE[LR][NA]\d+:", s):
            continue                                  # commands are already printed
        out.append(s)
    return out


def wrap_steps(state):
    """Print a header before each task2 step and its reasons / result after it."""
    for fname, num, title, purpose in STEPS:
        orig = getattr(T, fname)

        def wrapped(*a, _f=fname, _n=num, _t=title, _p=purpose, _o=orig, **k):
            state["step"] = _n
            OUT.say(f"\n== STEP {_n} · {_t} " + "=" * max(4, 48 - len(_t) - len(_n)))
            OUT.say(f"   {_p}")
            t0 = time.time()
            r = _o(*a, **k)
            OUT.flush_cmds()
            for d in decisions(r):
                OUT.say(f"   why: {d}")
            status = r.get("status", "OK" if r.get("safe", True) else "UNSAFE")
            mark = "OK " if status == "OK" else "!! "
            OUT.say(f"   {mark}{summary(_f, r)}")
            if status != "OK":
                OUT.say(f"   !! status {status}")
            if _f == "dodge_plan":
                OUT.say("   sending:")                    # run_task2 sends these right after
            else:
                OUT.say(f"   ({time.time() - t0:.1f} s)")
            state["times"][_n] = time.time() - t0
            return r
        setattr(T, fname, wrapped)


def make_snap(robot, state):
    def snap():
        res, path = robot.snap(state["step"])
        OUT.flush_cmds()
        if res:
            ok = res["conf"] >= C.RECOGNISED_CONF
            OUT.say(f"     [SNAP] {res['label']} {res['conf']:.2f}"
                    f"{'' if ok else '  (below ' + format(C.RECOGNISED_CONF, 'g') + ' - not trusted)'}   {path}")
        else:
            OUT.say(f"     [SNAP] no arrow   {path}")
        return res
    return snap


# =============================================================================
# Main
# =============================================================================

def next_run():
    pat = re.compile(r"^task2_run_(\d+)")
    nums = [int(m.group(1)) for n in os.listdir(PHOTO_DIR) if (m := pat.match(n))]
    return max(nums, default=0) + 1


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--go", action="store_true", help="start without waiting for Enter")
    ap.add_argument("--no-bi", action="store_true", help="skip the gyro bias calibration")
    ap.add_argument("--no-gyro", action="store_true", help="track heading from the turn models only")
    ap.add_argument("--sim", action="store_true", help="simulated arena, no robot")
    ap.add_argument("--no-ie", action="store_true",
                    help="old firmware: follow obstacles with FW3 + IR steps instead of IE")
    # Arena values: --sim ONLY. The real run never knows them (task2.py finds
    # everything with the camera, ultrasonic and IR). Defaults come from the
    # "EDIT HERE" block in show_path.py, so there is one place to edit.
    ap.add_argument("--g1", type=float)
    ap.add_argument("--g2", type=float)
    ap.add_argument("--length", type=float)
    ap.add_argument("--a1")
    ap.add_argument("--a2")
    ap.add_argument("--wall", type=float)
    a = ap.parse_args()
    arena = ("g1", "g2", "length", "a1", "a2", "wall")
    if not a.sim:
        given = [f"--{k}" for k in arena if getattr(a, k) is not None]
        if given:
            sys.exit(f"{' '.join(given)} only work with --sim: the real run must not be told the layout.")
        return a
    import show_path as SP                      # the EDIT HERE block
    defaults = {"g1": SP.G1_CM, "g2": SP.G2_CM, "length": SP.LENGTH_CM,
                "a1": SP.ARROW_1, "a2": SP.ARROW_2, "wall": SP.WALL_CM}
    for k in arena:
        if getattr(a, k) is None:
            setattr(a, k, defaults[k])
    a.a1 = "LEFT" if str(a.a1).upper().startswith("L") else "RIGHT"
    a.a2 = "LEFT" if str(a.a2).upper().startswith("L") else "RIGHT"
    return a


def main():
    global OUT
    a = parse_args()
    os.makedirs(PHOTO_DIR, exist_ok=True)
    run = next_run()
    log_path = os.path.join(PHOTO_DIR, f"task2_run_{run}.log")
    OUT = Out(log_path)
    OUT.logf(f"--- Task 2 run {run}  {datetime.now():%Y-%m-%d %H:%M:%S}  {'SIM' if a.sim else a.port}")

    print(f"TASK 2 · run {run} · {'SIMULATION' if a.sim else 'robot on ' + a.port}")
    print(f"   AC at obstacle 1 {C.T2_AC_OBS1_CM:g} cm · IR gap {C.T2_IR_GAP_OBS1_CM:g} cm · "
          f"min clearance {C.MIN_CLEARANCE_CM:g} cm · arrow trusted at conf >= {C.RECOGNISED_CONF:g}")
    print(f"   IR legs: {'FW3 + IR steps (--no-ie)' if a.no_ie else 'one IE command each'}")
    print(f"   log: {log_path}")

    robot = cam = None
    try:
        if a.sim:
            robot = SimRobot(a)
            print(f"   sim arena: G1 {a.g1:g} · G2 {a.g2:g} · L {a.length:g} · wall {a.wall:g} · "
                  f"arrows {a.a1}/{a.a2}")
        else:
            try:
                robot = Stm(a.port, BAUD)
            except Exception as e:
                print(f"!! cannot open {a.port}: {e}  (STM plugged in? another script using it?)")
                return 1
            print("   camera + YOLO loading ...")
            try:
                cam = Camera(run)
            except Exception as e:
                print(f"!! camera / model failed: {e}")
                return 1
            robot.snap = cam.snap
            if not a.no_bi:
                print("   BI: gyro calibration - keep the robot still ...")
                line = robot.bias()
                print(f"   {line}")
                if line is None or "UNSTABLE" in line:
                    print("!! gyro bias not OK - run again with the robot still (or --no-bi)")
                    return 1
            print("   US {} cm · IR L/R {}".format(f1(robot.read_us()),
                                                   " / ".join(f1(v) for v in robot.read_ir())))

        if not a.go:
            input("\nRobot in the carpark, facing obstacle 1. Enter = START (Ctrl+C = STOP) ")

        state = {"step": "0", "times": {}}
        wrap_steps(state)
        heading = None if (a.sim or a.no_gyro) else robot.read_heading
        t0 = time.time()
        out = T.run_task2(robot.send, robot.read_us, robot.read_ir_left, robot.read_ir_right,
                          make_snap(robot, state), read_heading=heading,
                          drive_edge=None if a.no_ie else robot.drive_edge)
        total = time.time() - t0

        OUT.say("\n" + "=" * 60)
        arrows = out.get("arrows") or (out.get("start", {}).get("arrow"), None)
        OUT.say(f"RESULT  {out['status']}  ·  {total:.1f} s  ·  arrows {arrows[0]} / {arrows[1]}")
        OUT.say("   step times: " + "  ".join(f"{k}:{v:.1f}s" for k, v in state["times"].items()))
        if a.sim:
            p = robot.w.p
            mc = robot.w.minclear
            OUT.say(f"   sim: final pose ({p.x:.1f}, {p.y:.1f}, {p.h:.0f} deg) · closest {mc[0]:.1f} cm "
                    f"to {mc[1]} during {mc[2]}")
        return 0 if out["status"] == "OK" else 2

    except MoveFailed as e:
        OUT.say(f"\n!! STOPPED: {e}")
        robot.stop()
        return 3
    except KeyboardInterrupt:
        OUT.say("\n!! Ctrl+C - STOP sent")
        if robot:
            robot.stop()
        return 130
    finally:
        if cam:
            cam.close()
        if robot:
            robot.close()
        OUT.close()


if __name__ == "__main__":
    sys.exit(main())
