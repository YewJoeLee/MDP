# V5: execution uses stationary bursts + checked backup + replanning.
# Legacy correction/AC controls are retained but do not affect V5 execution.
#!/usr/bin/env python3
"""
simulator.py  -  MDP Task 1 (Explore) simulator.  Run:  python simulator.py

Put this file next to planner.py, config.py and rpi_stm_conn.py (algo_rpi_code).

What it does
------------
* Shows the 200 x 200 cm arena, the 30 x 30 start box and the robot at the
  START_POSE_CM from config.py.
* Plans with your real planner.py using the CURRENT config values. Sliders on
  the right change those values live (in memory); the plan is recomputed.
  Nothing is written to config.py unless you press "Save to config.py"
  (a timestamped backup is made first).
* Executes the plan with your REAL rpi_stm_conn.run_commands(): the same
  SNAP -> (fail) -> correct_snap_angle -> AC20 -> SNAP -> RA logic that runs on
  the RPi. Only the hardware underneath is simulated: a fake STM (drive, turn,
  AC20 ultrasonic align, RA), a fake camera and a fake ultrasonic sensor.
* The simulated robot is NOT perfect. Turn/straight/drift errors are adjustable
  ("Hardware" tab), so the true robot drifts away from the planned pose (grey
  dashed ghost) and photos can genuinely fail.
* Click the arena to add / rotate / remove obstacles.

Everything about the hardware error model and the camera model is an
ASSUMPTION for testing the logic. Tune it against measurements from the robot.
Standard library only (tkinter).
"""
import ast
import contextlib
import copy
import math
import os
import queue
import random
import re
import shutil
import sys
import threading
import time
import types
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    HAVE_TK = True
except Exception:          # engine still usable without a display / tkinter
    HAVE_TK = False
    tk = None
    ttk = types.SimpleNamespace(Frame=object)

import config
import planner

try:
    import serial  # noqa: F401
except ImportError:        # pyserial is not needed: the STM is simulated
    _stub = types.ModuleType("serial")
    _stub.Serial = object
    sys.modules["serial"] = _stub
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "rpi"))
import rpi_stm_conn as stm

try:
    import test_maps
except Exception:
    test_maps = None

# ---------------------------------------------------------------------------
# config.py keys the simulator can edit / reset / save
# ---------------------------------------------------------------------------
CFG_KEYS = [
    "GRID", "ROBOT_WIDTH_CM", "ROBOT_LENGTH_CM", "MIN_CLEARANCE_CM",
    "POSITION_MARGIN_CM", "BOUNDARY_MARGIN_CM", "START_POSE_CM", "TURN_MODELS",
    "MAX_STRAIGHT_COMMAND_CM", "STRAIGHT_COST_PER_CM", "TURN_COST",
    "PLAN_TIME_LIMIT_S", "ALIGN_ENABLED", "SENSOR_FORWARD_CM",
    "ALIGN_MAX_TRAVEL_CM", "VIEW_SENSOR_DISTANCES_CM",
]
ORIG = {k: copy.deepcopy(getattr(config, k)) for k in CFG_KEYS}

# ---------------------------------------------------------------------------
# Simulation-only parameters (NOT in config.py)
# ---------------------------------------------------------------------------
SIM_DEFAULTS = dict(
    # hardware error model
    turn_bias_deg=0.0,      # + overshoot / - undershoot of every 90 degree turn
    turn_sd_deg=0.0,        # random turn error (std dev, degrees)
    turn_pos_sd_cm=0.0,     # random error of the turn end position (cm)
    straight_scale_pct=0.0, # + drives too far, - too short (percent)
    straight_sd_cm=0.0,     # random straight-distance error (cm)
    drift_deg_per_100=0.0,  # heading drift while driving straight (deg / 100 cm)
    drift_sd_deg=0.0,       # random extra drift (deg per 100 cm)
    us_sd_mm=0.0,           # ultrasonic reading noise (mm)
    us_cone_deg=0.0,        # >0: also sample rays +/- this angle, keep the nearest
    ac_cap_cm=10,           # firmware AC20 total travel cap (10 = current firmware)
    seed=1,
    # camera model (a guess - tune it)
    cam_fov=60.0,           # horizontal field of view (deg)
    cam_ideal_min=18.0,     # best camera->face distance range (cm)
    cam_ideal_max=32.0,
    cam_angle_tol=35.0,     # viewing angle at which confidence hits 0 (deg)
    cam_max_conf=0.92,
    conf_threshold=0.25,    # rpi_client_android_v2 uses conf=0.25
    force="photo1",         # "physics" | "photo1" | "both"
    # RPi behaviour switches (patched into rpi_stm_conn while simulating)
    angle_correction=True,
    nudge_cm=10,
    approach=True,          # send AC20 after a failed first photo
    return_=True,           # send RA after the second photo
)

PRESETS = {
    "Perfect robot": dict(turn_bias_deg=0, turn_sd_deg=0, turn_pos_sd_cm=0,
                          straight_scale_pct=0, straight_sd_cm=0,
                          drift_deg_per_100=0, drift_sd_deg=0, us_sd_mm=0),
    "Realistic (small errors)": dict(turn_bias_deg=0, turn_sd_deg=2.0,
                                     turn_pos_sd_cm=1.0, straight_scale_pct=-1.0,
                                     straight_sd_cm=0.5, drift_deg_per_100=0.5,
                                     drift_sd_deg=0.3, us_sd_mm=5),
    "Turns undershoot ~8 deg": dict(turn_bias_deg=-8, turn_sd_deg=1.5,
                                    turn_pos_sd_cm=1.0, straight_scale_pct=0,
                                    straight_sd_cm=0.3, drift_deg_per_100=1.0,
                                    drift_sd_deg=0.3, us_sd_mm=5),
    "Turns overshoot ~8 deg": dict(turn_bias_deg=8, turn_sd_deg=1.5,
                                   turn_pos_sd_cm=1.0, straight_scale_pct=0,
                                   straight_sd_cm=0.3, drift_deg_per_100=-1.0,
                                   drift_sd_deg=0.3, us_sd_mm=5),
    "Turns undershoot ~20 deg (bad)": dict(turn_bias_deg=-20, turn_sd_deg=3.0,
                                           turn_pos_sd_cm=1.5, straight_scale_pct=0,
                                           straight_sd_cm=0.3, drift_deg_per_100=1.0,
                                           drift_sd_deg=0.3, us_sd_mm=5),
    "Turns overshoot ~20 deg (bad)": dict(turn_bias_deg=20, turn_sd_deg=3.0,
                                          turn_pos_sd_cm=1.5, straight_scale_pct=0,
                                          straight_sd_cm=0.3, drift_deg_per_100=-1.0,
                                          drift_sd_deg=0.3, us_sd_mm=5),
    "Sloppy": dict(turn_bias_deg=-4, turn_sd_deg=4.0, turn_pos_sd_cm=2.0,
                   straight_scale_pct=2.0, straight_sd_cm=1.0,
                   drift_deg_per_100=2.0, drift_sd_deg=1.0, us_sd_mm=10),
}

FACE_N = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
HEAD_DEG = {"N": 0.0, "E": 90.0, "S": 180.0, "W": 270.0}
CPC_A, CPC_B = 72.26, 75.48          # encoder counts per cm (stm_code.c), for the fake DONE line


def letter(th):
    return "NESW"[int(round((th % 360.0) / 90.0)) % 4]


def fvec(th):
    r = math.radians(th)
    return math.sin(r), math.cos(r)


def rvec(th):
    r = math.radians(th)
    return math.cos(r), -math.sin(r)


def body_poly(x, y, th):
    """Robot footprint corners (world cm). Width is left/right, length is front/back."""
    hw, hl = config.ROBOT_WIDTH_CM / 2.0, config.ROBOT_LENGTH_CM / 2.0
    fx, fy = fvec(th)
    rx, ry = rvec(th)
    pts = []
    for right, ahead in ((hw, hl), (hw, -hl), (-hw, -hl), (-hw, hl)):
        pts.append((x + rx * right + fx * ahead, y + ry * right + fy * ahead))
    return pts


def _pt_seg(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def poly_rect_gap(poly, rect):
    """Distance between a convex polygon and an axis-aligned rect (0 if they overlap)."""
    x0, y0, x1, y1 = rect
    rp = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    def axes(p):
        for i in range(len(p)):
            (x, y), (u, v) = p[i], p[(i + 1) % len(p)]
            yield (-(v - y), u - x)

    overlap = True
    for ax in list(axes(poly)) + list(axes(rp)):
        pa = [px * ax[0] + py * ax[1] for px, py in poly]
        pb = [px * ax[0] + py * ax[1] for px, py in rp]
        if max(pa) < min(pb) or max(pb) < min(pa):
            overlap = False
            break
    if overlap:
        return 0.0
    best = math.inf
    for pts, segs in ((poly, rp), (rp, poly)):
        for p in pts:
            for i in range(len(segs)):
                best = min(best, _pt_seg(p, segs[i], segs[(i + 1) % len(segs)]))
    return best


def ray_dist(o, th, rects, ext):
    """Distance along heading th from o to the nearest obstacle rect or arena wall (cm)."""
    dx, dy = fvec(th)
    best = math.inf
    for (x0, y0, x1, y1) in rects:
        tmin, tmax = 0.0, math.inf
        ok = True
        for oo, dd, lo, hi in ((o[0], dx, x0, x1), (o[1], dy, y0, y1)):
            if abs(dd) < 1e-12:
                if oo < lo or oo > hi:
                    ok = False
                    break
            else:
                t1, t2 = (lo - oo) / dd, (hi - oo) / dd
                if t1 > t2:
                    t1, t2 = t2, t1
                tmin, tmax = max(tmin, t1), min(tmax, t2)
        if ok and tmin <= tmax:
            best = min(best, tmin)
    for oo, dd in ((o[0], dx), (o[1], dy)):
        if dd > 1e-12:
            best = min(best, (ext - oo) / dd)
        elif dd < -1e-12:
            best = min(best, (0 - oo) / dd)
    return best


# ---------------------------------------------------------------------------
# Robot motion (true hardware, with errors) and the ideal model (planner belief)
# ---------------------------------------------------------------------------
class Mover:
    def __init__(self, x, y, th, sp, rng, noisy=True):
        self.x, self.y, self.th = float(x), float(y), float(th)
        self.sp, self.rng, self.noisy = sp, rng, noisy

    def pose(self):
        return (self.x, self.y, self.th)

    def straight(self, d):
        """Drive d cm (+ forward, - backward). Returns the list of poses (1 cm apart)."""
        sign = 1 if d >= 0 else -1
        dist = abs(d)
        sp = self.sp
        if self.noisy:
            act = max(0.0, dist * (1 + sp["straight_scale_pct"] / 100.0)
                      + self.rng.gauss(0, sp["straight_sd_cm"]))
            yaw = (sp["drift_deg_per_100"] * act / 100.0
                   + self.rng.gauss(0, sp["drift_sd_deg"]) * math.sqrt(act / 100.0))
        else:
            act, yaw = float(dist), 0.0
        n = max(1, int(math.ceil(act)))
        step = act / n
        th0 = self.th
        out = []
        for i in range(1, n + 1):
            th = th0 + yaw * i / n
            fx, fy = fvec(th)
            self.x += fx * step * sign
            self.y += fy * step * sign
            self.th = th
            out.append((self.x, self.y, self.th))
        return out

    def turn(self, name):
        """One 90 degree turn using config.TURN_MODELS. Returns (poses, actual_angle_deg)."""
        a, r, q = config.TURN_MODELS[name]
        sp = self.sp
        phi, e0, e1 = 90.0, 0.0, 0.0
        if self.noisy:
            phi = max(10.0, 90.0 + sp["turn_bias_deg"] + self.rng.gauss(0, sp["turn_sd_deg"]))
            e0 = self.rng.gauss(0, sp["turn_pos_sd_cm"])
            e1 = self.rng.gauss(0, sp["turn_pos_sd_cm"])
        frac = phi / 90.0
        n = 30
        x0, y0, th0 = self.x, self.y, self.th
        fx, fy = fvec(th0)
        rx, ry = rvec(th0)
        out = []
        for k in range(1, n + 1):
            u = k / n
            t = u * frac * math.pi / 2
            ahead = a * math.sin(t) + e0 * u
            right = r * (1 - math.cos(t)) + e1 * u
            self.x = x0 + fx * ahead + rx * right
            self.y = y0 + fy * ahead + ry * right
            self.th = th0 + q * phi * u
            out.append((self.x, self.y, self.th))
        return out, phi


def nominal_polyline(start, commands):
    """Noise-free path (list of (x,y)) of a command list, turns sampled as arcs."""
    m = Mover(start[0], start[1], HEAD_DEG[start[2]], SIM_DEFAULTS, random.Random(0), noisy=False)
    pts = [(m.x, m.y)]
    for c in commands:
        u = c.upper()
        mm = re.fullmatch(r"(FW|BW)(\d+)", u)
        if mm:
            path = m.straight((1 if mm.group(1) == "FW" else -1) * int(mm.group(2)))
            pts.extend((p[0], p[1]) for p in path[::max(1, len(path) // 6)] + path[-1:])
        elif u in config.TURN_MODELS:
            path, _ = m.turn(u)
            pts.extend((p[0], p[1]) for p in path)
    return pts, m.pose()


# ---------------------------------------------------------------------------
# Recording
# ---------------------------------------------------------------------------
class Run:
    def __init__(self):
        self.frames = []
        self.events = []
        self.photos = []        # dicts: frame, oid, attempt, ok, conf, forced
        self.attempts = {}      # oid -> [(ok, conf), ...]
        self.hazards = []       # (kind, who, gap)
        self.closest = None     # (gap, who) smallest obstacle clearance seen
        self.completed = True
        self.corrections = 0
        self.summary = {}


class _Tee:
    def __init__(self, sim):
        self.sim, self.buf = sim, ""

    def write(self, s):
        self.buf += s
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            self.sim.log(line)
        return len(s)

    def flush(self):
        pass


class _DummySer:
    def close(self):
        pass


SIM_LOCK = threading.Lock()


class Sim:
    """A fake robot. Its .send() replaces rpi_stm_conn.send_and_wait."""

    def __init__(self, obstacles, plan, sp, record=True, hazard_step=1):
        self.plan_v5 = plan
        self.sp = sp
        self.rec = record
        self.hstep = max(1, hazard_step)
        self.rng = random.Random(sp["seed"])
        self.obs = {o[0]: o for o in obstacles}
        self.rects = {o[0]: planner.obstacle_rect(o) for o in obstacles}
        self.ext = config.GRID * config.CELL_CM
        sx, sy, sh = plan["start"]
        self.start = (sx, sy, sh)
        self.true = Mover(sx, sy, HEAD_DEG[sh], sp, self.rng, True)
        self.ideal = Mover(sx, sy, HEAD_DEG[sh], sp, self.rng, False)
        self.run = Run()
        self.ap_moved = 0
        self.cur_ob = None
        self.rb = math.hypot(config.ROBOT_WIDTH_CM / 2.0, config.ROBOT_LENGTH_CM / 2.0)
        self.hz_n = 0

    # ---- recording helpers -------------------------------------------------
    def event(self, text, tag="log"):
        if self.rec:
            r = self.run
            r.events.append(dict(text=text, tag=tag, f0=len(r.frames), f1=len(r.frames)))

    def _frame(self, pose, planned, src=None, **kw):
        if not self.rec:
            return
        r = self.run
        r.frames.append(dict(pose=pose, planned=planned, ev=len(r.events) - 1, src=src, **kw))
        r.events[-1]["f1"] = len(r.frames) - 1

    def log(self, text, tag="log"):
        if not self.rec:
            return
        self.event(text, tag)
        self._frame(self.true.pose(), self.ideal.pose())

    # ---- clearance / collision bookkeeping ----------------------------------
    def hazard(self, pose):
        x, y, th = pose
        clr = config.MIN_CLEARANCE_CM
        poly = None
        gmin, who = math.inf, None
        for oid, rect in self.rects.items():
            gc = planner.rect_gap((x, y, x, y), rect)
            if gc - self.rb > clr + 0.5:
                continue
            if poly is None:
                poly = body_poly(x, y, th)
            g = poly_rect_gap(poly, rect)
            if g < gmin:
                gmin, who = g, oid
        obst_gap = (gmin, who)
        wall_gap = None
        if min(x, y, self.ext - x, self.ext - y) - self.rb <= 1.0:
            poly = poly or body_poly(x, y, th)
            wg = min(min(p[0] for p in poly), min(p[1] for p in poly),
                     self.ext - max(p[0] for p in poly), self.ext - max(p[1] for p in poly))
            wall_gap = wg
        res = None
        if who is not None:
            r = self.run
            if r.closest is None or gmin < r.closest[0]:
                r.closest = (gmin, who)
            if gmin <= 0:
                res = ("collision", who, gmin)
            elif gmin < clr:
                res = ("close", who, gmin)
        if wall_gap is not None:
            if wall_gap < 0:
                res = ("collision", "wall", wall_gap)
            elif wall_gap < config.POSITION_MARGIN_CM + config.BOUNDARY_MARGIN_CM and res is None:
                res = ("close", "wall", wall_gap)
        return res

    def _path_frames(self, path, ideal_path, src, **kw):
        """Add one frame per sample of a movement; returns the worst hazard seen."""
        n = len(path)
        worst = None
        for i, pose in enumerate(path):
            hz = None
            if i % self.hstep == 0 or i == n - 1:
                hz = self.hazard(pose)
                if hz and (worst is None or hz[0] == "collision" and worst[0] != "collision"
                           or (hz[0] == worst[0] and hz[2] < worst[2])):
                    worst = hz
            if self.rec:
                if ideal_path:
                    j = min(len(ideal_path) - 1, max(0, int(round((i + 1) / n * len(ideal_path))) - 1))
                    planned = ideal_path[j]
                else:
                    planned = self.ideal.pose()
                self._frame(pose, planned, src, hz=hz[0] if hz else None, **kw)
        return worst

    def _report_hazard(self, worst):
        if not worst:
            return
        kind, who, gap = worst
        self.run.hazards.append(worst)
        name = "the wall" if who == "wall" else f"obstacle {who}"
        if kind == "collision":
            self.log(f"!! COLLISION with {name}", "warn")
        else:
            lim = config.MIN_CLEARANCE_CM if who != "wall" else config.POSITION_MARGIN_CM + config.BOUNDARY_MARGIN_CM
            self.log(f"!  too close to {name}: {gap:.1f} cm (clearance setting {lim:g} cm)", "warn")
        if self.rec:
            self.run.frames[-1]["hz"] = kind

    # ---- ultrasonic -------------------------------------------------------
    def us_read(self, remember=True):
        x, y, th = self.true.pose()
        fx, fy = fvec(th)
        S = (x + fx * config.SENSOR_FORWARD_CM, y + fy * config.SENSOR_FORWARD_CM)
        cone = self.sp["us_cone_deg"]
        angs = [0.0] if cone <= 0 else [-cone, 0.0, cone]
        best = min(ray_dist(S, th + a, list(self.rects.values()), self.ext) for a in angs)
        mm = int(round(best * 10 + self.rng.gauss(0, self.sp["us_sd_mm"])))
        mm = mm if mm > 0 else -1
        if self.rec and remember:
            end = (S[0] + fx * best, S[1] + fy * best)
            self._frame(self.true.pose(), self.ideal.pose(), None,
                        beam=(S, end, f"US {mm} mm" if mm > 0 else "US no echo"), us=mm)
        return mm

    # ---- fake STM ---------------------------------------------------------
    def send(self, ser, cmd):
        caller = sys._getframe(1).f_code.co_name
        src = "plan" if caller == "run_commands" else ("correct" if caller == "do" else "helper")
        up = cmd.upper().strip()
        try:
            if up.startswith("AC"):
                line = self.do_ac(cmd)
            elif re.fullmatch(r"RA\d*", up):
                line = self.do_ra(cmd)
            else:
                line = self.do_move(cmd, up, src)
        except Exception as e:                       # never crash the simulation
            line = f"ERR SIM [{cmd}] {e}"
        print(f"  <- {line}")
        return (not line.startswith("ERR")), line

    def do_move(self, cmd, up, src):
        tag = "plan" if src == "plan" else "correct"
        if src == "correct":
            self.run.corrections += 1
        label = "plan" if src == "plan" else "angle correction"
        m = re.fullmatch(r"(FW|BW)(\d+)", up)
        if m:
            d = int(m.group(2))
            if d <= 0:
                return f"ERR BADDIST [{cmd}]"
            sign = 1 if m.group(1) == "FW" else -1
            self.event(f"{cmd}   ({label})", tag)
            path = self.true.straight(sign * d)
            ideal = self.ideal.straight(sign * d) if src == "plan" else None
            worst = self._path_frames(path, ideal, tag)
            self._report_hazard(worst)
            return f"DONE {cmd} A:{int(d * CPC_A)} B:{int(d * CPC_B)} HE:0 OK"
        if up in config.TURN_MODELS:
            self.event(f"{cmd}   ({label})", tag)
            path, phi = self.true.turn(up)
            ideal = self.ideal.turn(up)[0] if src == "plan" else None
            worst = self._path_frames(path, ideal, tag)
            self._report_hazard(worst)
            return f"DONE {cmd} IMU:{int(phi * 10)}"
        return f"ERR UNKNOWN [{cmd}]"

    def drive(self, d, tag):
        path = self.true.straight(d)
        worst = self._path_frames(path, None, tag)
        self._report_hazard(worst)

    def do_ac(self, cmd):
        target = int(re.sub(r"\D", "", cmd) or 0) * 10
        self.event(f"{cmd}   (ultrasonic align)", "ac")
        d = self.us_read()
        self.ap_moved = 0
        cap = self.sp["ac_cap_cm"]
        travelled = 0                 # sum of |steps|, both directions (matches stm_code.c)
        while 0 < d < 1000 and travelled < cap:
            err = d - target
            if -20 < err < 20:        # +/-2 cm tolerance
                break
            cm = min(abs(err) // 10, cap - travelled)
            if cm <= 0:
                break
            sdir = 1 if err > 0 else -1
            self.drive(sdir * cm, "ac")
            self.ap_moved += sdir * cm
            travelled += cm
            d = self.us_read()
        if 0 < d < 1000:
            return f"DONE {cmd} US:{d} MOVED:{self.ap_moved}"
        return f"DONE {cmd} NOOBJ MOVED:{self.ap_moved}"

    def do_ra(self, cmd):
        back = self.ap_moved
        self.event(f"{cmd}   (return after align: undo {back:+d} cm)", "ra")
        if back != 0:
            self.drive(-back, "ra")
        else:
            self._frame(self.true.pose(), self.ideal.pose(), "ra")
        self.ap_moved = 0
        return f"DONE {cmd} UNDID:{back}"

    # ---- fake camera -------------------------------------------------------
    def camera_view(self, oid):
        x, y, th = self.true.pose()
        fx, fy = fvec(th)
        P = (x + fx * config.SENSOR_FORWARD_CM, y + fy * config.SENSOR_FORWARD_CM)
        _, gx, gy, face = self.obs[oid]
        s = config.CELL_CM
        C = ((gx + .5) * s, (gy + .5) * s)
        n = FACE_N[face]
        F = (C[0] + n[0] * s / 2, C[1] + n[1] * s / 2)
        v = (F[0] - P[0], F[1] - P[1])
        dist = math.hypot(*v)
        dot = fx * v[0] + fy * v[1]
        cross = fx * v[1] - fy * v[0]
        bearing = -math.degrees(math.atan2(cross, dot))          # + = face is to the right
        dotn = max(-1.0, min(1.0, -(fx * n[0] + fy * n[1])))
        view_err = math.degrees(math.acos(dotn))                 # 0 = looking straight at the face
        in_front = (P[0] - F[0]) * n[0] + (P[1] - F[1]) * n[1] > 0
        sp = self.sp
        lo, hi = sp["cam_ideal_min"], sp["cam_ideal_max"]
        if dist < lo:
            f_d = max(0.0, (dist - 6.0) / max(1e-6, lo - 6.0))
        elif dist > hi:
            f_d = max(0.0, 1.0 - (dist - hi) / max(1e-6, 70.0 - hi))
        else:
            f_d = 1.0
        half = sp["cam_fov"] / 2.0
        b = abs(bearing)
        f_b = 1.0 if b <= half / 2 else max(0.0, 1.0 - (b - half / 2) / (half / 2))
        tol = sp["cam_angle_tol"]
        f_v = 1.0 if view_err <= 10 else max(0.0, 1.0 - (view_err - 10) / max(1e-6, tol - 10))
        conf = sp["cam_max_conf"] * f_d * f_b * f_v if (in_front and dot > 0) else 0.0
        return dict(conf=conf, bearing=bearing, view_err=view_err, dist=dist, P=P, th=th,
                    f_d=f_d, f_b=f_b, f_v=f_v)

    def on_snap(self, oid_text):
        oid = int(oid_text)
        self.cur_ob = oid
        print(f"    [CAMERA] capture image for obstacle {oid}")
        if oid not in self.obs:
            print("    [DETECT] nothing found (unknown obstacle)")
            return False
        att = len(self.run.attempts.setdefault(oid, [])) + 1
        v = self.camera_view(oid)
        conf, forced = v["conf"], False
        force = self.sp["force"]
        if (force == "photo1" and att == 1) or (force == "both" and att <= 2):
            conf, forced = min(conf, 0.10), True
        ok = conf >= self.sp["conf_threshold"]
        sym = 11 + oid % 9
        if ok:
            print(f"    [DETECT] Number {oid % 9 + 1} -> {sym} ({conf:.2f})")
            print(f"    [ANDROID] TARGET,{oid},{sym}")
        else:
            print(f"    [DETECT] nothing found (best conf {conf:.2f} < {self.sp['conf_threshold']:.2f})"
                  + ("  [forced fail]" if forced else ""))
            print(f"    [ANDROID] MSG,Obstacle {oid}: no image recognised")
        self.run.attempts[oid].append((ok, conf))
        tp, ip = self.true.pose(), self.ideal.pose()
        off = math.hypot(tp[0] - ip[0], tp[1] - ip[1])
        dth = ((tp[2] - ip[2] + 180) % 360) - 180
        self.event(f"SNAP{oid} photo #{att}: {'RECOGNISED' if ok else 'FAILED'}  conf {conf:.2f}"
                   f"{' (forced)' if forced else ''}   [{off:.1f} cm / {dth:+.1f} deg from plan]", "photo_ok" if ok else "photo_bad")
        self._frame(tp, ip, None, fov=(v["P"], v["th"], self.sp["cam_fov"]),
                    photo=dict(oid=oid, attempt=att, ok=ok, conf=conf, forced=forced, view=v))
        if self.rec:
            self.run.photos.append(dict(frame=len(self.run.frames) - 1, oid=oid, attempt=att, ok=ok, conf=conf))
        return ok

    def check_side(self):
        oid = self.cur_ob
        if oid is None or oid not in self.obs:
            return None
        x, y, th = self.true.pose()
        fx, fy = fvec(th)
        P = (x + fx * config.SENSOR_FORWARD_CM, y + fy * config.SENSOR_FORWARD_CM)
        _, gx, gy, _ = self.obs[oid]
        s = config.CELL_CM
        C = ((gx + .5) * s, (gy + .5) * s)
        v = (C[0] - P[0], C[1] - P[1])
        dot = fx * v[0] + fy * v[1]
        cross = fx * v[1] - fy * v[0]
        bearing = -math.degrees(math.atan2(cross, dot))
        half = self.sp["cam_fov"] / 2.0
        if dot <= 0 or abs(bearing) > half:
            print("    [CAMERA] obstacle not in view (largest dark region too small)")
            return None
        px = 208 + 208 * math.tan(math.radians(bearing)) / math.tan(math.radians(half))
        side = "left" if px < 208 else "right"
        print(f"    [CAMERA] obstacle body centre {px:.0f}px vs frame centre 208px -> {side}")
        return side

    # ---- run the REAL rpi_stm_conn.run_commands -----------------------------
    def execute(self, commands):
        sx, sy, sh = self.start
        self.event(f"START  robot at ({sx:.1f}, {sy:.1f}) facing {sh}", "start")
        self._frame(self.true.pose(), self.ideal.pose())
        sp = self.sp
        names = ["open_serial", "send_and_wait", "time", "ANGLE_CORRECTION_ENABLED",
                 "ANGLE_NUDGE_CM", "APPROACH_CMD", "RETURN_CMD"]
        with SIM_LOCK:
            saved = {n: getattr(stm, n) for n in names}
            try:
                stm.open_serial = lambda: _DummySer()
                stm.send_and_wait = self.send
                stm.time = types.SimpleNamespace(sleep=lambda s: None, time=time.time)
                stm.ANGLE_CORRECTION_ENABLED = bool(sp["angle_correction"])
                stm.ANGLE_NUDGE_CM = int(sp["nudge_cm"])
                stm.APPROACH_CMD = "AC20" if sp["approach"] else None
                stm.RETURN_CMD = "RA" if sp["return_"] else None
                tee = _Tee(self)
                with contextlib.redirect_stdout(tee):
                    def replan(pose, order):
                        return planner.plan(list(self.obs.values()), start_pose=pose,
                                            target_ids=order, preferred_order=order)
                    ok = stm.run_plan_v5(self.plan_v5, self.on_snap, replan,
                        lambda oid: print(f"[ANDROID] MSG,Obstacle {oid}: no image found"))
                self.run.completed = bool(ok)
            finally:
                for n, v in saved.items():
                    setattr(stm, n, v)
        tp, ip = self.true.pose(), self.ideal.pose()
        drift = math.hypot(tp[0] - ip[0], tp[1] - ip[1])
        self.event(f"END    robot is {drift:.1f} cm from where the planner thinks it is", "end")
        self._frame(tp, ip)
        return self.run


def simulate(obstacles, plan, sp, record=True, hazard_step=1):
    sim = Sim(obstacles, plan, sp, record, hazard_step)
    run = sim.execute(plan["commands"])
    tp, ip = sim.true.pose(), sim.ideal.pose()
    run.summary = dict(
        drift_end=math.hypot(tp[0] - ip[0], tp[1] - ip[1]),
        drift_deg=abs(((tp[2] - ip[2] + 180) % 360) - 180),
        collisions=sum(1 for h in run.hazards if h[0] == "collision"),
        close=sum(1 for h in run.hazards if h[0] == "close"),
    )
    status = {}
    for oid in plan["order"]:
        att = run.attempts.get(oid, [])
        if att and att[0][0]:
            status[oid] = "ok1"
        elif len(att) > 1 and att[1][0]:
            status[oid] = "ok2"
        elif att:
            status[oid] = "fail"
        else:
            status[oid] = "none"
    run.summary["status"] = status
    return run


def monte_carlo(obstacles, plan, sp, n, progress=None):
    """Repeat the run with n random seeds for three retry strategies."""
    variants = [
        ("A  as it is now: correction + AC20", dict(angle_correction=True, approach=True)),
        ("B  AC20 only (no angle correction)", dict(angle_correction=False, approach=True)),
        ("C  just photograph twice (no moves)", dict(angle_correction=False, approach=False)),
    ]
    out = []
    total = len(variants) * n
    done = 0
    for name, ov in variants:
        per = {oid: dict(ok1=0, ok2=0, fail=0) for oid in plan["order"]}
        coll = close = 0
        drift = []
        for i in range(n):
            s2 = dict(sp)
            s2.update(ov)
            s2["seed"] = int(sp["seed"]) * 100003 + i
            s2["force"] = "physics"
            run = simulate(obstacles, plan, s2, record=False, hazard_step=3)
            for oid, st in run.summary["status"].items():
                per[oid]["fail" if st == "none" else st] += 1
            coll += 1 if run.summary["collisions"] else 0
            close += 1 if run.summary["close"] else 0
            drift.append(run.summary["drift_end"])
            done += 1
            if progress:
                progress(done, total)
        out.append(dict(name=name, per=per, coll=coll, close=close, n=n,
                        drift=sum(drift) / max(1, len(drift))))
    return out


def format_mc(res):
    lines = []
    for r in res:
        n = r["n"]
        tot_ok1 = sum(v["ok1"] for v in r["per"].values())
        tot_ok2 = sum(v["ok2"] for v in r["per"].values())
        tot_f = sum(v["fail"] for v in r["per"].values())
        tot = max(1, tot_ok1 + tot_ok2 + tot_f)
        lines.append(r["name"])
        lines.append(f"   photos OK first try {100 * tot_ok1 / tot:5.1f}%   recovered on 2nd {100 * tot_ok2 / tot:5.1f}%"
                     f"   still failed {100 * tot_f / tot:5.1f}%")
        first_fail = tot_ok2 + tot_f
        lines.append(f"   of the photos that failed the 1st time, recovered on the 2nd: "
                     + (f"{100 * tot_ok2 / first_fail:.0f}%" if first_fail else "n/a (none failed)"))
        lines.append(f"   runs with a collision {100 * r['coll'] / n:5.1f}%   runs too close {100 * r['close'] / n:5.1f}%"
                     f"   mean end drift {r['drift']:.1f} cm")
        for oid, v in r["per"].items():
            lines.append(f"      obstacle {oid}: 1st {100 * v['ok1'] / n:3.0f}%  2nd {100 * v['ok2'] / n:3.0f}%  fail {100 * v['fail'] / n:3.0f}%")
        lines.append("")
    return "\n".join(lines)


def make_plan(obstacles):
    """Run the real planner with the CURRENT config values."""
    return planner.plan(list(obstacles), start_pose=config.START_POSE_CM)


def lit(v):
    """Python literal for writing a value back into config.py."""
    if isinstance(v, bool):
        return "True" if v else "False"
    if isinstance(v, float):
        return repr(round(v, 4))
    if isinstance(v, int):
        return repr(v)
    if isinstance(v, str):
        return '"%s"' % v
    if isinstance(v, tuple):
        return "(" + ", ".join(lit(x) for x in v) + ("," if len(v) == 1 else "") + ")"
    if isinstance(v, dict):
        return "{" + ", ".join('"%s": %s' % (k, lit(x)) for k, x in v.items()) + "}"
    return repr(v)


# ===========================================================================
#                                   GUI
# ===========================================================================
COL = dict(plan="#1f77b4", correct="#ff7f0e", ac="#2ca02c", ra="#9467bd")
LIST_STYLE = {          # tag -> (foreground, background)
    "start": ("#000000", "#e8eaf6"), "end": ("#000000", "#e8eaf6"),
    "plan": ("#0d47a1", "#ffffff"), "correct": ("#bf5b00", "#fff3e0"),
    "ac": ("#1b7a1b", "#ffffff"), "ra": ("#6a3fa0", "#ffffff"),
    "photo_ok": ("#0b5d0b", "#d8f5d8"), "photo_bad": ("#8a0000", "#fbdcdc"),
    "warn": ("#b00000", "#ffffff"), "log": ("#666666", "#ffffff"),
}
FACES = ["N", "E", "S", "W"]
HELP_TEXT = """HOW TO USE
  Arena     left-click empty cell = add obstacle (face from 'New obstacle face')
            left-click obstacle   = rotate its image face  N > E > S > W
            right-click / shift-click obstacle = remove it
  Config    sliders edit config.py values IN MEMORY and replan. Nothing is written
            to disk until you press 'Save to config.py' (a backup is made first).
  Hardware  makes the simulated robot imperfect (turn / drift / ultrasonic noise),
            sets the camera model, and lets you force photos to fail.
  Playback  Play / step by event / drag the slider. Click a row in Steps to jump.

COLOURS
  blue line     planned moves the robot executes
  orange line   moves done by the angle correction (undo straight/turn, nudge, redo)
  green line    AC20 ultrasonic align movement
  purple line   RA (reverse of the align)
  grey dashed   where the PLANNER thinks the robot is (ghost) and the planned path
  red beam      ultrasonic reading;  yellow wedge = camera view when a photo is taken
  face bar      gold = not photographed yet, green = OK first try, light green = OK
                on 2nd try, orange = 1st photo failed, red = both failed

MODELS ARE ASSUMPTIONS
  The hardware error model and camera confidence model are guesses for testing the
  LOGIC. They are not calibrated to your robot.
"""


class Scrollable(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.cv = tk.Canvas(self, highlightthickness=0, borderwidth=0)
        self.vsb = ttk.Scrollbar(self, orient="vertical", command=self.cv.yview)
        self.inner = ttk.Frame(self.cv)
        self.inner.bind("<Configure>", lambda e: self.cv.configure(scrollregion=self.cv.bbox("all")))
        self.win = self.cv.create_window((0, 0), window=self.inner, anchor="nw")
        self.cv.bind("<Configure>", lambda e: self.cv.itemconfigure(self.win, width=e.width))
        self.cv.configure(yscrollcommand=self.vsb.set)
        self.cv.pack(side="left", fill="both", expand=True)
        self.vsb.pack(side="right", fill="y")
        self.bind("<Enter>", self._bind_wheel)
        self.bind("<Leave>", self._unbind_wheel)

    def _bind_wheel(self, _e):
        self.cv.bind_all("<MouseWheel>", self._wheel)
        self.cv.bind_all("<Button-4>", lambda e: self.cv.yview_scroll(-2, "units"))
        self.cv.bind_all("<Button-5>", lambda e: self.cv.yview_scroll(2, "units"))

    def _unbind_wheel(self, _e):
        self.cv.unbind_all("<MouseWheel>")
        self.cv.unbind_all("<Button-4>")
        self.cv.unbind_all("<Button-5>")

    def _wheel(self, e):
        self.cv.yview_scroll(int(-e.delta / 120) * 2, "units")


class Param:
    """label + slider + typed entry bound to a getter/setter."""

    def __init__(self, app, parent, label, lo, hi, res, get, set_, group, integer=False):
        self.app, self.get, self.set, self.group, self.integer = app, get, set_, group, integer
        f = ttk.Frame(parent)
        f.pack(fill="x", padx=6, pady=1)
        ttk.Label(f, text=label, anchor="w").grid(row=0, column=0, columnspan=2, sticky="w")
        self.scale = tk.Scale(f, from_=lo, to=hi, resolution=res, orient="horizontal",
                              showvalue=False, length=230, sliderlength=14, command=self._on_scale)
        self.scale.grid(row=1, column=0, sticky="we")
        self.sv = tk.StringVar()
        self.entry = ttk.Entry(f, textvariable=self.sv, width=8)
        self.entry.grid(row=1, column=1, padx=(6, 0))
        self.entry.bind("<Return>", self._on_entry)
        self.entry.bind("<FocusOut>", self._on_entry)
        f.columnconfigure(0, weight=1)
        self._busy = False
        self.refresh()

    def _fmt(self, v):
        return str(int(round(v))) if self.integer else ("%g" % round(v, 4))

    def refresh(self):
        self._busy = True
        try:
            v = self.get()
            self.scale.set(v)
            self.sv.set(self._fmt(v))
        finally:
            self._busy = False

    def _on_scale(self, v):
        if self._busy:
            return
        val = float(v)
        val = int(round(val)) if self.integer else val
        if self.get() == val:
            return
        self.set(val)
        self.sv.set(self._fmt(val))
        self.app.changed(self.group)

    def _on_entry(self, _e=None):
        try:
            val = float(self.sv.get())
        except ValueError:
            self.refresh()
            return
        val = int(round(val)) if self.integer else val
        if self.get() == val:
            return
        self.set(val)
        self._busy = True
        try:
            self.scale.set(val)
        finally:
            self._busy = False
        self.app.changed(self.group)


class App:
    def __init__(self, root):
        self.root = root
        root.title("MDP Task 1 simulator")
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        root.geometry(f"{min(1380, sw - 40)}x{min(860, sh - 90)}+10+10")
        root.minsize(900, 560)

        self.sp = dict(SIM_DEFAULTS)
        self.sp.update(PRESETS["Realistic (small errors)"])
        self.obstacles = self._initial_obstacles()
        self.plan = None
        self.plan_obs = []
        self.nominal = []
        self.run = None
        self.fi = 0
        self.playing = False
        self.planning = False
        self.dirty = False
        self._plan_job = None
        self._sim_job = None
        self._redraw_job = None
        self._prog = False
        self.q = queue.Queue()
        self.geom = (0, 0, 1, 200)
        self.params = {"plan": [], "sim": []}

        self.auto_plan = tk.BooleanVar(value=True)
        self.v_halo = tk.BooleanVar(value=True)
        self.v_path = tk.BooleanVar(value=True)
        self.v_trail = tk.BooleanVar(value=True)
        self.v_ghost = tk.BooleanVar(value=True)
        self.v_goals = tk.BooleanVar(value=False)
        self.v_beam = tk.BooleanVar(value=True)
        self.v_fine = tk.BooleanVar(value=True)
        self.new_face = tk.StringVar(value="N")

        self._build()
        root.after(100, self._poll)
        root.after(60, lambda: self.request_plan(delay=10, force=True))

    # ------------------------------------------------------------------ setup
    def _initial_obstacles(self):
        try:
            obs = planner.normalize_obstacles(list(test_maps.OBSTACLES))
            return obs
        except Exception:
            return [(1, 10, 10, "W")]

    def _build(self):
        root = self.root
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        left = ttk.Frame(root)
        left.grid(row=0, column=0, sticky="nsew")
        left.rowconfigure(0, weight=1)
        left.columnconfigure(0, weight=1)
        right = ttk.Frame(root, width=450)
        right.grid(row=0, column=1, sticky="ns")
        right.grid_propagate(False)
        right.rowconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)

        self.cv = tk.Canvas(left, bg="#eceff1", highlightthickness=0)
        self.cv.grid(row=0, column=0, sticky="nsew")
        self.cv.bind("<Configure>", lambda e: self._sched_redraw())
        self.cv.bind("<Button-1>", self._click)
        self.cv.bind("<Button-3>", self._rclick)
        self.cv.bind("<Button-2>", self._rclick)
        self.cv.bind("<Motion>", self._hover)

        info = ttk.Frame(left)
        info.grid(row=1, column=0, sticky="we", padx=6)
        self.v_pose = tk.StringVar()
        self.v_event = tk.StringVar()
        self.v_score = tk.StringVar()
        ttk.Label(info, textvariable=self.v_pose, font=("Consolas", 9)).pack(anchor="w")
        ttk.Label(info, textvariable=self.v_event, font=("Consolas", 9, "bold")).pack(anchor="w")
        ttk.Label(info, textvariable=self.v_score, font=("Consolas", 9)).pack(anchor="w")

        pb = ttk.Frame(left)
        pb.grid(row=2, column=0, sticky="we", padx=6, pady=2)
        ttk.Button(pb, text="|<", width=3, command=lambda: self.goto(0)).pack(side="left")
        ttk.Button(pb, text="< event", width=8, command=self.prev_event).pack(side="left")
        self.btn_play = ttk.Button(pb, text="Play", width=7, command=self.toggle_play)
        self.btn_play.pack(side="left", padx=2)
        ttk.Button(pb, text="event >", width=8, command=self.next_event).pack(side="left")
        ttk.Button(pb, text=">|", width=3, command=lambda: self.goto(10 ** 9)).pack(side="left")
        self.slider = tk.Scale(pb, from_=0, to=1, orient="horizontal", showvalue=False,
                               command=self._on_slider, sliderlength=14)
        self.slider.pack(side="left", fill="x", expand=True, padx=6)
        ttk.Label(pb, text="speed").pack(side="left")
        self.speed = tk.Scale(pb, from_=1, to=15, orient="horizontal", length=90, showvalue=False, sliderlength=12)
        self.speed.set(3)
        self.speed.pack(side="left")

        tg = ttk.Frame(left)
        tg.grid(row=3, column=0, sticky="we", padx=6)
        for text, var in (("1 cm grid", self.v_fine), ("clearance halo", self.v_halo), ("planned path", self.v_path),
                          ("true trail", self.v_trail), ("planner ghost", self.v_ghost),
                          ("goal poses", self.v_goals), ("beam / camera", self.v_beam)):
            ttk.Checkbutton(tg, text=text, variable=var, command=self._sched_redraw).pack(side="left", padx=3)
        lg = ttk.Frame(left)
        lg.grid(row=4, column=0, sticky="we", padx=6)
        for text, col in (("plan move", COL["plan"]), ("angle correction", COL["correct"]),
                          ("AC20 align", COL["ac"]), ("RA", COL["ra"]), ("planner's belief", "#777777")):
            tk.Label(lg, text="■ " + text, fg=col, font=("Arial", 9, "bold")).pack(side="left", padx=5)
        self.status = tk.StringVar(value="starting ...")
        self.hover = tk.StringVar()
        sb = ttk.Frame(root)
        sb.grid(row=1, column=0, columnspan=2, sticky="we")
        self.lbl_status = ttk.Label(sb, textvariable=self.status, anchor="w")
        self.lbl_status.pack(side="left", padx=6)
        ttk.Label(sb, textvariable=self.hover, anchor="e").pack(side="right", padx=6)

        nb = self.nb = ttk.Notebook(right)
        nb.grid(row=0, column=0, sticky="nsew")
        self.tab_steps = ttk.Frame(nb)
        self.tab_cfg = Scrollable(nb)
        self.tab_hw = Scrollable(nb)
        self.tab_obs = ttk.Frame(nb)
        self.tab_help = ttk.Frame(nb)
        nb.add(self.tab_steps, text="Steps")
        nb.add(self.tab_cfg, text="Config")
        nb.add(self.tab_hw, text="Hardware")
        nb.add(self.tab_obs, text="Obstacles")
        nb.add(self.tab_help, text="Help")
        self._build_steps()
        self._build_cfg()
        self._build_hw()
        self._build_obs()
        t = tk.Text(self.tab_help, wrap="word", font=("Consolas", 9), padx=6, pady=6)
        t.insert("1.0", HELP_TEXT)
        t.configure(state="disabled")
        t.pack(fill="both", expand=True)
        root.bind("<space>", self._key_space)
        root.bind("<Right>", lambda e: self._key(lambda: self.next_event()))
        root.bind("<Left>", lambda e: self._key(lambda: self.prev_event()))

    def _key(self, fn):
        if isinstance(self.root.focus_get(), (tk.Entry, ttk.Entry, ttk.Combobox, tk.Text)):
            return
        fn()

    def _key_space(self, e):
        self._key(self.toggle_play)

    # ---- steps tab ----------------------------------------------------------
    def _build_steps(self):
        f = self.tab_steps
        f.rowconfigure(1, weight=1)
        f.columnconfigure(0, weight=1)
        self.txt_sum = tk.Text(f, height=13, wrap="word", font=("Consolas", 9), padx=4, pady=4)
        self.txt_sum.grid(row=0, column=0, columnspan=2, sticky="we")
        self.lb = tk.Listbox(f, font=("Consolas", 9), activestyle="none", exportselection=False)
        self.lb.grid(row=1, column=0, sticky="nsew")
        vs = ttk.Scrollbar(f, orient="vertical", command=self.lb.yview)
        vs.grid(row=1, column=1, sticky="ns")
        self.lb.configure(yscrollcommand=vs.set)
        self.lb.bind("<<ListboxSelect>>", self._on_list_select)
        bb = ttk.Frame(f)
        bb.grid(row=2, column=0, columnspan=2, sticky="we", pady=2)
        ttk.Button(bb, text="Plan now", width=9, command=lambda: self.request_plan(delay=10, force=True)).pack(side="left", padx=2)
        ttk.Button(bb, text="Re-simulate", width=11, command=self.resim).pack(side="left", padx=2)
        ttk.Button(bb, text="Copy commands", width=14, command=self.copy_commands).pack(side="left", padx=2)
        ttk.Checkbutton(f, text="auto-replan when config / obstacles change", variable=self.auto_plan
                        ).grid(row=3, column=0, columnspan=2, sticky="w", padx=4, pady=(0, 4))

    # ---- config tab ---------------------------------------------------------
    def _hdr(self, parent, text):
        ttk.Label(parent, text=text, font=("TkDefaultFont", 9, "bold")).pack(anchor="w", padx=6, pady=(8, 0))

    def _add(self, parent, label, lo, hi, res, get, set_, group, integer=False):
        p = Param(self, parent, label, lo, hi, res, get, set_, group, integer)
        self.params[group].append(p)
        return p

    def _build_cfg(self):
        P = self.tab_cfg.inner
        c = lambda k, cast=float: (lambda: getattr(config, k), lambda v: setattr(config, k, cast(v)))
        row = lambda label, lo, hi, res, key, cast=float, integer=False: self._add(
            P, label, lo, hi, res, *c(key, cast), "plan", integer)

        self._hdr(P, "Arena and robot (config.py)")
        row("GRID  (cells of 10 cm)", 10, 30, 1, "GRID", int, True)
        row("ROBOT_WIDTH_CM", 10, 30, 0.5, "ROBOT_WIDTH_CM")
        row("ROBOT_LENGTH_CM", 15, 35, 0.5, "ROBOT_LENGTH_CM")
        row("SENSOR_FORWARD_CM  (sensor ahead of centre)", 0, 20, 0.5, "SENSOR_FORWARD_CM")
        self._hdr(P, "Start pose")
        sg = lambda i: (lambda: config.START_POSE_CM[i])

        def ss(i):
            def f(v):
                t = list(config.START_POSE_CM)
                t[i] = float(v)
                config.START_POSE_CM = tuple(t)
            return f
        self._add(P, "START x (cm, robot centre)", 0, 200, 0.5, sg(0), ss(0), "plan")
        self._add(P, "START y (cm, robot centre)", 0, 200, 0.5, sg(1), ss(1), "plan")
        fh = ttk.Frame(P)
        fh.pack(fill="x", padx=6, pady=2)
        ttk.Label(fh, text="START heading").pack(side="left")
        self.cb_head = ttk.Combobox(fh, values=FACES, width=4, state="readonly")
        self.cb_head.set(config.START_POSE_CM[2])
        self.cb_head.pack(side="left", padx=6)
        self.cb_head.bind("<<ComboboxSelected>>", self._on_head)

        self._hdr(P, "Clearances")
        row("MIN_CLEARANCE_CM  (to obstacles)", 0, 30, 0.5, "MIN_CLEARANCE_CM")
        row("POSITION_MARGIN_CM", 0, 5, 0.5, "POSITION_MARGIN_CM")
        row("BOUNDARY_MARGIN_CM", 0, 10, 0.5, "BOUNDARY_MARGIN_CM")

        self._hdr(P, "Photo alignment (planner side)")
        self.v_align = tk.BooleanVar(value=config.ALIGN_ENABLED)
        ttk.Checkbutton(P, text="ALIGN_ENABLED (AC20 before SNAP, RA after)",
                        variable=self.v_align, command=self._on_align).pack(anchor="w", padx=6)
        row("ALIGN_MAX_TRAVEL_CM  (strip kept clear)", 1, 60, 1, "ALIGN_MAX_TRAVEL_CM")
        self._add(P, "VIEW_SENSOR_DISTANCES_CM  (planned photo distance)", 10, 60, 1,
                  lambda: config.VIEW_SENSOR_DISTANCES_CM[0],
                  lambda v: setattr(config, "VIEW_SENSOR_DISTANCES_CM", (float(v),)), "plan")

        self._hdr(P, "Turn models (forward cm, right cm)  RT90 LT90 XL90 XR90")
        for name in ("RT90", "LT90", "XL90", "XR90"):
            for i, what in ((0, "forward"), (1, "right")):
                def g(n=name, k=i):
                    return config.TURN_MODELS[n][k]

                def s(v, n=name, k=i):
                    d = dict(config.TURN_MODELS)
                    t = list(d[n])
                    t[k] = int(round(v))
                    d[n] = tuple(t)
                    config.TURN_MODELS = d
                self._add(P, f"{name}  {what} cm", -60, 60, 1, g, s, "plan", True)

        self._hdr(P, "Search / costs")
        row("MAX_STRAIGHT_COMMAND_CM", 10, 150, 5, "MAX_STRAIGHT_COMMAND_CM", int, True)
        row("PLAN_TIME_LIMIT_S", 5, 120, 5, "PLAN_TIME_LIMIT_S")

        def dget(k, sub):
            return lambda: getattr(config, k)[sub]

        def dset(k, sub):
            def f(v):
                d = dict(getattr(config, k))
                d[sub] = float(v)
                setattr(config, k, d)
            return f
        for k, sub, label in (("STRAIGHT_COST_PER_CM", "FW", "cost per cm  FW"),
                              ("STRAIGHT_COST_PER_CM", "BW", "cost per cm  BW"),
                              ("TURN_COST", "LT90", "cost  LT90"), ("TURN_COST", "RT90", "cost  RT90"),
                              ("TURN_COST", "XL90", "cost  XL90"), ("TURN_COST", "XR90", "cost  XR90")):
            self._add(P, label, 0.01, 40, 0.01 if k.startswith("STR") else 0.5, dget(k, sub), dset(k, sub), "plan")

        bf = ttk.Frame(P)
        bf.pack(fill="x", padx=6, pady=10)
        ttk.Button(bf, text="Reset to config.py", command=self.reset_cfg).pack(side="left", padx=2)
        ttk.Button(bf, text="Copy changed lines", command=self.copy_changed).pack(side="left", padx=2)
        ttk.Button(bf, text="Save to config.py", command=self.save_config).pack(side="left", padx=2)
        self.v_diff = tk.StringVar()
        ttk.Label(P, textvariable=self.v_diff, foreground="#b00000", wraplength=400).pack(anchor="w", padx=6, pady=(0, 10))

    def _on_head(self, _e=None):
        t = list(config.START_POSE_CM)
        t[2] = self.cb_head.get()
        config.START_POSE_CM = tuple(t)
        self.changed("plan")

    def _on_align(self):
        config.ALIGN_ENABLED = bool(self.v_align.get())
        self.changed("plan")

    # ---- hardware tab -------------------------------------------------------
    def _build_hw(self):
        P = self.tab_hw.inner
        sp = self.sp

        def sg(k):
            return lambda: sp[k]

        def ss(k, cast=float):
            return lambda v: sp.__setitem__(k, cast(v))
        add = lambda label, lo, hi, res, k, integer=False: self._add(
            P, label, lo, hi, res, sg(k), ss(k, int if integer else float), "sim", integer)

        self._hdr(P, "Preset for the imperfect robot")
        f = ttk.Frame(P)
        f.pack(fill="x", padx=6, pady=2)
        self.cb_preset = ttk.Combobox(f, values=list(PRESETS), state="readonly", width=30)
        self.cb_preset.set("Realistic (small errors)")
        self.cb_preset.pack(side="left")
        self.cb_preset.bind("<<ComboboxSelected>>", self._on_preset)

        self._hdr(P, "Movement errors (simulation only)")
        add("Turn bias (deg)  - undershoot / + overshoot", -20, 20, 0.5, "turn_bias_deg")
        add("Turn random error, std (deg)", 0, 10, 0.5, "turn_sd_deg")
        add("Turn end-position error, std (cm)", 0, 6, 0.5, "turn_pos_sd_cm")
        add("Straight distance error (%)", -8, 8, 0.5, "straight_scale_pct")
        add("Straight random error, std (cm)", 0, 4, 0.1, "straight_sd_cm")
        add("Heading drift on straights (deg / 100 cm)", -6, 6, 0.1, "drift_deg_per_100")
        add("Heading drift random, std (deg / 100 cm)", 0, 4, 0.1, "drift_sd_deg")
        add("Random seed", 1, 999, 1, "seed", True)

        self._hdr(P, "Ultrasonic sensor and AC20 firmware")
        add("Ultrasonic noise, std (mm)", 0, 40, 1, "us_sd_mm")
        add("Ultrasonic beam half-angle (deg, 0 = single ray)", 0, 30, 1, "us_cone_deg")
        add("AC20 total travel cap (cm)  [10 = current firmware]", 3, 60, 1, "ac_cap_cm", True)

        self._hdr(P, "Camera model (assumption)")
        add("Field of view (deg)", 30, 100, 1, "cam_fov")
        add("Best distance min (cm, camera to face)", 8, 40, 1, "cam_ideal_min")
        add("Best distance max (cm)", 15, 60, 1, "cam_ideal_max")
        add("Viewing angle where confidence hits 0 (deg)", 15, 80, 1, "cam_angle_tol")
        add("Max confidence", 0.3, 1.0, 0.01, "cam_max_conf")
        add("Detection threshold (client uses 0.25)", 0.05, 0.95, 0.01, "conf_threshold")

        self._hdr(P, "Force failures (to watch the correction path)")
        self.cb_force = ttk.Combobox(P, state="readonly", width=44, values=[
            "physics only (no forced failure)", "force the 1st photo to fail", "force 1st AND 2nd photos to fail"])
        self.cb_force.set(["physics only (no forced failure)", "force the 1st photo to fail",
                           "force 1st AND 2nd photos to fail"][{"physics": 0, "photo1": 1, "both": 2}[sp["force"]]])
        self.cb_force.pack(anchor="w", padx=6, pady=2)
        self.cb_force.bind("<<ComboboxSelected>>", self._on_force)

        self._hdr(P, "Legacy retry controls (inactive in V5)")
        self.v_ac_on = tk.BooleanVar(value=sp["angle_correction"])
        self.v_appr = tk.BooleanVar(value=sp["approach"])
        self.v_ret = tk.BooleanVar(value=sp["return_"])
        for text, var, key in (("ANGLE_CORRECTION_ENABLED (undo / nudge / redo the last turn)", self.v_ac_on, "angle_correction"),
                               ("APPROACH_CMD = AC20 before the 2nd photo", self.v_appr, "approach"),
                               ("RETURN_CMD = RA after the 2nd photo", self.v_ret, "return_")):
            ttk.Checkbutton(P, text=text, variable=var,
                            command=lambda v=var, k=key: (sp.__setitem__(k, bool(v.get())), self.changed("sim"))
                            ).pack(anchor="w", padx=6)
        add("ANGLE_NUDGE_CM", 1, 30, 1, "nudge_cm", True)

        self._hdr(P, "Monte Carlo (legacy strategy labels; V5 policy is fixed)")
        mf = ttk.Frame(P)
        mf.pack(fill="x", padx=6, pady=2)
        ttk.Label(mf, text="runs").pack(side="left")
        self.mc_n = tk.IntVar(value=40)
        ttk.Spinbox(mf, from_=5, to=500, textvariable=self.mc_n, width=5).pack(side="left", padx=4)
        self.btn_mc = ttk.Button(mf, text="Run Monte Carlo", command=self.run_mc)
        self.btn_mc.pack(side="left", padx=4)
        ttk.Label(P, text="Uses the hardware errors above, ignores 'force failure', and\nre-runs with 3 strategies "
                          "(A as-is, B AC20 only, C photo twice).", foreground="#555").pack(anchor="w", padx=6)
        self.txt_mc = tk.Text(P, height=22, width=52, font=("Consolas", 8), wrap="none")
        self.txt_mc.pack(fill="x", padx=6, pady=4)

    def _on_preset(self, _e=None):
        self.sp.update(PRESETS[self.cb_preset.get()])
        for p in self.params["sim"]:
            p.refresh()
        self.changed("sim")

    def _on_force(self, _e=None):
        self.sp["force"] = ["physics", "photo1", "both"][self.cb_force.current()]
        self.changed("sim")

    # ---- obstacles tab --------------------------------------------------------
    def _build_obs(self):
        f = self.tab_obs
        top = ttk.Frame(f)
        top.pack(fill="x", padx=6, pady=4)
        ttk.Label(top, text="New obstacle face:").pack(side="left")
        ttk.Combobox(top, values=FACES, width=4, state="readonly", textvariable=self.new_face).pack(side="left", padx=4)
        self.lb_obs = tk.Listbox(f, height=12, font=("Consolas", 10), exportselection=False)
        self.lb_obs.pack(fill="x", padx=6)
        self.lb_obs.bind("<<ListboxSelect>>", self._on_obs_select)
        form = ttk.Frame(f)
        form.pack(fill="x", padx=6, pady=4)
        self.e_x, self.e_y = tk.StringVar(), tk.StringVar()
        ttk.Label(form, text="x").grid(row=0, column=0)
        ttk.Entry(form, textvariable=self.e_x, width=5).grid(row=0, column=1, padx=3)
        ttk.Label(form, text="y").grid(row=0, column=2)
        ttk.Entry(form, textvariable=self.e_y, width=5).grid(row=0, column=3, padx=3)
        ttk.Label(form, text="face").grid(row=0, column=4)
        self.e_face = ttk.Combobox(form, values=FACES, width=4, state="readonly")
        self.e_face.grid(row=0, column=5, padx=3)
        ttk.Button(form, text="Apply to selected", command=self.apply_selected).grid(row=0, column=6, padx=6)
        b = ttk.Frame(f)
        b.pack(fill="x", padx=6, pady=2)
        ttk.Button(b, text="Remove selected", command=self.remove_selected).pack(side="left", padx=2)
        ttk.Button(b, text="Clear all", command=self.clear_obs).pack(side="left", padx=2)
        b2 = ttk.Frame(f)
        b2.pack(fill="x", padx=6, pady=2)
        ttk.Button(b2, text="Load test_maps.py", command=self.load_test_maps).pack(side="left", padx=2)
        ttk.Label(b2, text="random:").pack(side="left", padx=(10, 2))
        self.rand_n = tk.IntVar(value=6)
        ttk.Spinbox(b2, from_=1, to=12, textvariable=self.rand_n, width=4).pack(side="left")
        ttk.Button(b2, text="Generate", command=self.random_obs).pack(side="left", padx=4)
        ttk.Button(f, text="Copy OBSTACLES = [...] for test_maps.py", command=self.copy_obs).pack(anchor="w", padx=8, pady=2)
        ttk.Label(f, text="Paste a list, e.g.  [(1,7,7,'W'),(2,5,12,'S')]").pack(anchor="w", padx=8, pady=(8, 0))
        self.e_paste = tk.StringVar()
        ttk.Entry(f, textvariable=self.e_paste).pack(fill="x", padx=8)
        ttk.Button(f, text="Apply pasted list", command=self.apply_paste).pack(anchor="w", padx=8, pady=2)
        ttk.Label(f, text="Cells are (x, y) in 10 cm units, origin bottom-left.\nCell (7,7) covers x 70-80, y 70-80 cm.",
                  foreground="#555").pack(anchor="w", padx=8, pady=6)
        self._refresh_obs_list()

    # ------------------------------------------------------------------ obstacles
    def _refresh_obs_list(self):
        self.lb_obs.delete(0, "end")
        for o in self.obstacles:
            self.lb_obs.insert("end", f"#{o[0]:<3} cell ({o[1]:>2},{o[2]:>2})   face {o[3]}   "
                                      f"x {o[1] * 10}-{o[1] * 10 + 10}  y {o[2] * 10}-{o[2] * 10 + 10} cm")

    def _on_obs_select(self, _e=None):
        s = self.lb_obs.curselection()
        if not s:
            return
        o = self.obstacles[s[0]]
        self.e_x.set(str(o[1]))
        self.e_y.set(str(o[2]))
        self.e_face.set(o[3])

    def obstacles_changed(self):
        self._refresh_obs_list()
        self._sched_redraw()
        self.request_plan(delay=250)

    def _next_id(self):
        used = {o[0] for o in self.obstacles}
        i = 1
        while i in used:
            i += 1
        return i

    def apply_selected(self):
        s = self.lb_obs.curselection()
        if not s:
            return
        o = self.obstacles[s[0]]
        try:
            x, y = int(self.e_x.get()), int(self.e_y.get())
        except ValueError:
            return
        self.obstacles[s[0]] = (o[0], x, y, self.e_face.get() or o[3])
        self.obstacles_changed()

    def remove_selected(self):
        s = self.lb_obs.curselection()
        if s:
            del self.obstacles[s[0]]
            self.obstacles_changed()

    def clear_obs(self):
        self.obstacles = []
        self.obstacles_changed()

    def load_test_maps(self):
        try:
            import importlib
            importlib.reload(test_maps)
            self.obstacles = planner.normalize_obstacles(list(test_maps.OBSTACLES))
        except Exception as e:
            self.set_status(f"Could not load test_maps.py: {e}", True)
            return
        self.obstacles_changed()

    def random_obs(self):
        rng = random.Random()
        n, g = int(self.rand_n.get()), config.GRID
        res, tries = [], 0
        while len(res) < n and tries < 2000:
            tries += 1
            x, y = rng.randint(2, g - 3), rng.randint(2, g - 3)
            if x < 6 and y < 6:
                continue
            if any(max(abs(x - o[1]), abs(y - o[2])) < 3 for o in res):
                continue
            res.append((len(res) + 1, x, y, rng.choice(FACES)))
        self.obstacles = res
        self.obstacles_changed()

    def copy_obs(self):
        text = "OBSTACLES = [" + ", ".join(f'({o[0]}, {o[1]}, {o[2]}, "{o[3]}")' for o in self.obstacles) + "]"
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.set_status("Copied: " + text)

    def apply_paste(self):
        try:
            obs = planner.normalize_obstacles(ast.literal_eval(self.e_paste.get()))
        except Exception as e:
            self.set_status(f"Bad list: {e}", True)
            return
        self.obstacles = obs
        self.obstacles_changed()

    # ------------------------------------------------------------------ canvas input
    def _cell_at(self, ev):
        ox, oy, s, ext = self.geom
        cx, cy = (ev.x - ox) / s, ext - (ev.y - oy) / s
        if not (0 <= cx < ext and 0 <= cy < ext):
            return None
        return int(cx // config.CELL_CM), int(cy // config.CELL_CM)

    def _find(self, cell):
        for i, o in enumerate(self.obstacles):
            if (o[1], o[2]) == cell:
                return i
        return None

    def _click(self, ev):
        self.cv.focus_set()
        cell = self._cell_at(ev)
        if cell is None:
            return
        i = self._find(cell)
        if ev.state & 0x1 and i is not None:            # shift-click removes
            del self.obstacles[i]
        elif i is not None:
            o = self.obstacles[i]
            self.obstacles[i] = (o[0], o[1], o[2], FACES[(FACES.index(o[3]) + 1) % 4])
        else:
            self.obstacles.append((self._next_id(), cell[0], cell[1], self.new_face.get()))
        self.obstacles_changed()

    def _rclick(self, ev):
        cell = self._cell_at(ev)
        i = self._find(cell) if cell else None
        if i is not None:
            del self.obstacles[i]
            self.obstacles_changed()

    def _hover(self, ev):
        cell = self._cell_at(ev)
        if cell is None:
            self.hover.set("")
            return
        ox, oy, s, ext = self.geom
        self.hover.set(f"cell ({cell[0]},{cell[1]})   x {(ev.x - ox) / s:.0f} cm   y {ext - (ev.y - oy) / s:.0f} cm")

    # ------------------------------------------------------------------ change handling
    def set_status(self, text, bad=False):
        self.status.set(text)
        self.lbl_status.configure(foreground="#b00000" if bad else "#000000")

    def changed(self, group):
        if group == "plan":
            self.request_plan(delay=450)
        else:
            if self._sim_job:
                self.root.after_cancel(self._sim_job)
            self._sim_job = self.root.after(150, self.resim)
        self._update_diff()

    def _update_diff(self):
        diffs = [k for k in CFG_KEYS if getattr(config, k) != ORIG[k]]
        self.v_diff.set("Changed vs config.py:  " + ", ".join(diffs) if diffs else "Values match config.py")

    def request_plan(self, delay=400, force=False):
        if self._plan_job:
            self.root.after_cancel(self._plan_job)
            self._plan_job = None
        if not self.auto_plan.get() and not force:
            self.set_status("Config / obstacles changed - press 'Plan now' (auto-replan is off)")
            return
        self._plan_job = self.root.after(delay, self.start_plan)

    def start_plan(self):
        self._plan_job = None
        if self.planning:
            self.dirty = True
            return
        self.planning, self.dirty = True, False
        obs = list(self.obstacles)
        self.set_status("Planning with the current config values ... (a low clearance can take a few seconds)")

        def work():
            try:
                self.q.put(("plan", obs, make_plan(obs), None))
            except Exception as e:
                self.q.put(("plan", obs, None, e))
        threading.Thread(target=work, daemon=True).start()

    def _poll(self):
        try:
            while True:
                msg = self.q.get_nowait()
                if msg[0] == "plan":
                    self.planning = False
                    if self.dirty:
                        self.start_plan()
                    else:
                        self._apply_plan(msg[1], msg[2], msg[3])
                elif msg[0] == "mc_progress":
                    self.set_status(f"Monte Carlo {msg[1]}/{msg[2]} ...")
                elif msg[0] == "mc_done":
                    self.btn_mc.configure(state="normal")
                    self.txt_mc.delete("1.0", "end")
                    self.txt_mc.insert("1.0", msg[1])
                    self.set_status("Monte Carlo finished (see Hardware tab, bottom)")
                elif msg[0] == "mc_err":
                    self.btn_mc.configure(state="normal")
                    self.set_status(f"Monte Carlo failed: {msg[1]}", True)
        except queue.Empty:
            pass
        self.root.after(100, self._poll)

    def _apply_plan(self, obs, res, err):
        self.playing = False
        self.btn_play.configure(text="Play")
        if err is not None:
            self.plan, self.run, self.nominal = None, None, []
            self.set_status(f"Planner error: {err}", True)
            self._fill_summary(f"PLANNER ERROR\n{err}\n\nThe planner rejected this map / these config values.")
            self.lb.delete(0, "end")
            self.v_pose.set("")
            self.v_event.set("")
            self.v_score.set("")
            self._sched_redraw()
            return
        self.plan, self.plan_obs = res, obs
        self.nominal = []
        for seg in res["segments"]:
            pts, _ = nominal_polyline(seg["start_pose"], [c for c in seg["hardware_commands"]])
            self.nominal.append((seg["obstacle_id"], pts))
        self.set_status(f"Planned {len(res['commands'])} commands in {res['planning_ms']:.0f} ms"
                        + (f"  ({len(res['skipped'])} obstacle(s) skipped)" if res["skipped"] else ""))
        self.resim()

    # ------------------------------------------------------------------ simulation
    def resim(self):
        self._sim_job = None
        if not self.plan:
            return
        try:
            self.run = simulate(self.plan_obs, self.plan, self.sp)
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.set_status(f"Simulation error: {e}", True)
            return
        self.playing = False
        self.btn_play.configure(text="Play")
        r = self.run
        self.lb.delete(0, "end")
        for i, e in enumerate(r.events):
            self.lb.insert("end", f"{i:>3} {e['text']}")
            fg, bg = LIST_STYLE.get(e["tag"], LIST_STYLE["log"])
            self.lb.itemconfig(i, foreground=fg, background=bg)
        n = max(1, len(r.frames) - 1)
        self.slider.configure(to=n)
        self._fill_summary(self._summary_text())
        self.fi = len(r.frames) - 1
        self.goto(self.fi)

    def _summary_text(self):
        p, r = self.plan, self.run
        lines = [f"Order {p['order']}   cost {p['total_cost']}   planning {p['planning_ms']:.0f} ms",
                 "Plan: " + " ".join(p["commands"]) if p["commands"] else "Plan: (no commands)"]
        for s in p["skipped"]:
            lines.append(f"SKIPPED obstacle {s['obstacle_id']}: {s['reason']}")
        lines.append("")
        names = dict(ok1="photo OK on the 1st try", ok2="recovered on the 2nd photo",
                     fail="FAILED both photos", none="never photographed")
        for oid, st in r.summary["status"].items():
            att = "  ".join(f"#{i + 1}:{'ok' if ok else 'fail'} {cf:.2f}" for i, (ok, cf) in enumerate(r.attempts.get(oid, [])))
            lines.append(f"Obstacle {oid}: {names[st]}   [{att}]")
        s = r.summary
        lines.append(f"End: robot is {s['drift_end']:.1f} cm / {s['drift_deg']:.1f} deg from the planner's pose")
        lines.append(f"Correction moves sent: {r.corrections}")
        if s["collisions"] or s["close"]:
            lines.append(f"HAZARDS: {s['collisions']} collision(s), {s['close']} too-close event(s)  (see red rows)")
        if r.closest:
            lines.append(f"Closest approach to an obstacle: {r.closest[0]:.1f} cm (obstacle {r.closest[1]}),"
                         f" setting MIN_CLEARANCE_CM = {config.MIN_CLEARANCE_CM:g}")
        if not r.completed:
            lines.append("Run ended early (a command returned an error).")
        return "\n".join(lines)

    def _fill_summary(self, text):
        self.txt_sum.configure(state="normal")
        self.txt_sum.delete("1.0", "end")
        self.txt_sum.insert("1.0", text)
        self.txt_sum.configure(state="disabled")

    def run_mc(self):
        if not self.plan:
            self.set_status("Plan first.", True)
            return
        plan, obs, sp, n = self.plan, self.plan_obs, dict(self.sp), int(self.mc_n.get())
        self.btn_mc.configure(state="disabled")
        self.txt_mc.delete("1.0", "end")
        self.txt_mc.insert("1.0", "running ...")

        def work():
            try:
                res = monte_carlo(obs, plan, sp, n, lambda d, t: self.q.put(("mc_progress", d, t)))
                self.q.put(("mc_done", format_mc(res)))
            except Exception as e:
                self.q.put(("mc_err", str(e)))
        threading.Thread(target=work, daemon=True).start()

    # ------------------------------------------------------------------ playback
    def goto(self, fi, from_slider=False):
        if not self.run or not self.run.frames:
            self.fi = 0
            self._sched_redraw()
            return
        fi = max(0, min(int(fi), len(self.run.frames) - 1))
        self.fi = fi
        if not from_slider:
            self.slider.configure(command=lambda v: None)
            self.slider.set(fi)
            self.slider.configure(command=self._on_slider)
        f = self.run.frames[fi]
        ev = f["ev"]
        if 0 <= ev < self.lb.size():
            self.lb.selection_clear(0, "end")
            self.lb.selection_set(ev)
            self.lb.see(ev)
        tp, ip = f["pose"], f["planned"]
        drift = math.hypot(tp[0] - ip[0], tp[1] - ip[1])
        dth = ((tp[2] - ip[2] + 180) % 360) - 180
        self.v_pose.set(f"true ({tp[0]:6.1f},{tp[1]:6.1f}) {tp[2] % 360:5.1f} deg   planner believes "
                        f"({ip[0]:6.1f},{ip[1]:6.1f}) {letter(ip[2])}   off by {drift:4.1f} cm / {dth:+5.1f} deg")
        self.v_event.set(self.run.events[ev]["text"] if 0 <= ev < len(self.run.events) else "")
        self.v_score.set(self._score_text(fi))
        self._sched_redraw()

    def _score_text(self, fi):
        parts = []
        for oid in (self.plan["order"] if self.plan else []):
            ph = [p for p in self.run.photos if p["oid"] == oid and p["frame"] <= fi]
            if not ph:
                parts.append(f"#{oid}: -")
            else:
                parts.append(f"#{oid}: " + ">".join(("OK " if p["ok"] else "X ") + f"{p['conf']:.2f}" for p in ph))
        return "   ".join(parts)

    def _on_slider(self, v):
        self.goto(int(float(v)), from_slider=True)

    def _on_list_select(self, _e=None):
        s = self.lb.curselection()
        if not s or not self.run:
            return
        e = self.run.events[s[0]]
        self.goto(e["f0"] if e["tag"] == "log" else e["f1"])

    def _event_indices(self):
        return [i for i, e in enumerate(self.run.events) if e["tag"] != "log"]

    def next_event(self):
        if not self.run or not self.run.frames:
            return
        cur = self.run.frames[self.fi]["ev"]
        ce = self.run.events[cur]
        if ce["tag"] != "log" and self.fi < ce["f1"]:
            return self.goto(ce["f1"])
        for i in self._event_indices():
            if i > cur:
                return self.goto(self.run.events[i]["f1"])
        self.goto(10 ** 9)

    def prev_event(self):
        if not self.run or not self.run.frames:
            return
        cur = self.run.frames[self.fi]["ev"]
        prev = [i for i in self._event_indices() if i < cur]
        self.goto(self.run.events[prev[-1]]["f1"] if prev else 0)

    def toggle_play(self):
        if not self.run:
            return
        if self.playing:
            self.playing = False
            self.btn_play.configure(text="Play")
            return
        if self.fi >= len(self.run.frames) - 1:
            self.fi = 0
        self.playing = True
        self.btn_play.configure(text="Pause")
        self._tick()

    def _tick(self):
        if not self.playing or not self.run:
            return
        nf = self.fi + int(self.speed.get())
        last = len(self.run.frames) - 1
        if nf >= last:
            self.goto(last)
            self.playing = False
            self.btn_play.configure(text="Play")
            return
        self.goto(nf)
        self.root.after(30, self._tick)

    # ------------------------------------------------------------------ config file actions
    def reset_cfg(self):
        for k in CFG_KEYS:
            setattr(config, k, copy.deepcopy(ORIG[k]))
        for p in self.params["plan"]:
            p.refresh()
        self.cb_head.set(config.START_POSE_CM[2])
        self.v_align.set(config.ALIGN_ENABLED)
        self.changed("plan")

    def _changed_items(self):
        return [(k, getattr(config, k)) for k in CFG_KEYS if getattr(config, k) != ORIG[k]]

    def copy_changed(self):
        items = self._changed_items()
        if not items:
            self.set_status("Nothing differs from config.py")
            return
        text = "\n".join(f"{k} = {lit(v)}" for k, v in items)
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.set_status("Copied the changed config lines to the clipboard")

    def save_config(self, path=None, quiet=False):
        path = path or os.path.join(HERE, "config.py")
        items = self._changed_items()
        if not items:
            self.set_status("Nothing differs from config.py")
            return
        if not quiet and not messagebox.askyesno(
                "Save to config.py",
                "Overwrite these values in config.py?\n\n" + "\n".join(k for k, _ in items) +
                "\n\nA timestamped backup is made first. Restart algo_server.py afterwards."):
            return
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
        missing = []
        for k, v in items:
            if k == "TURN_MODELS":
                block = "TURN_MODELS = {\n    " + ",\n    ".join(
                    ", ".join(f'"{n}": {lit(v[n])}' for n in pair if n in v)
                    for pair in (("RT90", "LT90"), ("XL90", "XR90"))) + ",\n}"
                text, cnt = re.subn(r"(?ms)^TURN_MODELS\s*=\s*\{.*?^\}", lambda m: block, text, count=1)
            else:
                text, cnt = re.subn(rf"(?m)^({k}\s*=\s*)([^#\n]*?)(\s*(?:#.*)?)$",
                                    lambda m, v=v: m.group(1) + lit(v) + m.group(3), text, count=1)
            if cnt == 0:
                missing.append(k)
        backup = f"{path}.{datetime.now():%Y%m%d_%H%M%S}.bak"
        shutil.copy2(path, backup)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(text)
        for k, v in items:
            if k not in missing:
                ORIG[k] = copy.deepcopy(v)
        self._update_diff()
        msg = f"Saved {len(items) - len(missing)} value(s) to config.py (backup: {os.path.basename(backup)})"
        if missing:
            msg += f"; could not find in file: {', '.join(missing)}"
        self.set_status(msg, bool(missing))

    def copy_commands(self):
        if self.plan:
            self.root.clipboard_clear()
            self.root.clipboard_append("\n".join(self.plan["commands"]))
            self.set_status("Copied the command list")

    # ------------------------------------------------------------------ drawing
    def _sched_redraw(self):
        if self._redraw_job is None:
            self._redraw_job = self.root.after(15, self._do_redraw)

    def _do_redraw(self):
        self._redraw_job = None
        try:
            self.redraw()
        except Exception:
            import traceback
            traceback.print_exc()

    def redraw(self):
        c = self.cv
        c.delete("all")
        W, H = c.winfo_width(), c.winfo_height()
        if W < 80 or H < 80:
            return
        cell = config.CELL_CM
        ext = config.GRID * cell
        m = 34
        s = max(0.4, min((W - 2 * m) / ext, (H - 2 * m) / ext))
        ox, oy = (W - ext * s) / 2, (H - ext * s) / 2
        self.geom = (ox, oy, s, ext)
        X = lambda x: ox + x * s
        Y = lambda y: oy + (ext - y) * s
        pts = lambda poly: [v for p in poly for v in (X(p[0]), Y(p[1]))]

        c.create_rectangle(X(0), Y(ext), X(ext), Y(0), fill="white", outline="#263238", width=2)
        fine = self.v_fine.get() and s >= 2.2          # 1 cm grid needs enough pixels per cm
        step = 1 if fine else cell
        for k in range(1, int(ext / step)):
            v = k * step
            if v % (5 * cell) == 0:
                col = "#90a4ae"
            elif v % cell == 0:
                col = "#cfd8dc"
            else:
                col = "#dfe5e8"
            c.create_line(X(v), Y(0), X(v), Y(ext), fill=col)
            c.create_line(X(0), Y(v), X(ext), Y(v), fill=col)
        for v in range(0, int(ext) + 1, 20):
            c.create_text(X(v), Y(0) + 12, text=str(v), fill="#455a64", font=("Arial", 8))
            c.create_text(X(0) - 14, Y(v), text=str(v), fill="#455a64", font=("Arial", 8))
        c.create_text(X(ext), Y(0) + 24, text="x (cm)", anchor="e", fill="#455a64", font=("Arial", 8))
        c.create_text(X(0) - 14, Y(ext) - 12, text="y (cm)", anchor="w", fill="#455a64", font=("Arial", 8))
        c.create_rectangle(X(0), Y(30), X(30), Y(0), outline="#1565c0", dash=(5, 3), width=2)
        c.create_text(X(15), Y(30) - 8, text="30x30 start", fill="#1565c0", font=("Arial", 8))

        run, fi = self.run, self.fi
        frame = run.frames[fi] if run and run.frames else None

        # obstacle status colours at this frame
        status = {}
        if run:
            for p in run.photos:
                if p["frame"] <= fi:
                    status[p["oid"]] = (p["attempt"], p["ok"])
        skipped = {s["obstacle_id"] for s in self.plan["skipped"]} if self.plan else set()

        if self.v_halo.get():
            hm = config.MIN_CLEARANCE_CM + config.POSITION_MARGIN_CM
            for o in self.obstacles:
                x0, y0 = o[1] * cell, o[2] * cell
                c.create_rectangle(X(x0 - hm), Y(y0 + cell + hm), X(x0 + cell + hm), Y(y0 - hm),
                                   outline="#ef9a9a", dash=(3, 3))
        if self.v_goals.get() and self.plan:
            for seg in self.plan["segments"]:
                gx, gy, gh = seg["goal_pose"]
                c.create_polygon(pts(body_poly(gx, gy, HEAD_DEG[gh])), outline="#2e7d32", fill="", dash=(2, 2))
        if self.v_path.get():
            for oid, pl in self.nominal:
                if len(pl) > 1:
                    c.create_line(*pts(pl), fill="#90a4ae", dash=(3, 3), width=1)
                    c.create_text(X(pl[-1][0]) + 4, Y(pl[-1][1]) - 8, text=f"to #{oid}", fill="#78909c", font=("Arial", 8))

        # trail of the true robot
        if run and self.v_trail.get():
            groups, last_pt, cur = [], None, None
            for f in run.frames[:fi + 1]:
                src = f["src"]
                if src is None:
                    continue
                p = (f["pose"][0], f["pose"][1])
                if cur is None or cur[0] != src:
                    cur = [src, [last_pt] if last_pt else []]
                    groups.append(cur)
                cur[1].append(p)
                last_pt = p
            for src, pl in groups:
                if len(pl) > 1:
                    c.create_line(*pts(pl), fill=COL.get(src, "#333"), width=3 if src in ("correct", "ac", "ra") else 2)

        # obstacles
        for o in self.obstacles:
            oid, gx, gy, face = o
            x0, y0 = gx * cell, gy * cell
            c.create_rectangle(X(x0), Y(y0 + cell), X(x0 + cell), Y(y0), fill="#37474f", outline="#000")
            if fine:                                   # show the 10 x 10 one-cm cells it covers
                for k in range(1, int(cell)):
                    c.create_line(X(x0 + k), Y(y0), X(x0 + k), Y(y0 + cell), fill="#546e7a")
                    c.create_line(X(x0), Y(y0 + k), X(x0 + cell), Y(y0 + k), fill="#546e7a")
            att = status.get(oid)
            if oid in skipped:
                bar = "#9e9e9e"
            elif att is None:
                bar = "#e0b000"
            elif att[1]:
                bar = "#2e9d2e" if att[0] == 1 else "#8bc34a"
            else:
                bar = "#ff9800" if att[0] == 1 else "#d32f2f"
            t = 3
            rect = {"N": (x0, y0 + cell, x0 + cell, y0 + cell + t), "S": (x0, y0 - t, x0 + cell, y0),
                    "E": (x0 + cell, y0, x0 + cell + t, y0 + cell), "W": (x0 - t, y0, x0, y0 + cell)}[face]
            c.create_rectangle(X(rect[0]), Y(rect[3]), X(rect[2]), Y(rect[1]), fill=bar, outline="#000")
            c.create_text(X(x0 + cell / 2), Y(y0 + cell / 2), text=str(oid), fill="white", font=("Arial", 10, "bold"))
            if oid in skipped:
                c.create_line(X(x0), Y(y0), X(x0 + cell), Y(y0 + cell), fill="#ff5252", width=2)
                c.create_line(X(x0), Y(y0 + cell), X(x0 + cell), Y(y0), fill="#ff5252", width=2)
            if att is not None and run:
                seq = ">".join(("OK" if p["ok"] else "X") + f"{p['conf']:.2f}"[1:] for p in run.photos
                               if p["oid"] == oid and p["frame"] <= fi)
                c.create_text(X(x0 + cell) + 4, Y(y0 + cell) - 2, text=seq, anchor="sw", fill="#000",
                              font=("Arial", 8, "bold"))

        # ghost + robot
        if frame:
            tp, ip = frame["pose"], frame["planned"]
        else:
            sx, sy, sh = config.START_POSE_CM
            tp = ip = (sx, sy, HEAD_DEG.get(sh, 0.0))
        if self.v_ghost.get():
            c.create_polygon(pts(body_poly(*ip)), fill="", outline="#666", dash=(4, 3), width=2)
        hz = frame.get("hz") if frame else None
        poly = body_poly(*tp)
        c.create_polygon(pts(poly), fill="#bbdefb", outline={"collision": "#d50000", "close": "#ff6d00"}.get(hz, "#0d47a1"),
                         width=3 if hz else 2)
        c.create_line(X(poly[0][0]), Y(poly[0][1]), X(poly[3][0]), Y(poly[3][1]), fill="#d50000", width=4)
        fx, fy = fvec(tp[2])
        c.create_line(X(tp[0]), Y(tp[1]), X(tp[0] + fx * 10), Y(tp[1] + fy * 10), fill="#0d47a1", width=2, arrow="last")
        sx_, sy_ = tp[0] + fx * config.SENSOR_FORWARD_CM, tp[1] + fy * config.SENSOR_FORWARD_CM
        c.create_oval(X(sx_) - 3, Y(sy_) - 3, X(sx_) + 3, Y(sy_) + 3, fill="#ffeb3b", outline="#333")

        if frame and self.v_beam.get():
            if frame.get("beam"):
                S, E, label = frame["beam"]
                c.create_line(X(S[0]), Y(S[1]), X(E[0]), Y(E[1]), fill="#e53935", width=2, dash=(6, 3))
                c.create_text(X((S[0] + E[0]) / 2), Y((S[1] + E[1]) / 2) - 9, text=label, anchor="s",
                              fill="#b71c1c", font=("Arial", 9, "bold"))
            if frame.get("fov"):
                P, th, fov = frame["fov"]
                a = fvec(th - fov / 2)
                b = fvec(th + fov / 2)
                L = 60
                c.create_line(X(P[0]), Y(P[1]), X(P[0] + a[0] * L), Y(P[1] + a[1] * L), fill="#ffb300", width=2)
                c.create_line(X(P[0]), Y(P[1]), X(P[0] + b[0] * L), Y(P[1] + b[1] * L), fill="#ffb300", width=2)
                c.create_line(X(P[0] + a[0] * L), Y(P[1] + a[1] * L), X(P[0] + b[0] * L), Y(P[1] + b[1] * L),
                              fill="#ffb300", width=1, dash=(3, 3))
                ph = frame.get("photo")
                if ph:
                    v = ph["view"]
                    txt = (("RECOGNISED" if ph["ok"] else "FAILED") + f"  conf {ph['conf']:.2f}"
                           f"{'  (forced)' if ph['forced'] else ''}\n"
                           f"dist {v['dist']:.0f} cm   bearing {v['bearing']:+.0f} deg   view angle {v['view_err']:.0f} deg")
                    c.create_text(W / 2, 3, text=txt, anchor="n", fill="#1b5e20" if ph["ok"] else "#b71c1c",
                                  font=("Arial", 10, "bold"))
        if run is None and self.plan is None:
            c.create_text(W / 2, H / 2, text="no plan yet", fill="#b0bec5", font=("Arial", 18))


def selftest():
    obs = [(1, 10, 10, "W"), (2, 5, 17, "S")]
    plan = make_plan(obs)
    sp = dict(SIM_DEFAULTS)
    sp.update(PRESETS["Realistic (small errors)"])
    run = simulate(obs, plan, sp)
    print("commands:", plan["commands"])
    print("frames:", len(run.frames), "events:", len(run.events), "summary:", run.summary)
    print(format_mc(monte_carlo(obs, plan, sp, 5)))


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    if not HAVE_TK:
        print("tkinter is not available in this Python. Install the standard python.org build (it includes tkinter).")
        return
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
