"""
Task 2 (Fastest car) - the TEAM's algorithm (KC v13 route, KC_stm32_link.py +
KC_task2_o1_calibration.py) running on YOUR firmware commands.

Same calling style as task2.run_task2(): send(cmd) blocks until DONE, read_us(),
read_ir_left(), read_ir_right(), snap(), optional read_heading() and drive_edge()
(the STM IE command). Only these commands are ever sent:
    FW<cm> BW<cm> LT<deg> RT<deg>   and IE<L|R><N|A><cm> through drive_edge()

TEAM ROUTE (KC v13)                                 -> how it is sent here
 0. Ultrasonic approach to O1, stop at 65 cm          FW (US median) + bounded <=10 cm fixes
    Arrow: two consecutive confident photos (>=0.85)  snap() up to 4 times
 1. O1 side-lane exit: 45 out, 28 cm, 45 back,        LT45/RT45, FW28, RT45/LT45, FW n
    short forward until the side IR is ~2 cm past       (n worked out from YOUR turn models;
    O1's rear edge; stays in the side lane              the team hard-coded 12 cm for theirs)
 2. O2 coarse centring: photos at 0 / 15 / 25 deg     LT/RT15, LT/RT10, snap at each
    inward, centre bearing = middle of the window
    where the arrow is seen, drive diagonally to      FW at that bearing to US 35 cm
    US 35 cm, then undo ONLY the heading              RT/LT<bearing>
 3. O2 loop (header plan of v9-v13): 90 out, slide    LT90/RT90, IE..N/A (front edge),
    along O2's face until the inside IR passes the      FW extra, LT90/RT90, IE (pass),
    edge + IR->rear margin, 90 in, pass O2 with the     FW, 90 in, FW across, 90 in,
    inside IR, cross behind O2 to the mirror lane,      IE..N (pass O2 going home)
    90 in, pass O2 again with the inside IR
 4. Home: inside IR passes O1, shift to centre        S-curve (LT/RT pairs), IE..N (O1)
 5. Park: both side IRs acquire the carpark walls,    FW5 + IR steps, then FW to US 18 cm
    then pull in with an ultrasonic wall guard 18 cm

WHAT IS NOT THE TEAM'S (their v13 stops at a safety gate after the first O2 90-degree
turn, so the rest of their route was never finished / validated):
  * every distance that depended on THEIR turn arcs is computed from YOUR measured
    turn models in config.py (TURN_MODELS / TURN45_MODELS / SMALL_TURN_MODELS), and
    every planned path is checked against the obstacles with task2._box_clear;
  * their obstacle2_loop_and_return() mirrors the sideways translation, which would end
    BEHIND O2 facing it; the header plan they wrote (cross behind O2, return down the
    opposite corridor, inner IR passes O2 then O1) is used instead;
  * their park() assumes the robot is already centred, so a centring S-curve (their
    "rough-centre" 45-in / 45-out trial) is added after O1 on the way home.
"""
import math
import re

import config as C
import task2 as B          # your helpers: _Pose, _box_clear, _us_median, NEEDS, T2 defaults

# ---------------------------------------------------------------------------
# TEAM CONSTANTS (KC_stm32_link.py v13 / KC_task2_o1_calibration.py)
# ---------------------------------------------------------------------------
O1_CAPTURE_US_CM = 65.0          # O1_CAPTURE_US_CM
APPROACH_TOL_CM = 2.0            # stop when within +-2 cm (calibration: 32-38 / 31-33 windows)
APPROACH_FIX_MAX_CM = 10         # calibration: final adjustments are <= 10 cm each
OBS1_ANGLE_DEG = 45              # OBS1_ANGLE_DEG
OBS1_MIN_STANDOFF_CM = 35.0      # calibration trials: 35 cm (32 for the centring trial)
OBS1_OUT_DIAGONAL_CM = 28        # OBS1_OUT_DIAGONAL_CM
OBS1_IR_PAST_REAR_CM = 2.0       # "10 cm measured + 2 cm margin": side IR 2 cm past O1's rear
O2_SWEEP_ANGLES_DEG = (0, 15, 25)
O2_CENTER_US_CM = 35.0
O2_SIDE_SEARCH_MAX_CM = 100
O2_PASS_SEARCH_MAX_CM = 80
RETURN_GAP_CM = 15.0             # NOT team: side gap to O2's end on the return corridor
WALL_GAP_MIN_CM = 50.0           # rules: arena wall >= 50 cm beyond each end of O2
O2_EDGE_UNCERT_CM = 2.0          # NOT team: IR edge position uncertainty added to O2's end
IR_EDGE_MARGIN_CM = 5.0          # IR_EDGE_MARGIN_CM
CARPARK_SEARCH_MAX_CM = 170
CARPARK_PULL_IN_MAX_CM = 40
CARPARK_WALL_STOP_CM = 18.0
PARK_IR_OBJECT_CM = 30.0         # IR_OBJECT_CM
PARK_IR_DANGER_CM = 7.0          # IR_DANGER_CM
PARK_STEP_CM = 5                 # O2_SIDE_CHUNK_CM used for chunked IR searches
ARROW_CONFIRM_CONF = 0.85        # recognise(): confidence >= 0.85, two in a row
ARROW_MAX_PHOTOS = 4

# ---------------------------------------------------------------------------
# YOUR ROBOT (config.py)
# ---------------------------------------------------------------------------
W, LEN, SENSOR_FWD = C.ROBOT_WIDTH_CM, C.ROBOT_LENGTH_CM, C.SENSOR_FORWARD_CM
IR_FWD, IR_INSET, IR_LAG = C.T2_IR_FWD_CM, C.T2_IR_INSET_CM, C.T2_IR_LAG_CM
IR_TO_REAR_CM = LEN / 2 + IR_FWD                       # team measured 18 on their car
IR_EDGE_EXTRA_CM = round(IR_TO_REAR_CM + IR_EDGE_MARGIN_CM)
IR_NEAR_CM = 40.0                                      # main_merged.c IE_NEAR_MAX_MM
IR_MAX_CM = 80.0                                       # rpi_client_task2.IR_MAX_CM (Sharp range)
US_MAX_CM = 300.0
NEED_PREF = B.NEEDS[0]                                 # clearance aim (cm)
NEED_MIN = B.CLEAR_CM + 1.0                            # hard minimum: rules + 1 cm
INF = 1e4

STAGES = ("obs1", "detect2", "o2turn", "obs2", "full")


def opposite(side):
    return "RIGHT" if side == "LEFT" else "LEFT"


def turn_cmd(side, deg):
    return ("LT" if side == "LEFT" else "RT") + str(int(round(deg)))


def turn_model(cmd):
    """(forward cm, right cm, clockwise deg) for LT<deg> / RT<deg> from config.py.
    90 / 45 / 10 use your measured models; other angles are cut from the 90-degree
    model with the same ellipse config.py uses for its placeholders."""
    m = re.match(r"^(LT|RT)(\d+)$", cmd)
    kind, deg = m.group(1), int(m.group(2))
    if deg == 90 and cmd in C.TURN_MODELS:
        f, r, q = C.TURN_MODELS[cmd]
        return float(f), float(r), 90.0 * q
    if deg == 45 and cmd in C.TURN45_MODELS:
        f, r, k = C.TURN45_MODELS[cmd]
        return float(f), float(r), 45.0 * k
    if cmd in C.SMALL_TURN_MODELS:
        f, r, dg = C.SMALL_TURN_MODELS[cmd]
        return float(f), float(r), float(dg)
    f90, r90, q = C.TURN_MODELS[kind + "90"]
    th = math.radians(deg)
    return f90 * math.sin(th), r90 * (1 - math.cos(th)), q * float(deg)


# =============================================================================
# Context: robot I/O + tracked pose + known obstacles
# =============================================================================

class Ctx:
    def __init__(self, send, read_us, read_ir_left, read_ir_right, snap,
                 read_heading=None, drive_edge=None, cp_front=33.0,
                 cp_half=20.0, cp_depth=50.0, arm=10.0):
        self._send, self._us, self._snap = send, read_us, snap
        self._irl, self._irr = read_ir_left, read_ir_right
        self.read_heading, self.drive_edge = read_heading, drive_edge
        self.pose = B._Pose(0.0)
        self.log = []
        self.boxes = {}
        x0 = cp_front - cp_depth - arm
        self.carpark = {"arm_left": (x0, cp_front, cp_half, cp_half + arm),
                        "arm_right": (x0, cp_front, -cp_half - arm, -cp_half),
                        "back": (x0, cp_front - cp_depth, -cp_half - arm, cp_half + arm)}
        self.cp_front, self.cp_depth = cp_front, cp_depth

    # ---- commands ----
    def cmd(self, c):
        self._send(c)
        self.log.append(c)

    def _sync_h(self):
        if self.read_heading is not None:
            try:
                h = self.read_heading()
                if h is not None:
                    self.pose.h = h
            except Exception:
                pass

    def fw(self, cm):
        cm = int(round(cm))
        word, left = ("FW" if cm > 0 else "BW"), abs(cm)
        while left > 0:
            st = min(left, C.MAX_STRAIGHT_COMMAND_CM)
            self.cmd(f"{word}{st}")
            self.pose.move(st if word == "FW" else -st)
            self._sync_h()
            left -= st

    def turn(self, side, deg):
        deg = int(round(deg))
        if deg < 1:
            return
        c = turn_cmd(side, deg)
        self.cmd(c)
        f, r, dg = turn_model(c)
        self.pose.move(f, r, dg)
        self._sync_h()

    # ---- sensors ----
    def us(self, lo=3.0, hi=US_MAX_CM):
        return B._us_median(self._us, lo=lo, hi=hi)

    def ir(self, side):
        return self._irl() if side == "LEFT" else self._irr()

    def snap(self):
        return self._snap()

    def ir_point(self, side, pose=None):
        """(x, y) of a side IR face."""
        p = pose or self.pose
        hr = math.radians(p.h); c, sn = math.cos(hr), math.sin(hr)
        sgn = 1 if side == "RIGHT" else -1
        off = W / 2 - IR_INSET
        return (p.x + IR_FWD * c + sgn * off * sn, p.y + IR_FWD * sn - sgn * off * c)

    def side_towards(self, y_target):
        """Which robot side (LEFT/RIGHT) points at the line y = y_target."""
        hr = math.radians(self.pose.h)
        right_y = -math.cos(hr)                     # y part of the right-hand vector
        return "RIGHT" if (y_target - self.pose.y) * right_y > 0 else "LEFT"

    # ---- IR edge leg (IE, or FW3 + IR steps on old firmware) ----
    def edge(self, side, max_cm, mode="auto"):
        """Drive forward until the side IR goes near -> far. mode 'auto': N if the IR
        sees the obstacle now, A (far counts at once) if it is already past it."""
        if mode == "auto":
            now = self.ir(side)
            need_seen = now is not None and now <= IR_NEAR_CM
        else:
            need_seen = mode == "N"
        p0 = self.pose.copy()
        if self.drive_edge is not None:
            r = dict(self.drive_edge(side, need_seen, max_cm))
        else:
            r = self._edge_steps(side, need_seen, max_cm)
        self.log.append(B._edge_log(r) + f"  (from x {p0.x:.0f} y {p0.y:.0f} h {p0.h:.0f})")
        self.pose.move(r["moved_cm"])
        self._sync_h()
        r["start_pose"] = p0
        r["need_seen"] = need_seen
        return r

    def _edge_steps(self, side, need_seen, max_cm, step=3):
        moved, seen, run, first_far, nears = 0, False, 0, None, []
        while True:
            v = self.ir(side)
            if v is not None and v <= IR_NEAR_CM:
                seen, run = True, 0
                nears.append(v)
            elif seen or not need_seen:
                if run == 0:
                    first_far = moved
                run += 1
                if run >= 2:
                    end = "EDGE" if seen else "NEVER"
                    break
            if moved >= max_cm:
                end, first_far = "MAX", None
                break
            st = min(step, int(max_cm - moved))
            self._send(f"FW{st}")
            moved += st
        nears.sort()
        return {"end": end, "edge_cm": first_far if end != "MAX" else None,
                "near_cm": nears[len(nears) // 2] if nears else None,
                "moved_cm": float(moved), "cmd": f"FW3-steps {side}"}

    # ---- geometry ----
    def clearance(self, seq, start=None, boxes=None):
        """Smallest gap (cm) between the robot and the known boxes along seq.
        seq items: ('FW', cm) or ('LT'|'RT', deg). Turns sampled 10x (like sim.py)."""
        p = (start or self.pose).copy()
        boxes = self.all_boxes() if boxes is None else boxes
        best = INF

        def chk(q):
            nonlocal best
            for b in boxes.values():
                best = min(best, B._box_clear(q, b, W, LEN))
        chk(p)
        for k, v in seq:
            if k == "FW":
                n = abs(v); sgn = 1 if v >= 0 else -1
                while n > 0:
                    st = min(2.0, n)
                    p.move(sgn * st); chk(p)
                    n -= st
            else:
                v = int(round(v))
                if v < 1:
                    continue
                f, r, dg = turn_model(f"{k}{v}")
                q = p.copy(); q.move(f, r, dg)
                for i in range(1, 11):
                    t = i / 10
                    mid = p.copy()
                    mid.x, mid.y, mid.h = p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t, p.h + (q.h - p.h) * t
                    chk(mid)
                p = q
        return best, p

    def need(self, want):
        """Clearance a move must keep: `want`, but never more than the robot already has
        right now (minus 1 cm), and never less than the rules' CLEAR_CM."""
        now, _ = self.clearance([])
        return max(B.CLEAR_CM, min(want, now - 1.0))

    def end_pose(self, seq, start=None):
        return self.clearance(seq, start, boxes={})[1]

    def all_boxes(self):
        b = dict(self.carpark)
        b.update(self.boxes)
        return b


def _t(side, deg):
    return ("LT" if side == "LEFT" else "RT", deg)


def approach(ctx, target, max_creep_cm=200, step_cm=10):
    """Team approach: drive to the US target (one long move), then bounded <= 10 cm
    corrections until within +-2 cm. Returns the last US reading (or None)."""
    u, crept = ctx.us(lo=5), 0
    while u is None and crept < max_creep_cm:
        ctx.fw(step_cm); crept += step_cm
        u = ctx.us(lo=5)
    if u is None:
        return None
    for i in range(4):
        if abs(u - target) <= APPROACH_TOL_CM:
            break
        d = u - target
        if i > 0:
            d = max(-APPROACH_FIX_MAX_CM, min(APPROACH_FIX_MAX_CM, d))
        if abs(d) < 1:
            break
        ctx.fw(d)
        u2 = ctx.us(lo=5)
        if u2 is None:
            break
        u = u2
    return u


def confirm_arrow(ctx, default=None):
    """Team recognise(): accept an arrow only when two photos in a row agree at
    >= 0.85. After ARROW_MAX_PHOTOS, falls back to the best photo (v13 detect_arrow
    accepted the first LEFT/RIGHT it got)."""
    prev, photos = None, []
    for _ in range(ARROW_MAX_PHOTOS):
        res = ctx.snap()
        photos.append(res)
        acc = res["label"] if res and res.get("conf", 0) >= ARROW_CONFIRM_CONF else None
        if acc is not None and acc == prev:
            return acc, res["conf"], "confirmed", photos
        prev = acc
    best = max((p for p in photos if p), key=lambda p: p["conf"], default=None)
    if best is None:
        return default, None, "default", photos
    how = "best" if best["conf"] >= C.RECOGNISED_CONF else "guess"
    return best["label"], best["conf"], how, photos


# =============================================================================
# Step 0 - approach O1 and read arrow 1
# =============================================================================

def approach_obs1(ctx, target=O1_CAPTURE_US_CM):
    ctx.log = []
    u = approach(ctx, target)
    if u is None:
        return {"status": "NO_OBSTACLE", "log": ctx.log}
    face = ctx.pose.x + SENSOR_FWD + u
    ctx.boxes["obs1"] = (face, face + 10.0, -5.0, 5.0)
    arrow, conf, how, _ = confirm_arrow(ctx, default=getattr(C, "T2_DEFAULT_ARROW_1", "LEFT"))
    ctx.log.append(f"arrow 1 {arrow} ({how})")
    return {"status": "OK", "arrow": arrow, "conf": conf, "arrow_how": how, "us": u,
            "D1": round(ctx.pose.x, 1), "obs1_face": round(face, 1), "log": ctx.log}


# =============================================================================
# Step 1 - O1 side-lane exit (45 out, 28 cm, 45 back, short forward)
# =============================================================================

def bypass_obs1(ctx, arrow):
    ctx.log = []
    return _bypass_obs1(ctx, arrow)


def _bypass_obs1(ctx, arrow):
    out, inn = arrow, opposite(arrow)
    o1_back = ctx.boxes["obs1"][1]
    diag, best = OBS1_OUT_DIAGONAL_CM, None
    while diag <= OBS1_OUT_DIAGONAL_CM + 40:
        seq = [_t(out, OBS1_ANGLE_DEG), ("FW", diag), _t(inn, OBS1_ANGLE_DEG)]
        end = ctx.end_pose(seq)
        ir_x = ctx.ir_point(inn, end)[0]
        side_fw = max(0, math.ceil(o1_back + OBS1_IR_PAST_REAR_CM - ir_x))
        seq.append(("FW", side_fw))
        c, _ = ctx.clearance(seq)
        if best is None or c > best[0]:
            best = (c, diag, side_fw, seq)
        if c >= ctx.need(NEED_PREF):
            break
        diag += 2
    c, diag, side_fw, seq = best
    if c < ctx.need(NEED_PREF):
        # team calibration trials stopped at 32-38 cm: creep closer and re-plan
        u = ctx.us(lo=5)
        if u is not None and u - 5 >= OBS1_MIN_STANDOFF_CM:
            step = min(10, int(u - OBS1_MIN_STANDOFF_CM))
            ctx.log.append(f"bypass from {u:.0f} cm only {c:.1f} cm clear (carpark?) - FW{step} closer")
            ctx.fw(step)
            return _bypass_obs1(ctx, arrow)
    if c < ctx.need(NEED_MIN):
        return {"status": "UNSAFE", "clear": round(c, 1), "log": ctx.log}
    if diag != OBS1_OUT_DIAGONAL_CM:
        ctx.log.append(f"team diagonal {OBS1_OUT_DIAGONAL_CM} cm too tight with your turns -> {diag} cm")
    ctx.turn(out, OBS1_ANGLE_DEG)
    ctx.fw(diag)
    ctx.turn(inn, OBS1_ANGLE_DEG)
    ctx.fw(side_fw)
    return {"status": "OK", "diag": diag, "side_fw": side_fw, "clear": round(c, 1),
            "lane_y": round(ctx.pose.y, 1), "log": ctx.log}


# =============================================================================
# Step 2 - O2 coarse centring: photos at 0/15/25 inward, diagonal US approach
# =============================================================================

def aim_obs2(ctx, o1_side):
    ctx.log = []
    inward, outward = opposite(o1_side), o1_side
    cur, dets, fine = 0, [], []
    for ang in O2_SWEEP_ANGLES_DEG:
        delta = ang - cur
        if delta > 0:
            for _ in range(5):                       # keep the swinging rear off O1
                c, _ = ctx.clearance([_t(inward, delta)])
                if c >= ctx.need(NEED_MIN):
                    break
                ctx.log.append(f"turn {delta} deg would pass {c:.1f} cm from O1 - FW3 first")
                ctx.fw(3)
            ctx.turn(inward, delta)
            cur = ang
        res = ctx.snap()
        if res and res.get("conf", 0) >= C.RECOGNISED_CONF:
            dets.append((cur, res["label"], res["conf"]))
            if res.get("box_cx") is not None and res.get("img_w"):
                b = B.bearing_deg(res["box_cx"], res["img_w"])        # + = left of centre
                fine.append(cur - b if inward == "RIGHT" else cur + b)
            ctx.log.append(f"{cur:2d} deg -> {res['label']} {res['conf']:.2f}")
        else:
            ctx.log.append(f"{cur:2d} deg -> no arrow")
    if not dets:
        ctx.turn(outward, cur)
        return {"status": "NO_ARROW", "log": ctx.log}
    labels = {l for _, l, _ in dets}
    if len(labels) > 1:
        top = max(dets, key=lambda d: d[2])
        if top[2] < ARROW_CONFIRM_CONF:
            ctx.turn(outward, cur)
            return {"status": "CONFLICT", "dets": dets, "log": ctx.log}
        ctx.log.append(f"conflicting arrows {dets} - keeping {top[1]} ({top[2]:.2f} >= 0.85)")
        dets = [d for d in dets if d[1] == top[1]]
    arrow2 = dets[0][1]
    lo, hi = min(a for a, _, _ in dets), max(a for a, _, _ in dets)
    sel = int(round((lo + hi) / 2))
    ctx.log.append(f"visible {lo}..{hi} deg -> centre bearing {sel} deg (team window midpoint)")
    if fine:
        # NOT team: the arrow's pixel column gives the bearing directly (their vision
        # server returned no box, hence the coarse window). Same idea, finer.
        f = sorted(fine)[len(fine) // 2]
        sel = int(round(max(0.0, min(40.0, f))))
        ctx.log.append(f"arrow box bearing -> centre bearing {sel} deg")
    corr = cur - sel
    if corr > 0:
        ctx.turn(outward, corr)
    elif corr < 0:
        ctx.turn(inward, -corr)
    u = approach(ctx, O2_CENTER_US_CM, max_creep_cm=60)
    if u is None:
        return {"status": "NO_US_O2", "arrow": arrow2, "log": ctx.log}
    hr = math.radians(ctx.pose.h)
    x2 = ctx.pose.x + (SENSOR_FWD + u) * math.cos(hr)
    ctx.boxes["obs2"] = (x2, x2 + 10.0, -INF, INF)        # ends not known yet
    normal = u * math.cos(math.radians(sel))
    ctx.turn(outward, sel)                                 # undo heading ONLY
    ctx.log.append(f"aimed US {u:.1f} cm, normal clearance ~{normal:.1f} cm, O2 face x {x2:.1f}")
    return {"status": "OK", "arrow": arrow2, "bearing": sel, "us": u, "normal": round(normal, 1),
            "x2": round(x2, 1), "y": round(ctx.pose.y, 1), "log": ctx.log}


# =============================================================================
# Step 3 - loop O2
# =============================================================================

def loop_obs2(ctx, arrow2, stop_after_turn=False):
    ctx.log = []
    out, inn = arrow2, opposite(arrow2)
    inside = inn                                       # IR facing O2 the whole loop
    x2 = ctx.boxes["obs2"][0]

    # 3a. first 90 out (team: o2turn), backing off if the arc would touch O2. The walls
    # are not seen yet: assume the closest the rules allow (O2 >= 30 cm, wall >= 50 past).
    # NOT team: the OUTER side IR faces that wall now (Sharp range 80 cm) - measure it.
    v = ctx.ir(out)
    ix, iy = ctx.ir_point(out)
    hr = math.radians(ctx.pose.h)
    dir_y = -math.cos(hr) if out == "RIGHT" else math.cos(hr)     # y of that side's normal
    reach = v if v is not None else IR_MAX_CM
    wy = iy + dir_y * reach
    sg = 1 if dir_y > 0 else -1
    prov = {"pwall": (x2 - 400, x2 + 400, min(wy, wy + 5 * sg), max(wy, wy + 5 * sg))}
    ctx.log.append(f"outer IR: wall {'%.0f cm' % v if v is not None else 'beyond ' + str(int(IR_MAX_CM)) + ' cm'}"
                   f" (y {wy:.0f})")
    if v is not None:
        ctx.boxes["wall_" + out.lower()] = prov["pwall"]
    pick = None
    for use_prov in (True, False):
        boxes = dict(ctx.all_boxes(), **prov) if use_prov else ctx.all_boxes()
        need_pref, need_min = ctx.need(NEED_PREF), ctx.need(NEED_MIN)
        good, ok = None, None
        for back in range(0, 31, 2):
            options = [[("FW", -back), _t(out, 90)]]
            options += [[("FW", -back), _t(out, 45), ("FW", d), _t(out, 45)] for d in (0, 4)]
            for seq in options:
                c, _ = ctx.clearance(seq, boxes=boxes)
                if c >= need_pref and good is None:
                    good = (c, seq)
                elif c >= need_min and (ok is None or c > ok[0] + 0.3):
                    ok = (c, seq)
            if good is not None:
                break
        pick = good or ok
        if pick is not None:
            break
        if use_prov:
            # NOT team: already outside a short O2 - back up, S-curve towards the
            # centre line, then the out-turn (all checked against the closest wall)
            for back in range(10, 71, 4):
                start = ctx.end_pose([("FW", -back)])
                for yt in (0.0, 0.5 * ctx.pose.y):
                    sq = _s_seq(ctx, start, yt, 45)
                    if sq is None:
                        continue
                    seq = [("FW", -back)] + sq + [_t(out, 90)]
                    c, _ = ctx.clearance(seq, boxes=boxes)
                    if c >= need_min and (pick is None or c > pick[0]):
                        pick = (c, seq)
                if pick is not None and pick[0] >= need_pref:
                    break
            if pick is not None:
                ctx.log.append("robot already beside O2's end - back up and re-centre before the out-turn")
                break
        return {"status": "UNSAFE_TURN", "log": ctx.log + ["no out-turn clears the side wall"]}
    if pick is None:
        return {"status": "UNSAFE_TURN", "log": ctx.log}
    c, seq = pick
    if seq[0][1]:
        ctx.log.append(f"BW{-int(round(seq[0][1]))} first; the out-turn keeps {c:.1f} cm clear")
    for k, v in seq:
        if k == "FW":
            ctx.fw(v)
        else:
            ctx.turn("LEFT" if k == "LT" else "RIGHT", v)
    if stop_after_turn:
        return {"status": "OK", "stage": "o2turn", "log": ctx.log}
    # facing the side wall now: the front US measures it (rules: >= 50 cm past O2)
    u = ctx.us(lo=3)
    if u is not None and u < 200:
        hr = math.radians(ctx.pose.h)
        wy = ctx.pose.y + (SENSOR_FWD + u) * math.sin(hr)
        sgn = 1 if wy > 0 else -1
        ctx.boxes["wall_" + out.lower()] = (x2 - 400, x2 + 400, min(wy, wy + 5 * sgn), max(wy, wy + 5 * sgn))
        ctx.log.append(f"side wall seen {u:.0f} cm ahead (y {wy:.0f})")
    # already past O2's end? (NOT team) back up until the inside IR sees O2 again
    v = ctx.ir(inside)
    if v is None or v > IR_NEAR_CM:
        backed = 0
        while backed < 60:
            ctx.fw(-3); backed += 3
            v = ctx.ir(inside)
            if v is not None and v <= IR_NEAR_CM:
                break
        ctx.log.append(f"inside IR was past O2's end - backed {backed} cm"
                       + ("" if v is not None and v <= IR_NEAR_CM else " (still nothing - short O2)"))

    # 3b. slide along O2's face until the inside IR passes its edge
    r = ctx.edge(inside, O2_SIDE_SEARCH_MAX_CM)
    if r["end"] == "MAX" or r["end"] == "WALL":
        return {"status": "O2_EDGE_" + r["end"], "log": ctx.log}
    p0 = r["start_pose"]
    hr = math.radians(p0.h)
    ir0 = ctx.ir_point(inside, p0)
    e = (r["edge_cm"] or 0.0) - IR_LAG
    y_edge = ir0[1] + e * math.sin(hr)
    half = max(abs(y_edge) + O2_EDGE_UNCERT_CM, 15.0)
    ctx.boxes["obs2"] = (x2, x2 + 10.0, -half, half)
    wy = half + WALL_GAP_MIN_CM                        # rules: walls >= 50 cm past O2's ends
    ctx.boxes.setdefault("wall_left", (x2 - 400, x2 + 400, wy, wy + 5))
    ctx.boxes.setdefault("wall_right", (x2 - 400, x2 + 400, -wy - 5, -wy))
    ctx.log.append(f"O2 edge at y {y_edge:.1f} -> length ~{2 * half:.0f} cm (centred)")

    # 3c. team: FW IR->rear+margin ({IR_EDGE_EXTRA_CM} cm here), then 90 in. With your
    # wide 90-degree arcs the extra is chosen so the arc clears O2 AND the wall, and the
    # 90 may be split into 45 + FW + 45 (tighter sweep past O2's corner).
    good, ok = None, None
    need_pref, need_min = ctx.need(NEED_PREF), ctx.need(NEED_MIN)
    for extra in range(0, IR_EDGE_EXTRA_CM + 21, 2):
        options = [[("FW", extra), _t(inn, 90)]]
        options += [[("FW", extra), _t(inn, 45), ("FW", d), _t(inn, 45)] for d in (0, 4, 8)]
        for seq in options:
            c, endp = ctx.clearance(seq)
            out_y = abs(endp.y)
            if c >= need_pref and (good is None or out_y < good[2]):
                good = (c, seq, out_y)
            elif c >= need_min and (ok is None or c > ok[0]):
                ok = (c, seq, out_y)
        if good is not None:
            break
    pick = good or ok
    if pick is None:
        return {"status": "UNSAFE_TURN2", "log": ctx.log}
    for k, v in pick[1]:
        if k == "FW":
            ctx.fw(v)
        else:
            ctx.turn("LEFT" if k == "LT" else "RIGHT", v)

    # 3d. pass O2 lengthwise with the inside IR
    r = ctx.edge(inside, O2_PASS_SEARCH_MAX_CM)
    if r["end"] in ("MAX", "WALL") and r["need_seen"]:
        return {"status": "O2_PASS_" + r["end"], "log": ctx.log}

    # 3e. cross behind O2 to the opposite corridor: FW n1, 90 in, FW n2, 90 in.
    # Team mirrored the outward translation; here the corridor is put RETURN_GAP_CM
    # off O2's far end so the inside IR (10-40 cm) can see O2 on the way back.
    sgn = 1 if ctx.pose.y > 0 else -1
    y_ret = -sgn * (half + W / 2 + RETURN_GAP_CM)
    plan = None
    for n1 in range(0, 81, 2):
        base_end = ctx.end_pose([("FW", n1), _t(inn, 90)])
        s = math.sin(math.radians(base_end.h))
        if abs(s) < 0.5:
            break
        fin0 = ctx.end_pose([_t(inn, 90)], start=base_end)
        n2 = (y_ret - fin0.y) / s
        if n2 < 0:
            continue
        seq = [("FW", n1), _t(inn, 90), ("FW", n2), _t(inn, 90)]
        c, _ = ctx.clearance(seq)
        if plan is None or c > plan[0]:
            plan = (c, seq)
        if c >= ctx.need(NEED_PREF):
            break
    if plan is None or plan[0] < ctx.need(NEED_MIN):
        return {"status": "UNSAFE_CROSS", "log": ctx.log}
    for k, v in plan[1]:
        if k == "FW":
            ctx.fw(v)
        else:
            ctx.turn("LEFT" if k == "LT" else "RIGHT", v)

    # 3f. back down the corridor: the inside IR passes O2 again
    ir_x = ctx.ir_point(inside)[0]
    if ir_x < x2:
        r = {"end": "PAST", "edge_cm": None}
        ctx.log.append("inside IR already past O2's face after the crossing turn")
    else:
        r = ctx.edge(inside, min(O2_PASS_SEARCH_MAX_CM, ir_x - x2 + 25),
                     mode="N" if ir_x > x2 + 10 else "auto")
    if r["end"] == "EDGE" and r["edge_cm"] is not None:
        # IR at the edge = O2's home face: re-sync the tracked x
        hr = math.radians(ctx.pose.h)
        ir_dx = IR_FWD * math.cos(hr)
        new_x = x2 - ir_dx + (r["moved_cm"] - r["edge_cm"] + IR_LAG) * math.cos(hr)
        ctx.log.append(f"x re-synced on O2 face: {ctx.pose.x:.1f} -> {new_x:.1f}")
        ctx.pose.x = new_x
    else:
        ctx.log.append(f"inside IR did not confirm O2 on the way back ({r['end']}) - odometry only")
    return {"status": "OK", "length": round(2 * half, 1), "y_edge": round(y_edge, 1),
            "y_return": round(ctx.pose.y, 1), "log": ctx.log}


# =============================================================================
# Step 4 - home: shift lane, inside IR passes O1, centre for the carpark
# =============================================================================

def _s_seq(ctx, start, y_target, ang):
    """[turn in ang, FW d, turn out ang] from pose start to reach y_target, or None."""
    hr = math.radians(start.h)
    dy = y_target - start.y
    inn = "RIGHT" if dy * (-math.cos(hr)) > 0 else "LEFT"
    out = opposite(inn)
    e0 = ctx.end_pose([_t(inn, ang), _t(out, ang)], start=start)
    mid = ctx.end_pose([_t(inn, ang)], start=start)
    sn = math.sin(math.radians(mid.h))
    if abs(sn) < 0.1:
        return None
    d = (y_target - e0.y) / sn
    if d < 0:
        return None
    return [_t(inn, ang), ("FW", d), _t(out, ang)]


def s_curve(ctx, y_target, angles=(45, 30, 20, 90), pre_max=60, front_limit=None):
    """Lateral shift to y_target with a turn pair (in, FW d, out), optionally after a
    straight pre-drive. Among the clear options the one that uses the least forward
    room wins. front_limit: robot front must end no further than this x (going home).
    Returns (clear, pre_fw, (in, out, angle), d) or None."""
    dy = y_target - ctx.pose.y
    if abs(dy) < 2:
        return (INF, 0, None, 0)
    good, ok = None, None
    need_pref, need_min = ctx.need(NEED_PREF), ctx.need(NEED_MIN)
    for pre in range(0, pre_max + 1, 2):
        start = ctx.end_pose([("FW", pre)])
        hr = math.radians(start.h)
        inn = "RIGHT" if dy * (-math.cos(hr)) > 0 else "LEFT"
        out = opposite(inn)
        for ang in angles:
            e0 = ctx.end_pose([_t(inn, ang), _t(out, ang)], start=start)
            mid = ctx.end_pose([_t(inn, ang)], start=start)
            sn = math.sin(math.radians(mid.h))
            if abs(sn) < 0.1:
                continue
            d = (y_target - e0.y) / sn
            if d < 0:
                continue
            seq = [("FW", pre), _t(inn, ang), ("FW", d), _t(out, ang)]
            c, endp = ctx.clearance(seq)
            hx = math.cos(math.radians(endp.h))
            front = endp.x + SENSOR_FWD * hx
            if front_limit is not None and front < front_limit:
                continue
            used = abs(endp.x - ctx.pose.x)
            cand = (c, pre, (inn, out, ang), d, used)
            if c >= need_pref and (good is None or used < good[4]):
                good = cand
            elif c >= need_min and (ok is None or c > ok[0]):
                ok = cand
        if good is not None:
            break
    pick = good or ok
    return None if pick is None else pick[:4]


def run_s_curve(ctx, plan):
    c, pre, turns, d = plan
    if not turns:
        return
    inn, out, ang = turns
    ctx.fw(pre)
    ctx.turn(inn, ang)
    ctx.fw(d)
    ctx.turn(out, ang)


def square_up(ctx, target_h):
    """Gyro only: turn the measured heading error out when it is >= 3 deg."""
    if ctx.read_heading is None:
        return
    err = ((ctx.pose.h - target_h + 180) % 360) - 180
    if abs(err) >= 3:
        ctx.log.append(f"heading error {err:+.1f} deg -> correcting")
        ctx.turn("RIGHT" if err > 0 else "LEFT", abs(err))


def return_home(ctx, g=getattr(C, "T2_IR_GAP_OBS1_CM", 10.0)):
    ctx.log = []
    side = 1 if ctx.pose.y > 0 else -1
    lane = side * (5.0 + g + W / 2 - IR_INSET)        # inside IR reads ~g beside O1
    # finish the shift with the side IR still before O1, so it can pass O1 (header plan)
    plan = s_curve(ctx, lane, front_limit=ctx.boxes["obs1"][1] + NEED_MIN - (SENSOR_FWD - IR_FWD))
    if plan is None:                                   # short O1-O2 gap: shift anyway
        plan = s_curve(ctx, lane)
        if plan is not None:
            ctx.log.append("gap too short to shift before O1 - shifting beside it (no IR pass)")
    if plan is None:
        return {"status": "UNSAFE_LANE", "log": ctx.log}
    run_s_curve(ctx, plan)
    square_up(ctx, 180.0)

    # inside IR passes O1 (team header: "inner IR passes obstacle 2, obstacle 1")
    inside = ctx.side_towards(0.0)
    o1 = ctx.boxes["obs1"]
    hr = math.radians(ctx.pose.h)
    ir_x = ctx.ir_point(inside)[0]
    dist = (ir_x - o1[0]) / max(0.5, -math.cos(hr)) + 40
    if ir_x < o1[0]:
        r = {"end": "PAST", "edge_cm": None}
    else:
        r = ctx.edge(inside, max(30, min(250, dist)), mode="N" if ir_x > o1[1] else "auto")
    ir_o1 = r.get("near_cm")
    if r["end"] == "EDGE" and r["edge_cm"] is not None:
        hx = math.cos(math.radians(ctx.pose.h))
        new_x = o1[0] - IR_FWD * hx + (r["moved_cm"] - r["edge_cm"] + IR_LAG) * hx
        ctx.log.append(f"x re-synced on O1 face: {ctx.pose.x:.1f} -> {new_x:.1f}")
        ctx.pose.x = new_x
    else:
        ctx.log.append(f"inside IR did not see O1 ({r['end']}) - odometry only")

    # centre for the carpark (team 'rough-centre' idea: 45 in, FW, 45 out)
    plan = s_curve(ctx, 0.0)                      # carpark arms are in the clearance check
    if plan is None:
        return {"status": "UNSAFE_CENTRE", "log": ctx.log}
    run_s_curve(ctx, plan)
    square_up(ctx, 180.0)
    return {"status": "OK", "lane": round(lane, 1), "ir_obs1": ir_o1,
            "y": round(ctx.pose.y, 1), "x": round(ctx.pose.x, 1), "log": ctx.log}


# =============================================================================
# Step 5 - park: both IRs find the carpark walls, then US wall guard
# =============================================================================

def park(ctx):
    ctx.log = []
    travelled, hits = 0, 0
    while travelled < CARPARK_SEARCH_MAX_CM:
        l, r = ctx.ir("LEFT"), ctx.ir("RIGHT")
        if (l is not None and l <= PARK_IR_DANGER_CM) or (r is not None and r <= PARK_IR_DANGER_CM):
            return {"status": "SIDE_DANGER", "ir": (l, r), "log": ctx.log}
        both = l is not None and r is not None and l <= PARK_IR_OBJECT_CM and r <= PARK_IR_OBJECT_CM
        hits = hits + 1 if both else 0
        if hits >= 2:
            ctx.log.append(f"both carpark walls acquired: IR L/R {l:.1f}/{r:.1f}")
            break
        u = ctx.us(lo=3)
        if u is not None and u <= CARPARK_WALL_STOP_CM + 2:
            ctx.log.append(f"back wall at {u:.1f} cm before both walls were seen")
            break
        ctx.fw(PARK_STEP_CM)
        travelled += PARK_STEP_CM
    else:
        return {"status": "NO_CARPARK", "log": ctx.log}
    pulled = 0
    for _ in range(3):
        u = ctx.us(lo=3)
        if u is None:
            break
        d = min(CARPARK_PULL_IN_MAX_CM - pulled, u - CARPARK_WALL_STOP_CM)
        if d < 1:
            break
        ctx.fw(d)
        pulled += int(round(d))
    u = ctx.us(lo=3)
    l, r = ctx.ir("LEFT"), ctx.ir("RIGHT")
    return {"status": "OK", "back_cm": u, "ir_lr": (l, r), "pulled": pulled, "log": ctx.log}


# =============================================================================
# Runner
# =============================================================================

def run_task2(send, read_us, read_ir_left, read_ir_right, snap, read_heading=None,
              drive_edge=None, stage="full",
              cp_front_from_start=getattr(C, "T2_CP_FRONT_FROM_START_CM", 33.0),
              cp_half=getattr(C, "T2_CARPARK_HALF_CM", 20.0),
              cp_depth=getattr(C, "T2_CARPARK_DEPTH_CM", 50.0),
              arm_t=getattr(C, "T2_CARPARK_ARM_CM", 10.0)):
    """Team route 0 -> 5. stage: obs1 | detect2 | o2turn | obs2 | full (team stages)."""
    ctx = Ctx(send, read_us, read_ir_left, read_ir_right, snap, read_heading, drive_edge,
              cp_front=cp_front_from_start, cp_half=cp_half, cp_depth=cp_depth, arm=arm_t)
    out = {"ctx": ctx}

    r0 = approach_obs1(ctx); out["start"] = r0
    if r0["status"] != "OK":
        out["status"] = "START_" + r0["status"]; return out
    a1 = r0["arrow"]

    r1 = bypass_obs1(ctx, a1); out["obs1"] = r1
    if r1["status"] != "OK" or stage == "obs1":
        out["status"] = "OK" if r1["status"] == "OK" else "OBS1_" + r1["status"]; return out

    r2 = aim_obs2(ctx, a1); out["obs2"] = r2
    if r2["status"] != "OK":
        out["status"] = "OBS2_" + r2["status"]; return out
    a2 = r2["arrow"]
    out["arrows"] = (a1, a2)
    if stage == "detect2":
        out["status"] = "OK"; return out

    r3 = loop_obs2(ctx, a2, stop_after_turn=(stage == "o2turn")); out["loop"] = r3
    if r3["status"] != "OK" or stage in ("o2turn", "obs2"):
        out["status"] = "OK" if r3["status"] == "OK" else "LOOP_" + r3["status"]; return out

    r4 = return_home(ctx); out["home"] = r4
    if r4["status"] != "OK":
        out["status"] = "HOME_" + r4["status"]; return out

    r5 = park(ctx); out["park"] = r5
    out["status"] = "OK" if r5["status"] == "OK" else "PARK_" + r5["status"]
    return out
