"""
Task 2 (Fastest car) - end to end: carpark -> obstacle 1 -> obstacle 2 -> back into the carpark.

FLOW
    0. start_to_obs1            carpark -> AC40 at obstacle 1 -> snap arrow 1
    1. dodge_plan               OUT45 -> FW d -> BACK45 past obstacle 1
    1b. pass_obs1_measure_ir    creep past obstacle 1, IR reading -> sideways offset Y0
    2. approach_and_snap_obs2   U1 (ultrasonic sees obs2) / U2 (angled 10-deg search) -> arrow 2
    3. loop_obs2                escape (Play A / B) + loop round obstacle 2, length measured
    4. return_to_carpark_edge   lane shift -> home drive past obstacle 1 -> stop at AC on the arm
    5. park_in_carpark          S-curve to the centre -> drive in (IRs confirm) -> AC on back wall
    run_task2() chains 0 -> 5.

IE (one-command IR edge legs)
    The STM cannot report sensors while it moves, so following an obstacle with the
    side IR used to be FW3 + IR read, over and over. If run_task2() is given
    drive_edge (the STM's IE command, see main_merged.c), each of those legs is ONE
    command: drive until the IR goes near -> far, report where. Without drive_edge
    the old FW3 stepping is used, so old firmware still works.
        drive_edge(side, need_seen, max_cm) -> {"end": "EDGE"|"NEVER"|"MAX"|"WALL",
            "edge_cm": travel at the first far reading (None if no edge),
            "near_cm": median IR reading beside the obstacle (None if never near),
            "moved_cm": total travel, "cmd": e.g. "IERN150"}
"""
import math
import statistics
import config as C

FOV_H_DEG = 62.2            # Pi Camera v2 horizontal field of view
SIN45 = math.sqrt(0.5)
PLAN_MARGIN_CM = 3.0        # extra gap on top of CLEAR_CM in every planned path
# Task 2 settings and the values they should have. Any that an older config.py lacks are
# filled in here, so the run still works (rpi_client_task2.py lists them as a warning).
T2_DEFAULTS = {
    "T2_AC_OBS1_CM": 40.0, "T2_IR_GAP_OBS1_CM": 12.0, "T2_MIN_CLEARANCE_CM": 5.0,
    "T2_CARPARK_CLEARANCE_CM": 5.0, "T2_TARGET_CLEARANCE_CM": 10.0, "T2_PHOTO_TURNS_MAX": 2,
    "T2_DEFAULT_ARROW_1": "LEFT", "T2_CARPARK_HALF_CM": 20.0, "T2_CARPARK_DEPTH_CM": 50.0,
    "T2_CP_FRONT_FROM_START_CM": 33.0, "T2_IR_FWD_CM": 0.0, "T2_IR_INSET_CM": 0.0,
    "T2_IR_LAG_CM": 0.0, "T2_U2_FAR_END_CM": 18.5, "T2_AC_STOP_CARPARK_CM": 70.0,
    "T2_CARPARK_ARM_CM": 10.0,
}
T2_MISSING = [k for k in T2_DEFAULTS if not hasattr(C, k)]
for _k in T2_MISSING:
    setattr(C, _k, T2_DEFAULTS[_k])

CLEAR_CM = getattr(C, "T2_MIN_CLEARANCE_CM", C.MIN_CLEARANCE_CM)          # obstacles, walls
CARPARK_CLEAR_CM = getattr(C, "T2_CARPARK_CLEARANCE_CM", C.MIN_CLEARANCE_CM)  # carpark arms / back
# Every plan first tries to keep TARGET_CLEAR_CM; only if none can, CLEAR_CM + PLAN_MARGIN_CM.
# Optional moves (10-deg photo / ultrasonic turns) need TARGET_CLEAR_CM or are skipped.
TARGET_CLEAR_CM = max(getattr(C, "T2_TARGET_CLEARANCE_CM", 0.0), CLEAR_CM + PLAN_MARGIN_CM)
NEEDS = (TARGET_CLEAR_CM, CLEAR_CM + PLAN_MARGIN_CM)      # preferred, then minimum
PHOTO_CLEAR_CM = TARGET_CLEAR_CM
                            # (turn arcs are only sampled at start / middle / end)


# =============================================================================
# Shared helpers
# =============================================================================

def bearing_deg(box_cx, img_w):
    """Angle from camera axis to a pixel column. + = left of centre, - = right."""
    fx = (img_w / 2) / math.tan(math.radians(FOV_H_DEG / 2))
    return math.degrees(math.atan((img_w / 2 - box_cx) / fx))


def _us_median(read_us, n=5, tol=2.0, lo=0.0, hi=400.0):
    """Median of n ultrasonic reads; None unless >= n-1 agree within tol and in [lo, hi]."""
    vals = [v for v in (read_us() for _ in range(n)) if v is not None and v > 0]
    if len(vals) < n - 1:
        return None
    med = statistics.median(vals)
    good = [v for v in vals if abs(v - med) <= tol]
    return med if len(good) >= n - 1 and lo <= med <= hi else None


def _median(v):
    """Plain median of a list, None if empty."""
    v = sorted(v)
    return v[len(v) // 2] if v else None


def _edge_log(r):
    """One log line for an IE leg."""
    e = "-" if r.get("edge_cm") is None else f"{r['edge_cm']:.1f}"
    nr = "-" if r.get("near_cm") is None else f"{r['near_cm']:.1f}"
    return f"{r.get('cmd', 'IE')}: {r['end']}, edge at {e} cm, IR beside {nr} cm, moved {r['moved_cm']:.1f} cm"


class _Pose:
    """Robot-centre pose. x = forward along the start heading, y = left of the
    centre line, h = heading in degrees (+ = turned left)."""
    def __init__(self, y):
        self.x, self.y, self.h = 0.0, y, 0.0

    def move(self, fwd, right=0.0, cw_deg=0.0):
        r = math.radians(self.h)
        self.x += fwd * math.cos(r) + right * math.sin(r)
        self.y += fwd * math.sin(r) - right * math.cos(r)
        self.h -= cw_deg

    def copy(self):
        p = _Pose(self.y); p.x, p.h = self.x, self.h
        return p


def _box_clear(p, box, W, Lr):
    """Min gap (cm) between the robot rectangle at pose p and an axis-aligned
    box (x0, x1, y0, y1). 0 = touching or overlapping. Use +-1e4 for 'infinite'."""
    x0, x1, y0, y1 = box
    hr = math.radians(p.h); c, sn = math.cos(hr), math.sin(hr)
    best = float("inf")
    for f in (1, -1):                                    # robot corners -> box
        for r in (1, -1):
            qx = p.x + f * Lr / 2 * c + r * W / 2 * sn
            qy = p.y + f * Lr / 2 * sn - r * W / 2 * c
            dx = max(x0 - qx, 0.0, qx - x1); dy = max(y0 - qy, 0.0, qy - y1)
            best = min(best, math.hypot(dx, dy))
    for qx in (x0, x1):                                  # box corners -> robot
        for qy in (y0, y1):
            if abs(qx) > 1e3 or abs(qy) > 1e3:
                continue
            lx = (qx - p.x) * c + (qy - p.y) * sn
            ly = -(qx - p.x) * sn + (qy - p.y) * c
            best = min(best, math.hypot(max(abs(lx) - Lr / 2, 0), max(abs(ly) - W / 2, 0)))
    return best


def _turn_model(cmd, m=1):
    """(forward, right, clockwise deg) for a 45 or small turn, mirrored if m = -1."""
    if cmd in C.TURN45_MODELS:
        f, r, k = C.TURN45_MODELS[cmd]; dg = 45 * k
    else:
        f, r, dg = C.SMALL_TURN_MODELS[cmd]
    return (f, r, dg) if m == 1 else (f, -r, -dg)


def _undo_turn(pose, cmd, m=1):
    """Pose after X<cmd> retraces the arc of <cmd> (XR10 after RT10, XL10 after LT10)."""
    f, r, dg = _turn_model(cmd, m)
    pose.h += dg
    hr = math.radians(pose.h)
    pose.x -= f * math.cos(hr) + r * math.sin(hr)
    pose.y -= f * math.sin(hr) - r * math.cos(hr)


def _recognised(photos):
    return any(p is not None and p["conf"] >= C.RECOGNISED_CONF for p in photos)


def _pick_arrow(photos, default):
    """Best of the photos -> (arrow, conf, how). how: "seen" (>= RECOGNISED_CONF),
    "guess" (best low-confidence box) or "default" (no arrow in any photo)."""
    got = [p for p in photos if p is not None]
    if not got:
        return default, None, "default"
    b = max(got, key=lambda p: p["conf"])
    return b["label"], b["conf"], ("seen" if b["conf"] >= C.RECOGNISED_CONF else "guess")


def _path_points(start, seq, m=1):
    """Poses along a planned sequence (turn names / straight cm). Turns are sampled at
    start, midpoint and end; straights every ~2 cm."""
    p, pts = start.copy(), [start.copy()]
    for it in seq:
        if isinstance(it, str):
            q = p.copy(); q.move(*_turn_model(it, m))
            mid = p.copy(); mid.x, mid.y, mid.h = (p.x + q.x) / 2, (p.y + q.y) / 2, (p.h + q.h) / 2
            pts += [mid, q]; p = q
        else:
            n = max(1, int(abs(it) // 2))
            for _ in range(n):
                p.move(it / n); pts.append(p.copy())
    return pts


def _path_room(start, seq, box, m=1):
    """Smallest gap (cm) between the robot and box along a planned sequence."""
    return min(_box_clear(q, box, C.ROBOT_WIDTH_CM, C.ROBOT_LENGTH_CM)
               for q in _path_points(start, seq, m))


def _plan_s_curve(start, target_y, m=1):
    """S-curve that ends straight at sideways position target_y.
    Tries 45-deg turns first, then small turns. Returns a list like
    ["RT45", 32, "LT45"] (in real command names) or [] if the shift is tiny."""
    dy = target_y - start.y
    if abs(dy) < 1.5:
        return []
    h = start.h
    # which real command moves +y from this heading: for heading ~0 that's a LEFT turn,
    # for heading ~180 it's a RIGHT turn (then mirrored by m)
    plus_is_left = math.cos(math.radians(h)) > 0
    plus_cmd, minus_cmd = ("LT", "RT") if plus_is_left else ("RT", "LT")
    if m == -1:
        plus_cmd, minus_cmd = minus_cmd, plus_cmd
    for big, ang in (("45", 45.0), (str(C.ANGLE_TURN_DEG), float(C.ANGLE_TURN_DEG))):
        out_c, back_c = (plus_cmd + big, minus_cmd + big) if dy > 0 else (minus_cmd + big, plus_cmd + big)
        if out_c not in C.TURN45_MODELS and out_c not in C.SMALL_TURN_MODELS:
            continue
        p = start.copy(); p.move(*_turn_model(out_c, m)); p.move(*_turn_model(back_c, m))
        rate = math.sin(math.radians(ang)) * (1 if dy > 0 else -1)
        d = (target_y - p.y) / rate
        if d >= 0:
            return [out_c, int(round(d)), back_c]
    return []


# =============================================================================
# Step 0 - carpark to AC40 at obstacle 1, read arrow 1
# =============================================================================

def start_to_obs1(send, read_us, snap, ac=40.0, us_max=200.0, max_drive_cm=200.0,
                  step_cm=10, default_arrow=getattr(C, "T2_DEFAULT_ARROW_1", "LEFT")):
    """
    Task 2, Step 0 - drive straight out of the carpark to AC<ac> in front of
    obstacle 1 and read its arrow.

    WHEN TO CALL
        Start of the run: robot parked in the carpark, straight, facing obstacle 1.

    WHAT IT DOES
        Reads the ultrasonic (obstacle 1 is straight ahead, 60-150 cm past the carpark),
        drives to `ac`, re-reads once to fine-tune, then snaps. If confidence is low it
        takes one 10-deg angled photo each side (RT10/XR10, LT10/XL10) and keeps the best.
        If the ultrasonic sees nothing it creeps forward in step_cm steps until it does.
        Photo turns: at most T2_PHOTO_TURNS_MAX (RT10 then LT10), only while the arrow is
        not recognised, each undone. Not recognised at all -> continue with the best
        low-confidence photo, or default_arrow if no photo had an arrow.

    INPUTS
        send(cmd)    : sends one STM command and BLOCKS until done.
        read_us()    : one ultrasonic reading (cm) or None.
        snap()       : photo + YOLO -> None or {"label", "conf", "box_cx", "img_w", ...}.
        ac           : stop distance ultrasonic -> obstacle 1 face (cm).
        us_max       : ignore readings above this (cm).
        max_drive_cm : give up if obstacle 1 isn't found within this (cm).
        step_cm      : creep step while the ultrasonic sees nothing (cm).

    OUTPUT (dict)
        status : "OK" | "NO_OBSTACLE".
        arrow  : "LEFT"/"RIGHT" - always set (see arrow_how).
        conf   : its confidence (None if no photo had an arrow).
        arrow_how : "seen" | "guess" (below RECOGNISED_CONF) | "default".
        us_ac  : ultrasonic at the AC stop (cm).
        D1     : robot-centre travel from the start to the AC stop (cm) - needed later
                 to find the carpark again.
        log    : commands sent.
    """
    log, D1 = [], 0.0

    def drive(cm):
        nonlocal D1
        cm = int(round(cm))
        cmd, left = ("FW" if cm > 0 else "BW"), abs(cm)
        while left > 0:
            st = min(left, C.MAX_STRAIGHT_COMMAND_CM)
            send(f"{cmd}{st}"); log.append(f"{cmd}{st}")
            D1 += st if cmd == "FW" else -st
            left -= st

    u = _us_median(read_us, lo=5, hi=us_max)
    while u is None and D1 < max_drive_cm:
        drive(step_cm)
        u = _us_median(read_us, lo=5, hi=us_max)
    if u is None:
        return {"status": "NO_OBSTACLE", "arrow": None, "conf": None, "arrow_how": None,
                "us_ac": None, "D1": D1, "log": log}
    drive(u - ac)
    u = _us_median(read_us, lo=5, hi=us_max)
    if u is not None and abs(u - ac) >= 1:
        drive(max(-10, min(10, u - ac)))

    log.append(f"US at the AC stop: {'-' if u is None else format(u, '.1f')} cm")

    # Arrow: straight photo, then while not recognised at most T2_PHOTO_TURNS_MAX 10-deg
    # photos (RT, then LT), each checked against the face first and undone with XR / XL,
    # which retraces the arc back to the AC pose.
    deg = C.ANGLE_TURN_DEG
    face = C.SENSOR_FORWARD_CM + (u if u is not None else ac)
    face_box = (face, face + 10.0, -1e4, 1e4)             # the face as a wall: worst case
    photos = [snap()]
    for t in ("RT", "LT")[:C.T2_PHOTO_TURNS_MAX]:
        if _recognised(photos):
            break
        name = f"{t}{deg}"
        room = _path_room(_Pose(0.0), [name], face_box)
        if room < PHOTO_CLEAR_CM:
            log.append(f"no room for the {name} photo ({room:.1f} cm to obstacle 1) - skipped")
            continue
        send(name); log.append(name)
        photos.append(snap())
        undo = "X" + t[0] + str(deg)
        send(undo); log.append(undo)
    arrow, conf, how = _pick_arrow(photos, default_arrow)
    if how != "seen":
        log.append(f"arrow 1 not recognised - continuing with {arrow} "
                   f"({'best low-confidence photo' if how == 'guess' else 'default'})")
    return {"status": "OK", "arrow": arrow, "conf": conf, "arrow_how": how,
            "us_ac": u, "D1": round(D1, 1), "log": log}


# =============================================================================
# Step 1 - dodge obstacle 1
# =============================================================================

def dodge_plan(arrow, g=10.0, s=0.0, h=5.0, ac=40.0, min_clear=CLEAR_CM):
    """
    Task 2, Step 1 - dodge OBSTACLE 1 after reading its arrow.

    WHEN TO CALL
        Robot is straight in front of obstacle 1 at AC<ac> (ultrasonic reads
        `ac` cm to its face) and the image has been recognised as LEFT/RIGHT.

    WHAT IT GIVES YOU
        The 3 moves (OUT45 -> FW d -> BACK45) that leave the robot driving
        straight, parallel to its original heading, with the side IR exactly
        `g` cm from obstacle 1's side as it passes. Then just keep driving FW
        toward obstacle 2 - no further turn is needed near obstacle 1.

    INPUTS
        arrow     : "LEFT" or "RIGHT" - the recognised arrow on obstacle 1.
        g         : target IR reading (cm) while passing obstacle 1. Default 10.
        s         : how far the IR's sensing face sits INSIDE the robot's side (cm).
                    0 = flush with the body edge. Measure on your robot.
        h         : half of obstacle 1's width (cm). 10x10 obstacle -> 5.
        ac        : the AC distance you stopped at (ultrasonic -> face, cm).
        min_clear : smallest allowed gap to any obstacle (cm). From config.
        Also read from config.py:
            TURN45_MODELS["LT45"], ["RT45"] : (forward cm, right cm) per turn
            SENSOR_FORWARD_CM               : robot centre -> ultrasonic (cm)
            ROBOT_WIDTH_CM                  : robot width (cm)

    OUTPUT (dict)
        cmds     : e.g. ["LT45", "FW10", "RT45"] - send these to the STM in order.
        gap_cm   : IR gap you will actually get after rounding FW to whole cm.
        x_end_cm : how far forward the robot centre has moved by the end of
                   the 3 moves (log it - it's part of the distance to obstacle 2).
        safe     : True only if (a) the front corner never comes within
                   min_clear of obstacle 1 during the OUT turn, and
                   (b) gap_cm >= min_clear. If False, do NOT execute.

    INTERNAL VARIABLES
        c                  : sin45 = cos45 = 0.707, converts diagonal moves to x/y.
        a, W               : SENSOR_FORWARD_CM, ROBOT_WIDTH_CM.
        F_out, S_out       : forward / sideways cm of the turn TOWARD the dodge side
                             (LT45 for LEFT arrow, RT45 for RIGHT arrow).
        F_back, S_back     : forward / sideways cm of the turn that straightens you out.
        Y_need             : sideways distance the robot centre must end at,
                             measured from obstacle 1's centre line
                             (= half obstacle + IR gap + half robot - IR inset).
        Y_turns            : sideways distance the two turns give on their own.
        d_raw, d           : exact / rounded diagonal FW between the turns.
        corner_fwd/_side   : where the robot's inner front corner is after
                             the OUT turn (the point most likely to clip obstacle 1).
        fwd_safe           : how far forward that corner is by the time it is
                             safely beside obstacle 1; must be <= face - min_clear.

    ASSUMES
        - Robot starts centred on obstacle 1 and dead straight.
        - Turn models are robot-centre endpoints relative to the start pose,
          and each 45 turn really ends at 45 degrees.
    """
    c = SIN45
    a, W = C.SENSOR_FORWARD_CM, C.ROBOT_WIDTH_CM
    lt = (C.TURN45_MODELS["LT45"][0], abs(C.TURN45_MODELS["LT45"][1]))
    rt = (C.TURN45_MODELS["RT45"][0], abs(C.TURN45_MODELS["RT45"][1]))
    (F_out, S_out), (F_back, S_back), out_cmd, back_cmd = (
        (lt, rt, "LT45", "RT45") if arrow == "LEFT" else (rt, lt, "RT45", "LT45"))

    Y_need  = h + g + W / 2 - s
    Y_turns = S_out + c * (F_back - S_back)
    d_raw   = (Y_need - Y_turns) / c
    d       = max(0, round(d_raw))
    g_act   = g + c * (d - d_raw)

    corner_fwd  = F_out + c * (a + W / 2)
    corner_side = S_out + c * (a - W / 2)
    fwd_safe    = corner_fwd + max(0.0, (h + min_clear) - corner_side)
    safe = fwd_safe <= ac + a - min_clear and g_act >= min_clear

    X_end = F_out + c * d + c * (F_back + S_back)
    return {"cmds": [out_cmd, f"FW{d}", back_cmd], "gap_cm": round(g_act, 1),
            "x_end_cm": round(X_end, 1), "safe": safe}


# =============================================================================
# Step 1b - pass obstacle 1 and measure the real sideways offset
# =============================================================================

def pass_obs1_measure_ir(send, read_ir, x_end_cm, ac=40.0, g=10.0, s=0.0, h=5.0,
                         ir_fwd=0.0, ir_near_max=40.0, spike_n=2, step_cm=3,
                         drive_edge=None, ir_side="RIGHT"):
    """
    Task 2, Step 1b - creep past obstacle 1 reading the side IR, so the robot's REAL
    sideways offset (Y0) is measured instead of assumed.

    WHEN TO CALL
        Right after dodge_plan()'s 3 moves. Robot straight, beside or just before
        obstacle 1.

    WHAT IT DOES
        FW in steps of step_cm, reading the IR on the obstacle 1 side, until the
        reading goes near -> far (IR has passed obstacle 1's back face) or until the
        IR must have passed it. Median of the near readings -> Y0.

    INPUTS
        send(cmd)   : sends one STM command and BLOCKS until done.
        read_ir()   : one reading (cm) of the IR facing obstacle 1 (right IR after a LEFT
                      dodge, left IR after a RIGHT dodge), None if out of range.
        x_end_cm    : dodge_plan()["x_end_cm"].
        ac          : AC distance used at obstacle 1 (cm).
        g           : planned IR gap - used as Y0 fallback if the IR never sees obstacle 1.
        s           : IR face inset from the robot's side (cm).
        h           : half of obstacle 1's width (cm).
        ir_fwd      : IR's forward offset from robot centre (cm).
        ir_near_max : readings at or below this mean "obstacle beside me".
        spike_n     : consecutive far readings that confirm the IR has passed it.
        step_cm     : FW step (cm) - only without drive_edge.
        drive_edge  : the STM IE command (see the module doc). Given -> ONE command
                      instead of FW<step_cm> + IR read repeated.
        ir_side     : "LEFT"/"RIGHT" - which IR read_ir / drive_edge watches.

    OUTPUT (dict)
        Y0         : robot centre's sideways offset from the centre line (cm, toward the
                     dodge side) = h + IR_reading + W/2 - s.
        ir_reading : median IR reading beside obstacle 1 (None if never seen).
        x_end_cm   : x_end_cm + distance crept (pass THIS to approach_and_snap_obs2).
        log        : commands sent.
    """
    a, W = C.SENSOR_FORWARD_CM, C.ROBOT_WIDTH_CM
    back_face = ac + a + 10                       # obstacle 1's back face, AC-pose frame
    max_cm = max(0.0, back_face - (x_end_cm + ir_fwd)) + 15
    log, nears, seen, far_run, travelled = [], [], False, 0, 0
    if drive_edge is not None:
        r = drive_edge(ir_side, True, max(1, int(round(max_cm))))
        log.append(_edge_log(r))
        travelled = r["moved_cm"]
        if r["near_cm"] is not None:
            nears = [r["near_cm"]]
        max_cm = -1                                # skip the stepping loop below
    while travelled < max_cm:
        r = read_ir()
        if r is not None and r <= ir_near_max:
            seen, far_run = True, 0
            nears.append(r)
        elif seen:
            far_run += 1
            if far_run >= spike_n:
                break
        send(f"FW{step_cm}"); log.append(f"FW{step_cm}")
        travelled += step_cm
    reading = _median(nears)
    Y0 = h + (reading if reading is not None else g) + W / 2 - s
    if reading is None:
        log.append("IR never saw obstacle 1 - using planned gap")
    return {"Y0": round(Y0, 1), "ir_reading": reading,
            "x_end_cm": round(x_end_cm + travelled, 1), "log": log}


# =============================================================================
# Step 2 - approach obstacle 2 and read its arrow
# =============================================================================

def approach_and_snap_obs2(send, read_us, snap, Y0, x_end_cm, dodge_side="LEFT",
                           ac_obs1=40.0, gap_min=60.0, target_front=60.0,
                           min_front_cm=25.0, search_step=10, max_search_steps=12,
                           us_max=170.0, cam_lat=0.0, cam_fwd=None,
                           bearing_tol_deg=5.0, read_heading=None, max_heading_err=3.0):
    """
    Task 2, Step 2 - from the end of the obstacle 1 dodge to obstacle 2's arrow.

    WHAT IT DOES
        U1 (ultrasonic sees obstacle 2 at the start):
            drive straight until the ultrasonic reads target_front, snap, done.
        U2 (ultrasonic sees nothing - obstacle 2 is short, beam misses it):
            1. drive straight until the robot's REAR has cleared obstacle 1,
            2. turn 10 deg toward the centre line (RT10 after a LEFT dodge),
            3. step forward along that angle; every step check the ultrasonic
               (the angle slowly brings it back in front of obstacle 2) and take a
               photo (the arrow's angle + known sideways offset gives distance),
               never letting the worst-case front gap drop below min_front_cm,
            4. snap from the angled pose,
            5. turn 10 deg back (LT10) so the robot is straight for the 45-deg exit.
        In both cases the robot's position is tracked through every move, and
        the sideways offset is cross-checked against the photo.
        Arrow not recognised: 10-deg photo turns toward the centre line, at most
        T2_PHOTO_TURNS_MAX from straight (U2's search angle counts), each checked
        against the face (backing up if needed) and undone. Still not recognised ->
        continue with the best low-confidence photo, or the dodge side.

    WHEN TO CALL
        After pass_obs1_measure_ir(). The robot is stopped and straight.

    INPUTS
        send(cmd)        : sends one STM command ("FW8", "RT10", "XR10", ...) and BLOCKS
                           until the STM reports done.
        read_us()        : one ultrasonic reading (cm, sensor -> obstacle), None if no echo.
        snap()           : photo + YOLO -> None, or the best box as
                           {"label": "LEFT"/"RIGHT", "conf": float, "box_cx": px, "img_w": px,
                            "body_edge_px": px or None}.
                           Return boxes >= MODEL_MIN_CONF (low-conf boxes still give distance).
                           body_edge_px (optional) = outer column of obstacle 2's dark body on
                           the robot's side (leftmost after a LEFT dodge) - used to pick the play.
        Y0               : pass_obs1_measure_ir()["Y0"] - robot centre's sideways offset from
                           the centre line (cm, positive, toward the dodge side).
        x_end_cm         : pass_obs1_measure_ir()["x_end_cm"] - robot-centre forward travel
                           since the AC40 pose at obstacle 1.
        dodge_side       : "LEFT" / "RIGHT" - which way obstacle 1 was dodged.
        ac_obs1          : the AC distance used at obstacle 1 (cm).
        gap_min          : smallest possible obs1 -> obs2 gap from the rules (60 cm).
        target_front     : ultrasonic -> face distance to snap from (cm).
        min_front_cm     : worst-case front-to-face distance never allowed below this (cm).
        search_step      : forward step along the 10-deg angle in U2 (cm).
        max_search_steps : hard cap on U2 search iterations.
        us_max           : ignore readings above this (cm).
        cam_lat, cam_fwd : camera offset from robot centre (cm, + = left / + = forward).
                           cam_fwd defaults to the ultrasonic's position.
        bearing_tol_deg  : photo-vs-expected angle difference that triggers a Y correction.
        read_heading()   : optional gyro heading vs the run's start heading (deg, + = left).
                           Warns if large; also syncs the tracked heading after every turn.
        Reads from config.py: SENSOR_FORWARD_CM, ROBOT_WIDTH_CM, ROBOT_LENGTH_CM,
        T2_MIN_CLEARANCE_CM, MAX_STRAIGHT_COMMAND_CM, RECOGNISED_CONF, ANGLE_TURN_DEG,
        SMALL_TURN_MODELS (RT10 / LT10).

    OUTPUT (dict)
        status      : "OK" | "NO_DISTANCE" (face distance unknown).
        case        : "U1" | "U2".
        arrow, conf : arrow and its confidence - arrow always set (see arrow_how).
        arrow_how   : "seen" | "guess" (below RECOGNISED_CONF) | "default" (= dodge_side).
        d_front     : ultrasonic -> obstacle 2 face distance, robot STRAIGHT (cm).
                      In U2 this can be well under target_front - that is fine, U2 means
                      obstacle 2 is short so the exit needs little room.
        Y           : robot centre's sideways offset from the centre line, toward the
                      dodge side (cm) -> feed this to the exit phase.
        face_source : "US" (ultrasonic) or "CAM" (triangulated from the arrow angle).
        x_face      : obstacle 2's face, forward from the robot centre at the START of this
                      step (cm) - used to work out the obs1 -> obs2 gap later.
        photo       : the straight snap() result at the snap pose (for far_end_from_photo).
        log         : every command and decision.
    """
    a, W, Lr = C.SENSOR_FORWARD_CM, C.ROBOT_WIDTH_CM, C.ROBOT_LENGTH_CM
    cam_fwd = a if cam_fwd is None else cam_fwd
    side = 1 if dodge_side == "LEFT" else -1
    deg = C.ANGLE_TURN_DEG
    toward, away = ("RT", "LT") if side == 1 else ("LT", "RT")   # toward = toward centre line
    pose, log = _Pose(side * Y0), []

    # Worst case: obstacle 2's face as close as the rules allow.
    min_face = ac_obs1 + 10 + gap_min - x_end_cm          # sensor -> face at the start
    x_face_min = a + min_face                             # in pose coordinates
    x_face, face_source = None, None

    # ---------------- helpers ----------------
    def cos_h():
        return math.cos(math.radians(pose.h))

    def drive(cm):
        cm = int(round(cm))
        cmd, left = ("FW" if cm > 0 else "BW"), abs(cm)
        while left > 0:
            step = min(left, C.MAX_STRAIGHT_COMMAND_CM)
            send(f"{cmd}{step}"); log.append(f"{cmd}{step}")
            pose.move(step if cmd == "FW" else -step)
            left -= step

    def turn(name):
        send(name); log.append(name)
        pose.move(*C.SMALL_TURN_MODELS[name])
        if read_heading is not None:
            pose.h = read_heading()                       # trust the gyro over the model

    def front_x():
        return pose.x + a * cos_h()

    def worst_gap():
        """Front-to-face distance if obstacle 2 were as close as the rules allow."""
        corner = (W / 2) * abs(math.sin(math.radians(pose.h)))
        return x_face_min - (front_x() + corner)

    def us_face(lo):
        r = _us_median(read_us, lo=lo, hi=us_max)
        return None if r is None else pose.x + (a + r) * cos_h()

    def cam_pos(p=None):
        p = p or pose
        hr = math.radians(p.h)
        return (p.x + cam_fwd * math.cos(hr) - cam_lat * math.sin(hr),
                p.y + cam_fwd * math.sin(hr) + cam_lat * math.cos(hr))

    def cam_face(res, p=None):
        """Face x from the arrow's angle (arrow sits on the centre line, y = 0),
        p = the pose the photo was taken from."""
        p = p or pose
        xc, yc = cam_pos(p)
        phi = math.radians(p.h + bearing_deg(res["box_cx"], res["img_w"]))
        if abs(math.degrees(phi)) < 0.5 or yc * math.tan(phi) >= 0:
            return None                                  # on-axis or wrong side
        xf = xc - yc / math.tan(phi)
        return xf if xf > p.x + a * math.cos(math.radians(p.h)) else None

    def drive_to_target():
        dx = (x_face - target_front) - front_x()
        if abs(dx) >= 1:
            drive(dx / cos_h())

    obs1_box = (ac_obs1 + a - x_end_cm, ac_obs1 + a + 10 - x_end_cm, -5.0, 5.0)

    def room_for(name):
        """Make sure the 10-deg photo turn `name` keeps clear of obstacle 2's face (the
        ultrasonic one if known, else the closest the rules allow), backing up first if
        it does not. False = no safe way -> skip the photo."""
        need = PHOTO_CLEAR_CM
        fx = x_face if x_face is not None else x_face_min
        face_box = (fx, fx + 10.0, -1e4, 1e4)
        if _path_room(pose, [name], obs1_box) < need:   # the turn swings the rear out
            log.append(f"no room for the {name} photo - the rear would swing near obstacle 1")
            return False
        room = _path_room(pose, [name], face_box)
        if room >= need:
            return True
        back = int(math.ceil(need - room)) + 1
        if _path_room(pose, [-back], obs1_box) < need:
            log.append(f"no room for the {name} photo and backing up would near obstacle 1 - skipped")
            return False
        log.append(f"backing up {back} cm for the {name} photo ({room:.1f} cm to the face)")
        drive(-back)
        return _path_room(pose, [name], face_box) >= need

    def snap_with_fallback():
        """Photo here, then while the arrow is not recognised 10-deg photo turns toward the
        centre line, at most T2_PHOTO_TURNS_MAX from straight in all (the U2 search angle
        counts). All undone with XR / XL (retraces each arc). Returns the first photo, the
        pose it was taken from, and every photo."""
        first, first_pose = snap(), pose.copy()
        photos, done = [first], []
        name = f"{toward}{deg}"
        undo = ("XR" if toward == "RT" else "XL") + str(deg)
        while (not _recognised(photos)
               and round(abs(pose.h) / deg) < C.T2_PHOTO_TURNS_MAX and room_for(name)):
            turn(name); done.append(name)
            photos.append(snap())
        for nm in reversed(done):
            send(undo); log.append(undo)
            _undo_turn(pose, nm)
            if read_heading is not None:
                pose.h = read_heading()
        return first, first_pose, photos

    def correct_y(res, p):
        """Ultrasonic gives distance, photo (taken at pose p) gives angle -> true sideways
        position. The error found at p is applied to the current pose."""
        xc, yc = cam_pos(p)
        phi = p.h + bearing_deg(res["box_cx"], res["img_w"])
        phi_exp = math.degrees(math.atan2(-yc, x_face - xc))
        if abs(phi - phi_exp) > bearing_tol_deg:
            yc_true = -(x_face - xc) * math.tan(math.radians(phi))
            old = side * pose.y
            pose.y += yc_true - yc
            log.append(f"Y corrected from photo: {old:.1f} -> {side * pose.y:.1f} cm "
                       f"(expected {phi_exp:.1f} deg, measured {phi:.1f} deg)")

    # ---------------- heading sanity ----------------
    if read_heading is not None:
        err = read_heading()
        if abs(err) > max_heading_err:
            log.append(f"WARN heading off by {err:.1f} deg at start")

    # ---------------- S0: stationary ultrasonic ----------------
    xf = us_face(lo=min_face - 10)

    if xf is not None:                                    # ========== U1 ==========
        case, x_face, face_source = "U1", xf, "US"
        log.append(f"U1: face {x_face - front_x():.1f} cm ahead")
        drive_to_target()
        xf = us_face(lo=target_front - 15)                # refine once
        if xf is not None:
            x_face = xf
            if abs((x_face - target_front) - front_x()) >= 1:
                drive(max(-10, min(10, (x_face - target_front) - front_x())))
        first, first_pose, photos = snap_with_fallback()

    else:                                                 # ========== U2 ==========
        case = "U2"
        log.append("U2: no echo - clear obs1, angle in, search")

        # 1. Rear clears obstacle 1 (+ clearance) before turning toward it.
        rear_clear = (ac_obs1 + a + 10 + CLEAR_CM + PLAN_MARGIN_CM + Lr / 2) - x_end_cm
        room = worst_gap() - min_front_cm
        angled_in = rear_clear <= room
        if rear_clear > 0:
            drive(min(rear_clear, room))
        if angled_in:
            # 2. Turn toward the centre line.
            turn(f"{toward}{deg}")
        else:
            log.append("U2: no room to angle in safely - searching straight")

        # 3. Stepped search along the angle.
        for _ in range(max_search_steps):
            xf = us_face(lo=min_front_cm - 5)
            if xf is not None:
                x_face, face_source = xf, "US"
                log.append(f"U2: ultrasonic locked on, face {x_face - front_x():.1f} cm")
                if x_face - front_x() > target_front:
                    drive_to_target()
                break
            res = snap()
            xf = cam_face(res) if res else None
            if xf is not None:
                x_face, face_source = xf, "CAM"
                gap = x_face - front_x()
                log.append(f"U2: arrow angle -> face {gap:.1f} cm")
                if gap > target_front + 5:
                    drive_to_target()                     # close in, then re-check
                    continue
                break
            if worst_gap() - search_step * cos_h() < min_front_cm:
                log.append("U2: search budget used up - snapping here")
                break
            drive(search_step)

        # 4. Snap from the (angled) pose.
        first, first_pose, photos = snap_with_fallback()
        if x_face is None and first is not None:
            xf = cam_face(first, first_pose)
            if xf is not None:
                x_face, face_source = xf, "CAM"

    # ---------------- Y cross-check (only when distance came from ultrasonic) ----------
    if face_source == "US" and first is not None:
        correct_y(first, first_pose)

    # ---------------- U2: straighten back ----------------
    if abs(pose.h) > 0.5:
        turn(f"{away}{deg}")

    if read_heading is not None:
        err = read_heading()
        if abs(err) > max_heading_err:
            log.append(f"WARN heading off by {err:.1f} deg before exit")

    d_front = None if x_face is None else round(x_face - (pose.x + a), 1)
    arrow, conf, how = _pick_arrow(photos, dodge_side)
    if how != "seen":
        log.append(f"arrow 2 not recognised - continuing with {arrow} "
                   f"({'best low-confidence photo' if how == 'guess' else 'default: the dodge side'})")
    status = "OK" if d_front is not None else "NO_DISTANCE"
    return {"status": status, "case": case, "arrow": arrow, "conf": conf, "arrow_how": how,
            "d_front": d_front, "Y": round(side * pose.y, 1),
            "face_source": face_source,
            "x_face": None if x_face is None else round(x_face, 1),
            "photo": first, "log": log}


def far_end_from_photo(u_edge_px, img_w, Y0, d_front=60.0, side="LEFT", cam_lat=0.0,
                       cam_fwd=None, edge_px=3, depth=10.0):
    """
    Task 2, Step 2b - estimate where obstacle 2's end (on the robot's side) is, from the
    straight snap photo. Used only to choose PLAY A vs PLAY B in loop_obs2().

    WHEN TO CALL
        After approach_and_snap_obs2() with case "U1" (robot straight at d_front).
        Only meaningful when the robot is on the same side as the end you will go
        round (dodge side == arrow 2's side).

    INPUTS
        u_edge_px : outer pixel column of obstacle 2's dark body on the robot's side
                    (leftmost column if the robot is LEFT of centre, rightmost if RIGHT).
        img_w     : photo width in px.
        Y0        : robot centre's sideways offset from the centre line (cm, positive).
        d_front   : ultrasonic -> face distance at the photo pose (cm).
        side      : "LEFT" / "RIGHT" - which side of the centre line the robot is on.
        cam_lat   : camera's sideways offset from robot centre (cm, + = left).
        cam_fwd   : camera's forward offset from robot centre (cm); default = ultrasonic's.
        edge_px   : body this close to the frame edge = "runs off frame".
        depth     : obstacle 2 thickness (cm), 10.

    OUTPUT
        Distance (cm) from the centre line to that end, or None if the body runs off
        the frame (long obstacle -> PLAY B).
    """
    a = C.SENSOR_FORWARD_CM
    cam_fwd = a if cam_fwd is None else cam_fwd
    u = u_edge_px if side == "LEFT" else img_w - u_edge_px       # mirror so "outward" = left
    lat = cam_lat if side == "LEFT" else -cam_lat
    if u <= edge_px:
        return None
    fx = (img_w / 2) / math.tan(math.radians(FOV_H_DEG / 2))
    th = math.atan((img_w / 2 - u) / fx)
    Y_cam, D_c = Y0 + lat, d_front + a - cam_fwd
    return Y_cam + (D_c if th >= 0 else D_c + depth) * math.tan(th)


# =============================================================================
# Step 3 - escape and loop around obstacle 2
# =============================================================================

def loop_obs2(send, read_us, read_ir, arrow2, Y, d_front, dodge_side="LEFT",
              far_end_est=None, play_a_max=45.0, gap_target=15.0,
              ir_fwd=0.0, s=0.0, lag=0.0, ir_near_max=40.0, spike_n=2,
              step_cm=3, wall_gap=50.0, wall_guard_cm=15.0, max_leg_cm=220.0,
              obs1_gap=60.0, read_heading=None, drive_edge=None):
    """
    Task 2, Step 3 - escape obstacle 2 on the arrow's side and loop around it,
    without knowing its length, keeping MIN_CLEARANCE from it and from a possible
    wall 50 cm past its ends. Ends heading back toward the carpark, clear of obstacle 2.

    THE PLAY  (described for a LEFT arrow; RIGHT is the exact mirror)
        Choose:  robot on the arrow's side and far end estimate <= play_a_max -> PLAY A,
                 else (unknown, long, or robot on the other side of the centre line) -> PLAY B.
        PLAY A (short/medium obstacle):
            [BW if it needs room] OUT45 -> FW d -> IN45   one S-curve into the middle of
                                    the safe lane (25 cm past the far end), heading across the top.
        PLAY B (long / unknown obstacle):
            drive so that after two OUT45s the side is gap_target from the face,
            OUT45 -> OUT45          now parallel to the face,
            FW until IR spikes      = far end measured  (ultrasonic guards the wall),
            FW adjust -> IN45 -> IN45   into the middle of the lane, heading across the top.
        BOTH, then:
            FW across the top until IR spikes  = back corner reached (x re-zeroed),
            FW adjust -> IN45 -> IN45   down the back face, side gap_target from it,
            FW until IR spikes      = near end measured  (ultrasonic guards the wall),
            FW adjust -> IN45 -> IN45   round the near end, now heading home.
        Far end + near end -> true length and true centre line; the robot's
        sideways position is corrected to that centre line before returning.

    WHEN TO CALL
        Right after approach_and_snap_obs2() returned status "OK": robot straight,
        ultrasonic d_front cm from obstacle 2's face, arrow known.

    INPUTS
        send(cmd)     : sends one STM command and BLOCKS until done.
        read_us()     : one ultrasonic reading (cm) or None.
        read_ir()     : one reading (cm) of the IR on the side FACING obstacle 2 during the
                        loop (right IR for a LEFT arrow, left IR for a RIGHT arrow), None if
                        out of range.
        arrow2        : "LEFT" / "RIGHT" - obstacle 2's arrow.
        Y             : approach_and_snap_obs2()["Y"] - robot centre's offset from the centre
                        line toward the obstacle 1 dodge side (cm).
        d_front       : approach_and_snap_obs2()["d_front"] (cm).
        dodge_side    : which way obstacle 1 was dodged ("LEFT"/"RIGHT").
        far_end_est   : distance (cm) from the centre line to the end you are going round
                        (far_end_from_photo, or a fixed short value after U2).
                        None = unknown -> PLAY B.
        play_a_max    : far ends up to this use PLAY A.
        gap_target    : side-to-face gap on the IR legs (cm).
        ir_fwd        : IR's forward offset from robot centre (cm).
        s             : IR face inset from the robot's side (cm).
        lag           : measured spike lag (cm) - how far past an edge the spike registers.
        ir_near_max   : IR readings at or below this mean "obstacle beside me".
        spike_n       : consecutive "far" readings that confirm a spike.
        step_cm       : FW step while watching for a spike (smaller = more precise, slower).
        wall_gap      : rules' minimum gap from obstacle end to wall (50).
        wall_guard_cm : stop if the ultrasonic (pointing at a possible wall) reads below this.
        max_leg_cm    : abort an IR leg that runs longer than this.
        obs1_gap      : obstacle 1's back face -> obstacle 2's front face (cm), for the
                        PLAY A clearance check (default = the rules' minimum, 60).
        read_heading(): optional gyro heading vs the run's start heading (deg, + = left).
                        If given, the tracked heading is synced to it after every turn.
        drive_edge    : the STM IE command (module doc). Given -> each IR leg is ONE
                        command (the STM stops at the edge and checks the wall itself)
                        instead of FW<step_cm> + IR + ultrasonic repeated.
        Reads from config.py: TURN45_MODELS, SENSOR_FORWARD_CM, ROBOT_WIDTH_CM,
        ROBOT_LENGTH_CM, T2_MIN_CLEARANCE_CM, MAX_STRAIGHT_COMMAND_CM.

    OUTPUT (dict)
        status      : "OK" | "ABORT_UNSAFE" (plan fails clearance - nothing executed)
                      | "ABORT_WALL" (ultrasonic guard tripped) | "ABORT_NO_EDGE" (IR leg too long).
        play        : "A" or "B".
        far_end, near_end : obstacle 2's ends from the corrected centre line (cm), measured
                      toward the arrow side (+ = the end you went round first).
        length      : far_end - near_end (cm), None if not both measured.
        centre_shift: how far the tracked centre line was off (cm) - already corrected.
        x           : robot centre's forward position from obstacle 2's FRONT face (cm,
                      + = past it). Use with D1/D2 for the return home.
        y           : robot centre's offset from the centre line (cm, + = left when facing
                      obstacle 2, i.e. the original "left").
        heading     : degrees from the original heading (about +-180 = facing home).
        log         : commands and measurements.

    ASSUMES
        TURN45_MODELS are robot-centre endpoints from the turn's start pose; obstacle 2
        is centred on the carpark centre line and 10 cm deep; turn arcs are checked only
        at their start, midpoint and end (re-check with your planner's polygon checker).
    """
    W, Lr, a = C.ROBOT_WIDTH_CM, C.ROBOT_LENGTH_CM, C.SENSOR_FORWARD_CM
    clear, depth = CLEAR_CM, 10.0
    m = 1 if arrow2 == "LEFT" else -1                 # work as if always rounding the +y end
    OUT, IN = ("LT45", "RT45") if m == 1 else ("RT45", "LT45")
    pose = _Pose(m * (Y if dodge_side == "LEFT" else -Y))
    pose.x = -(d_front + a)                           # obstacle 2's front face at x = 0
    lane_off = wall_gap / 2                           # middle of [clear+W/2, wall-clear-W/2]
    log, play = [], None
    far = near = None
    far_bound = False                                 # far is only an upper bound so far
    shift = 0.0

    # ---------------- helpers ----------------
    def turn(cmd):
        send(cmd); log.append(cmd); pose.move(*_turn_model(cmd, m))
        if read_heading is not None:
            pose.h = m * read_heading()                   # trust the gyro over the model

    def drive(cm):
        cm = int(round(cm))
        cmd, left = ("FW" if cm > 0 else "BW"), abs(cm)
        while left > 0:
            st = min(left, C.MAX_STRAIGHT_COMMAND_CM)
            send(f"{cmd}{st}"); log.append(f"{cmd}{st}")
            pose.move(st if cmd == "FW" else -st)
            left -= st

    def sim(seq, start=None):
        p = (start or pose).copy()
        for it in seq:
            p.move(*_turn_model(it, m)) if isinstance(it, str) else p.move(it)
        return p

    def ir_pos(p):
        hr = math.radians(p.h)
        side = W / 2 - s                               # IR on the right (inner) side
        return (p.x + ir_fwd * math.cos(hr) + side * math.sin(hr),
                p.y + ir_fwd * math.sin(hr) - side * math.cos(hr))

    def corners(p):
        hr = math.radians(p.h); c, sn = math.cos(hr), math.sin(hr)
        return [(p.x + f * Lr / 2 * c + r * W / 2 * sn, p.y + f * Lr / 2 * sn - r * W / 2 * c)
                for f in (1, -1) for r in (1, -1)]

    def clearance(p, far_y):
        """Min gap between robot rectangle and obstacle 2 (x 0..depth, y up to far_y)."""
        return _box_clear(p, (0.0, depth, -1e4, far_y), W, Lr)

    def drive_until_spike(need_seen, guard=True, max_cm=None):
        """FW in steps until the IR goes near -> far (spike_n far reads).
        Returns (result, pose_at_first_far, near_readings)."""
        max_cm = max_leg_cm if max_cm is None else max_cm
        if drive_edge is not None:                    # ONE STM command for the whole leg
            start = pose.copy()
            r = drive_edge("RIGHT" if m == 1 else "LEFT", need_seen, max(1, int(round(max_cm))))
            log.append(_edge_log(r))
            pose.move(r["moved_cm"])
            nears = [] if r["near_cm"] is None else [r["near_cm"]]
            if r["end"] in ("EDGE", "NEVER"):
                first_far = start.copy(); first_far.move(r["edge_cm"])
                return ("SPIKE" if r["end"] == "EDGE" else "NEVER_SAW"), first_far, nears
            return r["end"], pose.copy(), nears       # "WALL" / "MAX"
        seen, far_run, nears, first_far, travelled = False, 0, [], None, 0
        while True:
            r = read_ir()
            if r is not None and r <= ir_near_max:
                seen, far_run, first_far = True, 0, None
                nears.append(r)
            elif seen or not need_seen:
                if far_run == 0:
                    first_far = pose.copy()
                far_run += 1
                if far_run >= spike_n:
                    return ("SPIKE" if seen else "NEVER_SAW"), first_far, nears
            if guard:
                u = _us_median(read_us, n=3, tol=3.0)
                if u is not None and u < wall_guard_cm:
                    return "WALL", pose.copy(), nears
            if travelled >= max_cm:
                return "MAX", pose.copy(), nears
            drive(step_cm); travelled += step_cm

    def result(status):
        length = None if far is None or near is None else round(far - near, 1)
        return {"status": status, "play": play,
                "far_end": None if far is None else round(far, 1),
                "near_end": None if near is None else round(near, 1),
                "length": length, "centre_shift": round(shift, 1),
                "x": round(pose.x, 1), "y": round(m * pose.y, 1),
                "heading": round(m * pose.h, 1), "log": log}

    # ================= ESCAPE: PLAY A or B =================
    same_side = pose.y > 0                           # robot already on the arrow's side
    if same_side and far_end_est is not None and far_end_est <= play_a_max:
        play, far = "A", far_end_est
        y0 = sim([OUT, IN]).y
        d = max(0, round((far + lane_off - y0) / SIN45))
        obs1_box = (-obs1_gap - 10.0, -obs1_gap, -5.0, 5.0)

        def plan_ok(seq, need):
            end = sim(seq)
            if not far + clear + W / 2 <= end.y <= far + wall_gap - clear - W / 2:
                return False
            return all(clearance(p, far) >= need and
                       _box_clear(p, obs1_box, W, Lr) >= need and
                       max(cy for _, cy in corners(p)) <= far + wall_gap - need
                       for p in _path_points(pose, seq, m))

        pre = next((b for need in NEEDS for b in range(0, -62, -2)
                    if plan_ok(([b] if b else []) + [OUT, d, IN], need)), None)
        if pre is None:
            log.append("A: no safe S-curve even after backing up - not executed")
            return result("ABORT_UNSAFE")
        log.append(f"A: far end est {far:.1f} -> BW{-pre} {OUT} FW{d} {IN}, "
                   f"lane {sim(([pre] if pre else []) + [OUT, d, IN]).y:.1f}")
        if pre:
            drive(pre)
        turn(OUT); drive(d); turn(IN)

    else:
        play = "B"
        # Get parallel to the face. Robot on the arrow's side: OUT45 + reverse 45 (XR45 for
        # a LEFT arrow) ends ~9 cm CLOSER to the centre line, so the IR starts beside even
        # the shortest obstacle and the robot stays far from a wall. Its side gap is the
        # smallest that keeps the turns clear of a face of any length. Robot on the other
        # side (or the reverse turn would swing the rear near obstacle 1): OUT45 + OUT45,
        # which carries it ~30 cm out, across the centre line.
        obs1_box = (-obs1_gap - 10.0, -obs1_gap, -5.0, 5.0)
        face_box = (0.0, depth, -1e4, 1e4)
        need = clear + PLAN_MARGIN_CM
        entry = None
        BACK = "XR45" if m == 1 else "XL45"
        dx_e = sim([OUT, BACK]).x - pose.x
        # gaps to (obstacle 2's face, obstacle 1): both the target if possible, else the
        # minimum - still far safer than OUT45 + OUT45, which can end next to a wall
        for n_face, n_obs1 in ([(f, o) for f in NEEDS for o in NEEDS]
                               if same_side and BACK in C.TURN45_MODELS else ()):
            for g_e in range(int(gap_target), int(ir_near_max) - 4):    # IR reads <= 35
                x0 = -g_e - W / 2 - dx_e
                p0 = pose.copy(); p0.x = x0
                if _path_room(p0, [OUT, BACK], face_box, m) >= n_face:
                    if _path_room(pose, [x0 - pose.x, OUT, BACK], obs1_box, m) >= n_obs1:
                        entry = [OUT, BACK]
                    break
            if entry:
                break
        if entry is None:
            entry = [OUT, OUT]
            x0 = -gap_target - W / 2 - (sim([OUT, OUT]).x - pose.x)
        log.append(f"B: parallel to the face with {' + '.join(entry)}")
        drive(x0 - pose.x)
        u = _us_median(read_us, lo=5, hi=150)          # re-zero x on the face, then fine-tune
        if u is not None and abs((-u - a) - pose.x) > 5:   # beam missed the face (short obstacle,
            log.append(f"B: ultrasonic {u:.1f} ignored (expected ~{-pose.x - a:.1f})")  # robot off its end)
        elif u is not None:
            pose.x = -u - a
            if abs(x0 - pose.x) >= 1:
                drive(x0 - pose.x)
        for c in entry:                                # parallel to the face
            turn(c)

        res, p_sp, nears = drive_until_spike(need_seen=False)
        if nears:                                      # face is at x = 0
            pose.x -= ir_pos(pose)[0] + _median(nears)
        if res == "SPIKE":
            far = ir_pos(p_sp)[1] - lag
            target = far + lane_off
            log.append(f"B: far end measured {far:.1f}")
        elif res == "NEVER_SAW":                       # already past the end
            # lower bound: play_a_max only if a far end estimate existed (a short obstacle
            # would have been PLAY A), else the rules' shortest obstacle (30 cm -> 15)
            far_hi = ir_pos(pose)[1]
            far_lo = min(far_hi, play_a_max if far_end_est is not None else 15.0)
            lo, hi = far_hi + clear + W / 2, far_lo + wall_gap - clear - W / 2
            if lo > hi:
                return result("ABORT_UNSAFE")
            far, target = far_hi, (lo + hi) / 2
            far_bound = True                           # only an upper bound - the top leg measures it
            log.append(f"B: end already passed (between {far_lo:.0f} and {far_hi:.1f})")
        else:
            return result("ABORT_WALL" if res == "WALL" else "ABORT_NO_EDGE")

        dy_in = sim([IN, IN]).y - pose.y
        drive(target - dy_in - pose.y)
        turn(IN); turn(IN)                             # heading across the top

    # ================= ACROSS THE TOP =================
    # the IR should pass the back corner within this distance; if it never sees the end
    # (heading error, dark surface) finish the leg on the tracked position instead
    top_max = max(10.0, (depth + 15.0) - ir_pos(pose)[0])
    res, p_sp, nears = drive_until_spike(need_seen=True, guard=True, max_cm=top_max)
    if nears:
        far_meas = ir_pos(pose)[1] - _median(nears)    # IR looks down at the far end
        log.append(f"top: far end from IR {far_meas:.1f}")
        if play == "A" or far is None or far_bound:
            far = far_meas
    if res == "SPIKE":
        pose.x += depth - (ir_pos(p_sp)[0] - lag)      # re-zero x on the back corner
    elif res == "WALL":
        return result("ABORT_WALL")
    else:
        log.append("top: IR never confirmed the back corner - using tracked x")

    dx_in = sim([IN, IN]).x - pose.x
    drive((depth + gap_target + W / 2 - dx_in) - pose.x)
    turn(IN); turn(IN)                                 # heading down the back face

    # ================= DOWN THE BACK =================
    res, p_sp, nears = drive_until_spike(need_seen=False)
    if nears:                                          # back face is at x = depth
        pose.x += depth - (ir_pos(pose)[0] - _median(nears))
    if res == "SPIKE":
        near = ir_pos(p_sp)[1] + lag
        log.append(f"back: near end measured {near:.1f}")
    elif res == "NEVER_SAW":
        near = -far
        log.append("back: IR never saw the back face - assuming centred obstacle")
    else:
        return result("ABORT_WALL" if res == "WALL" else "ABORT_NO_EDGE")

    dy_in = sim([IN, IN]).y - pose.y                   # negative: turns carry you further down
    drive(pose.y - (near - lane_off - dy_in))
    turn(IN); turn(IN)                                 # heading home

    # ================= CENTRE-LINE CORRECTION =================
    shift = (far + near) / 2
    pose.y -= shift; far -= shift; near -= shift
    log.append(f"centre line corrected by {shift:.1f} cm, length {far - near:.1f} cm")
    return result("OK")


# =============================================================================
# Step 4 - drive home past obstacle 1 and stop at AC in front of the carpark edge
# =============================================================================

def return_to_carpark_edge(send, read_us, read_ir, loop_res, arrow2, gap12, g1,
                           cp_half=30.0, arm_t=10.0, arm_len=60.0, ac_stop=70.0,
                           lane_y=None, ir_fwd=0.0, s=0.0, lag=0.0, ir_near_max=40.0,
                           spike_n=2, step_cm=5, us_tol=25.0, us_tol_locked=4.0,
                           wall_gap=50.0, max_travel_cm=400.0, read_heading=None,
                           drive_edge=None):
    """
    Task 2, Step 4 - from the end of loop_obs2() (heading home, past obstacle 2's
    near end) to a stop in front of the CARPARK ARM's front edge, with the
    ultrasonic giving the AC distance. Prepares for turning into the carpark.

    WHAT IT DOES
        1. Plans a lane shift (S-curve: 45-deg turns, or 10-deg for small shifts) into
           lane_y, simulated against obstacle 2, obstacle 1, both carpark arms and the
           possible wall. Adds a straight FW first if needed. Nothing runs if unsafe.
        2. Drives home in steps:
             - ultrasonic: accepted only if within us_tol of the tracked prediction
               (rejects the carpark back wall seen through the opening); each accepted
               reading re-zeroes the carpark position.
             - IR passing obstacle 1: its reading corrects sideways position, its
               spike (obstacle 1's carpark-side face) corrects forward position.
        3. Stops at ac_stop from the arm edge, fine-tuned on the ultrasonic.
        4. Fallback if the ultrasonic never locks on: 10-deg turn toward the arm side,
           read, undo; then 10-deg the other way, read, undo; else tracked distance only.

    WHEN TO CALL
        Right after loop_obs2() returned status "OK".

    INPUTS
        send, read_us     : as before.
        read_ir()         : the same IR used in loop_obs2 (it faces the centre line now,
                            so it sees obstacle 1 on the way home).
        loop_res          : the dict returned by loop_obs2().
        arrow2            : obstacle 2's arrow ("LEFT"/"RIGHT"), same as loop_obs2.
        gap12             : obstacle 1's back face -> obstacle 2's front face (cm):
                            x_end_cm + x_face - (ac_obs1 + SENSOR_FORWARD_CM + 10)
                            (x_end_cm from pass_obs1_measure_ir, x_face from
                            approach_and_snap_obs2).
        g1                : carpark arm front edge -> obstacle 1's front face (cm):
                            D1 + ac_obs1 + SENSOR_FORWARD_CM - cp_front_from_start
                            (D1 from start_to_obs1, cp_front_from_start = start robot
                            centre -> arm front plane).
        cp_half           : half the carpark's inside width (60 / 2 = 30).
        arm_t             : thickness of the carpark arm's front edge (cm) - MEASURE IT.
        arm_len           : arm length (only for the clearance model).
        ac_stop           : ultrasonic -> arm edge distance to stop at (cm). Must leave room
                            for park_in_carpark's S-curve (~60 cm with the current models).
        lane_y            : home lane (cm from the centre line, on the robot's side);
                            default = middle of the arm, -(cp_half + arm_t/2).
        ir_fwd, s, lag    : IR mounting / spike lag (same as loop_obs2).
        ir_near_max, spike_n : IR "beside obstacle" threshold / far reads to confirm a spike.
        step_cm           : drive step while watching sensors.
        us_tol            : max difference (cm) between ultrasonic and prediction for the
                            FIRST lock-on.
        us_tol_locked     : max difference once locked - stops a slow drift onto the back
                            wall (seen through the opening) from dragging the stop point.
        wall_gap          : rules' wall gap past obstacle 2's ends (50).
        max_travel_cm     : hard stop on the home drive.
        read_heading()    : optional gyro heading vs the run's start heading (deg, + = left);
                            syncs the tracked heading after every turn.
        drive_edge        : the STM IE command (module doc). Given -> the home drive is a
                            few long moves: ultrasonic read, ONE IE leg past obstacle 1
                            (never past the stop point), ultrasonic read, one FW to just
                            short of the stop, then the final align - instead of FW5 steps.

    OUTPUT (dict)
        status        : "OK" | "ABORT_UNSAFE" (no safe lane shift - nothing executed).
        source        : how the stop distance was found: "US" | "US_ANGLED" | "ODOM".
        ac_cm         : ultrasonic -> carpark arm edge distance at the stop (cm).
        y             : robot centre's offset from the centre line (cm, + = original left).
        ir_obs1       : median IR reading passing obstacle 1 (None if not seen).
        expected_ir   : what that reading should be for lane_y.
        x_from_carpark: robot centre -> arm front plane (cm), along the home direction.
        obs1_from_carpark: obstacle 1's carpark-side face -> arm front plane (cm).
        heading       : degrees from the original heading (about +-180 = facing carpark).
        log           : commands and measurements.

    ASSUMES
        The carpark, obstacle 1 and obstacle 2 share the same centre line; turn models
        are robot-centre endpoints; arcs are checked at start / midpoint / end only.
    """
    W, Lr, a = C.ROBOT_WIDTH_CM, C.ROBOT_LENGTH_CM, C.SENSOR_FORWARD_CM
    clear, deg = CLEAR_CM, C.ANGLE_TURN_DEG
    m = 1 if arrow2 == "LEFT" else -1
    pose = _Pose(m * loop_res["y"])
    pose.x, pose.h = loop_res["x"], m * loop_res["heading"]
    far, near = loop_res["far_end"], loop_res["near_end"]
    log = []

    x1w = -gap12 - 10.0                                   # obstacle 1's carpark-side face
    x_cp = x1w - g1                                       # carpark arm front plane
    lane_y = -(cp_half + arm_t / 2) if lane_y is None else -abs(lane_y)
    expected_ir = -5.0 - lane_y - (W / 2 - s)
    boxes = [(0.0, 10.0, near, far),                                   # obstacle 2
             (x1w, x1w + 10.0, -5.0, 5.0),                             # obstacle 1
             (x_cp - arm_len, x_cp, -cp_half - arm_t, -cp_half),       # arm on robot's side
             (x_cp - arm_len, x_cp, cp_half, cp_half + arm_t),         # other arm
             (-1e4, 1e4, -1e4, near - wall_gap)]                       # possible wall

    # ---------------- helpers ----------------
    def turn(cmd):
        send(cmd); log.append(cmd); pose.move(*_turn_model(cmd, m))
        if read_heading is not None:
            pose.h = m * read_heading()                   # trust the gyro over the model

    def drive(cm):
        cm = int(round(cm))
        cmd, left = ("FW" if cm > 0 else "BW"), abs(cm)
        while left > 0:
            st = min(left, C.MAX_STRAIGHT_COMMAND_CM)
            send(f"{cmd}{st}"); log.append(f"{cmd}{st}")
            pose.move(st if cmd == "FW" else -st)
            left -= st

    def path_ok(seq, need=NEEDS[-1]):
        return all(_box_clear(q, b, W, Lr) >= need
                   for q in _path_points(pose, seq, m) for b in boxes)

    def front_x():
        return pose.x + a * math.cos(math.radians(pose.h))

    def ir_pos(p):
        hr = math.radians(p.h); side = W / 2 - s
        return (p.x + ir_fwd * math.cos(hr) + side * math.sin(hr),
                p.y + ir_fwd * math.sin(hr) - side * math.cos(hr))

    def result(status, source=None, nears=()):
        return {"status": status, "source": source,
                "ac_cm": round(front_x() - x_cp, 1), "y": round(m * pose.y, 1),
                "ir_obs1": _median(list(nears)), "expected_ir": round(expected_ir, 1),
                "x_from_carpark": round(pose.x - x_cp, 1),
                "obs1_from_carpark": round(x1w - x_cp, 1),
                "heading": round(m * pose.h, 1), "log": log}

    # ================= 1. LANE SHIFT =================
    plan = _plan_s_curve(pose, lane_y, m)
    pre = next((p for need in NEEDS for p in range(0, 122, 2)
                if path_ok(([p] if p else []) + plan, need)), None)
    if pre is None:
        log.append("lane shift: no safe plan found - nothing executed")
        return result("ABORT_UNSAFE")
    log.append(f"lane {pose.y:.1f} -> {lane_y:.1f}: pre-FW{pre} {plan}")
    if pre:
        drive(pre)
    for it in plan:
        turn(it) if isinstance(it, str) else drive(it)

    # ================= 2. HOME DRIVE =================
    seen, far_run, nears, first_far, ir_done = False, 0, [], None, False
    us_locked, travelled = False, 0.0
    last_u, last_fx = None, None                          # previous reading + front x then

    def us_check():
        """One ultrasonic reading: first lock-on (must shrink by what we drove since the
        last reading - a side wall seen at a slant gives a constant reading and is
        rejected), then stay re-zeroed on the arm."""
        nonlocal x_cp, us_locked, last_u, last_fx
        gap_pred = front_x() - x_cp
        u = _us_median(read_us, n=3, tol=3.0, lo=5, hi=400)
        if u is not None and not us_locked:
            moving = (last_u is not None and
                      abs((last_u - u) - (last_fx - front_x())) <= 2.5 and last_u - u > 1.0)
            if abs(u - gap_pred) <= us_tol and moving:
                x_cp, us_locked = front_x() - u, True
                log.append(f"US locked on the arm at {u:.1f} cm")
            elif abs(u - gap_pred) > us_tol:
                log.append(f"US {u:.1f} ignored (expected ~{gap_pred:.1f})")
            last_u, last_fx = u, front_x()
        elif u is not None and abs(u - gap_pred) <= us_tol_locked:
            x_cp = front_x() - u                          # stay re-zeroed on the arm
        elif u is not None:
            log.append(f"US {u:.1f} ignored (expected ~{gap_pred:.1f})")

    def obs1_fix(first_far_pose, near_list):
        """Obstacle 1 passed on the IR side: forward fix from where the reading went
        far, sideways fix from the reading beside it."""
        nonlocal x_cp
        dx_fix = x1w - (ir_pos(first_far_pose)[0] + lag)
        pose.x += dx_fix
        if us_locked:
            x_cp += dx_fix                                # keep the ultrasonic-based gap
        pose.y += (-5.0 - _median(near_list)) - ir_pos(pose)[1]
        log.append(f"obs1 passed: IR {_median(near_list):.1f} cm "
                   f"(expected {expected_ir:.1f}), position corrected")

    if drive_edge is not None:
        # ---- few long moves (IE firmware) ----
        us_check()
        room = (front_x() - x_cp) - ac_stop - 2.0         # never drive past the stop point
        if room >= 5:
            start = pose.copy()
            ir_side = "RIGHT" if m == 1 else "LEFT"
            r = drive_edge(ir_side, True, int(room))
            log.append(_edge_log(r))
            pose.move(r["moved_cm"]); travelled += r["moved_cm"]
            if r["near_cm"] is not None:
                nears = [r["near_cm"]]
            if r["end"] == "EDGE":
                ff = start.copy(); ff.move(r["edge_cm"])
                obs1_fix(ff, nears)
            else:
                log.append("obstacle 1 not passed before the stop point - no IR fix")
        us_check()
        for _ in range(3):                                # close in: one FW, re-read
            gap = (front_x() - x_cp) - ac_stop
            if gap <= 12 or travelled >= max_travel_cm:
                break
            drive(gap - 8); travelled += gap - 8
            us_check()
    else:
        # ---- FW<step_cm> + sensors, repeated (old firmware) ----
        while True:
            gap_pred = front_x() - x_cp
            us_check()
            gap_pred = front_x() - x_cp

            if not ir_done:                               # obstacle 1 on the IR side
                r = read_ir()
                if r is not None and r <= ir_near_max:
                    seen, far_run, first_far = True, 0, None
                    nears.append(r)
                elif seen:
                    if far_run == 0:
                        first_far = pose.copy()
                    far_run += 1
                    if far_run >= spike_n:
                        ir_done = True
                        obs1_fix(first_far, nears)
                        gap_pred = front_x() - x_cp

            if gap_pred - ac_stop <= 0.5:
                break
            if travelled >= max_travel_cm:
                log.append("home drive: travel cap reached")
                break
            stepcm = max(1, min(step_cm, round(gap_pred - ac_stop)))
            drive(stepcm); travelled += stepcm

    # ================= 3. FINAL ALIGN / FALLBACK =================
    if us_locked:
        source = "US"
        u2 = _us_median(read_us, n=5, tol=2.0, lo=5, hi=400)
        if u2 is not None and abs(u2 - (front_x() - x_cp)) <= us_tol_locked:
            x_cp = front_x() - u2
    else:
        source = None
        gap_pred = front_x() - x_cp
        L_cmd, R_cmd = ("LT", "RT") if m == 1 else ("RT", "LT")   # L_cmd = toward the arm
        for t in (L_cmd, R_cmd):
            name = f"{t}{deg}"
            if not path_ok([name], PHOTO_CLEAR_CM):
                log.append(f"no room for the angled {name} ultrasonic check - skipped")
                continue
            saved = pose.copy(); turn(name)
            u = _us_median(read_us, n=5, tol=3.0, lo=5, hi=200)
            hr = math.radians(pose.h)
            cand = None if u is None else pose.x + (a + u) * math.cos(hr)
            undo = "X" + t[0] + str(deg)
            send(undo); log.append(undo)
            pose.x, pose.y, pose.h = saved.x, saved.y, saved.h
            if cand is not None and abs((front_x() - cand) - gap_pred) <= us_tol:
                x_cp, source = cand, "US_ANGLED"
                log.append(f"angled US found the arm edge at {front_x() - cand:.1f} cm")
                break
        if source is None:
            source = "ODOM"
            log.append("ultrasonic never found the arm - stopping on tracked distance")

    if abs((front_x() - x_cp) - ac_stop) >= 1:
        drive((front_x() - x_cp) - ac_stop)
    return result("OK", source, nears)


# =============================================================================
# Step 5 - turn into the carpark and park
# =============================================================================

def park_in_carpark(send, read_us, read_ir_left, read_ir_right, home_res, arrow2,
                    cp_half=30.0, cp_depth=50.0, arm_t=10.0, entry_gap=10.0,
                    entry_min=-15.0, park_back_cm=None, s=0.0, ir_fwd=0.0, ir_in_max=35.0,
                    centre_tol=8.0, max_fixes=1, step_cm=4, us_tol=12.0,
                    us_tol_locked=4.0, max_travel_cm=120.0, depth_extra=50.0, read_heading=None):
    """
    Task 2, Step 5 - from the stop in front of the carpark arm (return_to_carpark_edge)
    into the carpark, centred, and stop clear of the back wall.

    THE PLAY
        1. S-curve to the centre line (RT45 -> FW d -> LT45 for the usual side; 10-deg
           turns if the shift is small), ending straight with the front entry_gap cm
           outside the opening - or as far as entry_min (negative = already inside) when
           obstacle 1 is close to the carpark and there is no room to finish outside. Simulated against both arms, the back wall AND obstacle 1
           first; shifts the start forward or back if it needs room. Nothing runs if no
           safe plan exists.
        2. Check the ultrasonic now sees the BACK WALL (about entry_gap + cp_depth away, or
           up to depth_extra more - that reading sets the real depth and the stop point).
           If it still sees the arm (about entry_gap) the S-curve fell short -> stop.
        3. Drive in in steps. Once both side IRs are between the arms they drop to about
           (inside width - robot width) / 2 each; left vs right gives the sideways error.
           If it is more than centre_tol: back out to the entry point, S-curve again (10-deg),
           drive in again (at most max_fixes times).
        4. AC on the back wall to park_back_cm and stop.

    WHEN TO CALL
        Right after return_to_carpark_edge() returned status "OK".

    INPUTS
        send, read_us              : as before.
        read_ir_left/read_ir_right : one reading (cm) from each side IR, None if out of range.
        home_res                   : the dict returned by return_to_carpark_edge().
        arrow2                     : obstacle 2's arrow (same as the earlier steps - it sets
                                     which side of the carpark the robot is on).
        cp_half                    : half the carpark's inside width (60 / 2 = 30).
        cp_depth                   : carpark inside depth, front plane -> back wall (50) -
                                     the MINIMUM; the real one is measured on entry.
        depth_extra                : how much deeper than cp_depth the carpark may be (cm).
        arm_t                      : arm / wall thickness (cm), for the clearance model.
        entry_gap                  : preferred front -> carpark front plane after the S-curve (cm).
        entry_min                  : closest allowed finish (cm, negative = inside the opening).
        park_back_cm               : ultrasonic -> back wall distance to stop at (cm).
                                     Default (measured depth - ROBOT_LENGTH_CM) / 2 = equal
                                     margin front and back.
        s, ir_fwd                  : IR face inset from the side / forward offset (cm).
        ir_in_max                  : side-IR readings at or below this mean "wall beside me".
        centre_tol                 : sideways error (cm) allowed before a re-entry fix.
        max_fixes                  : max re-entry corrections.
        step_cm                    : drive-in step (cm).
        us_tol                     : max ultrasonic-vs-prediction difference for the first check.
        us_tol_locked              : max difference while driving in (after the first check).
        max_travel_cm              : hard cap on the drive-in.
        read_heading()             : optional gyro heading vs the run's start heading (deg,
                                     + = left); syncs the tracked heading after every turn.
        Reads from config.py: TURN45_MODELS, SMALL_TURN_MODELS, ANGLE_TURN_DEG,
        SENSOR_FORWARD_CM, ROBOT_WIDTH_CM, ROBOT_LENGTH_CM, T2_MIN_CLEARANCE_CM, T2_CARPARK_CLEARANCE_CM,
        MAX_STRAIGHT_COMMAND_CM.

    OUTPUT (dict)
        status   : "OK" | "ABORT_UNSAFE" (no safe S-curve - nothing executed)
                   | "MISALIGNED" (ultrasonic still sees the arm after the S-curve).
        back_cm  : ultrasonic -> back wall at the stop (cm).
        y        : robot centre's offset from the carpark centre line (cm, + = original left).
        inside   : True if the whole robot is inside the carpark (front plane to back wall,
                   arm to arm) by the tracked position.
        fixes    : re-entry corrections used.
        ir_lr    : (left, right) IR readings used for the centring check.
        log      : commands and measurements.

    ASSUMES
        The carpark is centred on the same centre line as the obstacles, and both side
        IRs are mounted alike (same inset s and forward offset ir_fwd).
    """
    W, Lr, a = C.ROBOT_WIDTH_CM, C.ROBOT_LENGTH_CM, C.SENSOR_FORWARD_CM
    clear, cp_clear = CLEAR_CM, CARPARK_CLEAR_CM          # obstacle 1 / the carpark itself
    m = 1 if arrow2 == "LEFT" else -1
    ir_plus, ir_minus = (read_ir_right, read_ir_left) if m == 1 else (read_ir_left, read_ir_right)
    pose = _Pose(m * home_res["y"])
    pose.x, pose.h = home_res["x_from_carpark"], m * home_res["heading"]   # front plane x = 0
    park_back = (cp_depth - Lr) / 2 if park_back_cm is None else park_back_cm
    back_x = -cp_depth
    boxes = [(back_x - arm_t, 0.0, -cp_half - arm_t, -cp_half),          # arm on robot's side
             (back_x - arm_t, 0.0, cp_half, cp_half + arm_t),            # other arm
             (back_x - arm_t, back_x, -cp_half - arm_t, cp_half + arm_t)]  # back wall
    o1 = home_res.get("obs1_from_carpark")
    if o1 is not None:
        boxes.append((o1, o1 + 10.0, -5.0, 5.0))                       # obstacle 1
    log, fixes, ir_lr = [], 0, None

    # ---------------- helpers ----------------
    def turn(cmd):
        send(cmd); log.append(cmd); pose.move(*_turn_model(cmd, m))
        if read_heading is not None:
            pose.h = m * read_heading()                   # trust the gyro over the model

    def drive(cm):
        cm = int(round(cm))
        cmd, left = ("FW" if cm > 0 else "BW"), abs(cm)
        while left > 0:
            st = min(left, C.MAX_STRAIGHT_COMMAND_CM)
            send(f"{cmd}{st}"); log.append(f"{cmd}{st}")
            pose.move(st if cmd == "FW" else -st)
            left -= st

    def front_x():
        return pose.x + a * math.cos(math.radians(pose.h))

    def back_gap():
        return front_x() - back_x

    def path_ok(seq, need=NEEDS[-1]):
        """boxes[:3] are the carpark (arms, back wall), the rest obstacle 1."""
        return all(_box_clear(q, b, W, Lr) >= (cp_clear + PLAN_MARGIN_CM if i < 3 else need)
                   for q in _path_points(pose, seq, m) for i, b in enumerate(boxes))

    def plan_entry():
        """S-curve to y = 0 ending straight with the front entry_gap outside, plus the
        straight FW/BW before it. Returns (pre, plan) or None."""
        plan = _plan_s_curve(pose, 0.0, m)
        end = pose.copy()
        for it in plan:
            end.move(*_turn_model(it, m)) if isinstance(it, str) else end.move(it)
        end_front = end.x + a * math.cos(math.radians(end.h))
        pre0 = int(round(end_front - entry_gap))          # + = FW first, - = BW first
        for need in NEEDS:
            for k in range(0, 62, 2):                     # nearest working start first
                for pre in ((pre0,) if k == 0 else (pre0 + k, pre0 - k)):
                    if end_front - pre < entry_min:       # don't finish too deep inside
                        continue
                    if path_ok(([pre] if pre else []) + plan, need):
                        return pre, plan
        return None

    def run(pre, plan):
        if pre:
            drive(pre)
        for it in plan:
            turn(it) if isinstance(it, str) else drive(it)

    def inside():
        hr = math.radians(pose.h)
        rear = pose.x - (Lr / 2) * math.cos(hr)
        return (max(rear, front_x()) <= 0.0 and min(rear, front_x()) >= back_x
                and abs(pose.y) + W / 2 <= cp_half)

    def result(status):
        return {"status": status, "back_cm": round(back_gap(), 1),
                "y": round(m * pose.y, 1), "inside": inside(), "fixes": fixes,
                "ir_lr": ir_lr, "log": log}

    # ================= 1. S-CURVE TO THE CENTRE LINE =================
    pl = plan_entry()
    if pl is None:
        log.append("entry: no safe S-curve - nothing executed")
        return result("ABORT_UNSAFE")
    log.append(f"entry: y {m * pose.y:.1f} -> 0, pre {pl[0]}, {pl[1]}")
    run(*pl)

    # ================= 2. ULTRASONIC MUST SEE THE BACK WALL =================
    # The front plane is known from the arm (step 4), so the back wall reading gives the
    # REAL depth (cp_depth is the minimum) -> park in the middle of it.
    u = _us_median(read_us, lo=5, hi=200)
    depth = None if u is None else u - front_x()
    if depth is not None and cp_depth - us_tol <= depth <= cp_depth + depth_extra:
        back_x = -depth
        boxes[:3] = [(back_x - arm_t, 0.0, -cp_half - arm_t, -cp_half),
                     (back_x - arm_t, 0.0, cp_half, cp_half + arm_t),
                     (back_x - arm_t, back_x, -cp_half - arm_t, cp_half + arm_t)]
        if park_back_cm is None:
            park_back = (depth - Lr) / 2
        log.append(f"back wall: carpark depth {depth:.1f} cm -> stop {park_back:.1f} cm off it")
    elif u is not None and front_x() > 0 and abs(u - front_x()) <= min(us_tol, front_x() + 2):
        log.append(f"ultrasonic {u:.1f} = the arm, not the back wall - S-curve fell short")
        return result("MISALIGNED")
    else:
        log.append(f"ultrasonic {u} unexpected - continuing on tracked position")

    # ================= 3. DRIVE IN, CENTRING CHECK =================
    # Long moves: one FW until both side IRs are between the arms, read them, (re-enter
    # if off-centre), one FW to just short of the stop, then the final AC below.
    checked, travelled = False, 0.0
    for _ in range(2 + max_fixes * 2):
        u = _us_median(read_us, n=3, tol=3.0, lo=3, hi=200)
        if u is not None and abs(u - back_gap()) <= us_tol_locked:
            pose.x += (u + back_x) - front_x()
        if checked:
            break
        c = math.cos(math.radians(pose.h))                # about -1: heading into the carpark
        ir_x = pose.x + ir_fwd * c
        need = (ir_x + 4.0) / max(0.2, -c)                # IRs 4 cm inside the opening
        to_park = back_gap() - park_back
        if need > 0.5:
            drive(min(need, to_park)); travelled += min(need, to_park)
        rp, rm = ir_plus(), ir_minus()
        checked = True
        if rp is None or rm is None or rp > ir_in_max or rm > ir_in_max:
            log.append(f"inside: IRs did not both see the walls ({rm}, {rp}) - no centring check")
            continue
        y_meas = (rm - rp) / 2
        ir_lr = (rm, rp) if m == 1 else (rp, rm)          # (left, right)
        log.append(f"inside: IR left/right {ir_lr[0]:.1f}/{ir_lr[1]:.1f} "
                   f"-> off-centre {m * y_meas:.1f} cm")
        pose.y = y_meas
        if abs(y_meas) > centre_tol and fixes < max_fixes:
            fixes += 1
            drive(-(entry_gap - front_x()))               # back out to the entry point
            pl = plan_entry()
            if pl is None:
                log.append("fix: no safe re-entry plan - continuing as is")
            else:
                log.append(f"fix {fixes}: {pl}")
                run(*pl)
                checked = False

    rest = back_gap() - park_back
    if rest > 6:
        drive(rest - 3); travelled += rest - 3            # final AC below does the last cm
    if travelled >= max_travel_cm:
        log.append("drive-in: travel cap reached")

    # ================= 4. FINAL AC ON THE BACK WALL =================
    u = _us_median(read_us, n=5, tol=2.0, lo=3, hi=200)
    if u is not None and abs(u - back_gap()) <= us_tol_locked:
        pose.x += (u + back_x) - front_x()
    if abs(back_gap() - park_back) >= 1:
        drive(back_gap() - park_back)
    log.append(f"parked: back wall {back_gap():.1f} cm, inside = {inside()}")
    return result("OK")


# =============================================================================
# Runner - steps 0 to 5 in order
# =============================================================================

def run_task2(send, read_us, read_ir_left, read_ir_right, snap,
              cp_front_from_start=getattr(C, "T2_CP_FRONT_FROM_START_CM", 32.0),
              ac_obs1=getattr(C, "T2_AC_OBS1_CM", 40.0),
              ir_fwd=getattr(C, "T2_IR_FWD_CM", 0.0),
              s=getattr(C, "T2_IR_INSET_CM", 0.0),
              lag=getattr(C, "T2_IR_LAG_CM", 0.0),
              g=getattr(C, "T2_IR_GAP_OBS1_CM", 10.0),
              u2_far_end=getattr(C, "T2_U2_FAR_END_CM", 18.5),
              ac_stop=getattr(C, "T2_AC_STOP_CARPARK_CM", 70.0),
              arm_t=getattr(C, "T2_CARPARK_ARM_CM", 10.0),
              cp_half=getattr(C, "T2_CARPARK_HALF_CM", 30.0),
              cp_depth=getattr(C, "T2_CARPARK_DEPTH_CM", 50.0),
              read_heading=None, drive_edge=None):
    """
    Task 2 runner - the whole fastest-car run, steps 0 -> 5. Stops at the first step
    that is not OK and returns everything done so far.

    WHEN TO CALL
        Robot parked in the carpark, straight, facing obstacle 1, Android "start" received.

    WHAT IT DOES
        0.  start_to_obs1                       -> AC40 at obstacle 1, arrow 1, D1
        1.  dodge_plan(arrow1)                  -> send its 3 commands
        1b. pass_obs1_measure_ir                -> real Y0 from the IR
        2.  approach_and_snap_obs2              -> arrow 2 (U1 / U2)
            play choice: U2 -> far end = u2_far_end (short obstacle, PLAY A);
                         U1 + same side + body edge in photo -> far_end_from_photo;
                         otherwise None (PLAY B).
        3.  loop_obs2                           -> round obstacle 2, length measured
        4.  return_to_carpark_edge              -> stop at ac_stop in front of the arm
        5.  park_in_carpark                     -> centred inside, stopped off the back wall

    INPUTS  (everything after snap defaults to the T2_* values in config.py)
        send, read_us, snap         : as in the step functions.
        read_ir_left/read_ir_right  : one reading (cm) from each side IR, None if out of range.
        cp_front_from_start         : robot centre at the start -> carpark front plane (cm).
                                      Measure once for the way you place the car.
        ac_obs1                     : AC distance used at obstacle 1 (cm).
        ir_fwd, s, lag              : IR mounting / spike lag (both IRs assumed mounted alike).
        g                           : target IR gap passing obstacle 1 (cm).
        u2_far_end                  : far end assumed after U2 (short obstacle) - the middle of
                                      the possible 15-22 cm range.
        ac_stop, arm_t              : see return_to_carpark_edge.
        cp_half, cp_depth           : carpark inside half-width / depth (cm).
        drive_edge                  : the STM IE command (module doc) - IR legs in ONE command.
                                      None = old FW3 + IR stepping (old firmware).
        read_heading                : optional gyro: heading vs the start heading (deg, + = left).
                                      Strongly recommended - it keeps the tracked position honest
                                      when real turns differ from config.py.

    OUTPUT (dict)
        status : "OK" or the failing step's status prefixed with the step name.
        start, dodge, pass_obs1, obs2, loop, home, park : each step's own result dict.
        arrows : (arrow 1, arrow 2) for the Android / report.
    """
    out = {}
    a = C.SENSOR_FORWARD_CM

    # ---- 0. carpark -> obstacle 1 ----
    r0 = start_to_obs1(send, read_us, snap, ac=ac_obs1); out["start"] = r0
    if r0["status"] != "OK":
        out["status"] = "START_" + r0["status"]; return out
    arrow1 = r0["arrow"]

    # ---- 1. dodge obstacle 1 ----
    p1 = dodge_plan(arrow1, g=g, s=s, ac=ac_obs1); out["dodge"] = p1
    if not p1["safe"]:
        out["status"] = "DODGE_UNSAFE"; return out
    for cmd in p1["cmds"]:
        send(cmd)

    # ---- 1b. pass obstacle 1, measure Y0 ----
    ir1 = read_ir_right if arrow1 == "LEFT" else read_ir_left
    p1b = pass_obs1_measure_ir(send, ir1, p1["x_end_cm"], ac=ac_obs1, g=g, s=s, ir_fwd=ir_fwd,
                               drive_edge=drive_edge,
                               ir_side="RIGHT" if arrow1 == "LEFT" else "LEFT")
    out["pass_obs1"] = p1b

    # ---- 2. approach + snap obstacle 2 ----
    r2 = approach_and_snap_obs2(send, read_us, snap, p1b["Y0"], p1b["x_end_cm"],
                                dodge_side=arrow1, ac_obs1=ac_obs1, read_heading=read_heading)
    out["obs2"] = r2
    if r2["status"] != "OK":
        out["status"] = "OBS2_" + r2["status"]; return out
    arrow2 = r2["arrow"]
    out["arrows"] = (arrow1, arrow2)

    if r2["case"] == "U2":
        far_est = u2_far_end
    elif (arrow2 == arrow1 and r2["photo"] is not None
          and r2["photo"].get("body_edge_px") is not None):
        far_est = far_end_from_photo(r2["photo"]["body_edge_px"], r2["photo"]["img_w"],
                                     r2["Y"], r2["d_front"], side=arrow1)
    else:
        far_est = None

    # ---- 3. loop obstacle 2 ----
    ir2 = read_ir_right if arrow2 == "LEFT" else read_ir_left
    gap12 = p1b["x_end_cm"] + r2["x_face"] - (ac_obs1 + a + 10)
    r3 = loop_obs2(send, read_us, ir2, arrow2, r2["Y"], r2["d_front"], dodge_side=arrow1,
                   far_end_est=far_est, ir_fwd=ir_fwd, s=s, lag=lag, obs1_gap=gap12,
                   read_heading=read_heading, drive_edge=drive_edge)
    out["loop"] = r3
    if r3["status"] != "OK":
        out["status"] = "LOOP_" + r3["status"]; return out

    # ---- 4. home to the carpark edge ----
    g1 = r0["D1"] + ac_obs1 + a - cp_front_from_start
    # home lane: the side IR reads g (like on the way out) when passing obstacle 1
    home_lane = 5.0 + g + C.ROBOT_WIDTH_CM / 2 - s
    r4 = return_to_carpark_edge(send, read_us, ir2, r3, arrow2, gap12, g1, cp_half=cp_half,
                                lane_y=home_lane,
                                ac_stop=ac_stop, arm_t=arm_t, ir_fwd=ir_fwd, s=s, lag=lag,
                                read_heading=read_heading, drive_edge=drive_edge)
    out["home"] = r4
    if r4["status"] != "OK":
        out["status"] = "HOME_" + r4["status"]; return out

    # ---- 5. park ----
    r5 = park_in_carpark(send, read_us, read_ir_left, read_ir_right, r4, arrow2,
                         cp_half=cp_half, cp_depth=cp_depth, arm_t=arm_t, s=s, ir_fwd=ir_fwd, read_heading=read_heading)
    out["park"] = r5
    out["status"] = "OK" if r5["status"] == "OK" else "PARK_" + r5["status"]
    return out
