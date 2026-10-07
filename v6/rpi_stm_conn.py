#!/usr/bin/env python3
"""
v5. Send a sequence of commands to the STM32 over /dev/ttyACM0.

Standalone usage:
    python3 rpi_stm_conn.py                  # runs the COMMANDS list below
    python3 rpi_stm_conn.py FW50 RT90 FW30   # runs commands given on the command line
    python3 rpi_stm_conn.py AC20             # test the ultrasonic align on its own
    python3 rpi_stm_conn.py -f path.txt      # runs commands from a file (one per line);
                                             # path.txt.targets.json (written next to it by
                                             # rpi_client_android.py) gives the FAR/NEAR bounds

As a module (used by rpi_client_android.py):
    import rpi_stm_conn
    cmds = rpi_stm_conn.read_cmds_file(path)
    rpi_stm_conn.run_commands(cmds, on_snap=camera_fn, on_check_position=side_fn,
                              targets=planned_targets, on_result=report_fn)

The algo server only sends the ideal path plus SNAP<id>. Everything around a
photo happens here on the RPi (photo_cycle). SNAP<id> is never written to the STM.

v5 photo rules ("count" = number of TARGET images the model found in one photo,
i.e. classes with a numeric image ID; the "Bounding box" class is not counted):

    Snap 1 = IDEAL (the planned photo pose)
    CASE A  count == 1, conf >= RECOGNISED_CONF  -> report it. Done (1 photo).
    CASE B  count == 1, conf <  RECOGNISED_CONF  -> BW to the FAR bound, Snap FAR,
                                                   FW to the NEAR bound, Snap NEAR,
                                                   back to the photo pose.
                                                   Rank every box from all photos by
                                                   confidence; report the highest (max 3 photos).
    CASE C  count == 0                           -> report nothing (MSG to Android).
    CASE D  count >= 2                           -> IDEAL boxes are held, FW to the NEAR
                                                   bound, Snap NEAR, back. Rank every box
                                                   from IDEAL + NEAR by confidence and
                                                   report the highest.
                                                   (max 2 photos)

FAR / NEAR distances come from the planner (segment target["retry"]); they are
already cut short to keep MIN_CLEARANCE_CM. If a bound has no room, that photo
is skipped and the reason is printed. No angle correction runs in v5.

on_snap(obstacle_id, attempt, final, label=..., plan=...) returns a snap dict
(see trigger_camera in rpi_client_android.py). An older callback that returns
True/False (the simulator) still works: True = one confident image, False = none.

STM reply to AC<cm>:  DONE AC20 US:<mm> MOVED:<cm>  or  DONE AC20 NOOBJ MOVED:<cm>
STM reply to RA:      DONE RA UNDID:<cm>
"""

import inspect
import sys
import time
import serial
import config

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
STOP_ON_ERROR = True      # stop the sequence on ERR or timeout of a PLANNED move

# ---- v5 photo decision (see the module docstring) ----
RECOGNISED_CONF = config.RECOGNISED_CONF   # CASE A threshold (0.50)

# ---- v4 error correction: kept so older tools (simulator.py) still load,
# ---- but photo_cycle() in v5 never calls correct_snap_angle().
APPROACH_CMD = "AC15"
RETURN_CMD = "RA"
ANGLE_CORRECTION_ENABLED = True
MAX_CORRECTIONS = 0
ANGLE_NUDGE_CM = 10

SHOW_RAW_REPLIES = False  # True prints every raw STM reply line (debugging)

# ---- align debug (AC / RA) ----
SHOW_ALIGN_DEBUG = True   # US reading before AC and after RA, plus cap / undo checks
AC_MAX_TOTAL_CM = 10      # must match AC_MAX_TOTAL_CM in stm_code.c
AC_TOL_CM = 2             # must match AC_TOL_MM / 10 in stm_code.c

# Undoing a turn = the opposite-direction version of the same turn style.
REVERSE_TURN = {
    "LT90": "XL90",
    "XL90": "LT90",
    "RT90": "XR90",
    "XR90": "RT90",
}

# (last turn, side the obstacle appears on) -> which way to creep before
# redoing that turn, so the robot ends up shifted towards the obstacle.
TURN_CORRECTION_NUDGE = {
    ("LT90", "right"): "FW",
    ("LT90", "left"):  "BW",
    ("RT90", "left"):  "FW",
    ("RT90", "right"): "BW",
    ("XR90", "left"):  "BW",
    ("XR90", "right"): "FW",
    ("XL90", "right"): "BW",
    ("XL90", "left"):  "FW",
}

ALIGN_PREFIX = "AC"       # ultrasonic align commands; a failed align never stops the run

# Reply lines that mean "command finished"
DONE_PREFIXES = ("DONE", "ERR", "SPD", "SERVO", "BIAS", "YAW", "US ", "IR1")

RULE = "-" * 64

# Filled during run_commands, for summaries.
align_log = []   # one entry per AC/RA sent
snap_log = []    # one entry per obstacle: {"obstacle", "case", "snaps", "recognised", "decision"}
WIDE = "=" * 64


# ----------------------------------------------------------------------
# Serial I/O
# ----------------------------------------------------------------------

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
        if SHOW_RAW_REPLIES or is_gyro_dump or line.startswith("ERR"):
            print(f"        <- {line}")

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


def send_and_wait(ser, cmd):
    """Write one command, wait for its reply. Returns (result, line) like wait_reply."""
    ser.write((cmd + "\n").encode())
    ser.flush()
    return wait_reply(ser, cmd)


def parse_fields(line):
    """'DONE AC20 US:203 MOVED:-3' -> {'US': '203', 'MOVED': '-3'}"""
    fields = {}
    for token in (line or "").split():
        if ":" in token:
            key, _, value = token.partition(":")
            fields[key.upper()] = value
    return fields


def _int(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def mm_to_cm_text(value):
    v = _int(value)
    return "?" if v is None else f"{v / 10:.1f} cm"


def _status_text(result):
    if result is True:
        return "ok"
    if result is False:
        return "ERROR"
    return f"TIMEOUT ({REPLY_TIMEOUT_S} s)"


def _reply_warning(line):
    """Short warning taken from a DONE line, e.g. a stalled drive."""
    words = (line or "").split()
    for word in ("STALL", "TIMEOUT"):
        if word in words:
            return f"  [STM reported {word}]"
    return ""


def describe_align(line):
    """Readable summary of an AC / RA reply."""
    if not line:
        return ""
    f = parse_fields(line)
    words = line.split()
    parts = []
    if "US" in f:
        parts.append(f"distance now {mm_to_cm_text(f['US'])}")
    if "NOOBJ" in words:
        parts.append("no object seen")
    if _int(f.get("MOVED")) is not None:
        parts.append(f"moved {_int(f['MOVED']):+d} cm")
    if _int(f.get("UNDID")) is not None:
        parts.append(f"moved back {_int(f['UNDID']):+d} cm")
    # older firmware format: D/MV in mm
    if "D" in f:
        parts.append(f"first reading {mm_to_cm_text(f['D'])}")
    if "MV" in f:
        parts.append(f"moved {mm_to_cm_text(f['MV'])}")
    return ", ".join(parts)


def run_move(ser, cmd, label):
    """Send one movement command and print one tidy line. Returns (result, line)."""
    t0 = time.time()
    result, line = send_and_wait(ser, cmd)
    dt = time.time() - t0
    print(f"      {label:<9}{cmd:<8}{_status_text(result)} ({dt:.1f} s){_reply_warning(line)}")
    time.sleep(GAP_BETWEEN_CMDS_S)
    return result, line


_align_state = {"before_mm": None, "moved_cm": None, "target_cm": None}


def read_us_mm(ser):
    """One US median reading from the STM in mm, or None if unavailable."""
    result, line = send_and_wait(ser, "US")
    if result is not True or not line:
        return None
    words = line.split()
    value = _int(words[1]) if len(words) > 1 else None
    return value if value is not None and value > 0 else None


def _cm(mm):
    return "no reading" if mm is None else f"{mm / 10:.1f} cm"


def _align_report_ac(cmd, before_mm, line):
    f = parse_fields(line)
    target = _int(cmd[2:])
    after_mm = _int(f.get("US"))
    moved = _int(f.get("MOVED"))
    _align_state.update(before_mm=before_mm, moved_cm=moved, target_cm=target)
    print(f"      [ALIGN] {cmd}: target {target} cm sensor->face (stop within +-{AC_TOL_CM} cm)")
    if before_mm is not None and target is not None:
        print(f"        before : {_cm(before_mm)}   (off by {before_mm / 10 - target:+.1f} cm)")
    else:
        print(f"        before : {_cm(before_mm)}")
    if after_mm is not None and target is not None:
        err = after_mm / 10 - target
        ok = "OK" if abs(err) < AC_TOL_CM else "OUTSIDE TOLERANCE (hit cap or sensor noise)"
        print(f"        after  : {_cm(after_mm)}   (off by {err:+.1f} cm)  {ok}")
    else:
        print("        after  : no object seen")
    if moved is not None:
        ok = "OK" if abs(moved) <= AC_MAX_TOTAL_CM else "OVER CAP!"
        way = "forward" if moved > 0 else ("back" if moved < 0 else "no move")
        print(f"        moved  : {moved:+d} cm net ({way}), cap {AC_MAX_TOTAL_CM} cm  {ok}")
        if before_mm is not None and after_mm is not None:
            seen = (before_mm - after_mm) / 10
            print(f"        check  : ultrasonic changed {seen:+.1f} cm vs wheels {moved:+d} cm"
                  f"  (diff {seen - moved:+.1f})")


def _align_report_ra(line, after_ra_mm):
    undid = _int(parse_fields(line).get("UNDID"))
    moved = _align_state["moved_cm"]
    before = _align_state["before_mm"]
    print("      [ALIGN] RA: undo the last AC")
    if undid is None:
        print("        undid  : unknown (no UNDID in reply)")
    elif moved is None:
        print(f"        undid  : {undid:+d} cm   (no AC recorded on the Pi to compare)")
    else:
        ok = "MATCH" if undid == moved else "MISMATCH!"
        print(f"        undid  : {undid:+d} cm   AC moved {moved:+d} cm  {ok}")
    if before is not None and after_ra_mm is not None:
        diff = (after_ra_mm - before) / 10
        ok = "OK" if abs(diff) < AC_TOL_CM else "robot did not return to where AC started"
        print(f"        back at: {_cm(after_ra_mm)} vs {_cm(before)} before AC   "
              f"(diff {diff:+.1f} cm)  {ok}")
    _align_state.update(before_mm=None, moved_cm=None, target_cm=None)


def run_align(ser, cmd, label):
    """
    Send AC<cm> or RA. Never stops the run: a slightly-off photo beats
    aborting everything.
    """
    upper = cmd.upper()
    is_ac = upper.startswith(ALIGN_PREFIX)
    before_mm = read_us_mm(ser) if (SHOW_ALIGN_DEBUG and is_ac) else None
    result, line = send_and_wait(ser, cmd)
    detail = describe_align(line)
    print(f"      {label:<9}{cmd:<8}{_status_text(result)}" + (f" - {detail}" if detail else ""))
    align_log.append({"cmd": cmd, "status": _status_text(result), "detail": detail})
    time.sleep(GAP_BETWEEN_CMDS_S)
    if SHOW_ALIGN_DEBUG and result is True:
        if is_ac:
            _align_report_ac(cmd, before_mm, line)
        elif upper == "RA":
            _align_report_ra(line, read_us_mm(ser))
    return result, line


# ----------------------------------------------------------------------
# Photo + error correction
# ----------------------------------------------------------------------

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


def correct_snap_angle(ser, last_turn_cmd, last_straight_cmd, on_check_position):
    """
    One error correction. FW/BW drift leaves the robot slightly to one side of
    the photo pose, so:
      1. ask the camera which side of the frame the obstacle is on
      2. undo the last straight move, then the last turn
      3. nudge FW/BW ANGLE_NUDGE_CM along that earlier heading
      4. redo the turn and the straight move (back to the photo pose, shifted)
    The nudge is never undone.

    Returns True if the robot moved, False if the correction was skipped.
    """
    if not ANGLE_CORRECTION_ENABLED:
        print("      Correction moves are off (ANGLE_CORRECTION_ENABLED = False) - skipping")
        return False
    if last_turn_cmd not in REVERSE_TURN:
        print("      No turn before this photo, nothing to undo - skipping correction move")
        return False

    side = on_check_position() if on_check_position else None
    if side not in ("left", "right"):
        print("      Camera saw nothing (cannot tell left/right) - skipping correction move")
        return False

    print(f"      Obstacle is on the {side.upper()} of the frame (last turn was {last_turn_cmd})")

    if last_straight_cmd:
        run_move(ser, reverse_straight_cmd(last_straight_cmd), "undo")
    run_move(ser, REVERSE_TURN[last_turn_cmd], "undo")

    nudge_dir = TURN_CORRECTION_NUDGE.get((last_turn_cmd, side))
    if nudge_dir:
        run_move(ser, f"{nudge_dir}{ANGLE_NUDGE_CM}", "nudge")

    run_move(ser, last_turn_cmd, "replay")
    if last_straight_cmd:
        run_move(ser, last_straight_cmd, "replay")
    return True


def _snap_takes_attempt(fn):
    """True if on_snap accepts (obstacle_id, attempt, final)."""
    try:
        params = list(inspect.signature(fn).parameters.values())
    except (TypeError, ValueError):
        return False
    if any(p.kind == p.VAR_POSITIONAL for p in params):
        return True
    positional = [p for p in params if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)]
    return len(positional) >= 3


def _snap_takes_label(fn):
    """True if on_snap is the v5 camera (accepts label= and plan=)."""
    try:
        return "label" in inspect.signature(fn).parameters
    except (TypeError, ValueError):
        return False


def empty_snap(label, attempt, error=None):
    """A snap dict with nothing in it (same shape trigger_camera returns)."""
    return {"label": label, "attempt": attempt, "name": f"Snap{attempt}_{label}",
            "file": None, "targets": [], "ignored": [], "infer_ms": None, "error": error}


def _as_snap(raw, label, attempt):
    """Accept a v5 snap dict, or a v4-style True/False, and return a snap dict."""
    if isinstance(raw, dict):
        snap = empty_snap(label, attempt)
        snap.update(raw)
        snap["targets"] = sorted(snap.get("targets") or [], key=lambda d: -d["conf"])
        return snap
    snap = empty_snap(label, attempt, None if raw else "camera handler reported no image")
    if raw:   # old callback: True only says "recognised", no letter or confidence
        snap["targets"] = [{"letter": "?", "image_id": "?", "conf": 1.0, "cx": None, "w": None}]
    return snap


# ---- diagnosis helpers ------------------------------------------------

def ideal_case(targets):
    """
    The CASE the boxes on the IDEAL photo lead to, and a one-line reason.
    photo_cycle() and the photo overlay both use this, so they always agree.
    """
    n = len(targets)
    if n == 1 and targets[0]["conf"] >= RECOGNISED_CONF:
        return "A", f"1 image, conf {targets[0]['conf']:.2f} >= {RECOGNISED_CONF:.2f}"
    if n == 1:
        return "B", f"1 image but conf {targets[0]['conf']:.2f} < {RECOGNISED_CONF:.2f}"
    if n == 0:
        return "C", "no image ID found"
    ids = ", ".join(f"ID {t['image_id']} {t['conf']:.2f}" for t in targets)
    return "D", f"{n} images ({ids})"


def photo_position(label, ideal_cm, bound=None):
    """
    Where a photo is taken, relative to the planned (IDEAL) photo pose.
    FAR / NEAR keep their label even when clearance cut the move short;
    "short" and the tag ("FAR-short") say so.
        {"label", "tag", "sensor_cm", "wanted_cm", "move", "short", "limit", "text"}
    """
    if label == "IDEAL" or not bound:
        cm = ideal_cm
        return {"label": label, "tag": label, "sensor_cm": cm, "wanted_cm": cm,
                "move": "", "short": False, "limit": "",
                "text": f"IDEAL - planned photo pose, sensor->face {_cm_text(cm)}"}
    wanted = config.SNAP_FAR_MAX_SENSOR_CM if label == "FAR" else config.SNAP_NEAR_MIN_SENSOR_CM
    limit = bound.get("blocked_by") or bound.get("limited_by", "")
    short = bound.get("short", not str(bound.get("limited_by", "")).startswith("SNAP_"))
    cm = bound.get("sensor_to_face_cm")
    move = bound.get("command", "")
    text = (f"{label} - {move} from IDEAL ({_cm_text(ideal_cm)}), sensor->face {_cm_text(cm)}"
            + (f"  [wanted {wanted:g} cm, CUT SHORT by {limit}]" if short
               else f"  [full move, reached {wanted:g} cm]"))
    return {"label": label, "tag": label, "sensor_cm": cm,
            "wanted_cm": wanted, "move": move, "short": short, "limit": limit, "text": text}


def _iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / union if union > 0 else 0.0


def box_hints(snap):
    """
    For a photo with 2+ target boxes: why is each EXTRA box (2nd, 3rd ...) there?
    Returns a list of strings, one per extra box. Empty when count <= 1.
    """
    t = snap.get("targets") or []
    if len(t) < 2:
        return []
    top = t[0]
    width = snap.get("frame_w")
    hints = []
    for i, d in enumerate(t[1:], 2):
        why = []
        if top.get("xyxy") and d.get("xyxy"):
            iou = _iou(top["xyxy"], d["xyxy"])
            if iou >= 0.30:
                why.append(f"overlaps box 1 (IoU {iou:.2f}) -> same card read twice")
            else:
                why.append(f"separate from box 1 (IoU {iou:.2f})")
        if str(d["image_id"]) == str(top["image_id"]):
            why.append("same ID as box 1 (duplicate)")
        if d["conf"] < 0.40:
            why.append(f"weak ({d['conf']:.2f} < 0.40)")
        if width and d.get("cx") is not None:
            off = abs(d["cx"] - width / 2) / width
            if off > 0.30:
                why.append(f"near frame edge ({d['cx']:.0f}/{width} px) -> other obstacle?")
        hints.append(f"box {i} ID {d['image_id']} {d['conf']:.2f}: " + "; ".join(why))
    return hints


def _photo_record(s):
    """Compact record of one photo for the decision / run log."""
    pos = s.get("position") or {}
    return {"name": s["name"], "tag": pos.get("tag", s["label"]), "label": s["label"],
            "sensor_cm": pos.get("sensor_cm"), "wanted_cm": pos.get("wanted_cm"),
            "short": pos.get("short", False), "limit": pos.get("limit", ""),
            "why": s.get("why"), "count": len(s["targets"]),
            "boxes": [(d["image_id"], d["letter"], round(d["conf"], 2)) for d in s["targets"]],
            "hints": box_hints(s), "file": s.get("file"), "error": s.get("error"),
            "move_explain": explain_bound(s["bound"], "") if s.get("bound") else []}


# ---- printing ---------------------------------------------------------

def _pose_text(pose):
    if not pose:
        return "unknown"
    return f"({float(pose[0]):.1f}, {float(pose[1]):.1f}) facing {pose[2]}"


def _cm_text(value):
    return "?" if value is None else f"{float(value):.1f} cm"


def print_photo_header(obstacle_id, target):
    """What the robot is about to photograph, and from where."""
    print()
    print("    " + WIDE)
    print(f"    OBSTACLE {obstacle_id}  -  PHOTO CYCLE")
    print("    " + WIDE)
    if not target:
        print("    Plan info     : none for this obstacle (desired pose unknown)")
        print("                    FAR / NEAR retry photos are DISABLED without it")
        return
    cell, centre = target.get("obstacle_cell"), target.get("obstacle_centre_cm")
    face = target.get("face", "?")
    face_text = ""
    if centre and face in config.DIRECTION_STEP:
        nx, ny = config.DIRECTION_STEP[face]
        half = config.CELL_CM / 2
        face_text = f", face centre ({centre[0] + nx*half:.1f}, {centre[1] + ny*half:.1f}) cm"
    print(f"    Obstacle      : cell ({cell[0]}, {cell[1]}), image on {face} face{face_text}")
    print(f"    Desired pose  : {_pose_text(target.get('goal_view_pose'))}   (robot centre, cm)")
    lat = target.get("lateral_offset_cm", 0.0)
    print(f"    Sensor->face  : {_cm_text(target.get('expected_sensor_to_face_cm'))} planned"
          f"   lateral offset {lat:+.1f} cm")
    print_view_preference(target)
    retry = target.get("retry") or {}
    for key in ("far", "near"):
        b = retry.get(key)
        print(f"    Retry {key.upper():<4}    : (only used for CASE {'B' if key == 'far' else 'B / D'})")
        for line in explain_bound(b, "        "):
            print(line)


def print_view_preference(target, indent="    "):
    """Why the planner picked this distance: preferred distance and penalty paid."""
    pref = target.get("preferred_sensor_to_face_cm")
    if pref is None:
        if "preferred_sensor_to_face_cm" in target:
            print(f"{indent}Preference    : none (every distance costs the same)")
        return
    d = target.get("expected_sensor_to_face_cm")
    dist_pen, lat_pen = target.get("distance_penalty", 0.0), target.get("lateral_penalty", 0.0)
    if d is None:
        where = "?"
    elif abs(d - pref) < 0.5:
        where = "ON the preferred distance"
    elif d > pref:
        where = f"{d - pref:.0f} cm FURTHER than preferred (backward, {config.VIEW_FURTHER_COST_PER_CM:g}/cm)"
    else:
        where = f"{pref - d:.0f} cm CLOSER than preferred (forward, {config.VIEW_NEARER_COST_PER_CM:g}/cm)"
    print(f"{indent}Preference    : {pref:g} cm preferred -> {where}")
    print(f"{indent}View penalty  : {dist_pen + lat_pen:.1f} = distance {dist_pen:.1f} + lateral {lat_pen:.1f}"
          "   (0 = best possible pose)")


def print_snap(snap, desired_pose=None, sensor_cm=None):
    """One photo: where it was meant to be taken and everything the model saw."""
    print()
    pos = snap.get("position") or {}
    title = f"-- Snap {snap['attempt']}  {pos.get('tag', snap['label'])} "
    print("    " + title + "-" * max(4, 64 - len(title)))
    if pos.get("text"):
        print(f"    Position      : {pos['text']}")
    if snap.get("why"):
        print(f"    Taken because : {snap['why']}")
    if desired_pose is not None or sensor_cm is not None:
        print(f"    Desired pose  : {_pose_text(desired_pose)}   sensor->face {_cm_text(sensor_cm)}")
    if snap.get("file"):
        print(f"    Photo         : {snap['file']}")
    if snap.get("error"):
        print(f"    Camera        : {snap['error']}")
    ms = snap.get("infer_ms")
    boxes = len(snap["targets"]) + len(snap.get("ignored") or [])
    print(f"    Model saw     : {boxes} box(es) at conf >= {config.MODEL_MIN_CONF:.2f}"
          + (f"   ({ms:.0f} ms)" if ms is not None else ""))
    i = 0
    for d in snap["targets"]:
        i += 1
        where = ""
        if d.get("cx") is not None:
            where = f"   centre x {d['cx']:.0f} px, width {d['w']:.0f} px"
        print(f"        {i}. {str(d['letter']):<13} ID {str(d['image_id']):<3} conf {d['conf']:.2f}{where}")
    for d in snap.get("ignored") or []:
        i += 1
        print(f"        {i}. {str(d['letter']):<13} conf {d['conf']:.2f}   (not an image ID - not counted)")
    n = len(snap["targets"])
    print(f"    Count         : {n} target image{'s' if n != 1 else ''}")
    for h in box_hints(snap):
        print(f"    Extra box     : {h}")
    if snap["label"] == "IDEAL" and snap.get("case"):
        print(f"    IDEAL -> CASE : {snap['case']} ({snap.get('case_reason', '')})")


def vote_across_snaps(snaps):
    """
    CASE B ranking. For every image ID seen in any photo:
        seen  = in how many photos it appears (its best box per photo)
        avg   = average of those confidences
        best  = highest of those confidences, and which photo it came from
    Ranked by seen (most photos first), then avg, then best.
    """
    table = {}
    for snap in snaps:
        per_photo = {}
        for d in snap["targets"]:
            key = str(d["image_id"])
            if key not in per_photo or d["conf"] > per_photo[key]["conf"]:
                per_photo[key] = d
        for key, d in per_photo.items():
            row = table.setdefault(key, {"image_id": d["image_id"], "letter": d["letter"], "hits": []})
            row["hits"].append((d["conf"], snap))
    rows = []
    for row in table.values():
        confs = [c for c, _ in row["hits"]]
        best_conf, best_snap = max(row["hits"], key=lambda h: h[0])
        rows.append({"image_id": row["image_id"], "letter": row["letter"], "seen": len(confs),
                     "avg": sum(confs) / len(confs), "best": best_conf, "best_snap": best_snap})
    rows.sort(key=lambda r: (-r["seen"], -r["avg"], -r["best"]))
    return rows


def rank_by_conf(snaps):
    """CASE B: every target box from every photo, highest confidence first."""
    rows = [{"snap": sp, "det": d} for sp in snaps for d in sp["targets"]]
    rows.sort(key=lambda r: -r["det"]["conf"])
    return rows


def print_ranking(rows, photos):
    print()
    print(f"    RANKING over {photos} photo(s)  (every box, highest confidence first)")
    print(f"        {'#':<3}{'photo':<14}{'dist':>8}  {'ID':<5}{'image':<14}conf")
    for i, r in enumerate(rows, 1):
        pos = r["snap"].get("position") or {}
        dist = _cm_text(pos.get("sensor_cm"))
        mark = "  <- WINNER" if i == 1 else ""
        print(f"        {i:<3}{r['snap']['name']:<14}{dist:>8}  {str(r['det']['image_id']):<5}"
              f"{str(r['det']['letter']):<14}{r['det']['conf']:.2f}{mark}")


def explain_bound(b, indent="    "):
    """
    Lines describing how the planner chose a FAR / NEAR move: what it wanted,
    every distance it tried that broke clearance (grouped), and what it used.
    """
    if not b:
        return [f"{indent}not in plan (old algo server?) - photo disabled"]
    name = b.get("name", "?")
    cmd = "BW" if name == "FAR" else "FW"
    ideal = b.get("ideal_sensor_to_face_cm")
    wanted_s = b.get("wanted_sensor_to_face_cm")
    wanted_m = b.get("wanted_move_cm")
    out = []
    if wanted_m is not None:
        out.append(f"{indent}wanted {cmd}{wanted_m}: sensor->face {_cm_text(ideal)} -> {wanted_s:g} cm")
    tries = b.get("tries")
    if tries is None:          # plan from an older planner.py
        if b.get("available"):
            out.append(f"{indent}using {b['command']} -> {_cm_text(b['sensor_to_face_cm'])}  "
                       f"[limit: {b.get('limited_by', '?')}]  (update planner.py on the laptop for details)")
        else:
            out.append(f"{indent}{b.get('reason', 'no room')}")
        return out
    groups = []                # consecutive blocked tries with the same cause
    for t in tries:
        if t["ok"]:
            continue
        who = (t["problem"] or "").split(":")[0]
        if groups and groups[-1]["who"] == who:
            groups[-1]["last"] = t
        else:
            groups.append({"who": who, "first": t, "last": t})
    for g in groups:
        f, l = g["first"], g["last"]
        if f is l:
            out.append(f"{indent}{cmd}{f['move_cm']} ({f['sensor_to_face_cm']:g} cm) BLOCKED - {f['problem']}")
        else:
            out.append(f"{indent}{cmd}{f['move_cm']}..{cmd}{l['move_cm']} ({f['sensor_to_face_cm']:g}"
                       f"..{l['sensor_to_face_cm']:g} cm) BLOCKED - {g['who']}")
            out.append(f"{indent}    closest one ({cmd}{l['move_cm']}): {l['problem']}")
    if b.get("available"):
        gap = (wanted_s - b["sensor_to_face_cm"]) if name == "FAR" else (b["sensor_to_face_cm"] - wanted_s)
        if b.get("short"):
            out.append(f"{indent}-> using {b['command']}: sensor->face {_cm_text(b['sensor_to_face_cm'])} "
                       f"({abs(gap):.0f} cm short of {wanted_s:g}, obeys clearance)")
        else:
            out.append(f"{indent}-> using {b['command']}: sensor->face {_cm_text(b['sensor_to_face_cm'])} "
                       "(full move, nothing in the way)")
    else:
        out.append(f"{indent}-> {name} photo SKIPPED: {b.get('reason', 'no room')}")
    return out


def print_move_plan(b, from_offset=0):
    """Printed just before the robot drives to FAR / NEAR."""
    print(f"    Going {b.get('name', '?')} :")
    for line in explain_bound(b, "        "):
        print(line)
    if from_offset:
        net = b["move_cm"] - from_offset if b.get("name") == "NEAR" else -b["move_cm"] - from_offset
        print(f"        (robot is {abs(from_offset)} cm {'back' if from_offset < 0 else 'forward'} "
              f"of IDEAL now, so it drives {'FW' if net > 0 else 'BW'}{abs(net)})")


def print_vote(rows, photos):
    print()
    print(f"    VOTE over {photos} photo(s)  (rank: most photos seen in, then average conf)")
    print(f"        {'#':<3}{'ID':<5}{'image':<14}{'seen':<7}{'avg':<7}best (photo)")
    for i, r in enumerate(rows, 1):
        print(f"        {i:<3}{str(r['image_id']):<5}{str(r['letter']):<14}"
              f"{r['seen']}/{photos:<5}{r['avg']:<7.2f}{r['best']:.2f} ({r['best_snap']['name']})")


def _decision(case, reason, snap=None, det=None):
    return {"case": case, "reason": reason,
            "image_id": det["image_id"] if det else None,
            "letter": det["letter"] if det else None,
            "conf": det["conf"] if det else 0.0,
            "snap": snap["name"] if (snap and det) else None,
            "file": snap.get("file") if (snap and det) else None}


# ---- the photo cycle --------------------------------------------------

def take_snap(on_snap, obstacle_id, attempt, total, label="IDEAL", plan=None):
    """Call the camera once and return a snap dict (never raises)."""
    if on_snap is None:
        return empty_snap(label, attempt, "no camera handler - nothing captured")
    try:
        if _snap_takes_label(on_snap):
            raw = on_snap(obstacle_id, attempt, attempt == total, label=label, plan=plan)
        elif _snap_takes_attempt(on_snap):
            raw = on_snap(obstacle_id, attempt, attempt == total)
        else:
            raw = on_snap(obstacle_id)
    except Exception as error:          # a camera fault must not stop the run
        return empty_snap(label, attempt, f"camera handler failed: {error}")
    return _as_snap(raw, label, attempt)


def photo_cycle(ser, obstacle_id, last_turn_cmd, last_straight_cmd, on_snap, on_check_position,
                planned_align_pending=False, target=None, on_result=None):
    """
    v5 photo rules (CASE A/B/C/D, see the module docstring). The robot only
    ever drives straight along the photo heading for the extra photos and
    always comes back to the photo pose, so the rest of the plan is unchanged.

    target    : the planner's segment["target"] for this obstacle (desired pose,
                FAR / NEAR bounds). Without it no extra photos are taken.
    on_result : on_result(obstacle_id, decision) - sends the result to Android.

    planned_align_pending: the plan ran its own AC just before this SNAP and its
    RA has not run yet. If extra photos are needed, that AC is undone (RA) first
    so the FAR / NEAR distances are measured from the planned photo pose.

    Returns (recognised, planned_align_undone).
    """
    oid = str(obstacle_id)
    target = target or {}
    retry = target.get("retry") or {}
    far, near = retry.get("far") or {}, retry.get("near") or {}
    planned_align_undone = False
    snaps = []
    offset = 0            # cm driven forward (+) / back (-) from the photo pose
    moves_ok = True

    def drive_to(new_offset, why):
        nonlocal offset, moves_ok, planned_align_undone
        if planned_align_pending and not planned_align_undone:
            run_align(ser, "RA", "undo AC")
            planned_align_undone = True
        delta = new_offset - offset
        if delta == 0:
            return True
        result, _ = run_move(ser, f"{'FW' if delta > 0 else 'BW'}{abs(delta)}", why)
        if result is not True:
            moves_ok = False
            print("    !! Move failed: no more extra photos for this obstacle. The robot may")
            print("       no longer be on its planned photo pose.")
            return False
        offset = new_offset
        return True

    ideal_cm = target.get("expected_sensor_to_face_cm")
    skipped = []          # FAR / NEAR photos that were wanted but had no room

    def snap(label, total, desired, sensor_cm, bound=None, case=None, why=None):
        pos = photo_position(label, ideal_cm, bound)
        plan = {"pose": desired, "sensor_to_face_cm": sensor_cm, "position": pos,
                "case": case, "why": why,
                "classify": ideal_case if label == "IDEAL" else None}
        s = take_snap(on_snap, oid, len(snaps) + 1, total, label, plan=plan)
        s["position"] = pos
        s["why"] = why
        s["bound"] = bound
        if label == "IDEAL":
            s["case"], s["case_reason"] = ideal_case(s["targets"])
        else:
            s.setdefault("case", case)
        if not s.get("file"):           # no camera: still name it by position
            s["name"] = f"Snap{s['attempt']}_{pos['tag']}"
        snaps.append(s)
        print_snap(s, desired, sensor_cm)
        return s

    print_photo_header(oid, target)

    # ---- Snap 1: IDEAL -------------------------------------------------
    s1 = snap("IDEAL", 1, target.get("goal_view_pose"), target.get("expected_sensor_to_face_cm"))
    n = len(s1["targets"])
    print()

    if n == 1 and s1["targets"][0]["conf"] >= RECOGNISED_CONF:
        d = s1["targets"][0]
        print(f"    DECISION      : CASE A - 1 image, conf {d['conf']:.2f} >= {RECOGNISED_CONF:.2f}"
              "  ->  done, no extra photos")
        decision = _decision("A", "1 image, confident on the IDEAL photo", s1, d)

    elif n == 1:
        d = s1["targets"][0]
        plan_far, plan_near = bool(far.get("available")), bool(near.get("available"))
        total = 1 + plan_far + plan_near
        print(f"    DECISION      : CASE B - 1 image but conf {d['conf']:.2f} < {RECOGNISED_CONF:.2f}"
              "  ->  also take FAR and NEAR, then report the highest confidence of all photos")
        if not plan_far:
            print(f"                    FAR photo skipped: {far.get('reason', 'no plan info')}")
            skipped.append(f"FAR skipped: {far.get('reason', 'no plan info')}")
        if not plan_near:
            print(f"                    NEAR photo skipped: {near.get('reason', 'no plan info')}")
            skipped.append(f"NEAR skipped: {near.get('reason', 'no plan info')}")
        if total < 3:
            print(f"                    -> only {total} photo(s) for this CASE B")
        print()
        why_b = f"CASE B - IDEAL had 1 image, conf {d['conf']:.2f} < {RECOGNISED_CONF:.2f}"
        if plan_far:
            print_move_plan(far)
            if drive_to(-far["move_cm"], "to FAR"):
                snap("FAR", total, far["pose"], far["sensor_to_face_cm"], far, "B", why_b)
                print()
        if plan_near and moves_ok:
            print_move_plan(near, from_offset=offset)
            if drive_to(near["move_cm"], "to NEAR"):
                snap("NEAR", total, near["pose"], near["sensor_to_face_cm"], near, "B", why_b)
                print()
        if moves_ok:
            drive_to(0, "back")
        rows = rank_by_conf(snaps)
        print_ranking(rows, len(snaps))
        w = rows[0]
        why = (f"highest confidence of {len(rows)} box(es) over {len(snaps)} photo(s): "
               f"ID {w['det']['image_id']} {w['det']['conf']:.2f} on {w['snap']['name']}")
        print(f"    Winner        : {why}")
        decision = _decision("B", why, w["snap"], w["det"])
        decision["ranking"] = [(r["snap"]["name"], (r["snap"].get("position") or {}).get("sensor_cm"),
                                r["det"]["image_id"], r["det"]["letter"], round(r["det"]["conf"], 2))
                               for r in rows]

    elif n == 0:
        print("    DECISION      : CASE C - no image found  ->  report nothing (no extra photos)")
        decision = _decision("C", "no image on the IDEAL photo")

    else:
        ids = ", ".join(f"ID {t['image_id']} ({t['conf']:.2f})" for t in s1["targets"])
        print(f"    DECISION      : CASE D - {n} images on IDEAL  ->  held, move NEAR, "
              "rank IDEAL + NEAR boxes, report the top one")
        for i, t in enumerate(s1["targets"], 1):
            print(f"                    IDEAL box {i}: ID {t['image_id']} ({t['letter']}) conf {t['conf']:.2f}")
        for h in box_hints(s1):
            print(f"                    {h}")
        why_d = f"CASE D - IDEAL had {n} images ({ids})"
        s2, no_near = None, None
        if not near.get("available"):
            no_near = f"no room for a NEAR photo ({near.get('reason', 'no plan info')})"
            print(f"                    NEAR photo skipped: {near.get('reason', 'no plan info')}")
            skipped.append(f"NEAR skipped: {near.get('reason', 'no plan info')}")
        else:
            print()
            print_move_plan(near)
            if drive_to(near["move_cm"], "to NEAR"):
                s2 = snap("NEAR", 2, near["pose"], near["sensor_to_face_cm"], near, "D", why_d)
                print()
                drive_to(0, "back")
            else:
                no_near = "could not drive to the NEAR pose"
        print()
        if no_near:
            print(f"    NEAR result   : {no_near} - ranking the IDEAL boxes only")
        rows = rank_by_conf(snaps)          # IDEAL boxes + NEAR boxes, highest conf first
        print_ranking(rows, len(snaps))
        w = rows[0]
        why = (f"highest confidence of {len(rows)} box(es) over {len(snaps)} photo(s): "
               f"ID {w['det']['image_id']} {w['det']['conf']:.2f} on {w['snap']['name']}"
               + (f" ({no_near})" if no_near else ""))
        print(f"    Winner        : {why}")
        decision = _decision("D", why, w["snap"], w["det"])
        decision["ranking"] = [(r["snap"]["name"], (r["snap"].get("position") or {}).get("sensor_cm"),
                                r["det"]["image_id"], r["det"]["letter"], round(r["det"]["conf"], 2))
                               for r in rows]

    # ---- result ----------------------------------------------------------
    decision["photos"] = [_photo_record(s) for s in snaps]
    decision["skipped"] = skipped
    decision["ideal_case_reason"] = snaps[0].get("case_reason") if snaps else None
    print()
    print(f"    PHOTOS TAKEN  : {len(snaps)}  ->  "
          + "  |  ".join(f"{p['name']} {_cm_text(p['sensor_cm'])} count {p['count']}"
                         for p in decision["photos"]))
    for line in skipped:
        print(f"                    {line}")
    if decision["image_id"] is not None:
        print(f"    RESULT        : ID {decision['image_id']} ({decision['letter']}), conf "
              f"{decision['conf']:.2f}, from {decision['snap']}")
        if decision["file"]:
            print(f"                    {decision['file']}")
    else:
        print(f"    RESULT        : NOTHING  ({decision['reason']})")
    if on_result is not None:
        on_result(oid, decision)
    else:
        msg = (f"TARGET,{oid},{decision['image_id']}" if decision["image_id"] is not None
               else f"MSG,Obstacle {oid}: no image recognised")
        print(f"    Android       : (no Android here) would send {msg}")
    print("    " + WIDE)

    recognised = decision["image_id"] is not None
    snap_log.append({"obstacle": oid, "case": decision["case"], "snaps": len(snaps),
                     "recognised": recognised, "decision": decision})
    return recognised, planned_align_undone


def print_photo_summary():
    """Short table of every obstacle's photo outcome (standalone / simulator)."""
    if not snap_log:
        return
    print()
    print(WIDE)
    print("  PHOTO SUMMARY")
    print(WIDE)
    for e in snap_log:
        d = e["decision"]
        result = (f"ID {d['image_id']} ({d['letter']}, {d['conf']:.2f}) from {d['snap']}"
                  if e["recognised"] else "nothing")
        print(f"  Obstacle {e['obstacle']:<3} CASE {e['case']}  {e['snaps']} photo(s)  ->  {result}")
        for p in d.get("photos", []):
            print(f"      {p['name']:<22} {_cm_text(p['sensor_cm']):>8}  count {p['count']}  "
                  + (", ".join(f"ID {b[0]} {b[2]:.2f}" for b in p["boxes"]) or "-"))
    print(WIDE)


# ----------------------------------------------------------------------
# Run
# ----------------------------------------------------------------------

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


def split_legs(cmds):
    """Split the flat command list into legs, each ending at a SNAP<id>."""
    legs, current = [], []
    for cmd in cmds:
        current.append(cmd)
        if cmd.upper().startswith("SNAP"):
            legs.append(current)
            current = []
    if current:
        legs.append(current)
    return legs


def run_commands(cmds, on_snap=None, on_check_position=None, targets=None, on_result=None):
    """
    Run the planned commands leg by leg. SNAP<id> runs photo_cycle() instead
    of being sent to the STM.

    targets   : {obstacle id (str): planner segment "target"} - desired photo
                pose and FAR / NEAR bounds. Missing -> no extra photos.
    on_result : on_result(obstacle_id, decision), e.g. send TARGET to Android.

    Returns True if the whole sequence completed, False if a planned move
    failed or timed out while STOP_ON_ERROR is set.
    """
    ser = open_serial()
    completed = True
    align_log.clear()
    snap_log.clear()

    # Last planned turn / straight actually completed, for error correction.
    last_turn_cmd = None
    last_straight_cmd = None

    # Planned AC sent and its planned RA not yet run (see photo_cycle).
    planned_align_pending = False
    skip_planned_ra = False

    legs = split_legs(cmds)
    total = len(cmds)
    index = 0
    where = "Start"

    try:
        for leg_no, leg in enumerate(legs, 1):
            last = leg[-1].upper()
            dest = f"Obstacle {leg[-1][4:]}" if last.startswith("SNAP") else "End"
            moves = [c for c in leg if not c.upper().startswith("SNAP")]

            print()
            print(RULE)
            print(f"  Leg {leg_no}/{len(legs)}:  {where} -> {dest}")
            print(f"  Commands: {' '.join(moves) if moves else '(none - already in place)'}")
            print(RULE)

            for cmd in leg:
                index += 1
                upper = cmd.upper()

                if upper.startswith("SNAP"):
                    _ok, undone = photo_cycle(ser, cmd[4:], last_turn_cmd, last_straight_cmd,
                                              on_snap, on_check_position, planned_align_pending,
                                              target=(targets or {}).get(str(cmd[4:])),
                                              on_result=on_result)
                    if undone:
                        planned_align_pending = False
                        skip_planned_ra = True
                    continue

                label = f"[{index}/{total}]"
                skip_this_ra = skip_planned_ra and upper == "RA"
                skip_planned_ra = False          # only the RA straight after that SNAP
                if skip_this_ra:
                    print(f"      {label:<9}{cmd:<8}SKIPPED - planned align already undone before the extra photos")
                    continue

                if upper.startswith(ALIGN_PREFIX) or upper == "RA":
                    run_align(ser, cmd, label)
                    planned_align_pending = upper.startswith(ALIGN_PREFIX)
                    continue

                result, _line = run_move(ser, cmd, label)
                if result is True:
                    if upper in REVERSE_TURN:
                        last_turn_cmd = upper
                        last_straight_cmd = None
                    elif upper.startswith("FW") or upper.startswith("BW"):
                        last_straight_cmd = cmd
                elif STOP_ON_ERROR:
                    completed = False
                    break

            if not completed:
                break
            where = dest

    finally:
        ser.close()

    if on_result is None:
        print_photo_summary()
    print()
    print("Finished." if completed else "Stopped early: a planned move failed or timed out.")
    return completed


def targets_path(cmds_path):
    """Where rpi_client_android.py saves the plan's photo targets for a cmds file."""
    return cmds_path + ".targets.json"


def read_targets_file(cmds_path):
    """{obstacle id: target} saved next to a cmds file, or {} if there is none."""
    import json
    import os
    path = targets_path(cmds_path)
    if not os.path.exists(path):
        print(f"(no {path}: desired poses unknown, FAR / NEAR photos disabled)")
        return {}
    with open(path) as f:
        return json.load(f)


def load_commands():
    """Returns (commands, targets)."""
    args = sys.argv[1:]
    if not args:
        return COMMANDS, {}
    if args[0] == "-f":
        return read_cmds_file(args[1]), read_targets_file(args[1])
    return args, {}


def main():
    cmds, targets = load_commands()
    run_commands(cmds, targets=targets)


if __name__ == "__main__":
    main()
