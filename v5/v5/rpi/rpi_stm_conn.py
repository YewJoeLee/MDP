#!/usr/bin/env python3
"""
Send a sequence of commands to the STM32 over /dev/ttyACM0.

Standalone usage:
    python3 rpi_stm_conn.py                  # runs the COMMANDS list below
    python3 rpi_stm_conn.py FW50 RT90 FW30   # runs commands given on the command line
    python3 rpi_stm_conn.py AC20             # test the ultrasonic align on its own
    python3 rpi_stm_conn.py -f path.txt      # runs commands from a file (one per line)

As a module (used by rpi_client_android_v2.py):
    import rpi_stm_conn
    cmds = rpi_stm_conn.read_cmds_file(path)
    rpi_stm_conn.run_commands(cmds, on_snap=camera_fn, on_check_position=side_fn)

The algo server only sends the ideal path plus SNAP<id>. Everything around a
photo happens here on the RPi. SNAP<id> is never written to the STM:

    Snap 1                       photo straight away
    if not recognised, up to MAX_CORRECTIONS times:
        Error correction k       camera says which side the obstacle is on;
                                 undo the last straight move and the last turn,
                                 nudge FW/BW ANGLE_NUDGE_CM, redo the turn and
                                 the straight move. The nudge is kept (it IS the
                                 correction).
        AC20                     ultrasonic align
        Snap k+1
        RA                       undo exactly what AC20 moved
    stop as soon as a snap is recognised -> at most 1 + MAX_CORRECTIONS photos.

on_snap(obstacle_id, attempt, final) returns True if recognised. A callback
that only takes (obstacle_id) is still supported (the simulator uses one).

STM reply to AC<cm>:  DONE AC20 US:<mm> MOVED:<cm>  or  DONE AC20 NOOBJ MOVED:<cm>
STM reply to RA:      DONE RA UNDID:<cm>
"""

import inspect
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "algo"))
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

APPROACH_CMD = "AC15"     # ultrasonic align before the retry snap; None to disable
RETURN_CMD = "RA"         # undo the align after those snaps; None to disable

# ---- error correction (runs only when a snap is not recognised) ----
ANGLE_CORRECTION_ENABLED = True   # False: no undo/nudge/redo, still AC + retry snap
MAX_CORRECTIONS = 1 if config.ERROR_CORRECTION_ENABLED else 0
ANGLE_NUDGE_CM = 10               # how far each nudge creeps forward/back

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
snap_log = []    # one entry per obstacle: {"obstacle", "snaps", "recognised"}


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

        if line.startswith("ERR"):
            return False, line
        if line.startswith("DONE"):
            fields = line.split()
            if len(fields) < 2 or fields[1].upper() != cmd.upper():
                continue  # Ignore a stale completion from another command.
            failed = any(token in fields for token in ("STALL", "TIMEOUT", "IMUFAIL", "ABORT", "HEADINGERR"))
            return not failed, line
        if line.startswith(DONE_PREFIXES[2:]) and cmd[:2].upper() in ("BI", "YW", "US", "IR", "SU"):
            return True, line

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
    print(f"[MOVE][{label.upper()}] Sending {cmd}; waiting for STM completion...", flush=True)
    t0 = time.time()
    result, line = send_and_wait(ser, cmd)
    dt = time.time() - t0
    print(f"[STM] {line if line else 'No completion reply received'}", flush=True)
    print(f"[MOVE][{label.upper()}] {cmd}: {_status_text(result)} ({dt:.1f} s){_reply_warning(line)}", flush=True)
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


def take_snap(on_snap, obstacle_id, attempt, total):
    print(f"\n    Snap {attempt}/{total}  (obstacle {obstacle_id})")
    if on_snap is None:
        print("      No camera handler - skipping")
        return False
    if _snap_takes_attempt(on_snap):
        return bool(on_snap(obstacle_id, attempt, attempt == total))
    return bool(on_snap(obstacle_id))


def photo_cycle(ser, obstacle_id, last_turn_cmd, last_straight_cmd, on_snap, on_check_position,
                planned_align_pending=False):
    """
    Snap, then up to MAX_CORRECTIONS x (correct, AC, snap, RA).

    planned_align_pending: the plan ran its own AC just before this SNAP and its
    RA has not run yet. The STM remembers only the latest AC, so if the first
    snap fails that planned AC is undone here (RA) BEFORE the corrections start,
    otherwise the retry's AC would overwrite it and it would never be undone.

    Returns (recognised, planned_align_undone).
    """
    total = 1 + MAX_CORRECTIONS
    attempt = 1
    planned_align_undone = False
    success = take_snap(on_snap, obstacle_id, attempt, total)

    if not success and attempt < total and planned_align_pending:
        print()
        run_align(ser, "RA", "undo plan")
        planned_align_undone = True

    while not success and attempt < total:
        print(f"\n    Error correction {attempt}/{MAX_CORRECTIONS}  (obstacle {obstacle_id})")
        correct_snap_angle(ser, last_turn_cmd, last_straight_cmd, on_check_position)
        if APPROACH_CMD:
            run_align(ser, APPROACH_CMD, "align")
        attempt += 1
        success = take_snap(on_snap, obstacle_id, attempt, total)
        if APPROACH_CMD and RETURN_CMD:
            print()
            run_align(ser, RETURN_CMD, "return")

    outcome = (f"recognised on snap {attempt}" if success
               else f"NOT recognised after {attempt} snap(s)")
    print(f"\n    >> Obstacle {obstacle_id}: {outcome}")
    snap_log.append({"obstacle": obstacle_id, "snaps": attempt, "recognised": success})
    return success, planned_align_undone


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


def run_commands(cmds, on_snap=None, on_check_position=None):
    """
    Run the planned commands leg by leg. SNAP<id> runs photo_cycle() instead
    of being sent to the STM.

    Returns True if the whole sequence completed, False if a planned move
    failed or timed out while STOP_ON_ERROR is set.
    """
    if any(c.upper().startswith("SNAP") for c in cmds):
        raise ValueError("V5 image runs require run_plan_v5 and checked backup metadata")
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
                                              on_snap, on_check_position, planned_align_pending)
                    if undone:
                        planned_align_pending = False
                        skip_planned_ra = True
                    continue

                label = f"[{index}/{total}]"
                skip_this_ra = skip_planned_ra and upper == "RA"
                skip_planned_ra = False          # only the RA straight after that SNAP
                if skip_this_ra:
                    print(f"      {label:<9}{cmd:<8}SKIPPED - planned align already undone before error correction")
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

    print()
    print("Finished." if completed else "Stopped early: a planned move failed or timed out.")
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


def run_plan_v5(reply, on_snap, replan, on_not_found, serial_factory=None):
    """No angle correction, AC or undo/replay in the V5 execution path."""
    import recovery_config as policy
    def log(tag, message):
        print(f"[{tag}] {message}", flush=True)
    pending = list(reply["segments"])
    log("RUN", f"Starting V5: {len(pending)} planned obstacles; up to {policy.FRAMES_PER_POSE} frames per pose; at most one backup per obstacle.")
    ser = (serial_factory or open_serial)()
    snap_log.clear()
    try:
        while pending:
            seg = pending.pop(0)
            oid = seg["obstacle_id"]
            log("LEG", f"Obstacle {oid}: {len(snap_log)} completed, {len(pending)} queued after this obstacle.")
            log("POSE", f"Planned viewing pose: {seg.get('goal_pose', 'not supplied')} (nominal centre cm and heading).")
            backup = seg.get("backup")
            if backup:
                log("BACKUP PLAN", f"If needed: {backup['command']} to {backup['pose']}; reverse path and viewing range checked by planner.")
            else:
                log("BACKUP PLAN", "No safe backup within configured clearance/viewing range; stationary retries only.")
            # Only execute modelled motion, never legacy AC/RA photo sequences.
            for cmd in seg["hardware_commands"]:
                if cmd.startswith("SNAP") or cmd.startswith("AC") or cmd == "RA":
                    continue
                ok, line = run_move(ser, cmd, "planned")
                if ok is not True:
                    log("STOP", f"Obstacle {oid}: planned move {cmd} failed. No scan or further movement; actual pose is not confirmed.")
                    return False
            log("ARRIVED", f"Obstacle {oid}: planned moves completed; starting PRIMARY pose captures.")
            attempt = 0
            def burst(stage):
                nonlocal attempt
                log("SETTLE", f"Obstacle {oid} | {stage}: waiting {policy.SETTLE_SECONDS:.2f} s without moving.")
                time.sleep(policy.SETTLE_SECONDS)
                for frame in range(policy.FRAMES_PER_POSE):
                    attempt += 1
                    log("SNAP", f"Obstacle {oid} | {stage} | frame {frame+1}/{policy.FRAMES_PER_POSE} | saved attempt Snap_{attempt}")
                    found = (on_snap(oid, attempt, False) if _snap_takes_attempt(on_snap)
                             else on_snap(oid))
                    if found:
                        log("RECOGNISED", f"Obstacle {oid} at {stage} pose, attempt {attempt}. No further captures for this obstacle.")
                        return True
                    log("MISS", f"Obstacle {oid} | {stage} | frame {frame+1}: not recognised.")
                    if frame+1 < policy.FRAMES_PER_POSE:
                        log("RETRY", f"Staying at {stage} pose; next frame in {policy.FRAME_GAP_SECONDS:.2f} s.")
                        time.sleep(policy.FRAME_GAP_SECONDS)
                return False
            success = burst("PRIMARY")
            used_backup = False
            if not success and backup:
                log("BACKUP", f"Obstacle {oid}: primary attempts exhausted. Switching to BACKUP: {backup['command']} -> {backup['pose']}.")
                ok, line = run_move(ser, backup["command"], "backup")
                if ok is not True:
                    log("STOP", f"Obstacle {oid}: backup move failed. No backup scan; old route will not resume.")
                    return False
                used_backup = True
                log("ARRIVED", f"Obstacle {oid}: backup move completed; starting BACKUP pose captures.")
                success = burst("BACKUP")
            elif not success:
                log("BACKUP SKIPPED", f"Obstacle {oid}: no safe backup within configured clearance/viewing range.")
            else:
                log("BACKUP SKIPPED", f"Obstacle {oid}: recognised at primary pose; backup not needed.")
            if not success:
                log("NO IMAGE FOUND", f"Obstacle {oid}: {attempt} attempts exhausted. Reporting miss and continuing to remaining obstacles.")
                on_not_found(oid)
            snap_log.append({"obstacle": oid, "snaps": attempt, "recognised": success})
            log("OBSTACLE DONE", f"{oid}: {'recognised' if success else 'no image found'}; {attempt} frames; backup {'used' if used_backup else 'not used'}.")
            if used_backup and pending:
                remaining = [s["obstacle_id"] for s in pending]
                log("REPLAN", f"Robot is at nominal backup pose {backup['pose']}. Discarding old remaining commands; requesting route for {remaining}.")
                log("REPLAN", "Robot stays stopped while planner runs. All physical obstacles remain in the map.")
                updated = replan(backup["pose"], remaining)
                if updated is None or updated.get("status") != "SUCCESS":
                    log("STOP", "Replanning failed. Old commands will NOT be resumed.")
                    return False
                pending = list(updated["segments"])
                new_order = [item['obstacle_id'] for item in pending]
                log("REPLAN OK", f"Remaining order: {new_order}; {'unchanged' if new_order == remaining else 'changed from previous plan'}.")
                for skipped in updated.get("skipped", []):
                    log("UNREACHABLE", f"Obstacle {skipped['obstacle_id']}: {skipped.get('reason', 'no route after recovery')}")
                for item in pending:
                    log("NEW LEG", f"Obstacle {item['obstacle_id']}: {' '.join(item['hardware_commands'])}")
            elif used_backup:
                log("REPLAN SKIPPED", "No remaining obstacles; robot remains at the backup pose.")
            elif pending:
                log("CONTINUE", f"No recovery movement; continuing existing route to obstacle {pending[0]['obstacle_id']}.")
        recognised = sum(bool(item['recognised']) for item in snap_log)
        log("RUN COMPLETE", f"{len(snap_log)} obstacles attempted; {recognised} recognised; {len(snap_log)-recognised} without an image.")
        return True
    except Exception as error:
        log("STOP", f"Run interrupted: {type(error).__name__}: {error}. No further route commands will be sent.")
        raise
    finally:
        ser.close()
        log("SERIAL", "STM serial connection closed.")


if __name__ == "__main__":
    main()
