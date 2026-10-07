"""
RPi client for Android integration (v5).

v5: every SNAP runs the v5 photo rules in rpi_stm_conn.photo_cycle():
    CASE A 1 image, conf >= 0.50   -> report it (1 photo)
    CASE B 1 image, conf <  0.50   -> extra FAR + NEAR photos, report highest conf (max 3 photos)
    CASE C no image                -> report nothing
    CASE D 2+ images               -> hold them, NEAR photo, rank IDEAL+NEAR, report top (max 2 photos)
trigger_camera() only takes and reads the photo; report_result() talks to Android.
Photos: photos/Trial_<run>_Obstacle_<id>_Snap<n>_<IDEAL|FAR|NEAR>.jpg

v2: waits for Android's beginExplore (Task 1 Explore) before planning/driving,
streams ROBOT,x,y,dir to Android as commands finish, reports misses as MSG
instead of TARGET,<id>,None, and honours STOP between commands.

Receives the obstacle list from Android over Bluetooth (/dev/rfcomm0), sends it
to the algo server, receives the hardware command list, writes it to a cmds file,
then reads that file back and dispatches it:

    AC<cm>    -> ultrasonic align, sent to the STM just before each photo;
                 the reading and correction are printed
    SNAP<id>  -> rpi_stm_conn.photo_cycle(): trigger_camera() for each photo
                 (capture, YOLO, save), then report_result() sends
                 TARGET,<obstacle_id>,<image_id> or a "no image" MSG to Android
    anything  -> written to the STM over the UART by rpi_stm_conn

Usage:
    python3 rpi_client_android.py                        # 127.0.0.1, full run (Android map)
    python3 rpi_client_android.py 192.168.1.42           # connects to the laptop algo server
    python3 rpi_client_android.py --android-only         # test Bluetooth reception only
    python3 rpi_client_android.py 192.168.1.42 --dry-run # no camera, no serial

--dry-run only talks to the algo server and prints the plan, so it runs on
a machine with no camera and no /dev/ttyACM0 (e.g. the laptop).
"""
import ast
import json
import os
import re
import socket
import sys
import time
import traceback
import glob
from datetime import datetime

import config      # same config.py as the laptop: turn models, grid, thresholds

PORT = 5000
START_POSE = (1, 1, "N")

CMD_DIR = "cmds"
LATEST_NAME = "latest.txt"

PHOTO_DIR = "photos"
CONF_THRESHOLD = config.RECOGNISED_CONF   # CASE A threshold (0.50); set in config.py
MODEL_MIN_CONF = config.MODEL_MIN_CONF    # YOLO drops boxes below this (0.25)
SHOW_POSE = False          # True prints every dead-reckoned pose sent to Android
ALIGN_PREFIX = "AC"         # must match config.ALIGN_PREFIX on the laptop
ANDROID_PORT = "/dev/rfcomm0"
MODEL_PATH = "best_ncnn_model"
FRAME_SIZE = (416, 416)

# Android writes without "\n". Match complete tokens from the front of the buffer.
ADD_RE = re.compile(r"ADD,B?(\d+),\((\d+),(\d+)\)")
SUB_RE = re.compile(r"SUB,B?(\d+)")
FACE_RE = re.compile(r"FACE,B?(\d+),([NSEW]),\((\d+),(\d+)\)")
ROBOT_RE = re.compile(r"ROBOT,(\d+),(\d+),([NSEW])")
TOKEN_RE = re.compile(r"(beginExplore|beginFastest|STOP|tl|tr|f|r)")

letter2number = {
    "Bounding box": "bb",
    "Circle": 40,
    "Down Arrow": 37,
    "Left Arrow": 39,
    "Letter A": 20,
    "Letter B": 21,
    "Letter C": 22,
    "Letter D": 23,
    "Letter E": 24,
    "Letter F": 25,
    "Letter G": 26,
    "Letter H": 27,
    "Letter S": 28,
    "Letter T": 29,
    "Letter U": 30,
    "Letter V": 31,
    "Letter W": 32,
    "Letter X": 33,
    "Letter Y": 34,
    "Letter Z": 35,
    "Number 1": 11,
    "Number 2": 12,
    "Number 3": 13,
    "Number 4": 14,
    "Number 5": 15,
    "Number 6": 16,
    "Number 7": 17,
    "Number 8": 18,
    "Number 9": 19,
    "Right Arrow": 38,
    "Up arrow": 36,
}

picam2 = None
model = None
cv2 = None
android_fd = None

# Taken from config.py so the Android robot marker uses the same turn
# models as the planner (v4 kept a separate, out-of-date copy here).
TURN_MODELS = config.TURN_MODELS
HEADINGS = ["N", "E", "S", "W"]
HEAD_VEC = config.DIRECTION_STEP
CELL_CM = config.CELL_CM
GRID = config.GRID

stop_requested = False
run_trial = None           # Trial number for this run's photo names
photo_results = {}         # obstacle id -> final decision + every photo taken for it
planned_targets = {}       # obstacle id -> planner "target" info (aimed vs planned centre)
SHOW_TARGET_DEBUG = True   # print aimed vs planned obstacle centre per obstacle


class StopRequested(Exception):
    """Raised between STM commands when Android sends STOP."""


# ----------------------------------------------------------------------
# Live robot position -> Android
# ----------------------------------------------------------------------

def send_line(payload):
    """Write one newline-terminated message to Android. Never raises."""
    try:
        if android_fd is not None:
            os.write(android_fd, (payload + "\n").encode("utf-8"))
        else:
            with open(ANDROID_PORT, "w") as f:
                f.write(payload + "\n")
        return True
    except Exception as error:
        print(f"    [ANDROID] send failed ({payload}): {error}")
        return False


class PoseTracker:
    """
    Dead-reckons the robot pose (centre, cm) from the commands the STM
    confirms, and tells Android as ROBOT,<cell x>,<cell y>,<dir> whenever
    the cell or heading changes. Nominal model only, same as the planner.
    """

    def __init__(self, start_cm):
        self.x, self.y = float(start_cm[0]), float(start_cm[1])
        self.heading = str(start_cm[2]).upper()
        self.align_move_cm = 0.0
        self.last_sent = None

    def _advance(self, cm):
        dx, dy = HEAD_VEC[self.heading]
        self.x += dx * cm
        self.y += dy * cm

    def _turn(self, model):
        forward, right, quarter = model
        fx, fy = HEAD_VEC[self.heading]
        rx, ry = fy, -fx                      # right-hand vector of the heading
        self.x += fx * forward + rx * right
        self.y += fy * forward + ry * right
        self.heading = HEADINGS[(HEADINGS.index(self.heading) + quarter) % 4]

    def apply(self, cmd, reply_line=None):
        """Update pose for one successfully finished STM command."""
        upper = cmd.upper()
        if upper in TURN_MODELS:
            self._turn(TURN_MODELS[upper])
        elif upper.startswith("FW") and upper[2:].isdigit():
            self._advance(int(upper[2:]))
        elif upper.startswith("BW") and upper[2:].isdigit():
            self._advance(-int(upper[2:]))
        elif upper.startswith(ALIGN_PREFIX):
            moved = 0.0
            for token in (reply_line or "").split():
                upper_token = token.upper()
                try:
                    if upper_token.startswith("MOVED:"):
                        moved = float(int(token[6:]))    # current STM: cm
                    elif upper_token.startswith("MV:"):
                        moved = int(token[3:]) / 10.0    # older STM: mm
                except ValueError:
                    moved = 0.0
            self.align_move_cm = moved
            self._advance(moved)
        elif upper == "RA":
            self._advance(-self.align_move_cm)
            self.align_move_cm = 0.0
        else:
            return
        self.publish()

    def cell(self):
        cx = min(max(int(self.x // CELL_CM), 1), GRID - 2)
        cy = min(max(int(self.y // CELL_CM), 1), GRID - 2)
        return cx, cy, self.heading

    def publish(self, force=False):
        cell = self.cell()
        if force or cell != self.last_sent:
            self.last_sent = cell
            send_line(f"ROBOT,{cell[0]},{cell[1]},{cell[2]}")
            if SHOW_POSE:
                print(f"        [POSE] ({self.x:.1f}, {self.y:.1f}) cm {self.heading} "
                      f"-> cell {cell[0]},{cell[1]}")


def watch_for_stop():
    """Background thread during a run: sets stop_requested if Android sends STOP."""
    import threading

    def loop():
        global stop_requested
        buf = ""
        while android_fd is not None and not stop_requested:
            try:
                chunk = os.read(android_fd, 256)
            except OSError:
                return
            if not chunk:
                return
            buf += chunk.decode("utf-8", errors="replace")
            if "STOP" in buf:
                stop_requested = True
                print("\n[ANDROID] STOP received: halting after the current command.")
                return
            buf = buf[-16:]

    threading.Thread(target=loop, daemon=True).start()


# ----------------------------------------------------------------------
# Camera
# ----------------------------------------------------------------------

def start_camera():
    """
    Bring up the camera and load the model.

    Imports happen here rather than at the top of the file so that
    --dry-run works on a machine without picamera2 / ultralytics.

    Returns True if the camera is usable. On failure it prints the reason
    and returns False: the run still goes ahead so the movement half can
    be tested, and SNAP only logs instead of capturing.
    """
    global picam2, model, cv2

    try:
        from picamera2 import Picamera2
        from ultralytics import YOLO
        import cv2 as cv2_module

        cv2 = cv2_module

        os.makedirs(PHOTO_DIR, exist_ok=True)

        picam2 = Picamera2()
        picam2.preview_configuration.main.size = FRAME_SIZE
        picam2.preview_configuration.main.format = "BGR888"
        picam2.preview_configuration.align()
        picam2.configure("preview")
        picam2.start()

        print("Loading model ...")
        model = YOLO(MODEL_PATH)

        # Warm up now rather than in the middle of a run, where the first
        # slow inference would stall the robot at the first obstacle.
        warmup = picam2.capture_array()
        model(warmup, conf=MODEL_MIN_CONF, imgsz=416, verbose=False)

        print("Camera ready.")
        return True

    except Exception as error:
        print(f"Camera unavailable: {error}")
        traceback.print_exc()
        print("Continuing without capture. SNAP commands will only be logged.")
        picam2 = None
        model = None
        return False


def stop_camera():
    if picam2 is not None:
        try:
            picam2.stop()
        except Exception:
            pass


def send_to_android(obstacle_id, image_id):
    # A miss (image_id None) or the "bb" bounding-box class is not an image ID:
    # tell Android in words instead of sending TARGET,<id>,None.
    if image_id is None or not str(image_id).isdigit():
        message = f"MSG,Obstacle {obstacle_id}: no image recognised"
    else:
        message = f"TARGET,{obstacle_id},{image_id}"
    if send_line(message):
        print(f"    Android       : sent {message}")


def current_trial():
    """
    Trial number for this run: one more than the highest Trial_<n>_ already
    in PHOTO_DIR, so every run's photos have distinct names. Fixed for the run.
    """
    global run_trial
    if run_trial is None:
        highest = 0
        if os.path.isdir(PHOTO_DIR):
            for name in os.listdir(PHOTO_DIR):
                match = re.match(r"Trial_(\d+)_", name)
                if match:
                    highest = max(highest, int(match.group(1)))
        run_trial = highest + 1
    return run_trial


def photo_name(obstacle_id, attempt, label="IDEAL", sensor_cm=None, case=None, short=False):
    """
    Trial_<t>_Obstacle_<id>_Snap<n>_Case<X>_<cm>cm[_short]_<IDEAL|FAR|NEAR>.jpg
    The position tag is ALWAYS last. <cm> is the planned sensor->face distance,
    "_short" means clearance stopped the FAR / NEAR move before its target.
    Case<X> is the CASE the IDEAL photo gave (A-D), i.e. why extra photos were taken.
    """
    why = f"_Case{case}" if case else ""
    dist = f"_{float(sensor_cm):.0f}cm" if sensor_cm is not None else ""
    cut = "_short" if short else ""
    return f"Trial_{current_trial()}_Obstacle_{obstacle_id}_Snap{attempt}{why}{dist}{cut}_{label}.jpg"


def diagnosis_path():
    return os.path.join(PHOTO_DIR, f"Trial_{current_trial()}_diagnosis.txt")


def print_target_debug(obstacle_id, indent="      "):
    """One-line-per-item view of the planned photo pose and the FAR / NEAR bounds."""
    t = planned_targets.get(str(obstacle_id))
    if not SHOW_TARGET_DEBUG or not t:
        return
    gp = t["goal_view_pose"]
    print(f"{indent}photo pose ({gp[0]:.1f}, {gp[1]:.1f}) {gp[2]}, sensor->face "
          f"{t['expected_sensor_to_face_cm']:.1f} cm, lateral {t['lateral_offset_cm']:+.1f} cm")
    pref = t.get("preferred_sensor_to_face_cm")
    if pref is not None:
        d = t["expected_sensor_to_face_cm"]
        side = ("on preferred" if abs(d - pref) < 0.5 else
                f"{d - pref:+.0f} cm vs preferred {pref:g} ({'further/back' if d > pref else 'closer/forward'})")
        print(f"{indent}distance choice: {side}, view penalty {t.get('view_penalty', 0):.1f} "
              f"(distance {t.get('distance_penalty', 0):.1f} + lateral {t.get('lateral_penalty', 0):.1f})")
    retry = t.get("retry") or {}
    parts = []
    for key in ("far", "near"):
        b = retry.get(key)
        if not b:
            parts.append(f"{key.upper()} n/a")
        elif b.get("available"):
            cut = ""
            if b.get("short"):
                who = str(b.get("blocked_by") or "clearance").split(":")[0].split(" (")[0]
                cut = f" (wanted {b.get('wanted_sensor_to_face_cm', 0):g} cm, cut by {who})"
            parts.append(f"{key.upper()} {b['command']} -> {b['sensor_to_face_cm']:.1f} cm{cut}")
        else:
            parts.append(f"{key.upper()} no room")
    print(f"{indent}extra photos: {'   '.join(parts)}")


def read_detections(result, names):
    """
    Every box YOLO returned, split into:
        targets : classes with a numeric image ID  (these are the "count")
        ignored : anything else, e.g. "Bounding box"
    Each entry: {"letter", "image_id", "conf", "cx", "w"}  (cx, w in pixels).
    Both lists are sorted by confidence, highest first.
    """
    targets, ignored = [], []
    boxes = result.boxes
    if boxes is None:
        return targets, ignored
    for i in range(len(boxes)):
        letter = names[int(boxes.cls[i])]
        image_id = letter2number.get(letter)
        x1, _y1, x2, _y2 = (float(v) for v in boxes.xyxy[i].tolist())
        entry = {"letter": letter, "image_id": image_id, "conf": float(boxes.conf[i]),
                 "cx": (x1 + x2) / 2, "w": x2 - x1, "xyxy": (x1, _y1, x2, _y2)}
        (targets if image_id is not None and str(image_id).isdigit() else ignored).append(entry)
    targets.sort(key=lambda d: -d["conf"])
    ignored.sort(key=lambda d: -d["conf"])
    return targets, ignored


def trigger_camera(obstacle_id, attempt=1, final=True, label="IDEAL", plan=None):
    """
    Take ONE photo for rpi_stm_conn.photo_cycle() and read it with the model.

    Saves PHOTO_DIR/Trial_<run>_Obstacle_<id>_Snap<attempt>_<label>.jpg with
    every box drawn and the label / count written on it. Does NOT talk to
    Android and does NOT decide anything: photo_cycle() applies the v5 rules
    and report_result() sends the answer.

    Returns a snap dict:
        {"label", "attempt", "name", "file", "targets": [...], "ignored": [...],
         "infer_ms", "error"}
    """
    send_line("MSG,[Taking photo]")      # robot is stopped; replace the "Moving" status
    plan = plan or {}
    pos = plan.get("position") or {}
    tag = pos.get("tag", label)          # IDEAL / FAR / NEAR / FAR-short / NEAR-short
    name = f"Snap{attempt}_{tag}"
    snap = {"label": label, "attempt": attempt, "name": name, "file": None,
            "targets": [], "ignored": [], "infer_ms": None, "error": None}

    if picam2 is None or model is None:
        snap["error"] = "no camera / model loaded - nothing captured"
        return snap

    try:
        frame = picam2.capture_array()
        t0 = time.time()
        results = model(frame, conf=MODEL_MIN_CONF, imgsz=416, verbose=False)
        snap["infer_ms"] = (time.time() - t0) * 1000
        snap["targets"], snap["ignored"] = read_detections(results[0], model.names)
        snap["frame_h"], snap["frame_w"] = frame.shape[0], frame.shape[1]

        # Which CASE this photo belongs to: IDEAL works it out from its own
        # boxes (same rule as photo_cycle); FAR / NEAR carry the CASE that sent them.
        if plan.get("classify"):
            snap["case"], snap["case_reason"] = plan["classify"](snap["targets"])
        else:
            snap["case"], snap["case_reason"] = plan.get("case"), plan.get("why")

        # Saved photo = model boxes on the image + a black info strip underneath,
        # so the text never covers the card.
        boxed = results[0].plot()
        n = len(snap["targets"])
        sensor_cm = pos.get("sensor_cm", plan.get("sensor_to_face_cm"))
        lines = [f"Obs {obstacle_id} | Snap{attempt} {tag}{' (short)' if pos.get('short') else ''} | "
                 + (f"{float(sensor_cm):.0f}cm" if sensor_cm is not None else "?cm")
                 + f" | Trial {current_trial()}"]
        if label == "IDEAL":
            lines.append("planned photo pose")
        else:
            who = str(pos.get("limit", "")).split(":")[0].split(" (")[0]
            lines.append(f"{pos.get('move', '')} from IDEAL, wanted {pos.get('wanted_cm', 0):g}cm"
                         + (f", cut by {who}" if pos.get("short") else ", full move"))
        if label == "IDEAL":
            lines.append(f"CASE {snap['case']}: {snap['case_reason']}")
        else:
            lines.append(f"taken for CASE {snap['case'] or '?'}")
        lines.append(f"count {n}" + ("" if n else " - no image ID found"))
        for i, d in enumerate(snap["targets"][:4], 1):
            lines.append(f" {i}. ID {d['image_id']} {d['letter']} {d['conf']:.2f}  x={d['cx']:.0f}")
        if n > 4:
            lines.append(f" ... +{n - 4} more")

        font, scale, step = cv2.FONT_HERSHEY_SIMPLEX, 0.42, 17
        width = boxed.shape[1]
        max_chars = max(20, int(width / 7.4))
        strip = step * len(lines) + 8
        final_frame = cv2.copyMakeBorder(boxed, 0, strip, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        colour = {"A": (0, 255, 0), "B": (0, 200, 255), "C": (180, 180, 180), "D": (0, 0, 255)}
        y = boxed.shape[0] + step
        for k, text in enumerate(lines):
            c = colour.get(snap["case"], (255, 255, 255)) if k == 2 else (255, 255, 255)
            cv2.putText(final_frame, text[:max_chars], (6, y), font, scale, c, 1, cv2.LINE_AA)
            y += step

        os.makedirs(PHOTO_DIR, exist_ok=True)
        path = os.path.join(PHOTO_DIR, photo_name(obstacle_id, attempt, tag, sensor_cm, snap["case"],
                                                  pos.get("short", False)))
        if cv2.imwrite(path, final_frame):
            snap["file"] = path
        else:
            snap["error"] = f"photo NOT saved (write failed: {path})"
        return snap

    except Exception as error:
        traceback.print_exc()
        snap["error"] = f"capture / model failed: {error}"
        return snap


def report_result(obstacle_id, decision):
    """
    Called once per obstacle by photo_cycle() with its final v5 decision.
    Sends TARGET,<obstacle>,<image id> or a "no image" MSG to Android and
    keeps the result for the run summary.
    """
    photo_results[str(obstacle_id)] = decision
    send_to_android(obstacle_id, decision["image_id"])
    write_diagnosis(obstacle_id, decision)


def write_diagnosis(obstacle_id, decision):
    """Append one obstacle's photo story to photos/Trial_<t>_diagnosis.txt."""
    photos = decision.get("photos") or []
    out = [f"Obstacle {obstacle_id}: CASE {decision['case']}  {len(photos)} photo(s)  ->  "
           + (f"ID {decision['image_id']} ({decision['letter']}, {decision['conf']:.2f}) from {decision['snap']}"
              if decision["image_id"] is not None else "NOTHING reported"),
           f"    IDEAL gave : {decision.get('ideal_case_reason') or '?'}",
           f"    decision   : {decision['reason']}"]
    for p in photos:
        cm = "?" if p["sensor_cm"] is None else f"{float(p['sensor_cm']):.1f} cm"
        boxes = ", ".join(f"ID {b[0]} {b[1]} {b[2]:.2f}" for b in p["boxes"]) or "no image ID"
        extra = ""
        if p["label"] != "IDEAL":
            extra = (f"  (wanted {p['wanted_cm']:g} cm, CUT SHORT)" if p["short"]
                     else f"  (full move to {p['wanted_cm']:g} cm)")
        out.append(f"    {p['name']:<20} {cm:>8}{extra}")
        if p.get("why"):
            out.append(f"        taken because: {p['why']}")
        for line in p.get("move_explain") or []:
            out.append(f"        planner: {line}")
        out.append(f"        count {p['count']}: {boxes}")
        for h in p.get("hints") or []:
            out.append(f"        extra {h}")
        if p.get("error"):
            out.append(f"        camera: {p['error']}")
        if p.get("file"):
            out.append(f"        {p['file']}")
    for line in decision.get("skipped") or []:
        out.append(f"    {line}")
    if decision.get("ranking"):
        out.append("    ranking (highest confidence first):")
        for i, (name, cm, iid, letter, conf) in enumerate(decision["ranking"], 1):
            cm_t = "?" if cm is None else f"{float(cm):.0f}cm"
            out.append(f"        {i}. {name:<14} {cm_t:>5}  ID {iid} {letter} {conf:.2f}"
                       + ("  <- reported" if i == 1 else ""))
    out.append("")
    try:
        os.makedirs(PHOTO_DIR, exist_ok=True)
        with open(diagnosis_path(), "a", encoding="utf-8") as f:
            if f.tell() == 0:
                f.write(f"Trial {current_trial()}  {datetime.now():%Y-%m-%d %H:%M:%S}\n"
                        f"RECOGNISED_CONF {CONF_THRESHOLD:.2f}  MODEL_MIN_CONF {MODEL_MIN_CONF:.2f}\n\n")
            f.write("\n".join(out) + "\n")
    except OSError as error:
        print(f"    (could not write {diagnosis_path()}: {error})")


def check_obstacle_side():
    """
    Used only during angle correction: look for the obstacle's own dark
    body in the frame - not the printed image or bullseye - so this still
    works even when no symbol is fully visible. No model needed.

    Returns "left", "right", or None if the obstacle isn't in frame at all.
    """
    if picam2 is None:
        print("      Camera   : no camera, cannot check obstacle side")
        return None
    if cv2 is None:
        print("      Camera   : cv2 not available, cannot check obstacle side")
        return None

    MIN_OBSTACLE_AREA_PX = 800   # ignore small dark specks/shadows as noise

    try:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Otsu picks the split point between "dark" and "light" itself,
        # from this frame's own histogram - no manual brightness number.
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            print("      Camera   : no dark obstacle body found")
            return None

        biggest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest) < MIN_OBSTACLE_AREA_PX:
            print("      Camera   : obstacle not in view (largest dark region too small)")
            return None

        x, y, w, h = cv2.boundingRect(biggest)
        box_center_x = x + w / 2
        frame_center_x = FRAME_SIZE[0] / 2

        side = "left" if box_center_x < frame_center_x else "right"
        print(f"      Camera   : obstacle body centre {box_center_x:.0f}px vs "
              f"frame centre {frame_center_x:.0f}px -> {side}")
        return side

    except Exception as error:
        print(f"      Camera   : side check failed: {error}")
        traceback.print_exc()
        return None
    

# ----------------------------------------------------------------------
# Android Bluetooth
# ----------------------------------------------------------------------

def print_map(obstacles, start):
    print("    current map:")
    print(f"      start robot = {start}")
    if not obstacles:
        print("      (no obstacles yet)")
        return
    for obstacle_id in sorted(obstacles):
        _id, x, y, face = obstacles[obstacle_id]
        face_text = face if face else "(no face)"
        print(f"      obstacle {obstacle_id}: ({x}, {y}) face {face_text}")


def apply_snapshot(text, obstacles):
    """Replace the local map with Android's SEND ARENA list."""
    parsed = ast.literal_eval(text)
    obstacles.clear()
    for entry in parsed:
        obstacle_id, x, y, face = entry
        obstacles[int(obstacle_id)] = (int(obstacle_id), int(x), int(y), str(face or ""))
    return list(obstacles.values())


def extract_message(buffer):
    """
    Pull one complete Android message off the front of buffer.

    Returns (kind, payload, leftover) or (None, None, buffer) if incomplete.
    """
    stripped = buffer.lstrip("\r\n ")
    skipped = len(buffer) - len(stripped)
    prefix = buffer[:skipped]
    if not stripped:
        return None, None, buffer

    if stripped.startswith("["):
        depth = 0
        in_string = False
        for i, char in enumerate(stripped):
            if char == '"' and (i == 0 or stripped[i - 1] != "\\"):
                in_string = not in_string
            elif not in_string:
                if char == "[":
                    depth += 1
                elif char == "]":
                    depth -= 1
                    if depth == 0:
                        return "SNAPSHOT", stripped[: i + 1], prefix + stripped[i + 1 :]
        return None, None, buffer

    match = ADD_RE.match(stripped)
    if match:
        return "ADD", match, prefix + stripped[match.end() :]

    match = SUB_RE.match(stripped)
    if match:
        return "SUB", match, prefix + stripped[match.end() :]

    match = FACE_RE.match(stripped)
    if match:
        return "FACE", match, prefix + stripped[match.end() :]

    match = ROBOT_RE.match(stripped)
    if match:
        return "ROBOT", match, prefix + stripped[match.end() :]

    match = TOKEN_RE.match(stripped)
    if match:
        return "TOKEN", match.group(1), prefix + stripped[match.end() :]

    # Unknown leftover that cannot form a known message yet — wait for more
    # bytes unless this already looks like garbage (too long with no match).
    if len(stripped) > 256:
        print(f"[ANDROID] dropping unrecognised bytes: {stripped[:80]!r}")
        return "DROP", None, prefix + stripped[1:]

    return None, None, buffer


def open_android_port():
    """
    Open /dev/rfcomm0 for raw reads.

    Android does not send newlines, so this must not use text-mode readline.
    Retries until the device node exists (Bluetooth connected).
    """
    global android_fd

    print(f"Waiting for Android on {ANDROID_PORT} ...")
    print("Connect the tablet over Bluetooth first. Demo mode must be OFF.")
    print("Place obstacles, (optionally SEND ARENA), then tap Task 1 Explore to start.")
    print()

    while True:
        if os.path.exists(ANDROID_PORT):
            try:
                android_fd = os.open(ANDROID_PORT, os.O_RDWR | os.O_NOCTTY)
                print(f"Opened {ANDROID_PORT}. Listening for coordinates...")
                print()
                return
            except OSError as error:
                print(f"Could not open {ANDROID_PORT}: {error}. Retrying ...")
        time.sleep(1)


def close_android_port():
    global android_fd
    if android_fd is not None:
        try:
            os.close(android_fd)
        except OSError:
            pass
        android_fd = None


def wait_for_android_map(android_only):
    """
    Read Bluetooth until we have a usable obstacle list.

    Prints every received coordinate. ADD/SUB/FACE/ROBOT and SEND ARENA (`[...]`)
    only update the map. The run starts ONLY on `beginExplore` (Task 1 Explore),
    once at least one obstacle exists and every obstacle has a face.

    With --android-only, keeps printing until Ctrl+C and then returns
    whatever is currently stored.
    """
    global START_POSE

    open_android_port()

    obstacles = {}
    buffer = ""
    complete = False

    try:
        while True:
            chunk = os.read(android_fd, 1024)
            if not chunk:
                print("[ANDROID] Bluetooth closed.")
                break

            text = chunk.decode("utf-8", errors="replace")
            print(f"[ANDROID raw] {text!r}")
            buffer += text

            while True:
                kind, payload, buffer = extract_message(buffer)
                if kind is None:
                    break

                if kind == "DROP":
                    continue

                if kind == "ADD":
                    obstacle_id = int(payload.group(1))
                    x, y = int(payload.group(2)), int(payload.group(3))
                    previous = obstacles.get(obstacle_id)
                    face = previous[3] if previous else ""
                    obstacles[obstacle_id] = (obstacle_id, x, y, face)
                    print(f"[ANDROID] ADD obstacle {obstacle_id} -> ({x}, {y})")
                    print_map(obstacles, START_POSE)

                elif kind == "SUB":
                    obstacle_id = int(payload.group(1))
                    obstacles.pop(obstacle_id, None)
                    print(f"[ANDROID] SUB obstacle {obstacle_id}")
                    print_map(obstacles, START_POSE)

                elif kind == "FACE":
                    obstacle_id = int(payload.group(1))
                    face = payload.group(2)
                    x, y = int(payload.group(3)), int(payload.group(4))
                    obstacles[obstacle_id] = (obstacle_id, x, y, face)
                    print(f"[ANDROID] FACE obstacle {obstacle_id} -> ({x}, {y}) {face}")
                    print_map(obstacles, START_POSE)

                elif kind == "ROBOT":
                    START_POSE = (int(payload.group(1)), int(payload.group(2)), payload.group(3))
                    print(f"[ANDROID] ROBOT start -> {START_POSE}")
                    print_map(obstacles, START_POSE)

                elif kind == "SNAPSHOT":
                    apply_snapshot(payload, obstacles)
                    print("[ANDROID] SEND ARENA snapshot (waiting for beginExplore):")
                    print_map(obstacles, START_POSE)
                    send_line(f"MSG,Arena received ({len(obstacles)} obstacles). Press Task 1 Explore to start.")

                elif kind == "TOKEN":
                    print(f"[ANDROID] command {payload}")
                    if payload == "beginExplore":
                        if not obstacles:
                            print("[ANDROID] beginExplore but no obstacles yet - waiting.")
                            send_line("MSG,No obstacles received yet")
                        elif any(not o[3] for o in obstacles.values()):
                            print("[ANDROID] beginExplore but some obstacle has no face - waiting.")
                            send_line("MSG,Every obstacle needs a face before starting")
                        else:
                            complete = True

                if complete and not android_only:
                    return [obstacles[key] for key in sorted(obstacles)]

    except KeyboardInterrupt:
        print("\nStopped listening.")

    return [obstacles[key] for key in sorted(obstacles)]


# ----------------------------------------------------------------------
# Command file
# ----------------------------------------------------------------------

def write_cmds_file(commands):
    """
    Write the command list one per line, in the format rpi_stm_conn -f reads.

    Returns the path of the timestamped file. A copy also goes to
    cmds/latest.txt so the last run can be replayed with:
        python3 rpi_stm_conn.py -f cmds/latest.txt
    v5: the plan's photo targets (desired pose, FAR / NEAR bounds) are saved
    next to each file as <file>.targets.json, so a replay takes the same photos.
    """
    os.makedirs(CMD_DIR, exist_ok=True)

    body = "\n".join(commands) + "\n"
    targets = json.dumps(planned_targets, indent=1)

    path = os.path.join(CMD_DIR, f"{int(time.time() * 1000)}_cmds.txt")
    latest = os.path.join(CMD_DIR, LATEST_NAME)
    for p in (path, latest):
        with open(p, "w") as f:
            f.write(body)
        with open(p + ".targets.json", "w") as f:
            f.write(targets)

    return path


# ----------------------------------------------------------------------
# Networking
# ----------------------------------------------------------------------

def recv_json_line(sock, buffer=b""):
    """
    Read until we have one complete newline terminated JSON message.

    Buffers raw bytes to safely handle TCP stream fragmentation.
    Returns (message_dict, leftover_buffer).
    """
    while b"\n" not in buffer:
        chunk = sock.recv(4096)

        if not chunk:
            raise ConnectionError("Server closed the connection early.")

        buffer += chunk

    line_bytes, buffer = buffer.split(b"\n", 1)
    line = line_bytes.decode("utf-8")

    return json.loads(line), buffer


def parse_args():
    """Returns (host, dry_run, android_only). Flags may appear anywhere."""
    flags = {"--dry-run", "--android-only", "--from-android"}
    dry_run = "--dry-run" in sys.argv
    android_only = "--android-only" in sys.argv
    args = [a for a in sys.argv[1:] if a not in flags]
    host = args[0] if args else "127.0.0.1"
    return host, dry_run, android_only


def request_plan(host, obstacles):
    """Returns the reply dict, or None if anything went wrong."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as rpi_socket:

        # Without a timeout a blocked port makes this hang for over a
        # minute before failing, which looks like a crash.
        rpi_socket.settimeout(10)

        try:
            rpi_socket.connect((host, PORT))

        except socket.timeout:
            print("\nTimed out. The packets are not reaching the laptop.")
            print("Usually the Windows firewall is blocking inbound TCP 5000,")
            print("or the laptop and the Pi are on different networks.")
            return None

        except ConnectionRefusedError:
            print("\nConnection refused. The laptop was reachable but nothing")
            print("is listening on port 5000. Is algo_server.py running?")
            return None

        except OSError as error:
            print(f"\nCould not reach {host}: {error}")
            print("Try: ping " + host)
            return None

        # Planning takes a moment, so allow more time for the reply.
        rpi_socket.settimeout(60)
        print("Connected.")

        request = {
            "obstacles": [list(obstacle) for obstacle in obstacles],
            "start": list(START_POSE),
        }

        rpi_socket.sendall((json.dumps(request) + "\n").encode("utf-8"))
        print(f"Sent {len(obstacles)} obstacles.")

        reply, _leftover = recv_json_line(rpi_socket, b"")

        if reply["status"] != "SUCCESS":
            print("Algo server returned an error:", reply.get("message"))
            return None

        return reply


# ----------------------------------------------------------------------
# Dispatch
# ----------------------------------------------------------------------

def print_plan_summary(reply, total_obstacles, trial):
    """High-level view of the plan before anything moves."""
    order = reply["order"]
    print()
    print("=" * 64)
    print("  PLAN FROM ALGO SERVER")
    print("=" * 64)
    print(f"  Obstacles to visit : {len(order)} / {total_obstacles}")
    print(f"  Path               : {' -> '.join(['Start'] + [str(o) for o in order])}")
    if reply.get("skipped"):
        print("  Skipped            :")
        for entry in reply["skipped"]:
            print(f"      obstacle {entry['obstacle_id']}: {entry['reason']}")
    print(f"  Planning time      : {float(reply['planning_ms']):.0f} ms"
          f"   ({len(reply['commands'])} commands)")
    print(f"  Photos this run    : {os.path.abspath(PHOTO_DIR)}/")
    print(f"                       Trial_{trial}_Obstacle_<id>_Snap<n>_Case<X>_<cm>cm[_short]_<IDEAL|FAR|NEAR>.jpg")
    print(f"  Diagnosis log      : {os.path.abspath(diagnosis_path())}")
    print(f"  Photo rules        : A 1 image >= {CONF_THRESHOLD:.2f} done | B 1 image < {CONF_THRESHOLD:.2f}"
          " IDEAL+FAR+NEAR, highest conf | C none | D 2+ IDEAL+NEAR, highest conf")
    print("-" * 64)
    where = "Start"
    for seg in reply.get("segments", []):
        dest = f"Obstacle {seg['obstacle_id']}"
        print(f"  {where} -> {dest}:  {' '.join(seg['hardware_commands'])}")
        if seg.get("target"):
            planned_targets[str(seg["obstacle_id"])] = seg["target"]
            print_target_debug(seg["obstacle_id"], indent="    ")
        where = dest
    print("=" * 64)


def print_run_summary(order, trial):
    print()
    print("=" * 64)
    print(f"  RUN SUMMARY  (Trial {trial})")
    print("=" * 64)
    for obstacle_id in order:
        d = photo_results.get(str(obstacle_id))
        if d is None:
            print(f"  Obstacle {obstacle_id}: not photographed")
        elif d["image_id"] is not None:
            print(f"  Obstacle {obstacle_id}: CASE {d['case']}  ID {d['image_id']} ({d['letter']}, "
                  f"{d['conf']:.2f}) from {d['snap']}  ->  {d['file']}")
        else:
            print(f"  Obstacle {obstacle_id}: CASE {d['case']}  nothing reported - {d['reason']}")
        if d is not None:
            for p in d.get("photos") or []:
                cm = "?" if p["sensor_cm"] is None else f"{float(p['sensor_cm']):.0f}cm"
                print(f"      {p['name']:<20} {cm:>5}  count {p['count']}  "
                      + (", ".join(f"ID {b[0]} {b[2]:.2f}" for b in p["boxes"]) or "-"))
                for h in p.get("hints") or []:
                    print(f"          {h}")
            for line in d.get("skipped") or []:
                print(f"      {line}")
    print(f"  Diagnosis log: {os.path.abspath(diagnosis_path())}")
    print("=" * 64)


def dispatch(path, commands, dry_run, start_cm=None):
    if dry_run:
        print("\nDry run, not opening the serial port.")
        return True

    # Imported here so --dry-run works on a machine without pyserial.
    import rpi_stm_conn

    tracker = PoseTracker(start_cm or (15, 15, "N"))
    tracker.publish(force=True)          # show the start pose on the tablet
    original_send_and_wait = rpi_stm_conn.send_and_wait

    def tracked_send_and_wait(ser, cmd):
        if stop_requested:
            raise StopRequested()
        send_line("MSG,[Moving]")        # status box shows Moving while the wheels turn
        result, line = original_send_and_wait(ser, cmd)
        if result is True:               # only confirmed commands move the marker
            tracker.apply(cmd, line)
        return result, line

    rpi_stm_conn.send_and_wait = tracked_send_and_wait
    watch_for_stop()
    print("\nStarting run.")
    try:
        return rpi_stm_conn.run_commands(
            rpi_stm_conn.read_cmds_file(path),
            on_snap=trigger_camera,
            on_check_position=check_obstacle_side,   # v5: not used (no angle correction)
            targets=planned_targets,
            on_result=report_result,
        )
    except StopRequested:
        print("Run stopped by Android STOP.")
        send_line("MSG,Run stopped")
        return False
    finally:
        rpi_stm_conn.send_and_wait = original_send_and_wait


def main():
    host, dry_run, android_only = parse_args()

    if not dry_run and not android_only:
        start_camera()

    try:
        remaining = wait_for_android_map(android_only)
        if android_only:
            print()
            print("Android-only test finished. Not contacting the algo server.")
            if remaining:
                print("Final obstacles:", remaining)
            else:
                print("No obstacles received.")
            return
        if not remaining:
            print("No obstacles received from Android.")
            return

        print()
        print(f"Connecting to algo server at {host}:{PORT} ...")

        reply = request_plan(host, remaining)

        if reply is None:
            return

        trial = current_trial()
        print_plan_summary(reply, len(remaining), trial)

        commands = reply["commands"]

        if not commands:
            print("\nNo commands to run.")
            return

        path = write_cmds_file(commands)
        print(f"Wrote {len(commands)} commands to {path}")

        send_line("MSG,Run started")
        ok = dispatch(path, commands, dry_run, reply.get("start_cm"))

        # Tell Android straight away, before the summary prints.
        if ok:
            final_obstacle = reply["order"][-1] if reply["order"] else None
            done_msgs = []
            if final_obstacle is not None:
                done_msgs.append(f"MSG,Reached final obstacle {final_obstacle} "
                                 f"({len(reply['order'])}/{len(remaining)} visited).")
            done_msgs.append("MSG,Run complete. Done !")
            for i, done_msg in enumerate(done_msgs):
                if i:
                    time.sleep(0.2)      # gap so Android reads them as two messages
                send_line(done_msg)
                print(f"[ANDROID] sent {done_msg}")

        if not dry_run:
            print_run_summary(reply["order"], trial)

        print()
        if ok:
            print("Run complete.")
        else:
            print("Run ended early (error, timeout or STOP).")
            send_line("MSG,Run ended early")

    finally:
        stop_camera()
        close_android_port()


if __name__ == "__main__":
    main()