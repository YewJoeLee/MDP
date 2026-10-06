"""
RPi client for Android integration.

v2: waits for Android's beginExplore (Task 1 Explore) before planning/driving,
streams ROBOT,x,y,dir to Android as commands finish, reports misses as MSG
instead of TARGET,<id>,None, and honours STOP between commands.

Receives the obstacle list from Android over Bluetooth (/dev/rfcomm0), sends it
to the algo server, receives the hardware command list, writes it to a cmds file,
then reads that file back and dispatches it:

    AC<cm>    -> ultrasonic align, sent to the STM just before each photo;
                 the reading and correction are printed
    SNAP<id>  -> trigger_camera(): capture, run YOLO, save the annotated
                 frame, send TARGET,<obstacle_id>,<image_id> to Android
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

PORT = 5000
START_POSE = (1, 1, "N")

CMD_DIR = "cmds"
LATEST_NAME = "latest.txt"

PHOTO_DIR = "photos"
CONF_THRESHOLD = 0.5       # below this a snap counts as not recognised -> error correction
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

# Must match config.py on the laptop (forward cm, right cm, clockwise quarter-turns).
TURN_MODELS = {
    "RT90": (21, 31, 1), "LT90": (21, -35, -1),
    "XL90": (-26, -11, 1), "XR90": (-35, 13, -1),
}
HEADINGS = ["N", "E", "S", "W"]
HEAD_VEC = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
CELL_CM = 10
GRID = 20

stop_requested = False
run_trial = None           # Trial number for this run's photo names
photo_results = {}         # obstacle id -> result of its latest snap
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
        model(warmup, conf=0.25, imgsz=416, verbose=False)

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
        print(f"      Android  : sent {message}")


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


def photo_name(obstacle_id, attempt):
    return f"Trial_{current_trial()}_Obstacle_{obstacle_id}_Snap_{attempt}.jpg"


def print_target_debug(obstacle_id, indent="      "):
    """Planned obstacle centre vs the centre the robot is actually lined up on.

    The planner snaps the ideal viewing pose by up to VIEW_POSITION_TOLERANCE_CM
    (config.py) so it is reachable; this shows how far that snap moved it.
    Nominal (dead-reckoned) numbers from the plan, not a measurement.
    """
    t = planned_targets.get(str(obstacle_id))
    if not SHOW_TARGET_DEBUG or not t:
        return
    oc, ac = t["obstacle_centre_cm"], t["aimed_centre_cm"]
    ip, gp = t["ideal_view_pose"], t["goal_view_pose"]
    lat = t["lateral_offset_cm"]
    side = "on centre" if abs(lat) < 0.05 else (
        f"{abs(lat):.1f} cm {'RIGHT' if lat > 0 else 'LEFT'} of centre line")
    print(f"{indent}[TARGET] obstacle {obstacle_id} cell ({t['obstacle_cell'][0]}, {t['obstacle_cell'][1]})")
    print(f"{indent}  planned centre : ({oc[0]:.1f}, {oc[1]:.1f}) cm")
    print(f"{indent}  aimed centre   : ({ac[0]:.1f}, {ac[1]:.1f}) cm   "
          f"diff ({ac[0]-oc[0]:+.1f}, {ac[1]-oc[1]:+.1f})")
    print(f"{indent}  ideal pose     : ({ip[0]:.1f}, {ip[1]:.1f}) {ip[2]}")
    print(f"{indent}  planned pose   : ({gp[0]:.1f}, {gp[1]:.1f}) {gp[2]}   "
          f"robot {side}")
    print(f"{indent}  sensor->face   : {t['expected_sensor_to_face_cm']:.1f} cm "
          f"(ideal {t['planned_sensor_to_face_cm']:.1f}, leeway +-{t['tolerance_cm']:g})")


def trigger_camera(obstacle_id, attempt=1, final=True):
    """
    Called by rpi_stm_conn for every snap of an obstacle (attempt 1..3).

    Every attempt is saved as PHOTO_DIR/Trial_<run>_Obstacle_<id>_Snap_<n>.jpg
    with the image ID written on it. Recognised = a target class with
    confidence >= CONF_THRESHOLD. Android gets one result per obstacle:
    TARGET for the first recognised snap (which is also the last snap taken),
    or a "no image" MSG if the final snap still fails.

    Returns True if recognised (rpi_stm_conn then stops correcting).
    """
    send_line("MSG,[Taking photo]")      # robot is stopped; replace the "Moving" status
    if attempt == 1:
        print_target_debug(obstacle_id)
    name = photo_name(obstacle_id, attempt)
    result = {"attempt": attempt, "file": name, "recognised": False,
              "letter": None, "image_id": None, "confidence": 0.0}
    photo_results[str(obstacle_id)] = result

    if picam2 is None or model is None:
        print("      No camera - nothing captured")
        if final:
            send_to_android(obstacle_id, None)
        return False

    try:
        frame = picam2.capture_array()
        results = model(frame, conf=0.25, imgsz=416, verbose=False)
        final_frame = results[0].plot()
        boxes = results[0].boxes

        letter, image_id, confidence = None, None, 0.0
        if boxes is not None and len(boxes) > 0:
            best = boxes.conf.argmax()
            letter = model.names[int(boxes.cls[best])]
            image_id = letter2number.get(letter)
            confidence = float(boxes.conf[best])

        is_target = image_id is not None and str(image_id).isdigit()
        recognised = is_target and confidence >= CONF_THRESHOLD

        # Label the saved photo with the target ID (plot() only draws the class name).
        line1 = (f"Obstacle {obstacle_id} -> ID {image_id} ({confidence:.2f})"
                 if is_target else f"Obstacle {obstacle_id} -> no ID")
        line2 = f"Trial {current_trial()}  Snap {attempt}"
        for text, y in ((line1, 30), (line2, 58)):
            cv2.putText(final_frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 4, cv2.LINE_AA)   # dark outline for contrast
            cv2.putText(final_frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2, cv2.LINE_AA)

        os.makedirs(PHOTO_DIR, exist_ok=True)
        path = os.path.join(PHOTO_DIR, name)
        saved = cv2.imwrite(path, final_frame)

        if letter is None:
            print("      Detected : nothing")
        else:
            shown_id = image_id if is_target else "-"
            print(f"      Detected : {letter}  |  image ID {shown_id}  |  confidence {confidence:.2f}")
        print(f"      Saved    : {path}" + ("" if saved else "   (WRITE FAILED)"))
        if recognised:
            print(f"      Result   : RECOGNISED (confidence >= {CONF_THRESHOLD:.2f})")
        elif is_target:
            print(f"      Result   : NOT RECOGNISED (confidence {confidence:.2f} < {CONF_THRESHOLD:.2f})")
        else:
            print("      Result   : NOT RECOGNISED (no target image found)")

        result.update(recognised=recognised, letter=letter,
                      image_id=image_id, confidence=confidence)

        if recognised:
            send_to_android(obstacle_id, image_id)
        elif final:
            send_to_android(obstacle_id, None)
        return recognised

    except Exception as error:
        print(f"      Capture failed: {error}")
        traceback.print_exc()
        if final:
            send_to_android(obstacle_id, None)
        return False


def check_obstacle_side():
    """
    Used only during angle correction: look for the obstacle's own dark
    body in the frame - not the printed image or bullseye - so this still
    works even when no symbol is fully visible. No model needed.

    Prefers the blob whose pixel width matches what the real obstacle is
    expected to look like at the robot's normal photo-taking distance,
    instead of just picking the largest dark blob - that's how background
    clutter (shoes, furniture) at a different distance gets ignored even
    if it happens to have a bigger silhouette.

    Returns "left", "right", or None if the obstacle isn't in frame at all.
    """
    if picam2 is None:
        print("      Camera   : no camera, cannot check obstacle side")
        return None
    if cv2 is None:
        print("      Camera   : cv2 not available, cannot check obstacle side")
        return None

    MIN_OBSTACLE_AREA_PX = 800 #ignore small dark specks/shadows as noise
    EXPECTED_WIDTH_PX = 140 #The pixel width the real obstacle is expected to show up at, from the robot's normal photo-taking distance. 
    EXPECTED_WIDTH_TOLERANCE_PX = 45 #accept EXPECTED_WIDTH_PX +/- this

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

        candidates = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_OBSTACLE_AREA_PX:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            width_error = abs(w - EXPECTED_WIDTH_PX)
            if width_error > EXPECTED_WIDTH_TOLERANCE_PX:
                continue
            candidates.append((width_error, x, w))

        if not candidates:
            print("      Camera   : nothing matched the obstacle's expected size")
            return None

        # Smallest width_error wins: the blob closest to the obstacle's
        # expected on-frame size, not just the biggest blob.
        _, x, w = min(candidates, key=lambda c: c[0])
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
    """
    os.makedirs(CMD_DIR, exist_ok=True)

    body = "\n".join(commands) + "\n"

    path = os.path.join(CMD_DIR, f"{int(time.time() * 1000)}_cmds.txt")
    with open(path, "w") as f:
        f.write(body)

    with open(os.path.join(CMD_DIR, LATEST_NAME), "w") as f:
        f.write(body)

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
    print(f"                       Trial_{trial}_Obstacle_<id>_Snap_<1-3>.jpg")
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
        r = photo_results.get(str(obstacle_id))
        if r is None:
            print(f"  Obstacle {obstacle_id}: not photographed")
        elif r["recognised"]:
            print(f"  Obstacle {obstacle_id}: ID {r['image_id']} ({r['letter']}, "
                  f"{r['confidence']:.2f}) on snap {r['attempt']}  ->  {r['file']}")
        else:
            print(f"  Obstacle {obstacle_id}: not recognised after {r['attempt']} snap(s)"
                  f"  ->  last photo {r['file']}")
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
            on_check_position=check_obstacle_side
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