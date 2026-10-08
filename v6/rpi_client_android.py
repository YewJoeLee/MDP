"""
RPi client for Android integration (v5).

v5: every SNAP runs the v5 photo rules in rpi_stm_conn.photo_cycle().
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
import base64
from datetime import datetime

import config

PORT = 5000
START_POSE = (1, 1, "N")

CMD_DIR = "cmds"
LATEST_NAME = "latest.txt"

PHOTO_DIR = "photos"
CONF_THRESHOLD = config.RECOGNISED_CONF
MODEL_MIN_CONF = config.MODEL_MIN_CONF
SHOW_POSE = False
ALIGN_PREFIX = "AC"
ANDROID_PORT = "/dev/rfcomm0"
MODEL_PATH = "best_ncnn_model"
FRAME_SIZE = (416, 416)

ADD_RE = re.compile(r"ADD,B?(\d+),\((\d+),(\d+)\)")
SUB_RE = re.compile(r"SUB,B?(\d+)")
FACE_RE = re.compile(r"FACE,B?(\d+),([NSEW]),\((\d+),(\d+)\)")
ROBOT_RE = re.compile(r"ROBOT,(\d+),(\d+),([NSEW])")
TOKEN_RE = re.compile(r"(beginExplore|beginFastest|STOP|tl|tr|f|r)")

letter2number = {
    "Bounding box": "bb",
    "Circle": 40, "Down Arrow": 37, "Left Arrow": 39, "Right Arrow": 38, "Up arrow": 36,
    "Letter A": 20, "Letter B": 21, "Letter C": 22, "Letter D": 23, "Letter E": 24,
    "Letter F": 25, "Letter G": 26, "Letter H": 27, "Letter S": 28, "Letter T": 29,
    "Letter U": 30, "Letter V": 31, "Letter W": 32, "Letter X": 33, "Letter Y": 34, "Letter Z": 35,
    "Number 1": 11, "Number 2": 12, "Number 3": 13, "Number 4": 14, "Number 5": 15,
    "Number 6": 16, "Number 7": 17, "Number 8": 18, "Number 9": 19
}

picam2 = None
model = None
cv2 = None
android_fd = None

TURN_MODELS = config.TURN_MODELS
HEADINGS = ["N", "E", "S", "W"]
HEAD_VEC = config.DIRECTION_STEP
CELL_CM = config.CELL_CM
GRID = config.GRID

stop_requested = False
run_trial = None           
photo_results = {}         
planned_targets = {}       
SHOW_TARGET_DEBUG = True   


class StopRequested(Exception):
    pass


# ----------------------------------------------------------------------
# Live robot position -> Android
# ----------------------------------------------------------------------

def send_line(payload):
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
        rx, ry = fy, -fx
        self.x += fx * forward + rx * right
        self.y += fy * forward + ry * right
        self.heading = HEADINGS[(HEADINGS.index(self.heading) + quarter) % 4]

    def apply(self, cmd, reply_line=None):
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
                        moved = float(int(token[6:]))
                    elif upper_token.startswith("MV:"):
                        moved = int(token[3:]) / 10.0
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
    if image_id is None or not str(image_id).isdigit():
        message = f"MSG,Obstacle {obstacle_id}: no image recognised"
    else:
        message = f"TARGET,{obstacle_id},{image_id}"
    if send_line(message):
        print(f"    Android       : sent {message}")


def current_trial():
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
    return f"Trial_{current_trial()}_Obstacle_{obstacle_id}_Snap{attempt}_{label}.jpg"


def diagnosis_path():
    return os.path.join(PHOTO_DIR, f"Trial_{current_trial()}_diagnosis.txt")


def print_target_debug(obstacle_id, indent="      "):
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
    ang = t.get("angled") or {}
    aparts = []
    for side in ("right", "left"):
        o = ang.get(side)
        if not o:
            aparts.append(f"{side} n/a")
        elif o.get("available"):
            aparts.append(f"body {side.upper()} -> BW{o['start_back_cm']} + {o['turn']} (undo {o['undo']})")
        else:
            aparts.append(f"body {side.upper()} -> no angled pose")
    print(f"{indent}CASE C angled: {'   '.join(aparts)}")


def read_detections(result, names):
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


def find_obstacle_body(frame):
    width = frame.shape[1]
    out = {"side": None, "cx": None, "width": width, "area": 0, "why": ""}
    if cv2 is None:
        out["why"] = "cv2 not available"
        return out
    try:
        cropped_frame = frame[100:300, :]
        gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            out["why"] = "no dark region"
            return out
        biggest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest)
        out["area"] = area
        if area < config.BODY_MIN_AREA_PX:
            out["why"] = f"largest dark region {area:.0f} px < {config.BODY_MIN_AREA_PX} px"
            return out
        x, _y, w, _h = cv2.boundingRect(biggest)
        cx = x + w / 2
        out["cx"] = cx
        band = config.BODY_CENTRE_BAND * width
        if abs(cx - width / 2) <= band:
            out["side"] = "centre"
        else:
            out["side"] = "left" if cx < width / 2 else "right"
        return out
    except Exception as error:
        out["why"] = f"body check failed: {error}"
        return out


def trigger_camera(obstacle_id, attempt=1, final=True, label="IDEAL", plan=None):
    send_line("MSG,[Taking photo]")
    plan = plan or {}
    pos = plan.get("position") or {}
    tag = pos.get("tag", label)
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
        snap["body"] = find_obstacle_body(frame)      

        if plan.get("classify"):
            snap["case"], snap["case_reason"] = plan["classify"](snap["targets"])
        else:
            snap["case"], snap["case_reason"] = plan.get("case"), plan.get("why")

        boxed = results[0].plot()
        n = len(snap["targets"])
        sensor_cm = pos.get("sensor_cm", plan.get("sensor_to_face_cm"))
        lines = [f"Obs {obstacle_id} | Snap{attempt} {tag}{' (short)' if pos.get('short') else ''} | "
                 + (f"{float(sensor_cm):.0f}cm" if sensor_cm is not None else "angled")
                 + f" | Trial {current_trial()}"]
        if label == "IDEAL":
            lines.append("planned photo pose")
        elif label == "ANGLED":
            lines.append(pos.get("short_text", ""))
        else:
            who = str(pos.get("limit", "")).split(":")[0].split(" (")[0]
            lines.append(f"{pos.get('move', '')} from IDEAL, wanted {pos.get('wanted_cm', 0):g}cm"
                         + (f", cut by {who}" if pos.get("short") else ", full move"))
        if label == "IDEAL":
            lines.append(f"CASE {snap['case']}: {snap['case_reason']}")
        else:
            lines.append(f"taken for CASE {snap['case'] or '?'}")
        body = snap["body"].get("side")
        lines.append(f"count {n}" + ("" if n else " - no image ID found")
                     + f" | body {body.upper() if body else 'not found'}")
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
    photo_results[str(obstacle_id)] = decision
    sent = decision.get("sent", decision["image_id"] is not None)
    send_to_android(obstacle_id, decision["image_id"] if sent else None)
    if sent:
        print(f"    Android       : SENT TARGET,{obstacle_id},{decision['image_id']}")
    else:
        print(f"    Android       : no TARGET sent (MSG 'no image recognised')")
    write_diagnosis(obstacle_id, decision)


def write_diagnosis(obstacle_id, decision):
    photos = decision.get("photos") or []
    out = [f"Obstacle {obstacle_id}: CASE {decision['case']}  {len(photos)} photo(s)  ->  "
           + (f"ID {decision['image_id']} ({decision['letter']}, {decision['conf']:.2f}) from {decision['snap']}"
              if decision["image_id"] is not None else "NOTHING reported"),
           f"    IDEAL gave : {decision.get('ideal_case_reason') or '?'}",
           f"    decision   : {decision['reason']}",
           f"    sent       : {'YES' if decision.get('sent') else 'NO'}",
           "    why:"] + [f"      {l}" for l in decision.get("story", [])] + ["    photos:"]
    for p in photos:
        cm = "?" if p["sensor_cm"] is None else f"{float(p['sensor_cm']):.1f} cm"
        boxes = ", ".join(f"ID {b[0]} {b[1]} {b[2]:.2f}" for b in p["boxes"]) or "no image ID"
        extra = ""
        if p["label"] in ("FAR", "NEAR") and p.get("wanted_cm") is not None:
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
        if p.get("body"):
            out.append(f"        {p['body']}")
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
    if picam2 is None or cv2 is None:
        return None
    MIN_OBSTACLE_AREA_PX = 800  
    try:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None
        biggest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest) < MIN_OBSTACLE_AREA_PX: return None
        x, y, w, h = cv2.boundingRect(biggest)
        box_center_x = x + w / 2
        frame_center_x = FRAME_SIZE[0] / 2
        side = "left" if box_center_x < frame_center_x else "right"
        return side
    except Exception as error:
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
    parsed = ast.literal_eval(text)
    obstacles.clear()
    for entry in parsed:
        obstacle_id, x, y, face = entry
        obstacles[int(obstacle_id)] = (int(obstacle_id), int(x), int(y), str(face or ""))
    return list(obstacles.values())


def extract_message(buffer):
    stripped = buffer.lstrip("\r\n ")
    skipped = len(buffer) - len(stripped)
    prefix = buffer[:skipped]
    if not stripped: return None, None, buffer

    if stripped.startswith("["):
        depth = 0
        in_string = False
        for i, char in enumerate(stripped):
            if char == '"' and (i == 0 or stripped[i - 1] != "\\"): in_string = not in_string
            elif not in_string:
                if char == "[": depth += 1
                elif char == "]":
                    depth -= 1
                    if depth == 0: return "SNAPSHOT", stripped[: i + 1], prefix + stripped[i + 1 :]
        return None, None, buffer

    for kind, regex in [("ADD", ADD_RE), ("SUB", SUB_RE), ("FACE", FACE_RE), ("ROBOT", ROBOT_RE), ("TOKEN", TOKEN_RE)]:
        match = regex.match(stripped)
        if match:
            return kind, match if kind != "TOKEN" else match.group(1), prefix + stripped[match.end() :]

    if len(stripped) > 256:
        return "DROP", None, prefix + stripped[1:]

    return None, None, buffer


def open_android_port():
    global android_fd
    while True:
        if os.path.exists(ANDROID_PORT):
            try:
                android_fd = os.open(ANDROID_PORT, os.O_RDWR | os.O_NOCTTY)
                return
            except OSError as error:
                pass
        time.sleep(1)


def close_android_port():
    global android_fd
    if android_fd is not None:
        try: os.close(android_fd)
        except OSError: pass
        android_fd = None


def wait_for_android_map(android_only):
    global START_POSE
    open_android_port()
    obstacles = {}
    buffer = ""
    complete = False

    try:
        while True:
            chunk = os.read(android_fd, 1024)
            if not chunk: break
            buffer += chunk.decode("utf-8", errors="replace")

            while True:
                kind, payload, buffer = extract_message(buffer)
                if kind is None: break
                if kind == "DROP": continue

                if kind == "ADD":
                    obstacle_id = int(payload.group(1))
                    x, y = int(payload.group(2)), int(payload.group(3))
                    previous = obstacles.get(obstacle_id)
                    obstacles[obstacle_id] = (obstacle_id, x, y, previous[3] if previous else "")
                elif kind == "SUB":
                    obstacles.pop(int(payload.group(1)), None)
                elif kind == "FACE":
                    obstacle_id = int(payload.group(1))
                    obstacles[obstacle_id] = (obstacle_id, int(payload.group(3)), int(payload.group(4)), payload.group(2))
                elif kind == "ROBOT":
                    START_POSE = (int(payload.group(1)), int(payload.group(2)), payload.group(3))
                elif kind == "SNAPSHOT":
                    apply_snapshot(payload, obstacles)
                    send_line(f"MSG,Arena received ({len(obstacles)} obstacles). Press Task 1 Explore to start.")
                elif kind == "TOKEN" and payload == "beginExplore":
                    if obstacles and all(o[3] for o in obstacles.values()):
                        complete = True

                if complete and not android_only:
                    return [obstacles[key] for key in sorted(obstacles)]
    except KeyboardInterrupt: pass
    return [obstacles[key] for key in sorted(obstacles)]


# ----------------------------------------------------------------------
# Command file
# ----------------------------------------------------------------------

def write_cmds_file(commands):
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
# Networking & Execution
# ----------------------------------------------------------------------

def recv_json_line(sock, buffer=b""):
    while b"\n" not in buffer:
        chunk = sock.recv(4096)
        if not chunk: raise ConnectionError("Server closed the connection early.")
        buffer += chunk
    line_bytes, buffer = buffer.split(b"\n", 1)
    return json.loads(line_bytes.decode("utf-8")), buffer


def parse_args():
    flags = {"--dry-run", "--android-only", "--from-android"}
    dry_run = "--dry-run" in sys.argv
    android_only = "--android-only" in sys.argv
    args = [a for a in sys.argv[1:] if a not in flags]
    host = args[0] if args else "127.0.0.1"
    return host, dry_run, android_only


def request_plan(host, obstacles):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as rpi_socket:
        rpi_socket.settimeout(10)
        try:
            rpi_socket.connect((host, PORT))
        except (socket.timeout, ConnectionRefusedError, OSError): return None

        rpi_socket.settimeout(60)
        request = {"obstacles": [list(obstacle) for obstacle in obstacles], "start": list(START_POSE)}
        rpi_socket.sendall((json.dumps(request) + "\n").encode("utf-8"))
        reply, _ = recv_json_line(rpi_socket, b"")
        return reply if reply["status"] == "SUCCESS" else None


def print_plan_summary(reply, total_obstacles, trial):
    order = reply["order"]
    where = "Start"
    for seg in reply.get("segments", []):
        if seg.get("target"):
            planned_targets[str(seg["obstacle_id"])] = seg["target"]


def print_run_summary(order, trial):
    print("\n" + "=" * 64 + f"\n  RUN SUMMARY  (Trial {trial})\n" + "=" * 64)
    for obstacle_id in order:
        d = photo_results.get(str(obstacle_id))
        if d is not None:
            status = "SENT" if d.get("sent", True) and d["image_id"] is not None else "NOT sent"
            print(f"  Obstacle {obstacle_id}: CASE {d['case']} -> {status}")


def send_images_to_server(host, photo_results_dict):
    """Encodes recognized images as base64 and ships them to the laptop for tiling."""
    images_payload = []
    for obs_id, decision in photo_results_dict.items():
        if decision.get("sent") and decision.get("file"):
            try:
                with open(decision["file"], "rb") as f:
                    b64 = base64.b64encode(f.read()).decode('utf-8')
                images_payload.append({
                    "obstacle_id": obs_id,
                    "image_id": decision["image_id"],
                    "letter": decision.get("letter", ""),
                    "data": b64
                })
            except Exception as e:
                print(f"Failed to read image for obstacle {obs_id}: {e}")
                
    if not images_payload:
        print("No recognized images to send to PC.")
        return

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(10)
            s.connect((host, PORT))
            req = {"type": "DISPLAY_IMAGES", "images": images_payload}
            s.sendall((json.dumps(req) + "\n").encode("utf-8"))
            print("Successfully sent recognized images to PC for tiling.")
    except Exception as e:
        print(f"Error sending images to PC: {e}")


def dispatch(path, commands, dry_run, start_cm=None, host=None): # Added host parameter
    if dry_run: return True
    import rpi_stm_conn
    tracker = PoseTracker(start_cm or (15, 15, "N"))
    tracker.publish(force=True)
    original_send_and_wait = rpi_stm_conn.send_and_wait

    def tracked_send_and_wait(ser, cmd):
        if stop_requested: raise StopRequested()
        send_line("MSG,[Moving]")
        result, line = original_send_and_wait(ser, cmd)
        if result is True: tracker.apply(cmd, line)
        return result, line
        
    # --- NEW REAL-TIME CALLBACK ---
    def realtime_report(obstacle_id, decision):
        report_result(obstacle_id, decision)
        # Immediately send to PC if it's a valid, sent image
        if host and decision.get("sent"):
            send_images_to_server(host, {str(obstacle_id): decision})

    rpi_stm_conn.send_and_wait = tracked_send_and_wait
    watch_for_stop()
    try:
        return rpi_stm_conn.run_commands(
            rpi_stm_conn.read_cmds_file(path),
            on_snap=trigger_camera,
            on_check_position=check_obstacle_side,
            targets=planned_targets,
            on_result=realtime_report, # Replaced report_result with realtime_report
        )
    except StopRequested:
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
        if android_only or not remaining: return

        reply = request_plan(host, remaining)
        if reply is None: return

        trial = current_trial()
        print_plan_summary(reply, len(remaining), trial)
        commands = reply["commands"]
        if not commands: return

        path = write_cmds_file(commands)
        send_line("MSG,Run started")
        
        # Pass the host into dispatch so the real-time callback can use it
        ok = dispatch(path, commands, dry_run, reply.get("start_cm"), host=host)

        if ok:
            done_msgs = []
            if reply["order"]:
                done_msgs.append(f"MSG,Reached final obstacle {reply['order'][-1]}")
            done_msgs.append("MSG,Run complete. Done !")
            for done_msg in done_msgs:
                time.sleep(0.2)
                send_line(done_msg)

        if not dry_run:
            print_run_summary(reply["order"], trial)

    finally:
        stop_camera()
        close_android_port()


if __name__ == "__main__":
    main()

