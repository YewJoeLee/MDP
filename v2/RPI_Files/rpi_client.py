"""
RPi client for Android integration.

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
    payload = "TARGET," + str(obstacle_id) + "," + str(image_id) + "\n"
    try:
        if android_fd is not None:
            os.write(android_fd, payload.encode("utf-8"))
        else:
            with open(ANDROID_PORT, "w") as f:
                f.write(payload)
        print(f"    [ANDROID] TARGET,{obstacle_id},{image_id}")
    except Exception as error:
        print(f"    [ANDROID] failed: {error}")
        traceback.print_exc()


def trigger_camera(obstacle_id):
    """
    Called by rpi_stm_conn.run_commands whenever it meets SNAP<id>.

    Returns True if something was recognised (and sent to Android), False
    otherwise (no camera, nothing detected, or an error). rpi_stm_conn uses
    this return value to decide whether to attempt the angle correction.
    """
    print(f"    [CAMERA] capture image for obstacle {obstacle_id}")

    if picam2 is None or model is None:
        print("    [CAMERA] no camera, skipping capture")
        return False

    try:
        frame = picam2.capture_array()
        results = model(frame, conf=0.25, imgsz=416)
        final_frame = results[0].plot()
        boxes = results[0].boxes

        image_id = None
        recognised = False

        if boxes is not None and len(boxes) > 0:
            most_probable = boxes.conf.argmax()
            class_id = int(boxes.cls[most_probable])
            letter = model.names[class_id]
            image_id = letter2number.get(letter)
            confidence = float(boxes.conf[most_probable])
            print(f"    [DETECT] {letter} -> {image_id} ({confidence:.2f})")
            recognised = True   
        else:
            print("    [DETECT] nothing found")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(
            PHOTO_DIR, f"obstacle{obstacle_id}_{timestamp}.jpg")
        cv2.imwrite(filename, final_frame)
        print(f"    [CAMERA] saved {filename}")

        send_to_android(obstacle_id, image_id)

        return recognised

    except Exception as error:
        print(f"    [CAMERA] capture failed: {error}")
        traceback.print_exc()
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
    print("Place obstacles, then tap SEND ARENA (or Task 1 Explore).")
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

    Prints every received coordinate. Completes on SEND ARENA (`[...]`)
    or `beginExplore` if at least one obstacle was already added.

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
                    print("[ANDROID] SEND ARENA snapshot:")
                    print_map(obstacles, START_POSE)
                    complete = True

                elif kind == "TOKEN":
                    print(f"[ANDROID] command {payload}")
                    if payload == "beginExplore" and obstacles:
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

def print_plan(commands):
    """Numbered command list, with align and photo steps labelled."""
    print("Planned commands:")
    for n, command in enumerate(commands, 1):
        upper = command.upper()
        if upper.startswith("SNAP"):
            note = "  <- photo of obstacle " + command[4:]
        elif upper.startswith(ALIGN_PREFIX):
            target = command[len(ALIGN_PREFIX):] or "STM default"
            note = f"  <- ultrasonic align to {target} cm"
        else:
            note = ""
        print(f"  {n:>3}. {command}{note}")


def dispatch(path, commands, dry_run):
    print_plan(commands)
    print()

    if dry_run:
        print("Dry run, not opening the serial port.")
        return

    # Imported here so --dry-run works on a machine without pyserial.
    import rpi_stm_conn

    print("Dispatching commands:")
    rpi_stm_conn.run_commands(
        rpi_stm_conn.read_cmds_file(path),
        on_snap=trigger_camera,
        on_check_position=check_obstacle_side
    )


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

        print()
        print("Visit order      :", reply["order"])
        print("Total cost       :", reply["total_cost"])
        print("Planning time    :", reply["planning_ms"], "ms")
        print("Command count    :", len(reply["commands"]))

        if reply["skipped"]:
            print("Skipped obstacles:")
            for entry in reply["skipped"]:
                print("   obstacle", entry["obstacle_id"], "-", entry["reason"])

        commands = reply["commands"]

        if not commands:
            print("\nNo commands to run.")
            return

        path = write_cmds_file(commands)
        print(f"\nWrote {len(commands)} commands to {path}")

        print()
        dispatch(path, commands, dry_run)

        print()
        print("Run complete.")

    finally:
        stop_camera()
        close_android_port()


if __name__ == "__main__":
    main()