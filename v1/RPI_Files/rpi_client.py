"""
Fake RPi client. Stands in for the real Pi during the integration test.

Sends the obstacle list from test_maps.py to the algo server, receives the
hardware command list, writes it to a cmds file, then reads that file back
and dispatches it:

    AC<cm>    -> ultrasonic align, sent to the STM just before each photo;
                 the reading and correction are printed
    SNAP<id>  -> trigger_camera(): capture, run YOLO, save the annotated
                 frame, send TARGET,<obstacle_id>,<image_id> to Android
    anything  -> written to the STM over the UART by rpi_stm_conn

Usage:
    python3 rpi_client.py                        # 127.0.0.1, full run
    python3 rpi_client.py 192.168.1.42           # connects to the laptop
    python3 rpi_client.py --dry-run              # no camera, no serial
    python3 rpi_client.py 192.168.1.42 --dry-run

--dry-run only talks to the algo server and prints the plan, so it runs on
a machine with no camera and no /dev/ttyACM0 (e.g. the laptop).
"""

import socket
import json
import sys
import os
import time
import traceback
from datetime import datetime

from test_maps import OBSTACLES

PORT = 5000
START_POSE = (1, 1, "N")

CMD_DIR = "cmds"
LATEST_NAME = "latest.txt"

PHOTO_DIR = "photos"
ALIGN_PREFIX = "AC"         # must match config.ALIGN_PREFIX on the laptop
ANDROID_PORT = "/dev/rfcomm0"
MODEL_PATH = "best_ncnn_model"
FRAME_SIZE = (416, 416)

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
    try:
        with open(ANDROID_PORT, "w") as f:
            f.write("TARGET," + str(obstacle_id) + "," + str(image_id) + "\n")
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

    Returns "left", "right", or None if the obstacle isn't in frame at all.
    """
    if picam2 is None:
        print("    [CAMERA] no camera, cannot check obstacle side")
        return None
    if cv2 is None:
        print("    [CAMERA] cv2 not available, cannot check obstacle side")
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
            print("    [CAMERA] no dark obstacle body found")
            return None

        biggest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest) < MIN_OBSTACLE_AREA_PX:
            print("    [CAMERA] obstacle not in view (largest dark region too small)")
            return None

        x, y, w, h = cv2.boundingRect(biggest)
        box_center_x = x + w / 2
        frame_center_x = FRAME_SIZE[0] / 2

        side = "left" if box_center_x < frame_center_x else "right"
        print(f"    [CAMERA] obstacle body centre {box_center_x:.0f}px vs "
              f"frame centre {frame_center_x:.0f}px -> {side}")
        return side

    except Exception as error:
        print(f"    [CAMERA] side check failed: {error}")
        traceback.print_exc()
        return None


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
    """Returns (host, dry_run). --dry-run may appear in any position."""
    args = [a for a in sys.argv[1:] if a != "--dry-run"]
    dry_run = "--dry-run" in sys.argv

    host = args[0] if args else "127.0.0.1"

    return host, dry_run


def request_plan(host):
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
            "obstacles": [list(obstacle) for obstacle in OBSTACLES],
            "start": list(START_POSE),
        }

        rpi_socket.sendall((json.dumps(request) + "\n").encode("utf-8"))
        print(f"Sent {len(OBSTACLES)} obstacles.")

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
        on_check_position=check_obstacle_side,
    )


def main():
    host, dry_run = parse_args()

    if not dry_run:
        start_camera()

    try:
        print(f"Connecting to algo server at {host}:{PORT} ...")

        reply = request_plan(host)

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


if __name__ == "__main__":
    main()