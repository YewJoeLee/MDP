"""Task 2 test snaps: take a straight photo on the RPi and save it as
photos/task_2_snap_<x>.jpg, where x counts up (1, 2, 3, ...) and never
overwrites an earlier photo.

Usage (on the RPi, in this folder):
    python3 snap_task2.py              # one photo, then exit
    python3 snap_task2.py --loop       # photo every time you press Enter, q + Enter to quit
    python3 snap_task2.py --full       # full sensor resolution instead of 416x416

Same camera setup as rpi_client.py (Picamera2, 416x416, BGR888), so the
photos look like what YOLO sees during a run. Any other program still holding the camera
(e.g. a leftover rpi_client_task2.py) is stopped automatically.
"""
import argparse
import os
import re
import sys
import time

PHOTO_DIR = "photos"
PREFIX = "task_2_snap_"
FRAME_SIZE = (416, 416)          # same as rpi_client.py
WARMUP_S = 1.0                   # let exposure / white balance settle


def next_index():
    """1 + the highest x already in photos/ (gaps are not reused)."""
    pat = re.compile(rf"^{PREFIX}(\d+)\.jpg$")
    nums = [int(m.group(1)) for n in os.listdir(PHOTO_DIR) if (m := pat.match(n))]
    return max(nums, default=0) + 1


# =============================================================================
# Free the camera: stop any leftover program that still holds it
# =============================================================================

def camera_holders():
    """{pid: command line} of other processes with /dev/media* or /dev/video* open."""
    import glob, stat
    devs = set()
    for p in glob.glob("/dev/media*") + glob.glob("/dev/video*"):
        try:
            devs.add(os.stat(p).st_rdev)
        except OSError:
            pass
    me, found = os.getpid(), {}
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) == me:
            continue
        try:
            for fd in os.listdir(f"/proc/{d}/fd"):
                try:
                    st = os.stat(f"/proc/{d}/fd/{fd}")
                except OSError:
                    continue
                if stat.S_ISCHR(st.st_mode) and st.st_rdev in devs:
                    with open(f"/proc/{d}/cmdline", "rb") as f:
                        found[int(d)] = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
                    break
        except OSError:                      # process gone, or another user's (root) process
            continue
    return found


def free_camera(timeout_s=4.0):
    """Stop every other process holding the camera. Returns True when the camera is free.
    Ctrl+C first (lets the old script close the camera and stop the robot cleanly),
    then SIGTERM, then SIGKILL. SIGCONT wakes a script paused with Ctrl+Z."""
    import signal
    holders = camera_holders()
    if not holders:
        return True
    for pid, cmd in holders.items():
        print(f"   camera held by PID {pid} ({cmd or '?'}) - stopping it")
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for pid in list(holders):
            try:
                os.kill(pid, signal.SIGCONT)
                os.kill(pid, sig)
            except ProcessLookupError:
                holders.pop(pid)
            except PermissionError:
                print(f"   ! no permission to stop PID {pid} - run:  sudo kill {pid}")
                return False
        t_end = time.time() + timeout_s
        while time.time() < t_end:
            holders = {p: c for p, c in holders.items() if os.path.exists(f"/proc/{p}")}
            if not holders:
                time.sleep(0.5)                  # let libcamera release the pipeline
                return not camera_holders()
            time.sleep(0.1)
    return not camera_holders()


def open_camera(full):
    from picamera2 import Picamera2
    free_camera()
    cam = Picamera2()
    if full:
        cam.configure(cam.create_still_configuration(main={"format": "BGR888"}))
    else:
        cam.preview_configuration.main.size = FRAME_SIZE
        cam.preview_configuration.main.format = "BGR888"
        cam.preview_configuration.align()
        cam.configure("preview")
    cam.start()
    time.sleep(WARMUP_S)
    return cam


def snap(cam, cv2):
    frame = cam.capture_array()
    path = os.path.join(PHOTO_DIR, f"{PREFIX}{next_index()}.jpg")
    if not cv2.imwrite(path, frame):
        print(f"[ERROR] could not write {path}")
        return None
    h, w = frame.shape[:2]
    print(f"Saved {path}  ({w}x{h})")
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--loop", action="store_true", help="photo on every Enter, q to quit")
    ap.add_argument("--full", action="store_true", help="full sensor resolution")
    args = ap.parse_args()

    os.makedirs(PHOTO_DIR, exist_ok=True)
    try:
        import cv2
        cam = open_camera(args.full)
    except Exception as e:
        print(f"[ERROR] camera not available: {e}")
        print("Is rpi_client.py (or another camera program) still running?")
        sys.exit(1)

    try:
        if not args.loop:
            snap(cam, cv2)
            return
        print("Enter = take photo, q + Enter = quit")
        while True:
            if input().strip().lower() in ("q", "quit", "exit"):
                break
            snap(cam, cv2)
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        cam.stop()
        cam.close()


if __name__ == "__main__":
    main()
