"""Task 2 test snaps: take a straight photo on the RPi and save it as
photos/task_2_snap_<x>.jpg, where x counts up (1, 2, 3, ...) and never
overwrites an earlier photo.

Usage (on the RPi, in this folder):
    python3 snap_task2.py              # one photo, then exit
    python3 snap_task2.py --loop       # photo every time you press Enter, q + Enter to quit
    python3 snap_task2.py --full       # full sensor resolution instead of 416x416

Same camera setup as rpi_client.py (Picamera2, 416x416, BGR888), so the
photos look like what YOLO sees during a run. Stop rpi_client.py first:
only one program can hold the camera.
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


def open_camera(full):
    from picamera2 import Picamera2
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
