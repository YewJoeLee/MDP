"""
v5. Standalone script to test camera capture and YOLO inference on the Raspberry Pi
without running the STM motor controls, algo server, or Android Bluetooth.

Prints EVERY box the model returns and the v5 "count" (boxes with a numeric
image ID; "Bounding box" is not counted), plus which v5 case (A/B/C/D) the
photo would fall into at the IDEAL pose. Uses the same read_detections() as
rpi_client_android.py, so what you see here is what the run will see.

Usage:
    python3 test_camera.py              # Take one photo, run YOLO, save & print results
    python3 test_camera.py --loop       # Keep taking photos on Enter keypress
    python3 test_camera.py --conf 0.35  # Custom confidence threshold (default: 0.25)
"""

import argparse
import os
import sys
import time
from datetime import datetime

import config
from rpi_client_android import letter2number, read_detections  # noqa: F401 (same mapping as the run)

MODEL_PATH = "best_ncnn_model"
FRAME_SIZE = (416, 416)
PHOTO_DIR = "photos"


def main():
    parser = argparse.ArgumentParser(description="Test camera and YOLO detection on RPi.")
    parser.add_argument("--loop", action="store_true", help="Snap repeatedly on Enter keypress.")
    parser.add_argument("--conf", type=float, default=config.MODEL_MIN_CONF,
                        help=f"YOLO confidence threshold (default: {config.MODEL_MIN_CONF}).")
    parser.add_argument("--model", type=str, default=MODEL_PATH, help=f"Path to model (default: {MODEL_PATH}).")
    args = parser.parse_args()

    print("=" * 60)
    print(" CAMERA & YOLO TEST UTILITY")
    print("=" * 60)

    try:
        from picamera2 import Picamera2
        from ultralytics import YOLO
        import cv2
    except ImportError as e:
        print(f"[ERROR] Missing dependency: {e}")
        print("Please make sure picamera2, ultralytics, and opencv-python are installed.")
        sys.exit(1)

    os.makedirs(PHOTO_DIR, exist_ok=True)

    print(f"Initializing Picamera2 ({FRAME_SIZE[0]}x{FRAME_SIZE[1]})...")
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = FRAME_SIZE
    picam2.preview_configuration.main.format = "BGR888"
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()

    print(f"Loading YOLO model '{args.model}'...")
    try:
        model = YOLO(args.model)
    except Exception as e:
        print(f"[ERROR] Failed to load model '{args.model}': {e}")
        picam2.stop()
        sys.exit(1)

    # Warmup
    print("Warming up model...")
    warmup = picam2.capture_array()
    model(warmup, conf=args.conf, imgsz=416, verbose=False)
    print("Ready!\n")

    snap_count = 0
    try:
        while True:
            snap_count += 1
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"test_snap_{timestamp}_{snap_count}.jpg"
            filepath = os.path.join(PHOTO_DIR, filename)

            print(f"--- Snap #{snap_count} ---")
            frame = picam2.capture_array()
            start_t = time.time()
            results = model(frame, conf=args.conf, imgsz=416, verbose=False)
            infer_ms = (time.time() - start_t) * 1000

            annotated_frame = results[0].plot()
            targets, ignored = read_detections(results[0], model.names)
            n = len(targets)
            top = targets[0] if n else None

            if n == 1 and top["conf"] >= config.RECOGNISED_CONF:
                case = f"CASE A - 1 image, conf >= {config.RECOGNISED_CONF:.2f}: report it"
            elif n == 1:
                case = f"CASE B - 1 image, conf < {config.RECOGNISED_CONF:.2f}: run would take FAR + NEAR photos"
            elif n == 0:
                case = "CASE C - no image: run would report nothing"
            else:
                case = f"CASE D - {n} images: run would take a NEAR photo"

            # Stamp text on saved image
            line1 = (f"count {n}  top: ID {top['image_id']} {top['letter']} ({top['conf']:.2f})"
                     if top else f"count 0  (no image ID)")
            line2 = f"Time: {timestamp} | Latency: {infer_ms:.1f}ms"
            for text, y in ((line1, 30), (line2, 58)):
                cv2.putText(annotated_frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(annotated_frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.imwrite(filepath, annotated_frame)

            print(f"  Frame     : {frame.shape[1]} x {frame.shape[0]} px")
            print(f"  Inference : {infer_ms:.1f} ms")
            print(f"  Model saw : {n + len(ignored)} box(es) at conf >= {args.conf:.2f}")
            i = 0
            for d in targets:
                i += 1
                print(f"      {i}. {d['letter']:<13} ID {str(d['image_id']):<3} conf {d['conf']:.2f}"
                      f"   centre x {d['cx']:.0f} px, width {d['w']:.0f} px")
            for d in ignored:
                i += 1
                print(f"      {i}. {d['letter']:<13} conf {d['conf']:.2f}   (not an image ID - not counted)")
            print(f"  Count     : {n}")
            print(f"  v5 case   : {case}")
            print(f"  Saved to  : {filepath}\n")

            if not args.loop:
                break

            user_input = input("Press [Enter] to snap again, or 'q' to quit: ").strip().lower()
            if user_input == 'q':
                break

    finally:
        print("Stopping camera...")
        picam2.stop()
        print("Done.")


if __name__ == "__main__":
    main()
