"""
Standalone script to test camera capture and YOLO inference on the Raspberry Pi
without running the STM motor controls, algo server, or Android Bluetooth.

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

MODEL_PATH = "best_ncnn_model"
FRAME_SIZE = (416, 416)
PHOTO_DIR = "photos"

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


def main():
    parser = argparse.ArgumentParser(description="Test camera and YOLO detection on RPi.")
    parser.add_argument("--loop", action="store_true", help="Snap repeatedly on Enter keypress.")
    parser.add_argument("--conf", type=float, default=0.25, help="YOLO confidence threshold (default: 0.25).")
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
            boxes = results[0].boxes

            letter, image_id, confidence = None, None, 0.0
            if boxes is not None and len(boxes) > 0:
                best = boxes.conf.argmax()
                letter = model.names[int(boxes.cls[best])]
                image_id = letter2number.get(letter)
                confidence = float(boxes.conf[best])

            is_target = image_id is not None and str(image_id).isdigit()

            # Stamp text on saved image
            line1 = (f"Detected: {letter} -> ID {image_id} ({confidence:.2f})"
                     if is_target else f"Detected: {letter or 'None'}")
            line2 = f"Time: {timestamp} | Latency: {infer_ms:.1f}ms"
            for text, y in ((line1, 30), (line2, 58)):
                cv2.putText(annotated_frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(annotated_frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.imwrite(filepath, annotated_frame)

            print(f"  Inference : {infer_ms:.1f} ms")
            if letter:
                shown_id = image_id if is_target else "-"
                print(f"  Detected  : {letter} (ID: {shown_id}) with confidence: {confidence:.2f}")
            else:
                print("  Detected  : No object detected")
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
