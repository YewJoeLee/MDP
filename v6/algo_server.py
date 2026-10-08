"""
Algo server (v5). Runs on the laptop.

v5: every segment's "target" also carries "retry": the FAR (BW) and NEAR (FW)
straight moves the RPi may make for extra photos, already cut short to keep
MIN_CLEARANCE_CM. See planner.snap_retry_bounds().

Protocol: one JSON object per line, UTF-8, newline terminated.
"""

import socket
import json
import traceback
import queue
import math
import base64

try:
    import cv2
    import numpy as np
except ImportError:
    print("Warning: cv2 or numpy not installed. Tiled image display will not work.")
    cv2, np = None, None

import planner
import config

HOST = "0.0.0.0"
PORT = 5000

# Queue to safely pass image data from the socket thread to the main GUI thread
display_queue = queue.Queue()
accumulated_images = []

def show_tiled_images(images_data):
    """Decode, accumulate, resize, and tile the received images in real-time."""
    global accumulated_images
    if not cv2 or not np or not images_data:
        return

    # Decode and append new images to our running list
    for item in images_data:
        try:
            img_bytes = base64.b64decode(item["data"])
            np_arr = np.frombuffer(img_bytes, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                label = f"Obs: {item['obstacle_id']} | ID: {item['image_id']} ({item.get('letter','')})"
                cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                accumulated_images.append(img)
        except Exception as e:
            print(f"Error decoding image for obstacle {item.get('obstacle_id')}: {e}")

    if not accumulated_images:
        return

    # Calculate grid dimensions based on the TOTAL accumulated images
    n = len(accumulated_images)
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)

    # Uniform size for tiling
    h, w = 416, 416
    resized = [cv2.resize(img, (w, h)) for img in accumulated_images]

    # Pad with blank frames if the grid isn't full
    while len(resized) < rows * cols:
        resized.append(np.zeros((h, w, 3), dtype=np.uint8))

    # Stack horizontally then vertically
    grid_rows = []
    for r in range(rows):
        row_imgs = resized[r * cols : (r + 1) * cols]
        grid_rows.append(np.hstack(row_imgs))

    tiled = np.vstack(grid_rows)
    cv2.imshow("Task 1 - Recognized Images (Real-Time)", tiled)
    print(f"Updated PC display: now showing {n} image(s).")


def handle_request(payload):
    """Turn one decoded request dict into one response dict."""
    if not isinstance(payload, dict):
        raise ValueError("Request must be a JSON object")
        
    # Handle the end-of-run image transfer
    if payload.get("type") == "DISPLAY_IMAGES":
        display_queue.put(payload.get("images", []))
        return {"status": "SUCCESS", "message": "Images received for display."}

    raw_obstacles = payload.get("obstacles", [])

    if len(raw_obstacles) == 0:
        return {"status": "ERROR", "message": "No obstacles supplied."}

    obstacles = planner.normalize_obstacles(raw_obstacles)
    if "start_cm" in payload:
        start_pose = planner.normalize_pose(payload["start_cm"])
    elif "start" not in payload or tuple(payload["start"]) == config.LEGACY_DEFAULT_START:
        start_pose = config.START_POSE_CM
    else:
        legacy = planner.normalize_pose(payload["start"])
        start_pose = ((legacy[0]+0.5)*config.CELL_CM,
                      (legacy[1]+0.5)*config.CELL_CM, legacy[2])

    result = planner.plan(obstacles, start_pose=start_pose)

    if result["status"] != "SUCCESS":
        return {
            "status": "ERROR",
            "message": result.get("message", "Planning failed."),
            "skipped": result.get("skipped", []),
        }

    return {
        "status": "SUCCESS",
        "commands": result["commands"],
        "moves": result["optimal_moves"],
        "order": result["order"],
        "total_cost": result["total_cost"],
        "planning_ms": result["planning_ms"],
        "skipped": result["skipped"],
        "segments": result["segments"],
        "start_cm": result["start"],
        "pose_units": "cm",
        "route_policy": result["route_policy"],
    }


def serve_connection(conn, addr):
    """
    Read newline-delimited JSON requests until the peer hangs up.
    Buffers raw bytes to handle partial TCP packets.
    """
    buffer = b""
    with conn:
        while True:
            # Increased buffer chunk size to accommodate large base64 image payloads
            chunk = conn.recv(65536)

            if not chunk:
                print(f"[{addr}] disconnected")
                return

            buffer += chunk

            while b"\n" in buffer:
                line_bytes, buffer = buffer.split(b"\n", 1)
                line = line_bytes.decode("utf-8").strip()

                if not line:
                    continue

                try:
                    payload = json.loads(line)
                    response = handle_request(payload)
                except Exception as error:
                    traceback.print_exc()
                    response = {"status": "ERROR", "message": str(error)}

                if response.get("status") == "SUCCESS" and payload.get("type") != "DISPLAY_IMAGES":
                    print(f"[{addr}] order {response.get('order', [])} "
                          f"cost {response.get('total_cost', 0)} "
                          f"in {response.get('planning_ms', 0)} ms "
                          f"-> {len(response.get('commands', []))} commands")
                elif response.get("status") != "SUCCESS":
                    print(f"[{addr}] error: {response.get('message')}")

                encoded = (json.dumps(response) + "\n").encode("utf-8")
                conn.sendall(encoded)


def show_local_addresses():
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        primary = probe.getsockname()[0]
    except Exception:
        primary = None
    finally:
        probe.close()

    if primary:
        print(f"This laptop appears to be {primary}")
        print(f"On the Pi run:  python3 rpi_client_android.py {primary}")
    else:
        print("Could not auto-detect the Wi-Fi address. Run ipconfig and")
        print("use the IPv4 address of your wireless adapter.")


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    
    # Timeout allows the main thread to remain unblocked to service OpenCV GUI events
    server.settimeout(0.5) 
    server.listen(1)

    print(f"Algo server listening on port {PORT}...")
    show_local_addresses()
    print()

    try:
        while True:
            try:
                conn, addr = server.accept()
                print(f"Connected by {addr}")
                try:
                    serve_connection(conn, addr)
                except ConnectionResetError:
                    print(f"[{addr}] connection reset")
                except Exception:
                    traceback.print_exc()
            except socket.timeout:
                pass # Normal timeout, loop continues

            # Poll the queue for images to display
            while not display_queue.empty():
                images_data = display_queue.get()
                show_tiled_images(images_data)

            # Process GUI events to keep the image window responsive
            if cv2 is not None:
                cv2.waitKey(1)

    except KeyboardInterrupt:
        print("\nShutting down.")

    finally:
        server.close()
        if cv2 is not None:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

