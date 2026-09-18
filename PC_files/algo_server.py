"""
Algo server. Runs on the laptop.

Protocol: one JSON object per line, UTF-8, newline terminated.

Request:
    {"obstacles": [[1, 4, 13, "S"], [2, 13, 13, "N"]], "start": [1, 1, "N"]}

Response:
    {"status": "SUCCESS",
     "commands": ["FW50", "RT90", "SNAP6", ...],
     "moves": ["FW", "FW", ...],
     "order": [6, 2, 8],
     "total_cost": 212.5,
     "planning_ms": 110.6,
     "skipped": [{"obstacle_id": 3, "reason": "..."}]}

    {"status": "ERROR", "message": "..."}
"""

import socket
import json
import traceback

import planner

HOST = "0.0.0.0"
PORT = 5000


def handle_request(payload):
    """Turn one decoded request dict into one response dict."""
    raw_obstacles = payload.get("obstacles", [])

    if len(raw_obstacles) == 0:
        return {"status": "ERROR", "message": "No obstacles supplied."}

    obstacles = [tuple(obstacle) for obstacle in raw_obstacles]
    start_pose = tuple(payload.get("start", (1, 1, "N")))

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
    }


def serve_connection(conn, addr):
    """
    Read newline delimited JSON requests until the peer hangs up.

    A single conn.recv() is not enough: TCP is a byte stream, so one
    recv can return half a message or two messages joined together.
    We buffer and split on newlines instead.
    """
    buffer = ""

    with conn:
        while True:
            chunk = conn.recv(4096)

            if not chunk:
                print(f"[{addr}] disconnected")
                return

            buffer += chunk.decode("utf-8")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()

                if line == "":
                    continue

                try:
                    payload = json.loads(line)
                    response = handle_request(payload)

                except Exception as error:
                    traceback.print_exc()
                    response = {"status": "ERROR", "message": str(error)}

                if response["status"] == "SUCCESS":
                    print(f"[{addr}] order {response['order']} "
                          f"cost {response['total_cost']} "
                          f"in {response['planning_ms']} ms "
                          f"-> {len(response['commands'])} commands")
                    if response["skipped"]:
                        print(f"[{addr}] skipped: {response['skipped']}")
                else:
                    print(f"[{addr}] error: {response['message']}")

                encoded = (json.dumps(response) + "\n").encode("utf-8")
                conn.sendall(encoded)


def show_local_addresses():
    """
    Print the address the Pi should connect to.

    Opening a UDP socket to an arbitrary address does not send anything,
    but it makes the OS pick the interface it would route through, which
    is the Wi-Fi address we actually want.
    """
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
        print(f"On the Pi run:  python3 fake_rpi_client.py {primary}")
    else:
        print("Could not auto-detect the Wi-Fi address. Run ipconfig and")
        print("use the IPv4 address of your wireless adapter.")


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    print(f"Algo server listening on port {PORT}...")
    show_local_addresses()
    print()

    try:
        while True:
            conn, addr = server.accept()
            print(f"Connected by {addr}")

            try:
                serve_connection(conn, addr)
            except ConnectionResetError:
                print(f"[{addr}] connection reset")
            except Exception:
                traceback.print_exc()

    except KeyboardInterrupt:
        print("\nShutting down.")

    finally:
        server.close()


if __name__ == "__main__":
    main()
