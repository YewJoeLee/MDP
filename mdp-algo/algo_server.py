import socket
import json
import planner  # Imports planner.py from the same folder

HOST = '0.0.0.0'  # Listen on all local interfaces
PORT = 5000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)

print(f"Algo Server running. Listening on port {PORT}...")

while True:
    conn, addr = server.accept()
    print(f"Connected by RPi: {addr}")
    
    try:
        while True:
            data = conn.recv(1024).decode('utf-8')
            if not data:
                break
            
            # 1. Parse JSON received from RPi
            # Expected input format: {"obstacles": [[1, 5, 10, "N"]], "start": [1, 1, "N"]}
            payload = json.loads(data)
            
            # Extract obstacles and optional start pose
            obstacles = [tuple(obs) for obs in payload.get("obstacles", [])]
            start_pose = tuple(payload.get("start", (1, 1, "N")))
            
            # 2. Run pathfinding via planner.py
            result = planner.plan(obstacles, start_pose=start_pose)
            
            # Extract move list (e.g., ["FW", "FL", "SNAP1"])
            moves = result["optimal_moves"]
            
            # 3. Return movements back to RPi
            response = json.dumps({"status": "SUCCESS", "commands": moves}) + "\n"
            conn.sendall(response.encode('utf-8'))
            
    except Exception as e:
        print(f"Error handling request: {e}")
        error_resp = json.dumps({"status": "ERROR", "message": str(e)}) + "\n"
        conn.sendall(error_resp.encode('utf-8'))
    finally:
        conn.close()

