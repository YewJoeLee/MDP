import socket
import json

LAPTOP_IP = '192.168.X.X'  # Replace with Laptop's actual Wi-Fi IP
PORT = 5000

rpi_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
rpi_socket.connect((LAPTOP_IP, PORT))

# Forward obstacle coordinates received from Android to Algo
obstacle_payload = json.dumps({"x": 5, "y": 10, "dir": "N"})
rpi_socket.sendall(obstacle_payload.encode('utf-8'))

# Receive path from Algo
reply = rpi_socket.recv(1024).decode('utf-8')
movement_list = json.loads(reply)["commands"]

# Loop through and forward movements to STM / Image Recognition
for cmd in movement_list:
    if cmd == "SNAP":
        trigger_camera() #Replace with camera function
    else:
        send_to_stm_via_uart(cmd) #Replace with stm interface function

