import time
import threading
import zmq
from djitellopy import Tello
from enum import Enum
import socket
import json

class TelloCommand(Enum):
    TAKEOFF = 0
    LAND = 1
    EMERGENCY = 2
    UP = 3
    DOWN = 4
    LEFT = 5
    RIGHT = 6
    FORWARD = 7
    BACKWARD = 8
    ROTATE_CW = 9
    ROTATE_CCW = 10
    STOP = 11
    FLIP_LEFT = 12
    FLIP_RIGHT = 13
    FLIP_FORWARD = 14
    FLIP_BACK = 15
    SET_SPEED = 16
    UNKNOWN = 17
# ================= ZMQ =================
ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:6001")




# ================= TELLO =================
tello = Tello()
tello.connect()
print("[TELLO] Battery:", tello.get_battery())

running = True

# ================= TELEMETRY THREAD =================
def telemetry_loop():
    print("[TELLO] Telemetry thread started")
    last = time.time()

    while running:
        state = tello.get_current_state()
        now = time.time()
        dt = now - last
        last = now

        msg = {
            "type": "tello_state",
            "vgx": state.get("vgx", 0),
            "vgy": state.get("vgy", 0),
            "vgz": state.get("vgz", 0),
            "yaw": state.get("yaw", 0),
            "h": state.get("h", 0),
            "bat": state.get("bat", 0),
            "dt": dt
        }

        pub.send_json(msg)
        time.sleep(0.05)   # 20 Hz

# ================= COMMAND THREAD =================
def command_sequence():
    tello.streamon()
    print("[TELLO] Starting video stream...")

    time.sleep(2)

    HOST = "127.0.0.1"
    PORT = 5588

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    print(f"[TELLO CMD] Listening on tcp://{HOST}:{PORT}")

    conn, addr = server.accept()   # 🔒 BLOCKS
    print(f"[TELLO CMD] Client connected from {addr}")

    buffer = b""

    while True:
        data = conn.recv(4096)     # 🔒 BLOCKS
        if not data:
            print("[TELLO CMD] Client disconnected")
            break

        buffer += data

        # Handle stream framing (JSON per message)
        try:
            msg = json.loads(buffer.decode())
            buffer = b""
        except json.JSONDecodeError:
            continue  # wait for more data

        for cmd in msg["commands"]:
            cmd_str = cmd["command"]
            value = cmd["value"]

            try:
                key = int(cmd_str)
                cmd_enum = TelloCommand(key)
            except (ValueError, KeyError):
                print(f'key error key: <{cmd_str}>' )
                cmd_enum = TelloCommand.UNKNOWN

            print(f"[TELLO CMD] {cmd_enum} {value}")

            # OPTIONAL: execute command here
            # tello.send_command(f"{cmd_str.lower()} {value}")

        # ACK to client
        conn.sendall(b"OK")

    conn.close()
    server.close()
        

        

    
	

# ================= START =================
telemetry_thread = threading.Thread(target=telemetry_loop, daemon=True)
command_thread = threading.Thread(target=command_sequence)

telemetry_thread.start()
command_thread.start()

command_thread.join()
