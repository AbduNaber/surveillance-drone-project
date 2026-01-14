import socket
import json
import time

def send_waypoints():
    data = {
        "waypoints": [
            {"id": "A", "x": 4, "y": 35},
            {"id": "B", "x": 45, "y": 16}
        ]
    }
    
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("127.0.0.1", 5555))
        s.sendall(json.dumps(data).encode())
        s.close()
        print("Waypoints sent successfully")
    except Exception as e:
        print(f"Failed to send waypoints: {e}")

if __name__ == "__main__":
    time.sleep(1) # Give mission_planner time to start listening
    send_waypoints()
