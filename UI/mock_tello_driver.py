
import zmq
import time
import json
import math

def mock_tello_driver():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:6001")
    
    print("Mock Tello Driver started on tcp://*:6001")
    
    x, y, yaw = 0.0, 0.0, 0.0
    
    try:
        while True:
            # Simulate rotating and moving in a circle
            yaw = (yaw + 5) % 360
            
            # Simple circular motion
            # x = 1.0 * math.cos(math.radians(yaw)) * 100 # cm
            # y = 1.0 * math.sin(math.radians(yaw)) * 100 # cm
            
            # Just send velocity 0, position 0, but changing yaw
            msg = {
                "type": "tello_state",
                "bat": 85,
                "vgx": 0,
                "vgy": 0,
                "h": 100,
                "yaw": yaw
            }
            
            socket.send_json(msg)
            print(f"Sent: {msg}")
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Stopping mock driver...")
    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    mock_tello_driver()
