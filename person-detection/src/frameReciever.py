import socket
import numpy as np
import cv2

class FrameReceiver:
    def __init__(self, endpoint, width, height, channels=3):
        # endpoint in this context is just the port if we assume UDP bind
        # But existing code passed "tcp://localhost:5555"
        # We will parse the port from it or just hardcode if we want to be safe, 
        # but let's try to be flexible correctly.
        
        # For UDP server (receiver side), we bind to an address.
        # Let's assume endpoint like "tcp://localhost:5555" -> bind to "0.0.0.0:5555"
        
        try:
            port = int(endpoint.split(":")[-1])
        except ValueError:
            raise ValueError("Invalid endpoint format. Expected format like 'tcp://localhost:5555'")

        self.width = width
        self.height = height
        self.channels = channels

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Increase buffer size to handle bursty video
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        print(f"Binding UDP to 0.0.0.0:{port}")
        self.sock.bind(("0.0.0.0", port))

    def receive(self):
        try:
            # 65536 is max UDP size
            packet, _ = self.sock.recvfrom(65536) 
            
            # Decode JPEG
            frame_array = np.frombuffer(packet, dtype=np.uint8)
            frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)
            
            return frame
        except Exception as e:
            print(f"Error receiving frame: {e}")
            return None

    def close(self):
        self.sock.close()
