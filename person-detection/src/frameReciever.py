import zmq
import cv2
import numpy as np


class FrameReceiver:
    def __init__(self, endpoint="tcp://127.0.0.1:5577", width=960, height=720):
         # endpoint is now a ZMQ connection string like "tcp://127.0.0.1:5577"
        self.width = width
        self.height = height
        self.endpoint = endpoint

        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.connect(self.endpoint)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")

        # Optional: Conflate to always get latest frame if processing is slow
        self.sock.setsockopt(zmq.CONFLATE, 1)

        print(f"[FrameReceiver] Subscribed to {self.endpoint}")

    def receive(self):
        try:
            # Blocking receive, or use NOBLOCK if needed
            packet = self.sock.recv()

            frame_array = np.frombuffer(packet, dtype=np.uint8)
            frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)

            return frame

        except Exception as e:
            print(f"[FrameReceiver] Error receiving frame: {e}")
            return None

    def close(self):
        self.sock.close()
        self.ctx.term()
