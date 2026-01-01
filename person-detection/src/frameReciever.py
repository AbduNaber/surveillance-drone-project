import zmq
import numpy as np

class FrameReceiver:
    def __init__(self, endpoint, width, height, channels=3):
        self.width = width
        self.height = height
        self.channels = channels

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.connect(endpoint)

    def receive(self):
        packet = self.socket.recv()
        frame_array = np.frombuffer(packet, dtype=np.uint8)

        expected_size = self.width * self.height * self.channels
        if frame_array.size != expected_size:
            return None

        return frame_array.reshape(
            (self.height, self.width, self.channels)
        ).copy()

    def close(self):
        self.socket.close()
        self.context.term()
