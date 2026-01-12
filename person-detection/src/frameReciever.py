import socket
import struct
import cv2
import numpy as np


class FrameReceiver:
    def __init__(self, endpoint, width, height, channels=3,
                 mcast_group="239.255.0.1"):
        try:
            port = int(endpoint.split(":")[-1])
        except ValueError:
            raise ValueError("Invalid endpoint format. Expected 'tcp://localhost:5555'")

        self.width = width
        self.height = height
        self.channels = channels
        self.mcast_group = mcast_group
        self.port = port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        # Allow multiple receivers
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Increase buffer size
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)

        # ✅ Correct bind (all interfaces)
        self.sock.bind(("", self.port))

        # ✅ Join multicast group
        mreq = struct.pack(
            "4sl",
            socket.inet_aton(self.mcast_group),
            socket.INADDR_ANY
        )
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        print(f"[FrameReceiver] Listening on {self.mcast_group}:{self.port}")

    def receive(self):
        try:
            packet, _ = self.sock.recvfrom(65536)

            frame_array = np.frombuffer(packet, dtype=np.uint8)
            frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)

            return frame

        except Exception as e:
            print(f"[FrameReceiver] Error receiving frame: {e}")
            return None

    def close(self):
        self.sock.close()
