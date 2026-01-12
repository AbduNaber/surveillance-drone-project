import cv2
import time
import numpy as np
import zmq
from sort import Sort

from frameReciever import FrameReceiver
from visualizer import Visualizer
from personDetector import PersonDetector

class Notifier:
    def __init__(self, endpoint="tcp://*:6000"):
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.PUB)
        self.sock.bind(endpoint)

    def notify_new_person(self, track_id, bbox):
        msg = {
            "event": "new_person",
            "track_id": int(track_id),
            "bbox": [int(v) for v in bbox],
            "timestamp": time.time()
        }
        self.sock.send_json(msg)

class YoloWorker:
    def __init__(self):
        self.receiver = FrameReceiver(endpoint="udp://239.255.0.1:5577", width=960, height=720)
        self.detector = PersonDetector()
        self.visualizer = Visualizer()
        self.notifier = Notifier()

        # SORT: min_hits=1 ensures you see the person immediately
        self.tracker = Sort(max_age=15, min_hits=1, iou_threshold=0.3)
        self.active_ids = set()

    def run(self):
        print("[YOLO] Worker running...")
        try:
            while True:
                frame = self.receiver.receive()
                if frame is None: continue

                # 1. Detect
                raw_detections = self.detector.detect(frame)
                
                # 2. Prepare for SORT [x1, y1, x2, y2, score]
                to_track = []
                for d in raw_detections:
                    x1, y1, x2, y2 = d["bbox"]
                    to_track.append([x1, y1, x2, y2, d["confidence"]])

                detections_np = np.array(to_track) if len(to_track) > 0 else np.empty((0, 5))

                # 3. Update Tracker
                tracks = self.tracker.update(detections_np)

                # 4. Process Tracks & Notify
                current_ids = set()
                for track in tracks:
                    x1, y1, x2, y2, track_id = track
                    track_id = int(track_id)
                    current_ids.add(track_id)

                    if track_id not in self.active_ids:
                        self.notifier.notify_new_person(track_id, (x1, y1, x2, y2))

                self.active_ids = current_ids

                # 5. Visualize and Display
                frame = self.visualizer.draw(frame, tracks)
                cv2.imshow("YOLO Tracking", frame)

                if cv2.waitKey(1) & 0xFF == ord("q"): break
        finally:
            self.shutdown()

    def shutdown(self):
        self.receiver.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    YoloWorker().run()