import cv2
import numpy as np
from frameReciever import FrameReceiver
from visualizer import Visualizer
from personDetector import PersonDetector
class YoloWorker:
    def __init__(self):
        self.receiver = FrameReceiver(
            endpoint="tcp://localhost:5555",
            width=960,
            height=720
        )

        self.detector = PersonDetector()
        self.visualizer = Visualizer()

    def run(self):
        print("YOLO worker started")

        try:
            while True:
                frame = self.receiver.receive()
                if frame is None:
                    continue

                detections = self.detector.detect(frame)

                frame = self.visualizer.draw(frame, detections)

                cv2.imshow("YOLO Person Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.shutdown()

    def shutdown(self):
        print("Shutting down YOLO worker")
        self.receiver.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    worker = YoloWorker()
    worker.run()