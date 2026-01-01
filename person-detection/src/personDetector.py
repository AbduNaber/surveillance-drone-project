from ultralytics import YOLO

class PersonDetector:
    def __init__(self, model_path="yolov8s.pt", conf_thresh=0.4):
        self.model = YOLO(model_path)
        self.conf_thresh = conf_thresh

    def detect(self, frame):
        results = self.model(
            frame,
            conf=self.conf_thresh,
            classes=[0],  # person only
            verbose=False
        )

        detections = []

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])

                detections.append({
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "conf": conf
                })

        return detections
