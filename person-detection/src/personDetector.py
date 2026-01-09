from ultralytics import YOLO

class PersonDetector:
    def __init__(self, model_path="yolov8s.pt", conf_thresh=0.4):
        self.model = YOLO(model_path)
        self.conf_thresh = conf_thresh

    def detect(self, frame):
        results = self.model(
            frame,
            conf=self.conf_thresh,
            classes=[0],  # 0 is 'person' in COCO
            verbose=False,
            device=0,     # Ensure your GPU is available
            half=True
        )

        detections = []
        for r in results:
            for box in r.boxes:
                # Get coordinates and confidence
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])

                detections.append({
                    "bbox": [x1, y1, x2, y2],
                    "confidence": conf,
                    "class": "person"
                })
        return detections