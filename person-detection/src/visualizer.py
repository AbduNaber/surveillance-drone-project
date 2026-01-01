import cv2

class Visualizer:
    @staticmethod
    def draw(frame, detections):
        for d in detections:
            cv2.rectangle(
                frame,
                (d["x1"], d["y1"]),
                (d["x2"], d["y2"]),
                (0, 255, 0),
                2
            )
            cv2.putText(
                frame,
                f"Person {d['conf']:.2f}",
                (d["x1"], d["y1"] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )
        return frame
