import cv2

class Visualizer:
    @staticmethod
    def draw(frame, tracks):
        """
        Expects tracks as a list/array of [x1, y1, x2, y2, track_id]
        """
        for t in tracks:
            # SORT outputs floats, cv2 needs ints for coordinates
            x1, y1, x2, y2, track_id = map(int, t[:5])
            
            # Draw Bounding Box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw Track ID
            cv2.putText(
                frame,
                f"ID: {track_id}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )
        return frame