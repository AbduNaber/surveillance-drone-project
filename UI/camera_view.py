import cv2
import sys
import signal

UDP_URL = "udp://239.255.0.1:5577"

def main():
    cap = cv2.VideoCapture(UDP_URL)

    if not cap.isOpened():
        print("[Camera] Failed to open stream")
        return

    print("[Camera] Stream opened")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[Camera] Stream timeout")
            break

        cv2.imshow("Drone Camera", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("[Camera] Stream closed")

if __name__ == "__main__":
    main()
