
import cv2
import sys
import zmq
import numpy as np

# ZMQ Endpoint
ZMQ_URL = "tcp://127.0.0.1:5577"

def main():
    print(f"[Camera] Connecting to {ZMQ_URL}")
    
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.connect(ZMQ_URL)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    # Conflate to prevent lag if UI is slow
    sock.setsockopt(zmq.CONFLATE, 1)

    print("[Camera] Stream opened (ZMQ)")

    while True:
        try:
            # Check for window close event (handled by cv2.waitKey)
            
            packet = sock.recv()
            if len(packet) == 0:
                continue

            frame_array = np.frombuffer(packet, dtype=np.uint8)
            frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)

            if frame is None:
                continue

            cv2.imshow("Drone Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except Exception as e:
            print(f"[Camera] Error: {e}")
            break
        except KeyboardInterrupt:
            break

    sock.close()
    ctx.term()
    cv2.destroyAllWindows()
    print("[Camera] Stream closed")

if __name__ == "__main__":
    main()
