import zmq
import cv2
import numpy as np

def run_receiver():
    # 1. Setup ZMQ
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.connect("tcp://localhost:5555")
    print("Connected to C++ Publisher at tcp://localhost:5555")

    # 2. Define frame properties 
    # (Matches the FFmpeg test command: 640x480)
    width = 960
    height = 720
    channels = 3

    try:
        while True:
            # Receive the raw bytes
            packet = socket.recv()
            
            # Convert bytes to a numpy array (uint8)
            frame_array = np.frombuffer(packet, dtype=np.uint8)
            
            # Reshape the 1D array back into an image (H, W, C)
            try:
                frame = frame_array.reshape((height, width, channels))
                
                # Display the result
                cv2.imshow("Python ZMQ Receiver", frame)
            except ValueError as e:
                print(f"Frame size mismatch: {len(packet)} bytes received.")
                continue

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()
        socket.close()

if __name__ == "__main__":
    run_receiver()