import zmq
import time
import json
import random

def main():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    try:
        sock.bind("tcp://*:6000")
        print("Simulator bound to tcp://*:6000. Sending 'new_person' events...")
    except zmq.ZMQError as e:
        print(f"Could not bind to port 6000: {e}")
        return

    track_id = 1
    
    while True:
        bbox = [100, 100, 200, 200]
        msg = {
            "event": "new_person",
            "track_id": track_id,
            "bbox": [int(v) for v in bbox],
            "timestamp": time.time()
        }
        
        print(f"Sending: {msg}")
        sock.send_json(msg)
        
        # Simulate moving or new person
        if random.random() > 0.8:
            track_id += 1
            print(f"New person! track_id={track_id}")
            
        time.sleep(2)

if __name__ == "__main__":
    main()
