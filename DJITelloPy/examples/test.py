import time
import threading
import zmq
from djitellopy import Tello

# ================= ZMQ =================
ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:6000")

# ================= TELLO =================
tello = Tello()
tello.connect()
print("[TELLO] Battery:", tello.get_battery())

running = True

# ================= TELEMETRY THREAD =================
def telemetry_loop():
    print("[TELLO] Telemetry thread started")
    last = time.time()

    while running:
        state = tello.get_current_state()
        now = time.time()
        dt = now - last
        last = now

        msg = {
            "type": "tello_state",
            "vgx": state.get("vgx", 0),
            "vgy": state.get("vgy", 0),
            "vgz": state.get("vgz", 0),
            "yaw": state.get("yaw", 0),
            "h": state.get("h", 0),
            "bat": state.get("bat", 0),
            "dt": dt
        }

        pub.send_json(msg)
        time.sleep(0.05)   # 20 Hz

# ================= COMMAND THREAD =================
def command_sequence():

	tello.streamon()


	# wait user push q don@t get v'deo I w'll hande
	while True:
		if( input("Press q to quit: ") == 'q'):
			break

	tello.streamoff()
	

# ================= START =================
telemetry_thread = threading.Thread(target=telemetry_loop, daemon=True)
command_thread = threading.Thread(target=command_sequence)

telemetry_thread.start()
command_thread.start()

command_thread.join()
