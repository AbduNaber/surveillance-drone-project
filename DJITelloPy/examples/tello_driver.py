import time
import threading
import zmq
from djitellopy import Tello
from enum import Enum
import socket
import json
import os

class TelloCommand(Enum):
    TAKEOFF = 0
    LAND = 1
    EMERGENCY = 2
    UP = 3
    DOWN = 4
    LEFT = 5
    RIGHT = 6
    FORWARD = 7
    BACKWARD = 8
    ROTATE_CW = 9
    ROTATE_CCW = 10
    STOP = 11
    FLIP_LEFT = 12
    FLIP_RIGHT = 13
    FLIP_FORWARD = 14
    FLIP_BACK = 15
    SET_SPEED = 16
    UNKNOWN = 17
# ================= ZMQ =================
ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:6001")


class Command:
    def __init__(self, command: TelloCommand, value: int = 0):
        self.command = command
        self.value = value

# ================= TELLO =================
tello = Tello()
tello.connect()
print("[TELLO] Battery:", tello.get_battery())

running = True

def emergency_stop_callback():
    global running
    # listen tcp://*:6002
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect("tcp://127.0.0.1:6002")
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    print("emergency is started")
    while running:
        msg = sub.recv_json()
        if msg["type"] == "emergency_stop":
            running = False
            tello.emergency()
            print("!!!!!!emergency!!!!!")
            break
        if msg["type"] == "land":
            running = False
            tello.land()
            print("!!!!!!land!!!!!")
            break

    sub.close()
    ctx.term()
    

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

# ================= PERMISSION HELPER =================
def handle_rotation_with_permission(cmd_enum, initial_value):
    UI_HOST = "127.0.0.1"
    UI_PORT = 5590
    
    direction = "CW" if cmd_enum == TelloCommand.ROTATE_CW else "CCW"
    
    print(f"[TELLO CMD] Requesting permission for rotation: {direction} {initial_value}")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((UI_HOST, UI_PORT))
        
        # Send initial request
        req = {
            "type": "rotate_request",
            "cmd": direction,
            "value": initial_value
        }
        sock.sendall(json.dumps(req).encode('utf-8'))
        
        buffer = ""
        decoder = json.JSONDecoder()
        permission_granted = False # ONLY True if "done" received

        # Loop for interaction
        while True:
            try:
                data = sock.recv(4096)
            except socket.error as e:
                print(f"[TELLO CMD] Socket error during permission: {e}")
                break
                
            if not data:
                print("[TELLO CMD] UI disconnected during permission.")
                break
            
            buffer += data.decode('utf-8')
            
            while buffer:
                buffer = buffer.lstrip()
                if not buffer:
                    break
                
                try:
                    msg, idx = decoder.raw_decode(buffer)
                    buffer = buffer[idx:]
                except ValueError:
                    # Incomplete JSON
                    break
                
                action = msg.get("action")
                print(f"[TELLO CMD] Permission Msg: {action}")

                if action == "done":
                    print("[TELLO CMD] Rotation permission verified. Proceeding.")
                    permission_granted = True
                    sock.close()
                    return True
                
                elif action == "land":
                    print("[TELLO CMD] Use requested LAND. Aborting mission.")
                    try:
                        tello.land()
                        sock.sendall(b"ACK")
                    except Exception as e:
                        print(f"[TELLO CMD] Land error: {e}")
                    sock.close()
                    return False

                elif action == "rotate":
                    rot_dir = msg.get("direction", "CW")
                    rot_val = int(msg.get("value", 0))
                    print(f"[TELLO CMD] Performing interactive rotation: {rot_dir} {rot_val}")
                    
                    try:
                        if rot_dir == "CW":
                            tello.rotate_clockwise(rot_val)
                        else:
                            tello.rotate_counter_clockwise(rot_val)
                        
                        sock.sendall(b"ACK")
                    except Exception as e:
                        print(f"[TELLO CMD] Rotation error: {e}")
                        sock.sendall(b"ERR")
        
        sock.close()
        
        pass # Loop finished without 'done' or 'land' -> Disconnect
        
        # If we reached here, the loop finished. 
        # This means connection was made but dropped before "done" or "land".
        # We should ABORT for safety.
        print("[TELLO CMD] Permission dialog disconnected unexpectedly. Aborting.")
        return False
        
    except Exception as e:
        print(f"[TELLO CMD] Failed to connect to UI for permission: {e}")
        # Connect failed entirely -> Fallback to execution (Headless mode)
        print("[TELLO CMD] Fallback: Executing original rotation without permission.")
        if cmd_enum == TelloCommand.ROTATE_CW:
            tello.rotate_clockwise(initial_value)
        else:
            tello.rotate_counter_clockwise(initial_value)
            
        return True


# ================= COMMAND THREAD =================
def command_sequence():
    global running
    try:
        tello.streamon()
        print("[TELLO] Starting video stream...")
    except Exception as e:
        print(f"[TELLO] Failed to start stream: {e}")

    time.sleep(2)


    HOST = "127.0.0.1"
    PORT = 5588

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    
    # Set timeout for accept loop to allow checking 'running' flag
    server.settimeout(1.0)

    print(f"[TELLO CMD] Listening on tcp://{HOST}:{PORT}")

    while running:
        try:
            conn, addr = server.accept()
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[TELLO CMD] Accept error: {e}")
            continue

        print(f"[TELLO CMD] Client connected from {addr}")
        
        # Buffer for this connection
        buffer = ""
        decoder = json.JSONDecoder()
        
        # Inner loop for handling one connection
        while running:
            try:
                data = conn.recv(4096)
            except socket.error:
                break
                
            if not data:
                print("[TELLO CMD] Client disconnected")
                break

            # Decode bytes to string and append to buffer
            try:
                buffer += data.decode('utf-8')
            except UnicodeDecodeError:
                print("[TELLO CMD] Decode error, skipping chunk")
                continue

            # Process all complete JSON objects in the buffer
            while buffer:
                buffer = buffer.lstrip()
                if not buffer:
                    break
                
                try:
                    msg, idx = decoder.raw_decode(buffer)
                    buffer = buffer[idx:]
                except ValueError:
                    # Not enough data for a full JSON object yet
                    break
                
                # Check for commands list
                if "commands" not in msg:
                    continue

                commands: list[Command] = []
                for cmd in msg["commands"]:
                    cmd_str = cmd.get("command", "")
                    # handle value being potentially empty or string
                    val_raw = cmd.get("value", 0)
                    if val_raw == '':
                        val_raw = 0
                    
                    try:
                        value = int(val_raw)
                    except ValueError:
                        value = 0

                    try:
                        key = int(cmd_str)
                        cmd_enum = TelloCommand(key)
                    except (ValueError, KeyError, TypeError):
                        print(f'[TELLO CMD] Invalid key: <{cmd_str}>' )
                        cmd_enum = TelloCommand.UNKNOWN
                    
                    commands.append(Command(cmd_enum, value))

                print(f"[TELLO CMD] Received {len(commands)} commands")
                
                # Execute commands
                success = True
                for command in commands:  
                    print(f"[TELLO CMD] Executing command: {command.command}, value: {command.value}")
                    try:
                        if command.command == TelloCommand.TAKEOFF:
                            tello.takeoff()
                        elif command.command == TelloCommand.LAND:
                            tello.land()
                        elif command.command == TelloCommand.EMERGENCY:
                            tello.emergency()
                        elif command.command == TelloCommand.UP:
                            tello.move_up(command.value)
                        elif command.command == TelloCommand.DOWN:
                            tello.move_down(command.value)
                        elif command.command == TelloCommand.LEFT:
                            tello.move_left(command.value)
                        elif command.command == TelloCommand.RIGHT:
                            tello.move_right(command.value)
                        elif command.command == TelloCommand.FORWARD:
                            tello.move_forward(command.value)
                        elif command.command == TelloCommand.BACKWARD:
                            tello.move_back(command.value)
                        elif command.command == TelloCommand.ROTATE_CW:
                            if not handle_rotation_with_permission(command.command, command.value):
                                print("[TELLO CMD] Mission aborted by user.")
                                success = False
                                break
                        elif command.command == TelloCommand.ROTATE_CCW:
                             if not handle_rotation_with_permission(command.command, command.value):
                                print("[TELLO CMD] Mission aborted by user.")
                                success = False
                                break
                        elif command.command == TelloCommand.STOP:
                            tello.send_rc_control(0, 0, 0, 0)
                        elif command.command == TelloCommand.FLIP_LEFT:
                            tello.flip_left()
                        elif command.command == TelloCommand.FLIP_RIGHT:
                            tello.flip_right()
                        elif command.command == TelloCommand.FLIP_FORWARD:
                            tello.flip_forward()
                        elif command.command == TelloCommand.FLIP_BACK:
                            tello.flip_back()
                        elif command.command == TelloCommand.SET_SPEED:
                            tello.set_speed(command.value)
                        else:
                            print(f"[TELLO CMD] Unknown command: {command.command}")
                    except Exception as e:
                        print(f"[TELLO CMD] Execution Error: {e}")
                        success = False
                        # We continue executing other commands? 
                        # Usually if one fails, might want to stop, but user wanted "robustness".
                        # Let's continue but report error? 
                        # Actually if connection lost, tello object might raise, so maybe break?
                        # djitellopy usually catches internal errors but if it raises, it's serious.
                        # We'll allow continue for now.
                    
                    if success:
                         print(f"[TELLO CMD] Command OK: {command.command}")

                    time.sleep(0.3)  # small delay between commands
                
                commands.clear()
                
                # ACK to client
                try:
                    conn.sendall(b"OK")
                except socket.error:
                    print("[TELLO CMD] Failed to send ACK")
                    break

        # Connection closed
        conn.close()

    server.close()

# ================= START =================
telemetry_thread = threading.Thread(target=telemetry_loop, daemon=True)
command_thread = threading.Thread(target=command_sequence)
emergency_thread = threading.Thread(target=emergency_stop_callback)
emergency_thread.start()
telemetry_thread.start()
command_thread.start()


try:
    emergency_thread.join()
    command_thread.join()
except KeyboardInterrupt:
    print("\n[TELLO] Interrupted by user, shutting down...")
    tello.end()
    running = False
    pass

# ================= PARENT MONITOR =================
def monitor_parent(initial_ppid):
    """
    Monitors the parent process ID. If it changes (meaning the parent died),
    this process should exit.
    """
    while True:
        current_ppid = os.getppid()
        if current_ppid != initial_ppid:
            print(f"[Monitor] Parent process {initial_ppid} died. Exiting...")
            os._exit(0)  # Force exit
        time.sleep(1)

if __name__ == "__main__":

    
    # Start parent monitor
    initial_ppid = os.getppid()
    monitor_thread = threading.Thread(target=monitor_parent, args=(initial_ppid,), daemon=True)
    monitor_thread.start()

    # Wait for other threads (main thread is blocked here in original code?? No, original code joins at the end)
    # The original code has joins at the end of file which blocks the main thread.
    # So we just let those joins happen.
    pass
