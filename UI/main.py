import sys
import json
import socket
import threading
import math
import zmq

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QToolBar, QWidget,
    QSizePolicy, QGraphicsView, QGraphicsScene,
    QGraphicsPathItem, QMessageBox
)
from PyQt6.QtGui import QPainter, QPen, QBrush, QPainterPath
from PyQt6.QtCore import Qt, QEvent, pyqtSignal
from PyQt6.QtSvgWidgets import QGraphicsSvgItem

from battery_widget import BatteryWidget
from wifi_widget import WifiWidget


class Mainwindow(QMainWindow):
    # ✅ Signal to safely move data from socket thread → Qt main thread
    path_received = pyqtSignal(list)
    drone_update = pyqtSignal(float, float)
    person_detected = pyqtSignal(dict)

    def __init__(self):
        super().__init__()

        # ===== GRID CONFIG (MUST MATCH YAML) =====
        self.grid_width = 200
        self.grid_height = 200
        self.cell_size = 20  # world units per cell

        self.setWindowTitle("Drone Mission Planning Interface")
        self.setGeometry(200, 200, 1400, 900)

        self.wifi_widget = WifiWidget(self)
        self.battery_widget = BatteryWidget(self)

        self.waypoints = []
        self.max_waypoints = 2

        # Keep track of path line items so we can clear them cleanly
        self.path_items = []

        # Keep track of detected persons: track_id -> QGraphicsSvgItem
        self.person_items = {}

        self.setup_map()
        self.setup_top_toolbar()
        self.setup_main_toolbar()

        # ✅ connect signal to UI function (runs on Qt main thread)
        self.path_received.connect(self.on_path_received)
        self.drone_update.connect(self.update_drone_position)
        self.person_detected.connect(self.on_person_detected)

        # ✅ start TCP listener in background
        self.start_path_listener()
        self.start_tello_listener()
        self.start_person_listener()
        
        # ===== DRONE STATE =====
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.trajectory_initialized = False

        # ===== DRONE ICON =====
        # Using a distinct color for the drone on the map
        self.drone_item_visual = self.scene.addEllipse(
            -10, -10, 20, 20,
            QPen(Qt.GlobalColor.blue),
            QBrush(Qt.GlobalColor.blue)
        )
        self.drone_item_visual.setZValue(100) # High Z value to be on top

        # ===== TRAJECTORY =====
        self.trajectory_path = QPainterPath()
        self.trajectory_item = self.scene.addPath(
            self.trajectory_path,
            QPen(Qt.GlobalColor.blue, 3)
        )
        self.trajectory_item.setZValue(90)

        self.statusBar().showMessage("Ready")

    # ================= MAP =================
    def setup_map(self):
        self.view = QGraphicsView(self)
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)
        self.setCentralWidget(self.view)

        self.map_item = QGraphicsSvgItem("/home/abdu/surveillance_drone_project/UI/assets/map.svg")
        # Optional: prevent SVG from stealing clicks
        self.map_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)

        self.scene.addItem(self.map_item)

        bounds = self.map_item.boundingRect()
        self.map_width_scene = bounds.width()
        self.map_height_scene = bounds.height()

        self.scene.setSceneRect(bounds)

        self.view.resetTransform()
        self.view.fitInView(bounds, Qt.AspectRatioMode.KeepAspectRatio)
        self.view.scale(30.0, 30.0)

        self.view.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.view.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)

        self.view.viewport().installEventFilter(self)

    # ================= TOOLBARS =================
    def setup_top_toolbar(self):
        top = QToolBar("Status", self)
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, top)
        top.setMovable(False)

        spacer = QWidget(self)
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        top.addWidget(spacer)

        top.addWidget(self.wifi_widget)
        top.addWidget(self.battery_widget)

    def setup_main_toolbar(self):
        tb = self.addToolBar("Actions")

        find_way_btn = tb.addAction("Find Way")
        find_way_btn.triggered.connect(self.send_waypoints_to_mission_planner)

        clear_pins_btn = tb.addAction("Clear Pins")
        clear_pins_btn.triggered.connect(self.clear_persons)

    # ================= EVENTS =================
    def eventFilter(self, obj, event):
        if obj == self.view.viewport() and event.type() == QEvent.Type.MouseButtonPress:
            clicked = self.view.mapToScene(event.pos())

            if len(self.waypoints) == self.max_waypoints:
                self.clear_waypoints()

            if len(self.waypoints) == 0:
                self.add_waypoint(clicked, "A", Qt.GlobalColor.green)
            elif len(self.waypoints) == 1:
                self.add_waypoint(clicked, "B", Qt.GlobalColor.red)

        return False

    # ================= COORDINATE CONVERSION =================
    def scene_to_grid(self, scene_pos):
        nx = scene_pos.x() / self.map_width_scene
        ny = scene_pos.y() / self.map_height_scene

        world_x = nx * (self.grid_width * self.cell_size)
        world_y = ny * (self.grid_height * self.cell_size)

        gx = int(world_x / self.cell_size)
        gy = int(world_y / self.cell_size)

        gx = max(0, min(self.grid_width - 1, gx))
        gy = max(0, min(self.grid_height - 1, gy))

        return gx, gy

    def grid_to_scene(self, gx, gy):
        world_x = gx * self.cell_size
        world_y = gy * self.cell_size

        sx = (world_x / (self.grid_width * self.cell_size)) * self.map_width_scene
        sy = (world_y / (self.grid_height * self.cell_size)) * self.map_height_scene

        return sx, sy

    # ================= WAYPOINTS =================
    def add_waypoint(self, pos, label, color):
        size = 500

        path = QPainterPath()
        path.addEllipse(-size / 4, -size / 2, size / 2, size / 2)
        path.moveTo(0, size / 2)
        path.lineTo(-size / 6, 0)
        path.lineTo(size / 6, 0)
        path.closeSubpath()

        pin = QGraphicsPathItem(path)
        pin.setBrush(QBrush(color))
        pin.setPen(QPen(Qt.GlobalColor.black, 2))
        pin.setPos(pos)

        self.scene.addItem(pin)

        text = self.scene.addText(label)
        text.setDefaultTextColor(Qt.GlobalColor.white)
        text.setPos(pos.x(), pos.y())

        self.waypoints.append({
            "id": label,
            "pos": pos,
            "item": pin,
            "text": text
        })

    def clear_waypoints(self):
        for wp in self.waypoints:
            self.scene.removeItem(wp["item"])
            self.scene.removeItem(wp["text"])
        self.waypoints.clear()

    # ================= SEND TO C++ =================
    def send_waypoints_to_mission_planner(self):
        waypoints = []

        for wp in self.waypoints:
            gx, gy = self.scene_to_grid(wp["pos"])
            waypoints.append({"id": wp["id"], "x": gx, "y": gy})

        data = {"mission_id": 1, "waypoints": waypoints}

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(("127.0.0.1", 5555))
            sock.sendall(json.dumps(data).encode("utf-8"))
            sock.close()
            QMessageBox.information(self, "Success", "Waypoints sent to planner")
        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))

    # ================= RECEIVE PATH FROM C++ =================
    def start_path_listener(self):
        def server():
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("127.0.0.1", 5566))
            sock.listen(1)
            print("UI path listener started on 127.0.0.1:5566")

            while True:
                client, _ = sock.accept()

                # Read all data until client closes
                chunks = []
                while True:
                    chunk = client.recv(4096)
                    if not chunk:
                        break
                    chunks.append(chunk)

                client.close()

                try:
                    payload = json.loads(b"".join(chunks).decode("utf-8"))
                except Exception as e:
                    print("Invalid JSON from C++:", e)
                    continue

                if payload.get("type") == "path":
                    # ✅ Emit signal (safe), do not touch UI in this thread
                    self.path_received.emit(payload.get("path", []))

        threading.Thread(target=server, daemon=True).start()

    # ✅ This runs on the Qt main thread because it's triggered by a Qt signal
    def on_path_received(self, path):
        self.clear_path()
        self.draw_path(path)

    def draw_path(self, path):
        if not path or len(path) < 2:
            return

        pen = QPen(Qt.GlobalColor.green)
        pen.setWidth(5)

        for i in range(len(path) - 1):
            x1, y1 = self.grid_to_scene(path[i]["x"], path[i]["y"])
            x2, y2 = self.grid_to_scene(path[i + 1]["x"], path[i + 1]["y"])
            line = self.scene.addLine(x1, y1, x2, y2, pen)
            self.path_items.append(line)

    def clear_path(self):
        for item in self.path_items:
            self.scene.removeItem(item)
        self.path_items.clear()

    # ================= TELLO LISTENER =================
    def start_tello_listener(self):
        def listener():
            print("[UI] ZMQ listener starting on tcp://127.0.0.1:6001")

            ctx = zmq.Context()
            sock = ctx.socket(zmq.SUB)
            sock.connect("tcp://127.0.0.1:6001")
            sock.setsockopt_string(zmq.SUBSCRIBE, "")

            last_time = None 
            print("[UI] Waiting for Tello state via ZMQ...")

            while True:
                try:
                    msg = sock.recv_json()
                except Exception as e:
                    print(f"ZMQ Error: {e}")
                    continue
                
                # Basic validation
                if msg.get("type") != "tello_state":
                    continue

                now = time.time()
                if last_time is None:
                    last_time = now
                    dt = 0.1 # default first step
                else:
                    dt = msg.get("dt", now - last_time)
                
                last_time = now

                vgx = msg.get("vgx", 0)
                vgy = msg.get("vgy", 0)
                yaw_deg = msg.get("yaw", 0)

                vx = vgx / 100.0
                vy = vgy / 100.0
                yaw = math.radians(yaw_deg)

                wx = vx * math.cos(yaw) - vy * math.sin(yaw)
                wy = vx * math.sin(yaw) + vy * math.cos(yaw)

                self.drone_x += wx * dt
                self.drone_y += wy * dt

                self.drone_update.emit(self.drone_x, self.drone_y)
                #print("state ok")

        # Need to import time inside method or file level if not already
        import time 
        threading.Thread(target=listener, daemon=True).start()

    # ================= PERSON LISTENER =================
    def start_person_listener(self):
        def listener():
            print("[UI] Person listener starting on tcp://*:6000")
            ctx = zmq.Context()
            sock = ctx.socket(zmq.SUB)
            try:
                # User said "sends notify to tcp://*:6000", implying sender BINDS.
                # So we CONNECT to localhost:6000.
                sock.connect("tcp://127.0.0.1:6000")
                sock.setsockopt_string(zmq.SUBSCRIBE, "")
                
                while True:
                    try:
                        msg = sock.recv_json()
                        if msg.get("event") == "new_person":
                            self.person_detected.emit(msg)
                    except Exception as e:
                        print(f"Person ZMQ Error: {e}")
                        import time
                        time.sleep(1) 
            except Exception as e:
                print(f"Could not connect to person stream: {e}")

        threading.Thread(target=listener, daemon=True).start()

    def on_person_detected(self, msg):
        track_id = msg.get("track_id")
        if track_id is None:
            return

        # Position: Use current drone position
        pos = self.drone_item_visual.pos()
        
        if track_id in self.person_items:
            # Update position
            self.person_items[track_id].setPos(pos)
        else:
            # Create new pin
            pin = QGraphicsSvgItem("/home/abdu/surveillance_drone_project/UI/assets/person_icon.svg")
            
            # Optional: Scale pin if needed, e.g. pin.setScale(0.5)
            # pin.setFlags(QGraphicsSvgItem.GraphicsItemFlag.ItemIgnoresTransformations)
            
            self.scene.addItem(pin)
            pin.setPos(pos)
            pin.setZValue(50) # Below drone, above map
            
            # Center alignment attempt (if SVG bounds known)
            b = pin.boundingRect()
            pin.setTransformOriginPoint(b.center())
            
            # Center the item on the position by offsetting
            # pin.setPos(pos.x() - b.width()/2, pos.y() - b.height()/2)
            # Note: setPos sets the origin. If we want center at pos, we shift.
            # But setTransformOriginPoint is for rotation/scale.
            # Let's adjust offset.
            pin.setPos(pos.x() - b.width()/2, pos.y() - b.height()/2)
            
            self.person_items[track_id] = pin

    def clear_persons(self):
        for tid, item in self.person_items.items():
            self.scene.removeItem(item)
        self.person_items.clear()

    # ================= UI UPDATE =================
    def update_drone_position(self, x_m, y_m):
        # Scale logic: Assuming x_m, y_m are in meters.
        # Main map logic: 20 pixels = 1 unit? 
        # self.cell_size = 20 # world units per cell?? No, comment says "world units per cell".
        # Let's check grid_to_scene. 
        # world_x = gx * cell_size
        # Actually, let's look at scene_to_grid.
        # The map seems to be purely visual with grid overlay.
        # Let's try to just use a scaling factor similar to test.py but adapted to visibility.
        # test.py used scale=1000.
        # Here we have a map. Let's assume the drone starts at the center or 0,0 corresponds to a point.
        # For now, I'll use a direct visual scale.
        
        scale = 100
        
        # In test.py: sy = -y_m * scale.
        sx = x_m * scale
        sy = -y_m * scale 

        # We probably want to offset this to a starting position on the map if we knew it.
        # For now, relative to (0,0) of the scene (top-left of map typically, but sceneRect might be adjusted).
        # The map scene rect is set to bounds of svg.
        # Let's assume start at center of map for visibility if 0,0 is top left.
        
        center_x = self.map_width_scene / 2
        center_y = self.map_height_scene / 2
        
        final_x = center_x + sx
        final_y = center_y + sy

        self.drone_item_visual.setPos(final_x, final_y)

        # Tracjectory
        if not self.trajectory_initialized:
            self.trajectory_path.moveTo(final_x, final_y)
            self.trajectory_initialized = True
        else:
            self.trajectory_path.lineTo(final_x, final_y)

        self.trajectory_item.setPath(self.trajectory_path)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = Mainwindow()
    w.show()
    sys.exit(app.exec())
