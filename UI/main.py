import sys
import json
import socket
import threading

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

        self.setup_map()
        self.setup_top_toolbar()
        self.setup_main_toolbar()

        # ✅ connect signal to UI function (runs on Qt main thread)
        self.path_received.connect(self.on_path_received)

        # ✅ start TCP listener in background
        self.start_path_listener()

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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = Mainwindow()
    w.show()
    sys.exit(app.exec())
