import sys
import socket
import threading
import math
import time

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QGraphicsView, QGraphicsScene
)
from PyQt6.QtGui import QPen, QBrush, QPainterPath
from PyQt6.QtCore import Qt, pyqtSignal


class Mainwindow(QMainWindow):
    drone_update = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Tello Trajectory Debug UI")
        self.setGeometry(200, 200, 1200, 800)

        # ===== SCENE =====
        self.view = QGraphicsView(self)
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)
        self.setCentralWidget(self.view)

        self.scene.setSceneRect(-1000, -1000, 2000, 2000)
        self.view.setRenderHint(self.view.renderHints())
        self.view.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)

        # ===== DRONE ICON =====
        self.drone_item = self.scene.addEllipse(
            -10, -10, 20, 20,
            QPen(Qt.GlobalColor.blue),
            QBrush(Qt.GlobalColor.blue)
        )
        self.drone_item.setZValue(10)

        # ===== TRAJECTORY =====
        self.trajectory_path = QPainterPath()
        self.trajectory_item = self.scene.addPath(
            self.trajectory_path,
            QPen(Qt.GlobalColor.green, 3)
        )
        self.trajectory_item.setZValue(5)

        # ===== DEBUG TEXT =====
        self.debug_text = self.scene.addText("Waiting for Tello...")
        self.debug_text.setDefaultTextColor(Qt.GlobalColor.yellow)
        self.debug_text.setZValue(100)
        self.debug_text.setPos(-980, -980)

        # ===== DRONE STATE =====
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.trajectory_initialized = False

        self.drone_update.connect(self.update_drone_position)

        # Start listeners
        self.start_tello_listener()

    # ================= TELLO LISTENER =================
    def start_tello_listener(self):
        def listener():
            print("[UI] ZMQ listener starting on tcp://127.0.0.1:6000")

            import zmq

            ctx = zmq.Context()
            sock = ctx.socket(zmq.SUB)
            sock.connect("tcp://127.0.0.1:6000")
            sock.setsockopt_string(zmq.SUBSCRIBE, "")

            last_time = time.time()
            print("[UI] Waiting for Tello state via ZMQ...")

            while True:
                msg = sock.recv_json()
                print("[UI RAW]", msg)

                if msg.get("type") != "tello_state":
                    continue

                now = time.time()
                dt = msg.get("dt", now - last_time)
                last_time = now

                print(f"[DT] {dt:.3f}s")

                vgx = msg.get("vgx", 0)
                vgy = msg.get("vgy", 0)
                yaw_deg = msg.get("yaw", 0)

                vx = vgx / 100.0
                vy = vgy / 100.0
                yaw = math.radians(yaw_deg)

                wx = vx * math.cos(yaw) - vy * math.sin(yaw)
                wy = vx * math.sin(yaw) + vy * math.cos(yaw)

                print(f"[WORLD VEL] wx={wx:.3f} m/s wy={wy:.3f} m/s")

                self.drone_x += wx * dt
                self.drone_y += wy * dt

                print(f"[POSITION] x={self.drone_x:.2f} y={self.drone_y:.2f}")

                self.drone_update.emit(self.drone_x, self.drone_y)

        threading.Thread(target=listener, daemon=True).start()


    # ================= PARSER =================
    def parse_tello_state(self, msg):
        data = {}
        for item in msg.strip().split(";"):
            if ":" not in item:
                continue
            k, v = item.split(":")
            try:
                data[k] = float(v)
            except ValueError:
                print("[PARSE ERROR]", k, v)
        return data

    # ================= UI UPDATE =================
    def update_drone_position(self, x_m, y_m):
        scale = 1000  # exaggerate movement for visibility

        sx = x_m * scale
        sy = -y_m * scale  # invert Y for screen coords

        print(f"[UI] Move to sx={sx:.1f}, sy={sy:.1f}")

        # Move drone icon
        self.drone_item.setPos(sx, sy)

        # ---- TRAJECTORY FIX ----
        if not self.trajectory_initialized:
            self.trajectory_path.moveTo(sx, sy)
            self.trajectory_initialized = True
        else:
            self.trajectory_path.lineTo(sx, sy)

        self.trajectory_item.setPath(self.trajectory_path)

        # Debug overlay
        self.debug_text.setPlainText(
            f"x = {x_m:.2f} m\n"
            f"y = {y_m:.2f} m\n"
            f"sx = {sx:.1f}\n"
            f"sy = {sy:.1f}"
        )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = Mainwindow()
    w.show()
    sys.exit(app.exec())