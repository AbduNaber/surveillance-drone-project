import yaml
import sys
import cv2
import json
import socket
import threading
import math
import zmq
import subprocess
import time

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QToolBar, QWidget,
    QSizePolicy, QGraphicsView, QGraphicsScene,
    QGraphicsPathItem, QMessageBox, QDockWidget, QTextEdit,
    QDialog, QVBoxLayout, QLabel, QSpinBox, QPushButton, QHBoxLayout,
    QSlider, QTableWidget, QTableWidgetItem, QHeaderView
)
from PyQt6.QtGui import QPainter, QPen, QBrush, QPainterPath, QColor
from PyQt6.QtCore import Qt, QEvent, pyqtSignal, QObject, pyqtSlot
from PyQt6.QtSvgWidgets import QGraphicsSvgItem

from battery_widget import BatteryWidget
from wifi_widget import WifiWidget
from PyQt6.QtGui import QAction

class StatusDialog(QDialog):
    def __init__(self, parent, processes):
        super().__init__(parent)
        self.setWindowTitle("System Status")
        self.resize(600, 300)
        self.processes = processes
        
        layout = QVBoxLayout()
        
        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Process Name", "Status", "Action"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        
        layout.addWidget(self.table)
        
        self.refresh_table()
        
        btn_layout = QHBoxLayout()
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self.refresh_table)
        btn_layout.addWidget(refresh_btn)
        
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.accept)
        btn_layout.addWidget(close_btn)
        
        layout.addLayout(btn_layout)
        self.setLayout(layout)
        
    def refresh_table(self):
        self.table.setRowCount(0)
        
        for name, proc in self.processes.items():
            row = self.table.rowCount()
            self.table.insertRow(row)
            
            # Name Item
            name_item = QTableWidgetItem(name)
            self.table.setItem(row, 0, name_item)
            
            # Status Item
            status = "unknown"
            bg_color = QColor("gray")
            fg_color = QColor("white")
            
            is_running = False
            
            if proc is None:
                status = "Not Started"
                bg_color = QColor("#808080") # Grey
            elif proc.poll() is None:
                status = "RUNNING"
                bg_color = QColor("#4CAF50") # Green
                is_running = True
            else:
                status = f"STOPPED ({proc.returncode})"
                bg_color = QColor("#F44336") # Red
                
            status_item = QTableWidgetItem(status)
            status_item.setBackground(bg_color)
            status_item.setForeground(fg_color)
            status_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            status_item.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
            
            self.table.setItem(row, 1, status_item)
            
            # Action Item (Kill Button)
            if is_running:
                kill_btn = QPushButton("Kill Process")
                kill_btn.setStyleSheet("background-color: #F44336; color: white; font-weight: bold;")
                # Use lambda to capture current name
                kill_btn.clicked.connect(lambda checked, n=name: self.kill_process(n))
                self.table.setCellWidget(row, 2, kill_btn)
    
    def kill_process(self, name):
        proc = self.processes.get(name)
        if proc and proc.poll() is None:
            print(f">> [Status] Killing process: {name}")
            try:
                proc.terminate()
                # Optional: Force kill if needed
                # proc.kill()
                proc.wait(timeout=0.2)
            except Exception as e:
                print(f"Error killing process: {e}")
        
        # Refresh table to show new status
        self.refresh_table()
        
        # Update Main Window Buttons
        if self.parent():
            self.parent().update_ui_state()

class RotationControlDialog(QDialog):
    def __init__(self, parent, socket_conn, direction, initial_value):
        super().__init__(parent)
        self.setWindowTitle("Rotation Permission Request")
        self.socket_conn = socket_conn
        self.direction = direction # "CW" or "CCW"
        self.setModal(True)
        self.resize(300, 200)

        layout = QVBoxLayout()

        self.info_label = QLabel(f"Drone wants to rotate {direction} by {initial_value} degrees.")
        self.info_label.setWordWrap(True)
        layout.addWidget(self.info_label)

        # Input for value
        hbox_val = QHBoxLayout()
        hbox_val.addWidget(QLabel("Angle:"))
        self.spin_val = QSpinBox()
        self.spin_val.setRange(1, 360)
        self.spin_val.setValue(int(initial_value))
        hbox_val.addWidget(self.spin_val)
        layout.addLayout(hbox_val)

        # Action Buttons
        btn_layout = QHBoxLayout()
        
        self.btn_cw = QPushButton("Turn CW")
        self.btn_cw.clicked.connect(lambda: self.send_action("CW"))
        
        self.btn_ccw = QPushButton("Turn CCW")
        self.btn_ccw.clicked.connect(lambda: self.send_action("CCW"))

        btn_layout.addWidget(self.btn_cw)
        btn_layout.addWidget(self.btn_ccw)
        layout.addLayout(btn_layout)

        # Done Button
        self.btn_done = QPushButton("OK / Next Command")
        self.btn_done.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.btn_done.clicked.connect(self.finish_interaction)
        layout.addWidget(self.btn_done)

        # Cancel/Land Button
        self.btn_cancel = QPushButton("Cancel Flight (LAND)")
        self.btn_cancel.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        self.btn_cancel.clicked.connect(self.cancel_flight)
        layout.addWidget(self.btn_cancel)

        self.setLayout(layout)

    def send_action(self, direction):
        val = self.spin_val.value()
        msg = {"action": "rotate", "direction": direction, "value": val}
        
        # PREDICTIVE UPDATE (Dead Reckoning)
        try:
            if self.parent():
                step = val if direction == "CW" else -val
                self.parent().apply_rotation(step)
        except Exception as e:
            print(f"Predicitive rotation error: {e}")

        try:
            # Disable buttons while rotating
            self.set_buttons_enabled(False)
            self.repaint() # Force UI update
            
            self.socket_conn.sendall(json.dumps(msg).encode('utf-8'))
            
            # Wait for ACK/Completed from driver
            self.socket_conn.settimeout(10.0) 
            resp = self.socket_conn.recv(1024)
            print(f"[UI] Rotation ack: {resp}")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Communication error: {e}")
        finally:
            self.set_buttons_enabled(True)

    def finish_interaction(self):
        msg = {"action": "done"}
        try:
            self.socket_conn.sendall(json.dumps(msg).encode('utf-8'))
        except Exception as e:
            print(f"Error sending done: {e}")
        self.accept()

    def cancel_flight(self):
        msg = {"action": "land"}
        try:
            self.socket_conn.sendall(json.dumps(msg).encode('utf-8'))
        except Exception as e:
            print(f"Error sending land: {e}")
        self.reject() # Close dialog with reject code

    def set_buttons_enabled(self, enabled):
        self.btn_cw.setEnabled(enabled)
        self.btn_ccw.setEnabled(enabled)
        self.btn_done.setEnabled(enabled)
        self.btn_cancel.setEnabled(enabled)


class PermissionServer(QObject):
    request_received = pyqtSignal(object, dict) # socket, data

    def __init__(self, port=5590):
        super().__init__()
        self.port = port
        self.running = True

    def start(self):
        threading.Thread(target=self._server_loop, daemon=True).start()

    def _server_loop(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", self.port))
        server.listen(1)
        print(f"[UI] Permission Server listening on {self.port}")

        while self.running:
            try:
                conn, addr = server.accept()
                # Read the initial request
                data = conn.recv(4096)
                if not data:
                    conn.close()
                    continue
                
                try:
                    req = json.loads(data.decode('utf-8'))
                    self.request_received.emit(conn, req)
                except Exception as e:
                    print(f"Invalid permission request: {e}")
                    conn.close()

            except Exception as e:
                print(f"Permission server error: {e}")

class LogStream(QObject):
    """
    Terminal çıktılarını (print) yakalayıp PyQt sinyaline dönüştürür.
    Bu sayede threadlerden gelen print komutları arayüzü çökertmez.
    """
    new_text = pyqtSignal(str)

    def write(self, text):
        self.new_text.emit(str(text))

    def flush(self):
        pass

class Mainwindow(QMainWindow):

    path_received = pyqtSignal(list)
    drone_update = pyqtSignal(float, float, float)
    person_detected = pyqtSignal(dict)
    battery_update = pyqtSignal(int)

    def __init__(self):
        super().__init__()

        
        self.setAcceptDrops(True)

        # ===== PROCESS TRACKING =====
        self.drone_proc = None
        self.mission_proc = None
        self.yolo_proc = None
        self.default_map_path = "/home/abdu/surveillance_drone_project/missionPlanner/okul-map.svg"

        # ===== GRID CONFIG (MUST MATCH YAML) =====
        try:
            with open("/home/abdu/surveillance_drone_project/missionPlanner/params/general_params.yaml", "r") as f:
                params = yaml.safe_load(f)
                self.grid_width = params["grid"]["width"]
                self.grid_height = params["grid"]["height"]
                self.cell_size = params["grid"]["cell_size"]
                
                # Load Map Path from Params
                map_file_param = params.get("map", {}).get("map_file")
                if map_file_param:
                    self.default_map_path = map_file_param
                    print(f"[UI] Loaded map path: {self.default_map_path}")
                
                print(f"[UI] Loaded params: {self.grid_width}x{self.grid_height}, cell: {self.cell_size}")
        except Exception as e:
            print(f"[UI] ERROR loading params: {e}. Using defaults.")
            self.grid_width = 200
            self.grid_height = 200
            self.cell_size = 20
        # ==========================================
        # 2. ADIM: Sonra Matrisi Oluştur
        # ==========================================
        self.grid_matrix = [[0 for _ in range(self.grid_width)] for _ in range(self.grid_height)]
        
        # TEST İÇİN: Haritanın ortasına sanal bir duvar koyalım.
        # Bu koordinata tıkladığında pointer koymaması gerekir.
        # Ensure test wall is within bounds
        safe_y_start = min(90, self.grid_height - 10)
        safe_x_start = min(90, self.grid_width - 10)
        if safe_y_start > 0 and safe_x_start > 0:
             for y in range(safe_y_start, safe_y_start + 20):
                for x in range(safe_x_start, safe_x_start + 20):
                    if y < self.grid_height and x < self.grid_width:
                        self.grid_matrix[y][x] = 1  # 1 = Duvar

        self.setWindowTitle("Drone Mission Planning Interface")
        self.setGeometry(200, 200, 1400, 900)

        self.wifi_widget = WifiWidget(self)
        self.battery_widget = BatteryWidget(self)

        self.waypoints = []
        self.max_waypoints = 2

        # Keep track of path line items so we can clear them cleanly
        self.path_items = []
        self.person_items = {}

        self.setup_map()
        self.setup_log_dock()
        self.setup_yaw_slider()
        self.setup_top_toolbar()
        self.setup_main_toolbar()

        # ✅ connect signal to UI function (runs on Qt main thread)
        self.path_received.connect(self.on_path_received)
        self.drone_update.connect(self.update_drone_position)
        self.person_detected.connect(self.on_person_detected)
        self.battery_update.connect(self.on_battery_received)

        # ✅ start TCP listener in background
        self.start_path_listener()
        self.start_tello_listener()
        self.start_person_listener()
        
        # ✅ Start Permission Server
        self.perm_server = PermissionServer()
        self.perm_server.request_received.connect(self.handle_permission_request)
        self.perm_server.start()
        
        # ===== DRONE STATE =====
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.telemetry_yaw = 0.0 
        self.yaw_offset = 0.0
        # self.drone_yaw = 0.0 # Deprecated in favor of telemetry + offset
        #self.drone_scale = 20000  # pixels per meter (approx)
        self.trajectory_initialized = False
        self.low_battery_warned = False

        # ===== DRONE ICON =====
        # Uses independent item
        self.drone_item_visual = QGraphicsSvgItem("/home/abdu/surveillance_drone_project/UI/assets/drone_point.svg")
        self.scene.addItem(self.drone_item_visual)
        self.drone_item_visual.setZValue(100) # High Z value to be on top
        self.drone_item_visual.setScale(5)  # Scale down if too big burayı değiştirebilirsin
        # Center the item (approximation, better if we knew SVG size)
        # Using a fixed offset if we assume the icon is roughly centered in its viewbox
        # or we just rely on its own coordinate system.
        # Let's adjust origin for easier positioning
        d_bounds = self.drone_item_visual.boundingRect()
        self.drone_item_visual.setTransformOriginPoint(d_bounds.center())
        # To center it exactly on (0,0) initially -> we setPos later.
        # But visual origin is top-left.
        # We will handle centering logic in update_poisition or here by offset.
        # Let's just create it here.

        # ===== TRAJECTORY =====
        self.trajectory_path = QPainterPath()
        self.trajectory_item = self.scene.addPath(
            self.trajectory_path,
            QPen(Qt.GlobalColor.blue, 5)
        )
        self.trajectory_item.setZValue(90)
        self.camera_proc = None
        self.stream_proc = None

        self.ctx = zmq.Context()
        self.emergency_pub = self.ctx.socket(zmq.PUB)
        self.emergency_pub.bind("tcp://*:6002")



        self.statusBar().showMessage("Ready")
        # --- YENİ EKLENDİ: İlk test logu ---
        print("begining log panel.")

# ================= LOG DOCK SETUP (YENİ) =================
    def setup_log_dock(self):
        # 1. Dock Widget Oluştur
        self.log_dock = QDockWidget("System Logs", self)
        self.log_dock.setAllowedAreas(Qt.DockWidgetArea.LeftDockWidgetArea | Qt.DockWidgetArea.RightDockWidgetArea)
        
        # 2. İçine Metin Editörü Koy
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)  # Kullanıcı değiştiremesin
        self.log_text.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: Consolas;") # Hacker/Terminal teması
        self.log_dock.setWidget(self.log_text)

        # 3. Ana Pencereye Sol Tarafa Ekle
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.log_dock)
        
        # Başlangıçta kapalı olsun istiyorsan:
        # self.log_dock.hide()

        # 4. stdout ve stderr'i Yönlendir
        self.log_stream = LogStream()
        self.log_stream.new_text.connect(self.append_log)
        
        sys.stdout = self.log_stream
        sys.stderr = self.log_stream

    def setup_yaw_slider(self):
        # Create Dock
        self.yaw_dock = QDockWidget("Yaw Calibration", self)
        self.yaw_dock.setAllowedAreas(Qt.DockWidgetArea.RightDockWidgetArea | Qt.DockWidgetArea.BottomDockWidgetArea)
        
        # Widget container
        container = QWidget()
        layout = QVBoxLayout()
        
        # Label
        self.yaw_label = QLabel("Drone Angle: 0°")
        layout.addWidget(self.yaw_label)
        
        # Slider
        self.yaw_slider = QSlider(Qt.Orientation.Horizontal)
        self.yaw_slider.setRange(0, 360)
        self.yaw_slider.setValue(0)
        self.yaw_slider.valueChanged.connect(self.on_yaw_slider_change)
        layout.addWidget(self.yaw_slider)
        
        container.setLayout(layout)
        self.yaw_dock.setWidget(container)
        
        # Add to Bottom Right (Right area)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.yaw_dock)

    def on_yaw_slider_change(self, value):
        self.yaw_label.setText(f"Drone Angle Offset: {value}°")
        self.yaw_offset = float(value)
        self._update_visual_rotation()

    def _update_visual_rotation(self):
        # Combined rotation
        final_yaw = self.telemetry_yaw + self.yaw_offset
        if self.drone_item_visual:
            self.drone_item_visual.setRotation(final_yaw)

    def append_log(self, text):
        # İmleci sona taşı ve metni ekle
        self.log_text.moveCursor(self.log_text.textCursor().MoveOperation.End)
        self.log_text.insertPlainText(text)
        # Otomatik kaydırma
        self.log_text.ensureCursorVisible()
    # ================= MAP =================
    def setup_map(self):
        self.view = QGraphicsView(self)
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)
        self.setCentralWidget(self.view)
        self.view.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.view.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)

        self.map_item = QGraphicsSvgItem(self.default_map_path)
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

        # --- GRUP 1: Bağlantı ve Durum ---
        self.connect_act = QAction("Connect Drone", self)
        self.connect_act.triggered.connect(self.connect_drone)
        tb.addAction(self.connect_act)

        self.status_act = QAction("Check Status", self)
        self.status_act.triggered.connect(self.check_status)
        tb.addAction(self.status_act)

        tb.addSeparator() # Araya çizgi çeker

        # --- GRUP 2: Görev Planlama ---
        self.start_mission_act = QAction("Start Mission Planner", self)
        self.start_mission_act.triggered.connect(self.start_mission)
        tb.addAction(self.start_mission_act)

        # (Mevcut Find Way butonunu burada koruduk)
        self.start_flight_act = QAction("Start Flight", self)
        self.start_flight_act.triggered.connect(self.send_waypoints_to_mission_planner)
        tb.addAction(self.start_flight_act)

        tb.addSeparator()

        # --- GRUP 3: Kamera ve Algılama ---
        self.cam_act = QAction("Camera", self)
        self.cam_act.triggered.connect(self.toggle_camera)
        tb.addAction(self.cam_act)


        self.detect_act = QAction("Start Detection", self)
        self.detect_act.triggered.connect(self.start_detection)
        tb.addAction(self.detect_act)

        tb.addSeparator()

        # --- GRUP 4: Harita İşlemleri ---
        clear_act = QAction("Clear Map", self)
        clear_act.triggered.connect(self.clear_map_content)
        tb.addAction(clear_act)
        tb.addSeparator()

        self.land_button = QAction("Land Drone", self)
        self.land_button.triggered.connect(self.land_drone)
        tb.addAction(self.land_button)

        tb.addSeparator()
        # --- GRUP 5: Acil Durdurma (STOP) ---
        stop_act = QAction("EMERGENCY STOP", self)
        # İsteğe bağlı: Kırmızı renk vurgusu için stil eklenebilir ama QAction'da CSS zordur,
        # Genelde ikon kullanılır. Şimdilik metin kalsın.
        stop_act.triggered.connect(self.emergency_stop)
        tb.addAction(stop_act)

        tb.addSeparator()


        tb.addSeparator()
        # --- Mevcut Log Butonu ---
        self.show_logs_action = QAction("Show Logs", self)
        self.show_logs_action.setCheckable(True) 
        self.show_logs_action.setChecked(True)   
        self.show_logs_action.triggered.connect(self.toggle_logs)
        tb.addAction(self.show_logs_action)
        
        self.log_dock.visibilityChanged.connect(self.show_logs_action.setChecked)
        
        # Initial State Check
        self.update_ui_state()

    def update_ui_state(self):
        """
        Enables/Disables buttons based on running processes.
        """
        drone_running = self.drone_proc is not None and self.drone_proc.poll() is None
        mission_running = self.mission_proc is not None and self.mission_proc.poll() is None

        # CONNECT: allow only if not running
        self.connect_act.setEnabled(not drone_running)
        self.start_mission_act.setEnabled(not mission_running)

        # START FLIGHT: Only if BOTH are running
        if drone_running and mission_running:
            self.start_flight_act.setEnabled(True)
            self.start_flight_act.setToolTip("Ready to fly")
        else:
            self.start_flight_act.setEnabled(False)
            msg = []
            if not drone_running: msg.append("Drone not connected")
            if not mission_running: msg.append("Mission Planner not started")
            self.start_flight_act.setToolTip(", ".join(msg))

        # ACTIONS: Only if Drone is running
        self.cam_act.setEnabled(drone_running)
        self.detect_act.setEnabled(drone_running)
        self.land_button.setEnabled(drone_running) # Land requires connection





    # --- YENİ EKLENDİ: Log Toggle Fonksiyonu ---
    def toggle_logs(self, checked):
        if checked:
            self.log_dock.show()
        else:
            self.log_dock.hide()

    # ================= EVENTS =================
    def eventFilter(self, obj, event):
        # Ctrl + Mouse Wheel → Zoom
        if obj == self.view.viewport() and event.type() == QEvent.Type.Wheel:
            if event.modifiers() & Qt.KeyboardModifier.ControlModifier:
                zoom_factor = 1.15

                if event.angleDelta().y() > 0:
                    self.view.scale(zoom_factor, zoom_factor)
                else:
                    self.view.scale(1 / zoom_factor, 1 / zoom_factor)

                return True  # consume event

        # --- MOUSE TIKLAMA (GÜNCELLENDİ) ---
        if obj == self.view.viewport() and event.type() == QEvent.Type.MouseButtonPress:
            if event.button() == Qt.MouseButton.LeftButton:
                clicked_pos = self.view.mapToScene(event.pos())

                # 1. KONTROL: Tıklanan yer harita resminin sınırları içinde mi?
                # Eğer harita resminin dışındaki siyah boşluğa tıklarsan reddeder.
                if hasattr(self, "map_item"):
                    map_bounds = self.map_item.boundingRect()
                    if not map_bounds.contains(clicked_pos):
                    
                        return True

                # Koordinatları Grid'e çevir
                gx, gy = self.scene_to_grid(clicked_pos)

                # 2. KONTROL: Grid üzerinde duvar veya engel var mı?
                if not self.is_location_walkable(gx, gy):
                    # Kullanıcıya uyarı ver (Sol alttaki status bar'da yazar)
                    self.statusBar().showMessage(f"Engel! ({gx}, {gy}) noktasına gidilemez.", 2000)
                    return True

                # Her şey tamamsa Waypoint ekle
                if len(self.waypoints) == self.max_waypoints:
                    self.clear_waypoints()

                if len(self.waypoints) == 0:
                    self.add_waypoint(clicked_pos, "A", Qt.GlobalColor.green)
                elif len(self.waypoints) == 1:
                    self.add_waypoint(clicked_pos, "B", Qt.GlobalColor.red)
                
                return True

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
# ================= VALIDATION (KONTROL) =================
    def is_location_walkable(self, gx, gy):
        # ---------------------------------------------------------
        # AYAR: Güvenlik Payı (Duvarlardan kaç kare uzak durulsun?)
        # Örnek: 2 yaparsan duvara 2 kare (40 birim) yaklaşamazsın.
        margin = 2 
        # ---------------------------------------------------------

        # 1. Kenar Kontrolü (Sınırlara çok yakınsa YASAKLA)
        # Sol kenar (0) ve Sağ kenar (width) kontrolü
        if gx < margin or gx >= (self.grid_width - margin):
            return False
        
        # Üst kenar (0) ve Alt kenar (height) kontrolü
        if gy < margin or gy >= (self.grid_height - margin):
            return False
        
        # 2. İç Duvar (Matrix) Kontrolü
        try:
            if self.grid_matrix[gy][gx] == 1:
                return False
        except IndexError:
            return False

        return True
    
    def grid_to_scene(self, gx, gy):
        world_x = gx * self.cell_size
        world_y = gy * self.cell_size

        sx = (world_x / (self.grid_width * self.cell_size)) * self.map_width_scene
        sy = (world_y / (self.grid_height * self.cell_size)) * self.map_height_scene

        return sx, sy

    # ================= WAYPOINTS =================
    def add_waypoint(self, pos, label, color):
        size = 500 # burayı değiştirebilirsin

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
        
        # [FEATURE] Teleport drone to Waypoint A
        if label == "A":
            # 1. Scene → Grid
            gx, gy = self.scene_to_grid(pos)

            # 2. Grid → World (cm)
            world_x_cm = gx * self.cell_size
            world_y_cm = gy * self.cell_size

            # 3. World (cm) → meters
            new_drone_x = world_x_cm / 100.0
            new_drone_y = world_y_cm / 100.0

            print(
                f"[UI] Teleporting drone to A | "
                f"Grid=({gx},{gy}) "
                f"World=({world_x_cm:.1f}cm,{world_y_cm:.1f}cm)"
            )

            # 4. Set state and update visualization
            self.drone_x = new_drone_x
            self.drone_y = new_drone_y
            self.update_drone_position(self.drone_x, self.drone_y)

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
        print(f"[UI] Drawing path with {len(path)} points")
        print(path)
        if not path or len(path) < 2:
            return

        pen = QPen(Qt.GlobalColor.green)
        pen.setWidth(5) # burayı değiştirebilirsin

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

                vx = vgx / 100
                vy = vgy / 100
                # Tello Yaw 0 is North (Up), but Math 0 is East (Right).
                # So we subtract 90 degrees for movement calculation.
                yaw_math = math.radians(yaw_deg - 90)

                wx = vx * math.cos(yaw_math) - vy * math.sin(yaw_math)
                wy = vx * math.sin(yaw_math) + vy * math.cos(yaw_math)

                self.drone_x += wx * dt
                self.drone_y += wy * dt

                self.drone_update.emit(self.drone_x, self.drone_y, float(yaw_deg))
                self.battery_update.emit(int(msg.get("bat", 0)))
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
                        print("[UI] Person detected:", msg) 
                        if msg.get("event") == "new_person":
                            self.person_detected.emit(msg)
                    except Exception as e:
                        print(f"Person ZMQ Error: {e}")
                        import time
                        time.sleep(1) 
            except Exception as e:
                print(f"Could not connect to person stream: {e}")

        threading.Thread(target=listener).start()

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
            pin.setZValue(200) # Below drone, above map
            
            PERSON_RADIUS_CM = 25  # half meter diameter


            scene_per_cm_x = self.map_width_scene / (self.grid_width * self.cell_size)
            scene_per_cm_y = self.map_height_scene / (self.grid_height * self.cell_size)

            scene_per_cm = min(scene_per_cm_x, scene_per_cm_y)

            target_scene_size = PERSON_RADIUS_CM * 2 * scene_per_cm

            b = pin.boundingRect()
            scale = target_scene_size / max(b.width(), b.height())
            pin.setScale(scale)

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

    def on_battery_received(self, level):
        self.battery_widget.setValue(level)
        
        if level < 20 and not self.low_battery_warned:
            self.low_battery_warned = True 
            QMessageBox.warning(self, "Low Battery", f"Warning: Drone battery is low! ({level}%)")
            self.statusBar().showMessage(f"⚠️ LOW BATTERY: {level}%", 0) 
        elif level >= 20:
             self.low_battery_warned = False

    # ================= PERMISSION REQUEST =================
    @pyqtSlot(object, dict)
    def handle_permission_request(self, conn, data):
        # Retrieve info from data
        direction = data.get("cmd", "CW")
        value = data.get("value", 90)
        
        # Open Dialog
        dialog = RotationControlDialog(self, conn, direction, value)
        dialog.exec()
        
        # Close connection after dialog finishes (dialog sends 'done' before closing)
        try:
            conn.close()
        except:
            pass

    # ================= UI UPDATE =================
    def apply_rotation(self, delta_deg):
        """
        Manually updates the drone's rotation (Dead Reckoning).
        This is called by the permission dialog to simulate rotation immediately.
        """
        # For dead reckoning, we can update telemetry_yaw directly
        self.telemetry_yaw += delta_deg
        
        self._update_visual_rotation()
        
        # Sync slider if exists
        if hasattr(self, "yaw_slider"):
            self.yaw_slider.blockSignals(True)
            self.yaw_slider.setValue(int(self.drone_yaw % 360))
            self.yaw_slider.blockSignals(False)

    def set_drone_yaw(self, angle):
        """
        Sets the absolute rotation of the drone (Calibration).
        """
        # This function might be deprecated or used for hard-reset. 
        # If used for calibration, it should set offset.
        # But let's assume it sets the base telemetry value if called programmatically.
        self.telemetry_yaw = angle
        self._update_visual_rotation()
            
    def update_drone_position(self, x_m, y_m, yaw_deg=0.0):
        # 1. meters → centimeters
        x_cm = x_m * 100.0
        y_cm = y_m * 100.0

        # 2. centimeters → grid (float, not int!)
        gx = x_cm / self.cell_size
        gy = y_cm / self.cell_size

        # 3. grid → scene
        sx = (gx / self.grid_width)  * self.map_width_scene
        sy = (gy / self.grid_height) * self.map_height_scene

        # Update yaw state and visual
        self.telemetry_yaw = yaw_deg
        self._update_visual_rotation()

        if not self.drone_item_visual.isVisible():
            self.drone_item_visual.show()

        # Center icon
        bounds = self.drone_item_visual.boundingRect()
        self.drone_item_visual.setPos(
            sx - bounds.width() / 2,
            sy - bounds.height() / 2
        )

        # Trajectory
        if not self.trajectory_initialized:
            self.trajectory_path.moveTo(sx, sy)
            self.trajectory_initialized = True
        else:
            self.trajectory_path.lineTo(sx, sy)

        self.trajectory_item.setPath(self.trajectory_path)


    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.acceptProposedAction()

    def dropEvent(self, event):
        urls = event.mimeData().urls()
        if not urls:
            return

        file_path = urls[0].toLocalFile()

        if not file_path.lower().endswith(".svg"):
            QMessageBox.warning(self, "Invalid File", "Only SVG map files allowed")
            return

        self.change_map(file_path)


    def change_map(self, svg_path):
        # SADECE map'i değiştir, scene'e dokunma
        if hasattr(self, "map_item") and self.map_item:
            self.scene.removeItem(self.map_item)

        self.map_item = QGraphicsSvgItem(svg_path)
        self.map_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.map_item.setZValue(-1000)  # her şeyin arkasında

        self.scene.addItem(self.map_item)

    def connect_drone(self):
        print(">> [Command] Attempting to connect to drone...")
        self.drone_proc = subprocess.Popen(
            ["execs/tello_driver"]          # or "my_program.exe" on Windows
        )
        time.sleep(0.5)  # Give it a moment to start
        if self.drone_proc.poll() is None:
            print("Tello Driver started successfully.")
        else:
            print("Failed to start Tello Driver.")
        
        self.update_ui_state()

    def check_status(self):
        print(">> [Command] Checking drone status...")
        self.update_ui_state()
        
        # Collect processes
        procs = {
            "Tello Driver (Drone)": self.drone_proc,
            "Mission Planner": self.mission_proc,
            "Person Detection (Yolo)": self.yolo_proc,
            "Video Streamer": self.stream_proc,
            "Camera Viewer (UI)": self.camera_proc,
        }
        
        dlg = StatusDialog(self, procs)
        dlg.exec()


    def start_mission(self):

        print(">> [Command] Starting  mission...")
        self.mission_proc = subprocess.Popen(
            ["execs/mission_planner"]          
        )
        time.sleep(0.5)  # Give it a moment to start
        if self.mission_proc.poll() is None:
            print("Mission Planner started successfully.")
        else:
            print("Failed to start Mission Planner.")
        
        self.update_ui_state()

    def toggle_camera(self):


        if self.camera_proc and self.camera_proc.poll() is None:
            print("[Camera] Stopping camera process")
            self.camera_proc.terminate()
            self.camera_proc.wait()
            self.camera_proc = None
            return

        if self.stream_proc and self.stream_proc.poll() is None:
            print("[Camera] Stopping stream process")
            self.stream_proc.terminate()
            self.stream_proc.wait()
            self.stream_proc = None
        
        print("[Camera] Starting camera process")
        self.stream_proc = subprocess.Popen(
            ["/home/abdu/surveillance_drone_project/execs/video_streamer"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(0.5)  # Give it a moment to start
        if self.stream_proc.poll() is None:
            print("Tello Streamer started successfully.")
        else:
            print("Failed to start Tello Streamer.")

        self.camera_proc = subprocess.Popen(
            [sys.executable, "/home/abdu/surveillance_drone_project/UI/camera_view.py"]
            

        )


    def start_detection(self):
        

        print(">> [Command] Object detection (Detection) started.")

        yolo_python = "/home/abdu/surveillance_drone_project/person-detection/venv/bin/python"
        yolo_script = "/home/abdu/surveillance_drone_project/person-detection/src/YoloWorker.py"

        proc = subprocess.Popen(
            [yolo_python, yolo_script],
            cwd="/home/abdu/surveillance_drone_project/person-detection"
        )

        time.sleep(0.5)

        if proc.poll() is None:
            print("YoloWorker started successfully.")
            self.yolo_proc = proc
        else:
            print("Failed to start YoloWorker.")


    def clear_map_content(self):
        print(">> [Command] Clearing map content...")
        self.clear_waypoints() 
        self.clear_path()      
        self.clear_persons()
        
        # Clear drone trajectory
        self.trajectory_path = QPainterPath()
        self.trajectory_item.setPath(self.trajectory_path)
        self.trajectory_initialized = False
        
        # Hide drone until next update
        if hasattr(self, "drone_item_visual"):
            self.drone_item_visual.hide()
        
    def land_drone(self):
        msg = {"type": "land"}
        self.emergency_pub.send_json(msg)
        


    def emergency_stop(self):
        print("!! [EMERGENCY] STOP COMMAND SENT !!")
        # connect 6002 and send emergency stop with zmq

        msg = {"type": "emergency_stop"}
        self.emergency_pub.send_json(msg)
        self.emergency_pub.close()
        self.ctx.term()



if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = Mainwindow()
    w.show()
    sys.exit(app.exec())
