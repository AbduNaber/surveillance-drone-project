import sys
import json
import socket
import threading
import math
import zmq

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QToolBar, QWidget,
    QSizePolicy, QGraphicsView, QGraphicsScene,
    QGraphicsPathItem, QMessageBox,QDockWidget,QTextEdit
)
from PyQt6.QtGui import QPainter, QPen, QBrush, QPainterPath
from PyQt6.QtCore import Qt, QEvent, pyqtSignal,QObject
from PyQt6.QtSvgWidgets import QGraphicsSvgItem

from battery_widget import BatteryWidget
from wifi_widget import WifiWidget
from PyQt6.QtGui import QAction
# --- YENİ EKLENDİ: Logları Yakalamak İçin Sınıf ---
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
    # ✅ Signal to safely move data from socket thread → Qt main thread
    path_received = pyqtSignal(list)
    drone_update = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()


        self.setAcceptDrops(True)
        self.default_map_path = "/Users/esin/drone projemiz/src/assets/map.svg"

        # ===== GRID CONFIG (MUST MATCH YAML) =====
        self.grid_width = 200
        self.grid_height = 200
        self.cell_size = 20  # world units per cell
        # ==========================================
        # 2. ADIM: Sonra Matrisi Oluştur
        # ==========================================
        self.grid_matrix = [[0 for _ in range(self.grid_width)] for _ in range(self.grid_height)]
        
        # TEST İÇİN: Haritanın ortasına sanal bir duvar koyalım.
        # Bu koordinata tıkladığında pointer koymaması gerekir.
        for y in range(90, 110):
            for x in range(90, 110):
                self.grid_matrix[y][x] = 1  # 1 = Duvar

        self.setWindowTitle("Drone Mission Planning Interface")
        self.setGeometry(200, 200, 1400, 900)

        self.wifi_widget = WifiWidget(self)
        self.battery_widget = BatteryWidget(self)

        self.waypoints = []
        self.max_waypoints = 2

        # Keep track of path line items so we can clear them cleanly
        self.path_items = []

        self.setup_map()
        self.setup_log_dock()
        self.setup_top_toolbar()
        self.setup_main_toolbar()

        # ✅ connect signal to UI function (runs on Qt main thread)
        #self.path_received.connect(self.on_path_received)
        #self.drone_update.connect(self.update_drone_position)

        # ✅ start TCP listener in background
        #self.start_path_listener()
        #self.start_tello_listener()
        
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

        # --- GRUP 1: Bağlantı ve Durum ---
        connect_act = QAction("Connect Drone", self)
        connect_act.triggered.connect(self.connect_drone)
        tb.addAction(connect_act)

        status_act = QAction("Check Status", self)
        status_act.triggered.connect(self.check_status)
        tb.addAction(status_act)

        tb.addSeparator() # Araya çizgi çeker

        # --- GRUP 2: Görev Planlama ---
        start_mission_act = QAction("Start Mission Planner", self)
        start_mission_act.triggered.connect(self.start_mission)
        tb.addAction(start_mission_act)

        # (Mevcut Find Way butonunu burada koruduk)
        find_way_btn = tb.addAction("Find Way")
        find_way_btn.triggered.connect(self.send_waypoints_to_mission_planner)

        tb.addSeparator()

        # --- GRUP 3: Kamera ve Algılama ---
        cam_act = QAction("Camera", self)
        cam_act.triggered.connect(self.toggle_camera)
        tb.addAction(cam_act)

        detect_act = QAction("Start Detection", self)
        detect_act.triggered.connect(self.start_detection)
        tb.addAction(detect_act)

        tb.addSeparator()

        # --- GRUP 4: Harita İşlemleri ---
        clear_act = QAction("Clear Map", self)
        clear_act.triggered.connect(self.clear_map_content)
        tb.addAction(clear_act)

        # --- GRUP 5: Acil Durdurma (STOP) ---
        stop_act = QAction("STOP", self)
        # İsteğe bağlı: Kırmızı renk vurgusu için stil eklenebilir ama QAction'da CSS zordur,
        # Genelde ikon kullanılır. Şimdilik metin kalsın.
        stop_act.triggered.connect(self.emergency_stop)
        tb.addAction(stop_act)

        tb.addSeparator()

        # --- Mevcut Log Butonu ---
        self.show_logs_action = QAction("Show Logs", self)
        self.show_logs_action.setCheckable(True) 
        self.show_logs_action.setChecked(True)   
        self.show_logs_action.triggered.connect(self.toggle_logs)
        tb.addAction(self.show_logs_action)
        
        self.log_dock.visibilityChanged.connect(self.show_logs_action.setChecked)



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
            print("[UI] ZMQ listener starting on tcp://127.0.0.1:6000")

            ctx = zmq.Context()
            sock = ctx.socket(zmq.SUB)
            sock.connect("tcp://127.0.0.1:6000")
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

        # Need to import time inside method or file level if not already
        import time 
        threading.Thread(target=listener, daemon=True).start()

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
        # Buraya drone bağlantı kodu gelecek (örn: Tello connect)

    def check_status(self):
        print(">> [Command] Checking drone status...")
        # Batarya, ısı vb. verileri çekmek için

    def start_mission(self):
        print(">> [Command] Starting  mission...")
        # Otonom görevi başlatma sinyali

    def toggle_camera(self):
        print(">> [Command] Camera on/off toggled.")
        # Video akışını başlatmak/durdurmak için

    def start_detection(self):
        print(">> [Command] Object detection (Detection) started.")
        # YOLO veya OpenCV algoritmalarını tetiklemek için

    def clear_map_content(self):
        print(">> [Command] Clearing map content...")
        self.clear_waypoints() # Mevcut fonksiyonunuz
        self.clear_path()      # Mevcut fonksiyonunuz
        # Eğer drone ikonunu da sıfırlamak isterseniz buraya ekleyebilirsiniz.

    def emergency_stop(self):
        print("!! [EMERGENCY] STOP COMMAND SENT !!")
        # Drone'a acil iniş veya motor durdurma komutu (Land/Emergency)

# ================= YENİ BUTON FONKSİYONLARI =================
    
if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = Mainwindow()
    w.show()
    sys.exit(app.exec())
