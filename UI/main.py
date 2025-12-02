import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QToolBar, QWidget, QSizePolicy, QGraphicsView, QGraphicsScene
from PyQt6.QtGui import QPainter, QPen, QBrush
from PyQt6.QtCore import Qt, QEvent
from PyQt6.QtSvgWidgets import QGraphicsSvgItem

from drone_worker import DroneWorker
from battery_widget import BatteryWidget
from wifi_widget import WifiWidget
from PyQt6.QtCore import QTimer

class Mainwindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Mission Planning Interface")
        self.setGeometry(200, 200, 1400, 900)

        self.wifi_widget = WifiWidget(self)
        self.battery_widget = BatteryWidget(self)

        # A/B ve çizgi
        self.A_point = None
        self.B_point = None
        self.line_item = None
        self.text_items = []  # A ve B yazıları burada tutulacak

        self.setup_map()
        self.setup_top_toolbar()
        self.setup_main_toolbar()
        #self.setup_drone_worker()

        self.statusBar().showMessage("Ready")

    def setup_map(self):
        self.view = QGraphicsView(self)
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)
        self.setCentralWidget(self.view)

        # SVG yükle
        self.map_item = QGraphicsSvgItem("/home/abdu/surveillance_drone_project/UI/assets/map.svg")

    
            
        self.scene.addItem(self.map_item)

        # harita sınırlarını al
        bounds = self.map_item.boundingRect()

        # Scene'i ayarla
        self.scene.setSceneRect(bounds)

        # Açılışta haritayı ekrana fit et + biraz büyüt
        self.view.resetTransform()
        self.view.fitInView(bounds, Qt.AspectRatioMode.KeepAspectRatio)
        self.view.scale(30.0, 30.0)  # Haritayı biraz büyük başlatır ✅

        # Pan ve Render ayarları
        self.view.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.view.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)

        # Tıklama dinlemeyi bağla
        self.view.viewport().installEventFilter(self)

    def setup_top_toolbar(self):
        top = QToolBar("Status", self)
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, top)
        top.setMovable(False)

        record_btn = top.addAction("Record Drone Video")
        record_btn.triggered.connect(self.toggle_recording)

        spacer = QWidget(self)
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        top.addWidget(spacer)

        top.addWidget(self.wifi_widget)
        top.addWidget(self.battery_widget)

    def setup_main_toolbar(self):
        tb = self.addToolBar("Actions")
        tb.addAction("Add Spot")
        tb.addAction("Add Camera")
        tb.addAction("Add Danger Zone")
        tb.addAction('Set Waypoint')
        
    def setup_drone_worker(self):
        self.worker = DroneWorker()
        self.worker.start()

    def toggle_recording(self):
        if not self.worker.recording:
            self.worker.start_recording()
        else:
            self.worker.stop_recording()

    # --- Mouse ile A/B seçme ve çizgi çizme ---
    def eventFilter(self, obj, event):
        if obj == self.view.viewport() and event.type() == QEvent.Type.MouseButtonPress:
            clicked = self.view.mapToScene(event.pos())

            # İlk tık → A
            if self.A_point is None:
                self.clear_markers()
                self.A_point = clicked
                tA = self.scene.addText("A")
                tA.setPos(clicked.x() + 6, clicked.y() - 6)
                self.text_items.append(tA)

            # İkinci tık → B + çizgi
            elif self.B_point is None:
                self.B_point = clicked
                tB = self.scene.addText("B")
                tB.setPos(clicked.x() + 6, clicked.y() - 6)
                self.text_items.append(tB)
                self.draw_line()

            # 3. ve sonrası → reset ve yeni A
            else:
                self.clear_markers()
                self.A_point = clicked
                tA = self.scene.addText("A")
                tA.setPos(clicked.x() + 6, clicked.y() - 6)
                self.text_items.append(tA)

            return False  # event tüketilmez, crash olmaz ✅
        return False

    def draw_line(self):
        if self.A_point and self.B_point:
            pen = QPen(Qt.GlobalColor.red)
            pen.setWidthF(10.)
            self.line_item = self.scene.addLine(
                self.A_point.x(), self.A_point.y(),
                self.B_point.x(), self.B_point.y(),
                pen
            )

    def clear_markers(self):
        # Haritayı koru, sadece marker ve textleri sil ✅
        for item in list(self.scene.items()):
            if item is self.map_item:
                continue
            self.scene.removeItem(item)

        # reset
        self.A_point = None
        self.B_point = None
        self.line_item = None
        self.text_items = []

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = Mainwindow()
    w.show()
    sys.exit(app.exec())
