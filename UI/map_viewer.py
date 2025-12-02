import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsView, QGraphicsScene, QWidget, QVBoxLayout
from PyQt5.QtSvg import QGraphicsSvgItem
from PyQt5.QtGui import QWheelEvent
from PyQt5.QtCore import Qt

class MapViewer(QGraphicsView):
    def __init__(self, svg_path):
        super().__init__()

        # Drag ile kaydırma aktif
        self.setDragMode(QGraphicsView.ScrollHandDrag)

        # Scroll’un zoomu mouse altında yapsın
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)

        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)

        # SVG dosyasını ekle
        self.map_item = QGraphicsSvgItem(svg_path)
        self.scene.addItem(self.map_item)

        # View'e sığdır (siyah/boş gelmesini engeller)
        self.fitInView(self.map_item, Qt.KeepAspectRatio)

        self.scale_factor = 1.2

    def wheelEvent(self, event: QWheelEvent):
        if event.angleDelta().y() > 0:
            self.scale(self.scale_factor, self.scale_factor)
        else:
            self.scale(1 / self.scale_factor, 1 / self.scale_factor)

class MainWindow(QMainWindow):
    def __init__(self, svg_path):
        super().__init__()
        self.setWindowTitle("SVG Map Viewer")

        central = QWidget()
        layout = QVBoxLayout(central)
        self.setCentralWidget(central)

        viewer = MapViewer(svg_path)
        layout.addWidget(viewer)

        self.resize(1000, 700)

app = QApplication(sys.argv)

# 📌 Bunu SAKIN değiştirme, asset olarak yüklediğin SVG'nin yolu:
svg_file = "/Users/esin/drone projemiz/src/assets/map.svg"

window = MainWindow(svg_file)
window.show()

sys.exit(app.exec_())

