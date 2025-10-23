# gui.py
import sys, subprocess
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QFileDialog, QTextEdit

class MapGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Mission Planner")
        self.resize(600, 400)

        layout = QVBoxLayout(self)
        self.btn_open = QPushButton("Open Map (SVG)")
        self.text_output = QTextEdit()
        layout.addWidget(self.btn_open)
        layout.addWidget(self.text_output)

        self.btn_open.clicked.connect(self.open_map)

    def open_map(self):
        file, _ = QFileDialog.getOpenFileName(self, "Choose SVG Map", "", "SVG Files (*.svg)")
        if not file:
            return

        # Call C++ program to compute distances
        try:
            result = subprocess.run(
                ["./map-calc", file],
                capture_output=True,
                text=True,
                check=True
            )
            self.text_output.setPlainText(result.stdout)
        except subprocess.CalledProcessError as e:
            self.text_output.setPlainText("Error:\n" + e.stderr)

app = QApplication(sys.argv)
window = MapGUI()
window.show()
app.exec()
