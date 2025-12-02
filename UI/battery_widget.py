# battery_widget.py

from PyQt6.QtWidgets import (QWidget, QLabel, QHBoxLayout, QVBoxLayout, QFrame, 
                             QApplication, QMenu, QWidgetAction)
from PyQt6.QtGui import QPainter, QColor, QBrush, QPen, QPainterPath, QMouseEvent
from PyQt6.QtCore import Qt, QSize

battery_level = 10

# _BatteryIcon sınıfında herhangi bir değişiklik yapmamıza gerek yok.
class _BatteryIcon(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._battery_level = battery_level
        self.setMinimumSize(QSize(40, 18))
    def setValue(self, level):
        self._battery_level = level
        self.update()
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        margin = 2
        pen_width = 1.5 
        body_height = rect.height() - (margin + pen_width) * 2
        head_proportion = 0.15 
        body_width = rect.width() - (margin + pen_width) * 2 - body_height * head_proportion 
        body_x = margin + pen_width
        body_y = margin + pen_width
        corner_radius = body_height * 0.25
        head_height = body_height * 0.4
        head_width = body_height * head_proportion
        head_x = body_x + body_width
        head_y = body_y + (body_height - head_height) / 2
        head_corner_radius = head_width * 0.5
        path = QPainterPath()
        path.addRoundedRect(int(body_x), int(body_y), int(body_width), int(body_height), corner_radius, corner_radius)
        pen = QPen(QColor(200, 200, 200))
        pen.setWidthF(pen_width)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawPath(path)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.drawRoundedRect(int(head_x), int(head_y), int(head_width), int(head_height), head_corner_radius, head_corner_radius)
        fill_color = QColor(255, 255, 255)
        painter.setBrush(QBrush(fill_color))
        painter.setPen(Qt.PenStyle.NoPen)
        inner_margin = 2.5
        fill_x = body_x + inner_margin
        fill_y = body_y + inner_margin
        fillable_height = body_height - (inner_margin * 2)
        fillable_width = body_width - (inner_margin * 2)
        fill_width = (fillable_width * self._battery_level) / 100
        fill_corner_radius = corner_radius * 0.8
        painter.drawRoundedRect(
            int(fill_x), int(fill_y), int(fill_width), int(fillable_height), 
            fill_corner_radius, fill_corner_radius
        )


# --- YENİ: Dropdown menünün İÇERİĞİNİ oluşturacak özel widget ---
class _BatteryInfoWidget(QWidget):
    def __init__(self, battery_level, parent=None):
        super().__init__(parent)
        
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(8)

        # 1. Satır: Başlık ve yüzde
        title_layout = QHBoxLayout()
        title_label = QLabel("<b>Battery</b>") # Kalın yazı için HTML tag'ı
        title_label.setStyleSheet("color: white; font-size: 14px;")
        percent_label = QLabel(f"%{int(battery_level)}")
        percent_label.setStyleSheet("color: #dddddd; font-size: 14px;")
        title_layout.addWidget(title_label)
        title_layout.addStretch() # Araya boşluk koyarak yüzdeyi sağa iter
        title_layout.addWidget(percent_label)
        
        # 2. Satır ve sonrası için etiketler
        # Dronunuzdan bu verileri alabiliyorsanız, burayı dinamik hale getirebilirsiniz.
        source_label = QLabel("Power Source: Drone Battery")
        source_label.setStyleSheet("color: #cccccc;")
        energy_label = QLabel("No Apps Using Significant Energy") # Bu satırı kendi durumunuza göre değiştirebilirsiniz
        energy_label.setStyleSheet("color: #cccccc;")

        # Ayırıcı çizgi
        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setFrameShadow(QFrame.Shadow.Sunken)
        separator.setStyleSheet("background-color: #555; height: 1px; border: none;")

        # "Battery Settings..." butonu gibi davranan etiket
        settings_label = QLabel("Battery Details...")
        settings_label.setStyleSheet("color: #cccccc;")
        
        # Tüm elemanları ana layout'a ekle
        main_layout.addLayout(title_layout)
        main_layout.addWidget(separator)
        main_layout.addWidget(source_label)
        main_layout.addWidget(energy_label)
        main_layout.addWidget(settings_label)


# --- ANA WIDGET GÜNCELLEMESİ ---
class BatteryWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 5, 0)
        layout.setSpacing(5)
        self.label = QLabel(f"{battery_level}%", self)
        self.label.setStyleSheet("color: white; font-weight: bold; font-size: 12px;")
        self.icon = _BatteryIcon(self)
        layout.addWidget(self.label)
        layout.addWidget(self.icon)
        self.setLayout(layout)

    def setValue(self, level):
        self.label.setText(f"{int(level)}%")
        self.icon.setValue(level)

    def mousePressEvent(self, event: QMouseEvent):
        """Widget'a tıklandığında bu fonksiyon çalışır ve özel menüyü gösterir."""
        
        # 1. Özel içeriğimizi oluştur
        info_widget = _BatteryInfoWidget(self.icon._battery_level)

        # 2. İçeriği bir QMenu'ye yerleştirmek için QWidgetAction kullan
        action = QWidgetAction(self)
        action.setDefaultWidget(info_widget)

        # 3. Ana menü konteynerini oluştur
        menu = QMenu(self)
        menu.addAction(action)

        # 4. Menünün stilini ayarla (macOS görünümü için)
        menu.setStyleSheet("""
            QMenu {
                background-color: rgba(45, 45, 45, 0.9); /* Yarı saydam koyu gri */
                color: white;
                border: 1px solid #666;
                border-radius: 10px;
                padding: 5px;
            }
            QMenu::item {
                /* Seçim efektini kaldır, çünkü tüm widget tek parça */
                background-color: transparent;
            }
        """)

        # 5. Menüyü, pil ikonunun altında doğru konumda göster
        point = self.mapToGlobal(self.rect().bottomLeft())
        menu.exec(point)
        
        super().mousePressEvent(event)