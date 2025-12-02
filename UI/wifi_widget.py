# wifi_widget.py

from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QColor, QPen
from PyQt6.QtCore import Qt, QSize, QRectF

class WifiWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._signal_level = 4  # Başlangıç değeri: 4 (tam çekiyor)
        self.setMinimumSize(QSize(28, 22))

    def setValue(self, level):
        """
        Sinyal seviyesini güncellemek için bu fonksiyon çağrılır.
        Beklenen değer 0, 1, 2, 3, veya 4.
        """
        if 0 <= level <= 4:
            self._signal_level = level
            self.update()  # Widget'ı yeniden çizmek için tetikle

    def paintEvent(self, event):
        """Widget'ı ekrana çizen ana fonksiyon."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Renkleri belirle
        active_color = QColor(255, 255, 255)  # Beyaz (sinyalin olduğu kısım)
        inactive_color = QColor(80, 80, 80)   # Koyu Gri (sinyalin olmadığı kısım)

        # Kalem ayarları
        pen = QPen()
        pen.setWidth(2)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap) # Çizgi uçlarını yuvarlak yap

        # Sinyal yoksa (level 0), kırmızı bir 'X' çizelim
        if self._signal_level == 0:
            pen.setColor(QColor(200, 0, 0)) # Kırmızı
            painter.setPen(pen)
            rect = self.rect().adjusted(4, 4, -4, -4) # Kenarlardan boşluk bırak
            painter.drawLine(rect.topLeft(), rect.bottomRight())
            painter.drawLine(rect.topRight(), rect.bottomLeft())
            return # Fonksiyondan çık, başka bir şey çizme

        # Tüm sinyal yaylarını önce inaktif renkte çizelim
        pen.setColor(inactive_color)
        painter.setPen(pen)

        center_x = self.width() / 2
        center_y = self.height() - 5 # Merkezin dikey konumu (alta yakın)
        radii = [4, 8, 12, 16] # 4 yayın yarıçapı

        for r in radii:
            rect = QRectF(center_x - r, center_y - r, r * 2, r * 2)
            # 135 dereceden başla, saat yönünün tersine 90 derece çiz
            start_angle = 135 * 16
            span_angle = -90 * 16
            painter.drawArc(rect, start_angle, span_angle)

        # Şimdi sinyal seviyesine göre aktif olanları beyaz renkle üstüne çizelim
        pen.setColor(active_color)
        painter.setPen(pen)

        for i in range(self._signal_level):
            r = radii[i]
            rect = QRectF(center_x - r, center_y - r, r * 2, r * 2)
            start_angle = 135 * 16
            span_angle = -90 * 16
            painter.drawArc(rect, start_angle, span_angle)