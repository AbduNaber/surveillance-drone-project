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
        self.resize(400, 300)
        self.processes = processes
        
        layout = QVBoxLayout()
        
        self.table = QTableWidget()
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["Process Name", "Status"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        
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
            color = QColor("gray")
            
            if proc is None:
                status = "Not Started"
                color = QColor("#FFA500") # Orange
            elif proc.poll() is None:
                status = "RUNNING"
                color = QColor("#00FF00") # Green
            else:
                status = "STOPPED (Exit Code: {})".format(proc.returncode)
                color = QColor("#FF0000") # Red
                
            status_item = QTableWidgetItem(status)
            status_item.setForeground(QBrush(Qt.GlobalColor.black))
            status_item.setBackground(color)
            status_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            
            self.table.setItem(row, 1, status_item)

class RotationControlDialog(QDialog):
# ... (rest of the file)
