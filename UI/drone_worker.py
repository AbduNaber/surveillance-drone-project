import time
import os
import cv2
from PyQt6.QtCore import QThread, pyqtSignal
from djitellopy import Tello

class DroneWorker(QThread):
    battery_updated = pyqtSignal(int)
    flight_time_updated = pyqtSignal(int)
    telemetry_updated = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.tello = Tello()
        self.is_running = True
        self.recording = False
        self.writer = None
        self.output_dir = "records"
        self.output_file = f"{self.output_dir}/drone_motion.avi"

    def run(self):
        # klasör yoksa oluştur
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        try:
            self.tello.connect()
            self.tello.streamon()
            print("Worker: Drone bağlı ✅")
        except Exception as e:
            print(f"Worker: Drone bağlantı hatası ❌ {e}")
            return

        while self.is_running:
            try:
                battery = self.tello.get_battery()
                flight_time = self.tello.get_flight_time()

                telemetry = {
                    "battery": battery,
                    "flight_time": flight_time,
                }

                self.battery_updated.emit(battery)
                self.flight_time_updated.emit(flight_time)
                self.telemetry_updated.emit(telemetry)

                # Frame al
                frame = self.tello.get_frame_read().frame

                # 🎥 Kaydı yaz
                if self.recording and frame is not None:
                    if self.writer is None:
                        h, w, _ = frame.shape
                        self.writer = cv2.VideoWriter(
                            self.output_file,
                            cv2.VideoWriter_fourcc(*"XVID"),
                            20,
                            (w, h)
                        )
                        print(f"[RECORD] VideoWriter başladı {w}x{h} 📁 -> {self.output_file}")

                    self.writer.write(frame)

            except Exception as e:
                print(f"Worker: Polling hatası 🚫 {e}")
                self.is_running = False

            time.sleep(1.0)

        if self.writer:
            self.writer.release()

        self.tello.streamoff()
        print("Worker: Drone kapandı 🛑")

    def start_recording(self):
        self.recording = True
        print("Worker: Kayıt BAŞLADI ▶️🎥")

    def stop_recording(self):
        self.recording = False
        if self.writer:
            self.writer.release()
            self.writer = None

        print("Worker: Kayıt DURDU ⏹️🎥")
    
    def stop(self):
        self.is_running = False
        self.wait()
