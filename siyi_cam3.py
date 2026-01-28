









# import os
# import sys
# import threading
# import time
# import cv2
# from time import sleep

# # ---- Add parent directory to path ----
# current = os.path.dirname(os.path.realpath(__file__))
# parent_directory = os.path.dirname(current)
# sys.path.append(parent_directory)

# from siyi_sdk import SIYISDK


# class SIYICam:
#     def __init__(self, server_ip="192.168.144.25", port=37260):
#         self.server_ip = server_ip
#         self.cam = SIYISDK(server_ip=server_ip, port=port)

#         self.cap = None
#         self.running = False
#         self.latest_frame = None
#         self.frame_lock = threading.Lock()
#         self.reader_thread = None

#     def connect(self):
#         if not self.cam.connect():
#             print("❌ No connection to camera.")
#             exit(1)

#     def move(self, yaw_deg, pitch_deg):
#         print("Moving camera to yaw:", yaw_deg, "pitch:", pitch_deg)
#         self.cam.requestSetAngles(yaw_deg, pitch_deg)

#     def zoom(self, zoom_level):
#         self.cam.requestAbsoluteZoom(zoom_level)


#     # ---------------- CAMERA CONTROL ----------------
#     def setup_cam(self):
#         """Connect once to control interface and set angles."""
#         if not self.cam.connect():
#             raise RuntimeError("❌ Cannot connect to SIYI camera")

#         self.cam.requestFollowMode()
#         sleep(1)

#         print("✅ Motion mode:", self.cam._motionMode_msg.mode)

#         target_yaw_deg = 0.0
#         target_pitch_deg = -90.0
#         self.cam.requestSetAngles(target_yaw_deg, target_pitch_deg)

#         print("🎯 Camera attitude:", self.cam.getAttitude())
#         sleep(1)

#         # self.cam.disconnect()
#         print("🔌 Control interface disconnected")



#     # ---------------- STREAM ----------------
#     def open_stream(self):
#         if self.cap is not None:
#             return

#         gst_pipeline = (
#             f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency=10 ! "
#             "rtph265depay ! h265parse ! nvv4l2decoder ! "
#             "nvvidconv ! video/x-raw, format=BGRx ! "
#             "videoconvert ! video/x-raw, format=BGR ! "
#             "appsink drop=true sync=false"
#         )

#         print("📡 Opening GStreamer pipeline:\n", gst_pipeline)

#         self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

#         if not self.cap.isOpened():
#             raise RuntimeError("❌ Failed to open RTSP stream")

#         print("✅ RTSP stream opened")

#     # ---------------- THREAD ----------------
#     def _reader_loop(self):
#         """Blocking frame reader (runs in background thread)."""
#         while self.running:
#             ret, frame = self.cap.read()
#             if not ret:
#                 time.sleep(0.01)
#                 continue

#             with self.frame_lock:
#                 self.latest_frame = frame

#         print("🛑 Frame reader thread stopped")

#     def start(self):
#         """Start background frame capture."""
#         self.open_stream()
#         self.running = True
#         self.reader_thread = threading.Thread(
#             target=self._reader_loop,
#             daemon=True
#         )
#         self.reader_thread.start()
#         print("▶️ Frame capture started")

#     def get_frame(self):
#         """Non-blocking latest-frame access (async-safe)."""
#         with self.frame_lock:
#             return self.latest_frame

#     # ---------------- CLEANUP ----------------
#     def stop(self):
#         self.running = False
#         if self.reader_thread:
#             self.reader_thread.join(timeout=2)

#         if self.cap:
#             self.cap.release()

#         print("🧹 Camera resources released")
















import os
import sys
import threading
import time
import cv2
import queue
from datetime import datetime
from time import sleep

# ---- Add parent directory to path ----
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK

class SIYICam:
    def __init__(self, server_ip="192.168.144.25", port=37260):
        self.server_ip = server_ip
        self.cam = SIYISDK(server_ip=server_ip, port=port)

        self.cap = None
        self.running = False
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.reader_thread = None

        # --- Recording variables ---
        self.is_recording = False
        self.record_thread = None
        self.save_queue = queue.Queue()
        self.recording_path = ""

    # ... [connect, move, zoom, setup_cam functions remain the same] ...

    def connect(self):
        if not self.cam.connect():
            print("❌ No connection to camera.")
            exit(1)

    def move(self, yaw_deg, pitch_deg):
        print("Moving camera to yaw:", yaw_deg, "pitch:", pitch_deg)
        self.cam.requestSetAngles(yaw_deg, pitch_deg)

    def zoom(self, zoom_level):
        self.cam.requestAbsoluteZoom(zoom_level)

    def setup_cam(self):
        print("🔌 Connecting to SIYI camera...")
        if not self.cam.connect():
            raise RuntimeError("❌ Cannot connect to SIYI camera")
        self.cam.requestFollowMode()
        sleep(1)
        target_yaw_deg = 0.0
        target_pitch_deg = -90.0
        self.cam.requestSetAngles(target_yaw_deg, target_pitch_deg)
        sleep(1)

    # ---------------- RECORDING LOGIC ----------------

    def start_recording(self):
        """Initializes the folder structure and starts the background saving thread."""
        if self.is_recording:
            print("⚠️ Already recording.")
            return

        # Generate paths: captures/YYYY-MM-DD/HH_MM_SS
        now = datetime.now()
        date_folder = now.strftime("%Y-%m-%d")
        time_folder = now.strftime("%H_%M_%S")
        
        self.recording_path = os.path.join("captures", date_folder, time_folder)
        os.makedirs(self.recording_path, exist_ok=True)

        self.is_recording = True
        self.record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self.record_thread.start()
        print(f"📸 Recording started: {self.recording_path}")

    def stop_recording(self):
        """Stops the recording loop."""
        self.is_recording = False
        if self.record_thread:
            self.record_thread.join()
        print("🛑 Recording stopped and saved.")

    def _record_loop(self):
        """Background thread that pulls frames from the queue and saves them to disk."""
        frame_count = 0
        while self.is_recording or not self.save_queue.empty():
            try:
                # Get frame from queue with a timeout so it can exit loop if recording stops
                frame = self.save_queue.get(timeout=1)
                
                # Generate filename: HH_MM_SS_frame#.jpg
                timestamp = datetime.now().strftime("%H_%M_%S")
                filename = f"{timestamp}_{frame_count:04d}.jpg"
                filepath = os.path.join(self.recording_path, filename)

                cv2.imwrite(filepath, frame)
                frame_count += 1
                self.save_queue.task_done()
            except queue.Empty:
                continue

    # ---------------- STREAM & CAPTURE ----------------

    def open_stream(self):
        if self.cap is not None:
            return

        # gst_pipeline = (
        #     f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency=10 ! "
        #     "rtph265depay ! h265parse ! nvv4l2decoder ! "
        #     "nvvidconv ! video/x-raw, format=BGRx ! "
        #     "videoconvert ! video/x-raw, format=BGR ! "
        #     "appsink drop=true sync=false"
        # )

        gst_pipeline = (
            f"rtspsrc location=rtsp://192.168.144.25:8554/video2 protocols=tcp latency=100 ! "
            "rtph265depay ! h265parse ! avdec_h265 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=true sync=false"
        )




        print("📡 Opening GStreamer pipeline...")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            raise RuntimeError("❌ Failed to open RTSP stream")

    def _reader_loop(self):
        """Blocking frame reader (runs in background thread)."""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            # Update latest frame for display/processing
            with self.frame_lock:
                self.latest_frame = frame.copy()

            # If recording is active, push a copy to the save queue
            if self.is_recording:
                # We use .copy() to ensure the frame isn't overwritten before being saved
                try:
                    self.save_queue.put_nowait(frame.copy())
                except queue.Full:
                    pass # Skip frame if disk I/O is too slow

    def start(self):
        self.open_stream()
        self.running = True
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        print("▶️ Frame capture started")

    def get_frame(self):
        with self.frame_lock:
            return self.latest_frame

    def stop(self):
        self.stop_recording()
        self.running = False
        if self.reader_thread:
            self.reader_thread.join(timeout=2)
        if self.cap:
            self.cap.release()
        print("🧹 Camera resources released")