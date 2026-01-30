









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
import json
import piexif

# ---- Add parent directory to path ----
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK


class SIYICam:
    def __init__(self, server_ip="192.168.144.25", port=37260, telemetry_provider=None):
        self.server_ip = server_ip
        self.cam = SIYISDK(server_ip=server_ip, port=port)

        self.cap = None
        self.running = False
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.reader_thread = None
        self.telemetry_provider = telemetry_provider

        # --- Recording variables ---
        self.is_recording = False
        self.record_thread = None
        self.save_queue = queue.Queue()
        self.recording_path = ""

    # ... [connect, move, zoom, setup_cam functions remain the same] ...

    def set_telemetry_provider(self, provider):
        """Set or update the telemetry provider callback"""
        self.telemetry_provider = provider
        print(f"📊 Telemetry provider {'updated' if provider else 'cleared'}")

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

    # def _record_loop(self):
    #     """Background thread that pulls frames from the queue and saves them to disk."""
    #     frame_count = 0
    #     while self.is_recording or not self.save_queue.empty():
    #         try:
    #             # Get frame from queue with a timeout so it can exit loop if recording stops
    #             frame = self.save_queue.get(timeout=1)
                
    #             # Generate filename: HH_MM_SS_frame#.jpg
    #             timestamp = datetime.now().strftime("%H_%M_%S")
    #             filename = f"{timestamp}_{frame_count:04d}.jpg"
    #             filepath = os.path.join(self.recording_path, filename)

    #             cv2.imwrite(filepath, frame)
    #             frame_count += 1
    #             self.save_queue.task_done()
    #         except queue.Empty:
    #             continue

    def _record_loop(self):
        """
        Background thread with non-blocking async I/O for frame saving.
        Uses ThreadPoolExecutor for parallel disk writes.
        """
        from concurrent.futures import ThreadPoolExecutor
        
        frame_count_per_second = {}  # Track frames per second
        executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="frame_saver")
        
        def save_frame_with_metadata(frame, filepath, telemetry, frame_num_in_second):
            try:
                cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])

                exif_dict = {"0th": {}}

                exif_dict["0th"][piexif.ImageIFD.ImageDescription] = json.dumps({
                    "latitude": telemetry.get("lat"),
                    "longitude": telemetry.get("lng"),
                    "battery": telemetry.get("battery", 0),
                    "speed": telemetry.get("speed", 0),
                    "heading": telemetry.get("heading", 0),
                    "frame_num": frame_num_in_second,
                    "telemetry_age_ms": telemetry.get("age_ms", 0)
                }).encode("utf-8")

                exif_bytes = piexif.dump(exif_dict)
                piexif.insert(exif_bytes, filepath)

                return True
            except Exception as e:
                print(f"⚠️ Frame save failed: {e}")
                return False

        
        @staticmethod
        def _deg_to_dms(self, deg):
            d = int(deg)
            m = int((deg - d) * 60)
            s = int((((deg - d) * 60) - m) * 60 * 100)
            return ((d, 1), (m, 1), (s, 100))
        
        while self.is_recording or not self.save_queue.empty():
            try:
                frame_data = self.save_queue.get(timeout=1)
                frame = frame_data['frame']
                telemetry = frame_data['telemetry']
                # Generate timestamp-based filename
                now = datetime.now()
                timestamp_key = now.strftime("%Y%m%d_%H%M%S")
                
                # Track frame count per second
                if timestamp_key not in frame_count_per_second:
                    frame_count_per_second = {timestamp_key: 0}  # Clear old keys
                
                frame_count_per_second[timestamp_key] += 1
                frame_num = frame_count_per_second[timestamp_key]
                
                # Filename: captures/YYYY-MM-DD/HH_MM_SS/capture_YYYYMMDD_HHMMSS_01.jpg
                date_folder = now.strftime("%Y-%m-%d")
                time_folder = now.strftime("%H_%M_%S")
                folder_path = os.path.join("captures", date_folder, time_folder)
                os.makedirs(folder_path, exist_ok=True)
                
                filename = f"capture_{timestamp_key}_{frame_num:02d}.jpg"
                filepath = os.path.join(folder_path, filename)
                
                # Submit to thread pool (non-blocking)
                executor.submit(save_frame_with_metadata, frame, filepath, telemetry, frame_num)
                
                self.save_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"❌ Recording loop error: {e}")
        
        executor.shutdown(wait=True)
        print("✅ All frames saved to disk")


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
            f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency=100 ! "
            "rtph265depay ! h265parse ! avdec_h265 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=true sync=false"
        )

        # gst_pipeline = (
        #     f"rtspsrc location=rtsp://192.168.144.25:8554/video1 protocols=tcp latency=100 ! "
        #     "rtph265depay ! h265parse ! avdec_h265 ! "
        #     "videoconvert ! video/x-raw,format=BGR ! "
        #     "appsink drop=true sync=false"
        # )




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
                    telemetry = self.telemetry_provider() if self.telemetry_provider else {}

                    self.save_queue.put_nowait({
                        'frame':frame.copy(),
                        'telemetry':telemetry
                    })
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