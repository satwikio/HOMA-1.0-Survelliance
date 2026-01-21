# import sys
# import os
# from time import sleep
# import cv2

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

#     def connect(self):
#         if not self.cam.connect():
#             print("❌ No connection to camera.")
#             exit(1)

#     def move(self, yaw_deg, pitch_deg):
#         self.cam.requestSetAngles(yaw_deg, pitch_deg)

#     def zoom(self, zoom_level):
#         self.cam.requestAbsoluteZoom(zoom_level)

#     def setup_cam(self):
#         """Connect to the camera, set motion mode, and adjust angles"""
#         if not self.cam.connect():
#             print("❌ No connection to camera.")
#             exit(1)

#         self.cam.requestFollowMode()
#         sleep(2)

#         print("✅ Current motion mode:", self.cam._motionMode_msg.mode)

#         target_yaw_deg = -0.0
#         target_pitch_deg = -0.0
#         self.cam.requestSetAngles(target_yaw_deg, target_pitch_deg)

#         print("🎯 Attitude (yaw, pitch, roll):", self.cam.getAttitude())
#         sleep(2)

#         # self.cam.disconnect()
#         # print("🔌 Camera disconnected from control interface.")

#     def open_stream(self):
#         """Initialize RTSP stream and return a generator"""
#         gst_pipeline = (
#             f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency=10 ! "
#             "rtph265depay ! h265parse ! nvv4l2decoder ! "
#             "nvvidconv ! video/x-raw, format=BGRx ! "
#             "videoconvert ! video/x-raw, format=BGR ! "
#             "appsink drop=true sync=false"
#         )

#         print("Opening pipeline:\n", gst_pipeline)
#         self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

#         if not self.cap.isOpened():
#             raise RuntimeError("❌ Unable to open RTSP stream")

#         print("✅ RTSP stream opened successfully.")

#     def frames(self):
#         """A generator that yields frames indefinitely"""
#         if self.cap is None:
#             self.open_stream()

#         while True:
#             ret, frame = self.cap.read()
#             if not ret:
#                 print("⚠️ Empty frame received, retrying...")
#                 continue
#             yield frame

#     def release(self):
#         if self.cap:
#             self.cap.release()


# if __name__ == "__main__":
#     cam = SIYICam()
#     cam.setup_cam()
#     #cam.open_stream()

#     # Show stream just like original
#     for frame in cam.frames():
#         cv2.imshow("Main Stream", frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cam.release()
#     cv2.destroyAllWindows()














import os
import sys
import threading
import time
import cv2
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

    def connect(self):
        if not self.cam.connect():
            print("❌ No connection to camera.")
            exit(1)

    def move(self, yaw_deg, pitch_deg):
        self.cam.requestSetAngles(yaw_deg, pitch_deg)

    def zoom(self, zoom_level):
        self.cam.requestAbsoluteZoom(zoom_level)


    # ---------------- CAMERA CONTROL ----------------
    def setup_cam(self):
        """Connect once to control interface and set angles."""
        if not self.cam.connect():
            raise RuntimeError("❌ Cannot connect to SIYI camera")

        self.cam.requestFollowMode()
        sleep(1)

        print("✅ Motion mode:", self.cam._motionMode_msg.mode)

        target_yaw_deg = 0.0
        target_pitch_deg = -90.0
        self.cam.requestSetAngles(target_yaw_deg, target_pitch_deg)

        print("🎯 Camera attitude:", self.cam.getAttitude())
        sleep(1)

        self.cam.disconnect()
        print("🔌 Control interface disconnected")

    # ---------------- STREAM ----------------
    def open_stream(self):
        if self.cap is not None:
            return

        gst_pipeline = (
            f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency=10 ! "
            "rtph265depay ! h265parse ! nvv4l2decoder ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true sync=false"
        )

        print("📡 Opening GStreamer pipeline:\n", gst_pipeline)

        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            raise RuntimeError("❌ Failed to open RTSP stream")

        print("✅ RTSP stream opened")

    # ---------------- THREAD ----------------
    def _reader_loop(self):
        """Blocking frame reader (runs in background thread)."""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            with self.frame_lock:
                self.latest_frame = frame

        print("🛑 Frame reader thread stopped")

    def start(self):
        """Start background frame capture."""
        self.open_stream()
        self.running = True
        self.reader_thread = threading.Thread(
            target=self._reader_loop,
            daemon=True
        )
        self.reader_thread.start()
        print("▶️ Frame capture started")

    def get_frame(self):
        """Non-blocking latest-frame access (async-safe)."""
        with self.frame_lock:
            return self.latest_frame

    # ---------------- CLEANUP ----------------
    def stop(self):
        self.running = False
        if self.reader_thread:
            self.reader_thread.join(timeout=2)

        if self.cap:
            self.cap.release()

        print("🧹 Camera resources released")
