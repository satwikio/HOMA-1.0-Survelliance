# import sys
# import os
# from time import sleep
# import cv2

# # ---- Add parent directory to path ----
# current = os.path.dirname(os.path.realpath(__file__))
# parent_directory = os.path.dirname(current)
# sys.path.append(parent_directory)

# from siyi_sdk import SIYISDK
# # from stream import SIYIRTSP


# class SIYICam:
#     def __init__(self, server_ip="192.168.144.25", port=37260):
#         self.server_ip = server_ip
#         self.cam = SIYISDK(server_ip=server_ip, port=port)

#     def setup_cam(self):
#         """Connect to the camera, set motion mode, and adjust angles"""
#         if not self.cam.connect():
#             print("❌ No connection to camera.")
#             exit(1)

#         # Request follow mode
#         self.cam.requestFollowMode()
#         sleep(2)

#         print("✅ Current motion mode:", self.cam._motionMode_msg.mode)

#         # Set gimbal angles
#         target_yaw_deg = 0.0
#         target_pitch_deg = -90.0
#         self.cam.requestSetAngles(target_yaw_deg, target_pitch_deg)

#         print("🎯 Attitude (yaw, pitch, roll):", self.cam.getAttitude())
#         sleep(2)

#         # Disconnect SDK (you can keep connection open if you need control during stream)
#         self.cam.disconnect()
#         print("🔌 Camera disconnected from control interface.")

#     def stream_video(self):
#         """Start RTSP stream using GStreamer and display via OpenCV"""
#         gst_pipeline = (
#             f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency=10 ! "
#             "rtph265depay ! h265parse ! nvv4l2decoder ! "
#             "nvvidconv ! video/x-raw, format=BGRx ! "
#             "videoconvert ! video/x-raw, format=BGR ! "
#             "appsink drop=true sync=false"
#         )

#         print("Opening pipeline:\n", gst_pipeline)

#         cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

#         if not cap.isOpened():
#             print("❌ Unable to open RTSP stream via GStreamer")
#             exit()

#         print("✅ Stream opened successfully. Press 'q' to quit.")

#         while True:
#             ret, frame = cap.read()




import sys
import os
from time import sleep
sys.path.insert(0, "/home/nanosuper/opencv_build/build/lib/python3/")

import cv2

# ---- Add parent directory to path ----
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK
# from stream import SIYIRTSP


class SIYICam:
    def __init__(self, server_ip="192.168.144.25", port=37260):
        self.server_ip = server_ip
        self.cam = SIYISDK(server_ip=server_ip, port=port)

    def setup_cam(self):
        """Connect to the camera, set motion mode, and adjust angles"""
        if not self.cam.connect():
            print("❌ No connection to camera.")
            exit(1)

        # Request follow mode
        self.cam.requestFollowMode()
        sleep(0.5)

        print("✅ Current motion mode:", self.cam._motionMode_msg.mode)

        # Set gimbal angles
        target_yaw_deg = -60.0
        target_pitch_deg = -0.0
        self.cam.requestSetAngles(target_yaw_deg, target_pitch_deg)

        print("🎯 Attitude (yaw, pitch, roll):", self.cam.getAttitude())
        sleep(0.5)

        # Disconnect SDK (you can keep connection open if you need control during stream)
        # self.cam.disconnect()
        # print("🔌 Camera disconnected from control interface.")

    def stream_video(self):
        """Start RTSP stream using GStreamer and display via OpenCV"""
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
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true sync=false"
        )



        print("Opening pipeline:\n", gst_pipeline)

        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            print("❌ Unable to open RTSP stream via GStreamer")
            exit()

        print("✅ Stream opened successfully. Press 'q' to quit.")

        while True:
            ret, frame = cap.read()
            if not ret:
                print("⚠️ Empty frame received, retrying...")
                continue

            cv2.imshow("RTSP Stream", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if not ret:
                print("⚠️ Empty frame received, retrying...")
                continue

            cv2.imshow("RTSP Stream", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        print("👋 Stream closed.")


if __name__ == "__main__":
    cam = SIYICam()
    cam.setup_cam()
    cam.stream_video()



















