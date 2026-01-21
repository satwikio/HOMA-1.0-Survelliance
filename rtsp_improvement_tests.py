#!/usr/bin/env python3
"""
siyi_cam_improved.py

Improves RTSP stream quality by:
 - attempting to set camera exposure/gain/white balance
 - using a tuned GStreamer pipeline (H.265 hardware decode on NVIDIA)
 - applying fast CPU post-processing (denoise, CLAHE, unsharp mask, gamma)
"""

import sys
import os
from time import sleep, time
import cv2
import numpy as np

# ---- Add parent directory to path ----
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
if parent_directory not in sys.path:
    sys.path.append(parent_directory)

# Import SIYISDK (user-provided)
from siyi_sdk import SIYISDK


def postprocess(frame,
                denoise_h=10,
                clahe_clip=2.0,
                clahe_tile=(8, 8),
                unsharp_amount=1.5,
                unsharp_sigma=1.0,
                gamma_val=1.0):
    """
    Enhance frame with:
      1) fastNlMeansDenoisingColored
      2) CLAHE on L channel
      3) Unsharp mask (sharpening)
      4) Gamma correction
    """
    if frame is None:
        return None

    # 1) Denoise
    try:
        denoised = cv2.fastNlMeansDenoisingColored(frame, None,
                                                   h=denoise_h, hColor=denoise_h,
                                                   templateWindowSize=7, searchWindowSize=21)
    except Exception:
        denoised = frame.copy()

    # 2) CLAHE on L channel
    try:
        lab = cv2.cvtColor(denoised, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=clahe_tile)
        l2 = clahe.apply(l)
        lab2 = cv2.merge((l2, a, b))
        enhanced = cv2.cvtColor(lab2, cv2.COLOR_LAB2BGR)
    except Exception:
        enhanced = denoised

    # 3) Sharpen (Unsharp mask)
    try:
        gaussian = cv2.GaussianBlur(enhanced, (0, 0), sigmaX=unsharp_sigma)
        sharpened = cv2.addWeighted(enhanced, 1.0 + unsharp_amount * 0.5, gaussian, -unsharp_amount * 0.5, 0)
    except Exception:
        sharpened = enhanced

    # 4) Gamma correction
    try:
        if gamma_val != 1.0 and gamma_val > 0:
            invGamma = 1.0 / gamma_val
            table = np.array([((i / 255.0) ** invGamma) * 255
                              for i in np.arange(0, 256)]).astype("uint8")
            final = cv2.LUT(sharpened, table)
        else:
            final = sharpened
    except Exception:
        final = sharpened

    return final


class SIYICam:
    def __init__(self, server_ip="192.168.144.25", port=37260,
                 width=1280, height=720, framerate=25, latency=120, use_h265=True):
        self.server_ip = server_ip
        self.port = port
        self.cam = SIYISDK(server_ip=server_ip, port=port)
        self.cap = None
        self.width = width
        self.height = height
        self.framerate = framerate
        self.latency = latency
        self.use_h265 = use_h265

    def _attempt_camera_tuning(self):
        """
        Try several common method names on SIYISDK to set camera parameters.
        This does not assume exact method names — it probes likely names and calls them if present.
        Values used are conservative defaults and can be changed.
        """
        print("🔧 Attempting to tune camera parameters (if supported)...")
        methods_to_try = [
            # method_name, args tuple, kwargs dict
            ("setExposure", (20,), {}),            # milliseconds
            ("setExposureMs", (20,), {}),
            ("set_exposure", (20,), {}),
            ("requestSetExposure", (20,), {}),
            ("setGain", (4,), {}),
            ("set_gain", (4,), {}),
            ("requestSetGain", (4,), {}),
            ("setWhiteBalance", (4500,), {}),     # kelvin
            ("set_white_balance", (4500,), {}),
            ("requestSetWhiteBalance", (4500,), {}),
            ("setBrightness", (50,), {}),
            ("set_contrast", (50,), {}),
        ]

        available = dir(self.cam)
        for name, args, kwargs in methods_to_try:
            if name in available:
                try:
                    fn = getattr(self.cam, name)
                    print(f" ➜ calling {name}({', '.join(map(str, args))})")
                    fn(*args, **kwargs)
                    sleep(0.15)
                except Exception as e:
                    print(f"   (failed to call {name}: {e})")
            # don't error if not present

    def setup_cam(self):
        """Connect to the camera, set motion mode, adjust angles, and attempt to tune exposure/gain"""
        print("🔌 Connecting to camera control interface...")
        if not self.cam.connect():
            print("❌ No connection to camera control interface.")
            raise RuntimeError("Camera control connect failed")

        try:
            # request Follow Mode (as before)
            if hasattr(self.cam, "requestFollowMode"):
                self.cam.requestFollowMode()
                sleep(1)
                # print motion mode if available
                try:
                    mm = getattr(self.cam, "_motionMode_msg", None)
                    if mm is not None:
                        print("✅ Current motion mode:", getattr(mm, "mode", mm))
                except Exception:
                    pass

            # set a neutral angle (your original)
            try:
                target_yaw_deg = -0.0
                target_pitch_deg = -0.0
                if hasattr(self.cam, "requestSetAngles"):
                    self.cam.requestSetAngles(target_yaw_deg, target_pitch_deg)
                    sleep(0.1)
            except Exception:
                pass

            # Attempt parameter tuning (exposure/gain/white balance) gracefully
            self._attempt_camera_tuning()

            # print attitude if available
            if hasattr(self.cam, "getAttitude"):
                try:
                    print("🎯 Attitude (yaw, pitch, roll):", self.cam.getAttitude())
                except Exception:
                    pass

        finally:
            # Disconnect control interface (we only needed to set parameters)
            try:
                self.cam.disconnect()
                print("🔌 Camera disconnected from control interface.")
            except Exception:
                pass

    def open_stream(self):
        """Initialize RTSP stream and open cv2.VideoCapture via GStreamer pipeline"""
        # Use h265 or h264 pipeline depending on camera
        if self.use_h265:
            depay = "rtph265depay ! h265parse ! nvv4l2decoder"
        else:
            depay = "rtph264depay ! h264parse ! nvv4l2decoder"

        # NVVIDCONV caps ensure correct conversion and sizes
        gst_pipeline = (
            f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency={self.latency} ! "
            f"{depay} ! "
            f"nvvidconv ! video/x-raw, width={self.width}, height={self.height}, format=BGRx, framerate={self.framerate}/1 ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink max-buffers=2 drop=true sync=false"
        )

        print("📡 Opening pipeline:\n", gst_pipeline)
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        # fallback: try without nvv4l2decoder (software decode) if hardware decode fails
        if not self.cap.isOpened():
            print("⚠️ Primary pipeline failed to open. Trying software decode fallback...")
            fallback_pipeline = (
                f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency={self.latency} ! "
                "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw, format=BGR ! "
                "appsink max-buffers=2 drop=true sync=false"
            )
            print("📡 Fallback pipeline:\n", fallback_pipeline)
            self.cap = cv2.VideoCapture(fallback_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            raise RuntimeError("❌ Unable to open RTSP stream with either pipeline")

        print("✅ RTSP stream opened successfully.")

    def frames(self, retry_delay=0.1):
        """Generator yielding frames indefinitely; handles transient empty frames gracefully."""
        if self.cap is None:
            self.open_stream()

        while True:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                # Temporary failure — try to re-open capture after a short pause if the stream seems dead
                print("⚠️ Empty frame received, retrying...")
                sleep(retry_delay)
                # we continue and let VideoCapture handle reconnection
                continue
            yield frame


    # def open_stream(self, force_cpu_decode=False):
    #     """Initialize RTSP stream and open cv2.VideoCapture via GStreamer pipeline.

    #     - Tries hardware decode first (nvv4l2decoder) with conservative width/height.
    #     - If that fails or force_cpu_decode=True, uses avdec_h265 CPU decode.
    #     """
    #     # try to reduce buffer pressure on hardware decoder by using smaller default
    #     hw_width = max(640, min(self.width, 1280))   # keep at least 640, cap at 1280
    #     hw_height = int(hw_width * 9 / 16)

    #     # choose depay/parse elements for H.265 (your stream appears to be H.265)
    #     depay = "rtph265depay ! h265parse"

    #     # Hardware pipeline (Jetson) - conservative sizes to avoid NvMapMemAlloc errors
    #     hw_pipeline = (
    #         f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency={self.latency} ! "
    #         f"{depay} ! nvv4l2decoder ! nvvidconv ! video/x-raw, width={hw_width}, height={hw_height}, format=BGRx, framerate={self.framerate}/1 ! "
    #         "videoconvert ! video/x-raw, format=BGR ! "
    #         "appsink max-buffers=2 drop=true sync=false"
    #     )

    #     # CPU (software) decode pipeline for H.265 (fallback)
    #     sw_pipeline = (
    #         f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency={self.latency} ! "
    #         f"{depay} ! avdec_h265 ! videoconvert ! video/x-raw, format=BGR ! "
    #         "appsink max-buffers=2 drop=true sync=false"
    #     )

    #     # If the user specifically requests CPU decode, skip HW attempt
    #     if force_cpu_decode:
    #         print("⚠️ force_cpu_decode=True: opening software (CPU) decode pipeline.")
    #         pipeline = sw_pipeline
    #     else:
    #         print("📡 Trying hardware decode pipeline (may fail if GPU buffers unavailable)...")
    #         pipeline = hw_pipeline

    #     print("Opening pipeline:\n", pipeline)
    #     self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    #     # If hardware attempt failed and we didn't force CPU, try software fallback
    #     if (not self.cap.isOpened()) and (not force_cpu_decode):
    #         print("⚠️ Hardware pipeline failed; trying software (CPU) H.265 decode fallback...")
    #         print("Fallback pipeline:\n", sw_pipeline)
    #         self.cap = cv2.VideoCapture(sw_pipeline, cv2.CAP_GSTREAMER)

    #     if not self.cap.isOpened():
    #         # Give actionable advice in the error message
    #         raise RuntimeError(
    #             "❌ Unable to open RTSP stream with either pipeline.\n"
    #             "- Confirm the stream codec (H.265) and that gst plugins (avdec_h265, rtph265depay) are installed.\n"
    #             "- On NVIDIA Jetson: nvv4l2decoder may fail when GPU memory is low or resolution is too large; try reducing width/height or closing other GPU processes.\n"
    #             "- Try testing the pipelines directly with gst-launch-1.0 to see plugin errors."
    #         )

    #     print("✅ RTSP stream opened successfully.")


    def release(self):
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None


def main():
    # --- Configuration (tweak these) ---
    SERVER_IP = "192.168.144.25"
    PORT = 37260
    WIDTH = 1280
    HEIGHT = 720
    FRAMERATE = 25
    LATENCY = 120            # increase for fewer drops, decrease for lower delay
    USE_H265 = True

    cam = SIYICam(server_ip=SERVER_IP, port=PORT,
                  width=WIDTH, height=HEIGHT, framerate=FRAMERATE,
                  latency=LATENCY, use_h265=USE_H265)

    # set camera settings via control interface (best improvements usually come from here)
    try:
        cam.setup_cam()
    except Exception as e:
        print("⚠️ Warning: setup_cam failed or partial. Continuing to open stream. Error:", e)

    # open the RTSP stream
    try:
        cam.open_stream()
    except Exception as e:
        print("❌ Failed to open stream:", e)
        return

    # frame loop
    window_orig = "Main Stream - Orig"
    window_proc = "Main Stream - Processed"
    cv2.namedWindow(window_orig, cv2.WINDOW_NORMAL)
    cv2.namedWindow(window_proc, cv2.WINDOW_NORMAL)

    last_save_ts = 0
    save_count = 0

    try:
        for frame in cam.frames():
            # small defensive check
            if frame is None:
                continue

            proc = postprocess(frame,
                               denoise_h=8,
                               clahe_clip=2.0,
                               clahe_tile=(8, 8),
                               unsharp_amount=1.2,
                               unsharp_sigma=1.0,
                               gamma_val=1.0)

            # Show both for comparison
            cv2.imshow(window_orig, frame)
            cv2.imshow(window_proc, proc)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # save processed frame (debounce to avoid many quick saves)
                now = time()
                if now - last_save_ts > 0.5:
                    filename = f"frame_saved_{save_count:03d}.jpg"
                    cv2.imwrite(filename, proc)
                    print("💾 Saved", filename)
                    save_count += 1
                    last_save_ts = now

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        cam.release()
        cv2.destroyAllWindows()
        print("Exited cleanly.")


if __name__ == "__main__":
    main()
