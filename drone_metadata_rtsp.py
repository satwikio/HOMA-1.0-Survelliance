"""
capture_rtsp_with_mavsdk.py

Capture frames from an RTSP stream, attach MAVSDK telemetry + attitude as EXIF,
and save JPEGs to a timestamped folder (same metadata format as your USB-camera script).

Notes:
 - If siyi_sdk is available in your PYTHONPATH, the script will optionally use it to setup the camera.
 - Uses GStreamer pipeline (as in your example) to open RTSP. Adjust pipeline if required on your platform.
 - Uses asyncio with run_in_executor for non-blocking cap.read().
"""

import asyncio
import json
import os
import math
from datetime import datetime, timezone
from typing import Optional, Tuple
from time import sleep

import cv2
import piexif
from PIL import Image

from mavsdk import System

# Optional SIYI SDK (camera control). If not available, we proceed without it.
try:
    from siyi_sdk import SIYISDK
    SIYI_AVAILABLE = True
except Exception:
    SIYI_AVAILABLE = False

# ---------- Configuration ----------
# RTSP / camera control
RTSP_HOST = "192.168.144.25"
RTSP_PORT = 8554
RTSP_PATH = "main.264"  # as in your example
RTSP_USE_TCP = True
SIYI_SERVER_IP = "192.168.144.25"
SIYI_SERVER_PORT = 37260
USE_SIYI_CONTROL = SIYI_AVAILABLE and True  # set False to skip camera control even if siyi_sdk exists
SHOW_VIDEO_PREVIEW = 0  # set True to show live video window (press 'q' to quit)
PRINT_FRAMES = False  # set True to print frame array to console



# Capture / saving
SAVE_BASE_DIR = "./captures"
CAPTURE_INTERVAL_SEC = 0.03
JPEG_QUALITY = 100

# MAVSDK
MAVSDK_CONNECTION_URL = "serial:///dev/ttyACM0:57600"  # None will auto-discover. Or set e.g. "udp://:14540"

# GStreamer pipeline (NVidia elements in your example). If your system doesn't have nv* elements,
# replace with an appropriate pipeline or set 'use_gst_pipeline' to False and use a plain rtsp url.
use_gst_pipeline = True
# -----------------------------------

def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)

def decimal_to_dms_rational(dec: float):
    dec_abs = abs(dec)
    deg = int(dec_abs)
    rem = (dec_abs - deg) * 60
    minute = int(rem)
    sec = round((rem - minute) * 60, 6)
    def to_rational(value: float, denom=1000000):
        num = int(round(value * denom))
        return (num, denom)
    return [(deg, 1), (minute, 1), to_rational(sec, 1000000)]

def exif_gps_if_available(lat: Optional[float], lon: Optional[float], alt: Optional[float]):
    if lat is None or lon is None:
        return {}
    gps_ifd = {}
    gps_ifd[piexif.GPSIFD.GPSLatitudeRef] = b'N' if lat >= 0 else b'S'
    gps_ifd[piexif.GPSIFD.GPSLatitude] = decimal_to_dms_rational(lat)
    gps_ifd[piexif.GPSIFD.GPSLongitudeRef] = b'E' if lon >= 0 else b'W'
    gps_ifd[piexif.GPSIFD.GPSLongitude] = decimal_to_dms_rational(lon)
    if alt is not None:
        gps_ifd[piexif.GPSIFD.GPSAltitudeRef] = 0 if alt >= 0 else 1
        gps_ifd[piexif.GPSIFD.GPSAltitude] = (int(abs(alt) * 100), 100)
    return gps_ifd

def build_exif_bytes(
    timestamp_utc: datetime,
    lat: Optional[float],
    lon: Optional[float],
    alt: Optional[float],
    orientation: Optional[Tuple[float, float, float]],
    telemetry_raw: Optional[dict]
) -> bytes:
    zeroth_ifd = {
        piexif.ImageIFD.Make: u"RTSP+MAVSDK".encode("utf-8"),
        piexif.ImageIFD.Model: u"Network RTSP Camera".encode("utf-8"),
        piexif.ImageIFD.Software: u"capture_rtsp_with_mavsdk.py".encode("utf-8"),
        piexif.ImageIFD.DateTime: timestamp_utc.astimezone().strftime("%Y:%m:%d %H:%M:%S").encode("utf-8"),
    }
    exif_ifd = {
        piexif.ExifIFD.DateTimeOriginal: timestamp_utc.strftime("%Y:%m:%d %H:%M:%S").encode("utf-8"),
        piexif.ExifIFD.DateTimeDigitized: timestamp_utc.strftime("%Y:%m:%d %H:%M:%S").encode("utf-8"),
    }
    meta_summary = {
        "captured_utc": timestamp_utc.replace(tzinfo=timezone.utc).isoformat(),
        "gps": {"lat": lat, "lon": lon, "alt": alt},
        "orientation_deg": {"yaw": None, "pitch": None, "roll": None},
        "telemetry_raw": telemetry_raw
    }
    if orientation:
        yaw, pitch, roll = orientation
        # print(pitch)
        meta_summary["orientation_deg"] = {"ya": yaw, "pitch": pitch, "roll": roll}
    meta_json = json.dumps(meta_summary, ensure_ascii=False, indent=None)
    zeroth_ifd[piexif.ImageIFD.ImageDescription] = meta_json.encode("utf-8")
    user_comment_prefix = b'ASCII\0\0\0'
    exif_ifd[piexif.ExifIFD.UserComment] = user_comment_prefix + meta_json.encode("utf-8")
    gps_ifd = exif_gps_if_available(lat, lon, alt)
    exif_dict = {"0th": zeroth_ifd, "Exif": exif_ifd, "GPS": gps_ifd, "1st": {}}
    exif_bytes = piexif.dump(exif_dict)
    return exif_bytes

# ---------- MAVSDK helper ----------
class MAVDataFetcher:
    def __init__(self, connection_url: Optional[str] = None):
        self._system = System()
        self._connection_url = connection_url
        self._connected = False
        self._latest_position = None
        self._latest_attitude = None
        self._task = None

    async def connect(self):
        if self._connection_url:
            await self._system.connect(system_address=self._connection_url)
        else:
            await self._system.connect()
        # wait until connection established
        async for state in self._system.core.connection_state():
            if state.is_connected:
                self._connected = True
                print("[MAVSDK] Connected to system")
                break
        # start telemetry listeners in background
        self._task = asyncio.create_task(self._listen())

    async def _listen(self):
        async def position_listener():
            async for pos in self._system.telemetry.position():
                self._latest_position = pos
        async def attitude_listener():
            # attitude_euler gives yaw_deg,pitch_deg,roll_deg
            async for att in self._system.telemetry.attitude_euler():
                self._latest_attitude = att
        await asyncio.gather(position_listener(), attitude_listener())

    def is_connected(self) -> bool:
        return self._connected

    def latest_position(self) -> Optional[dict]:
        if self._latest_position is None:
            return None
        return {
            "latitude_deg": getattr(self._latest_position, "latitude_deg", None),
            "longitude_deg": getattr(self._latest_position, "longitude_deg", None),
            "absolute_altitude_m": getattr(self._latest_position, "absolute_altitude_m", None),
            "relative_altitude_m": getattr(self._latest_position, "relative_altitude_m", None)
        }

    def latest_attitude(self) -> Optional[dict]:
        if self._latest_attitude is None:
            return None
        return {
            "yaw_deg": getattr(self._latest_attitude, "yaw_deg", None),
            "pitch_deg": getattr(self._latest_attitude, "pitch_deg", None),
            "roll_deg": getattr(self._latest_attitude, "roll_deg", None)
        }

# ---------- SIYI wrapper (optional) ----------
class SIYICamController:
    def __init__(self, server_ip=SIYI_SERVER_IP, port=SIYI_SERVER_PORT):
        self.server_ip = server_ip
        self.port = port
        self.cam = None
        if SIYI_AVAILABLE:
            try:
                self.cam = SIYISDK(server_ip=self.server_ip, port=self.port)
            except Exception as e:
                print(f"[WARN] Could not instantiate SIYISDK: {e}")
                self.cam = None

    def is_available(self):
        return self.cam is not None

    def setup_cam(self, yaw=0.0, pitch=-90.0):
        if not self.cam:
            print("[WARN] SIYI control not available.")
            return False
        if not self.cam.connect():
            print("❌ No connection to SIYI camera control.")
            return False
        # Example: request follow mode and set angles like your sample
        try:
            self.cam.requestFollowMode()
            sleep(1)
            self.cam.requestSetAngles(yaw, pitch)
            sleep(1)
            print("🎯 Attitude (yaw, pitch, roll):", self.cam.getAttitude())
        except Exception as e:
            print(f"[WARN] SIYI control call failed: {e}")
        finally:
            self.cam.disconnect()
        return True

# ---------- Helper: build GStreamer pipeline or RTSP URL ----------
def make_rtsp_pipeline(host: str, port: int, path: str, use_tcp: bool = True) -> str:
    rtsp_url = f"rtsp://{host}:{port}/{path}"
    if not use_gst_pipeline:
        return rtsp_url
    # Use the same pipeline string you provided. Adjust latency if you want.
    protocols = "tcp" if use_tcp else "udp"
    # gst_pipeline = (
    #     f"rtspsrc location={rtsp_url} protocols={protocols} latency=0 ! "
    #     "rtph265depay ! h265parse ! nvv4l2decoder ! "
    #     "nvvidconv ! video/x-raw, format=BGRx ! "
    #     "videoconvert ! video/x-raw, format=BGR ! "
    #     "appsink drop=True sync=false"
    # )

    gst_pipeline = (
        f"rtspsrc location=rtsp://192.168.144.25:8554/video2 protocols=tcp latency=100 ! "
        "rtph265depay ! h265parse ! avdec_h265 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true sync=false"
    )
    return gst_pipeline

# ---------- Main capture loop ----------
async def main_loop():
    start_time = datetime.now()
    today_str = start_time.strftime("%Y-%m-%d")
    session_str = start_time.strftime("%H-%M-%S")
    save_dir = os.path.join(SAVE_BASE_DIR, today_str, session_str)
    ensure_dir(save_dir)
    print(f"[INFO] Saving captures to: {save_dir}")

    # optionally setup SIYI camera
    siyi_ctrl = SIYICamController(server_ip=SIYI_SERVER_IP, port=SIYI_SERVER_PORT)
    if USE_SIYI_CONTROL and siyi_ctrl.is_available():
        print("[INFO] Running SIYI camera setup (follow mode + set angles).")
        try:
            siyi_ctrl.setup_cam()
        except Exception as e:
            print(f"[WARN] SIYI setup failed: {e}")
    elif USE_SIYI_CONTROL:
        print("[WARN] SIYI control requested but SDK not available; skipping camera setup.")

    # start MAVSDK
    mav = MAVDataFetcher(connection_url=MAVSDK_CONNECTION_URL)
    try:
        await mav.connect()
    except Exception as e:
        print(f"[WARN] Could not connect to MAVSDK system: {e}")
        print("[WARN] Continuing without telemetry. Images saved without GPS/attitude.")

    # open RTSP stream
    pipeline = make_rtsp_pipeline(RTSP_HOST, RTSP_PORT, RTSP_PATH, use_tcp=RTSP_USE_TCP)
    print("[INFO] Opening stream:", pipeline if not use_gst_pipeline else "(GStreamer pipeline shown in code)")
    cap_flag = cv2.CAP_GSTREAMER if use_gst_pipeline else cv2.CAP_ANY
    cap = cv2.VideoCapture(pipeline, cap_flag)

    if not cap.isOpened():
        raise RuntimeError("Cannot open RTSP stream. Check pipeline/URL and GStreamer availability.")

    print("[INFO] Stream opened. Press Ctrl+C to stop (if running from terminal).")

    loop = asyncio.get_running_loop()
    try:
        while True:
            # read frame in threadpool to avoid blocking the event loop
            ret, frame = await loop.run_in_executor(None, cap.read)
            if PRINT_FRAMES:
                print("frame",frame)
            if not ret or frame is None:
                # small sleep to avoid busy-looping on stream interruption
                await asyncio.sleep(0.1)
                continue

            ts = datetime.now(timezone.utc)
            file_time_str = ts.astimezone().strftime("%H-%M-%S.%f")[:-3]
            filename = f"{file_time_str}.jpg"
            filepath = os.path.join(save_dir, filename)

            pos = mav.latest_position() if mav.is_connected() else None
            att = mav.latest_attitude() if mav.is_connected() else None

            lat = pos["latitude_deg"] if pos else None
            lon = pos["longitude_deg"] if pos else None
            alt = None
            if pos:
                alt = pos.get("absolute_altitude_m") or pos.get("relative_altitude_m")

            orientation_tuple = None
            if att:
                try:
                    yaw = att.get("yaw_deg")
                    pitch = att.get("pitch_deg")
                    roll = att.get("roll_deg")
                    orientation_tuple = (float(yaw) if yaw is not None else None,
                                         float(pitch) if pitch is not None else None,
                                         float(roll) if roll is not None else None)
                except Exception:
                    orientation_tuple = None

            telemetry_raw = {"position": pos, "attitude": att} if (pos or att) else None
            exif_bytes = build_exif_bytes(timestamp_utc=ts, lat=lat, lon=lon, alt=alt,
                                          orientation=orientation_tuple, telemetry_raw=telemetry_raw)

            
            if SHOW_VIDEO_PREVIEW:
                # --- Show live video preview ---
                cv2.imshow("RTSP Stream", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("[INFO] 'q' pressed, exiting capture loop.")
                    break


            # convert BGR (OpenCV) to RGB (PIL)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(rgb)
            try:
                pil_img.save(filepath, "JPEG", quality=JPEG_QUALITY, exif=exif_bytes)
            except Exception as e:
                print(f"[ERROR] Failed to save {filepath}: {e}")
                # try saving without EXIF as fallback
                try:
                    pil_img.save(filepath, "JPEG", quality=JPEG_QUALITY)
                except Exception as ee:
                    print(f"[ERROR] Fallback save also failed: {ee}")

            gps_str = f"{lat:.6f},{lon:.6f}" if (lat is not None and lon is not None) else "N/A"
            print(f"[SAVED] {filepath}  (gps={gps_str}, orient={orientation_tuple if orientation_tuple else 'N/A'})")

            await asyncio.sleep(CAPTURE_INTERVAL_SEC)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping capture (keyboard interrupt).")
    finally:
        cap.release()
        print("[INFO] Stream released. Exiting.")


if __name__ == "__main__":
    try:
        asyncio.run(main_loop())
    except Exception as e:
        print(f"[ERROR] {e}")
