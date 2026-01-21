#!/usr/bin/env python3
"""
capture_with_mavsdk_fixed.py

Same as before but fixes the Attitude import error and uses the telemetry objects directly.
"""

import asyncio
import json
import os
import math
from datetime import datetime, timezone
from typing import Optional, Tuple

import cv2
import piexif
from PIL import Image
from mavsdk import System

# ---------- Configuration ----------
WEBCAM_INDEX = 0
SAVE_BASE_DIR = "./captures"
CAPTURE_INTERVAL_SEC = 0.5
JPEG_QUALITY = 90
# MAVSDK_CONNECTION_URL = "udpin://0.0.0.0:14540"#None  # e.g. "udp://:14540" or None to auto-discover
MAVSDK_CONNECTION_URL = None
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
        piexif.ImageIFD.Make: u"Webcam+MAVSDK".encode("utf-8"),
        piexif.ImageIFD.Model: u"USB Camera".encode("utf-8"),
        piexif.ImageIFD.Software: u"capture_with_mavsdk_fixed.py".encode("utf-8"),
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
        meta_summary["orientation_deg"] = {"yaw": yaw, "pitch": pitch, "roll": roll}
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
        async for state in self._system.core.connection_state():
            if state.is_connected:
                self._connected = True
                print("[MAVSDK] Connected to system")
                break
        self._task = asyncio.create_task(self._listen())

    async def _listen(self):
        async def position_listener():
            async for pos in self._system.telemetry.position():
                self._latest_position = pos
        async def attitude_listener():
            async for att in self._system.telemetry.attitude_euler():
                self._latest_attitude = att
        await asyncio.gather(position_listener(), attitude_listener())

    def is_connected(self) -> bool:
        return self._connected

    def latest_position(self) -> Optional[dict]:
        if self._latest_position is None:
            return None
        # use attributes returned by telemetry.Position object
        return {
            "latitude_deg": getattr(self._latest_position, "latitude_deg", None),
            "longitude_deg": getattr(self._latest_position, "longitude_deg", None),
            "absolute_altitude_m": getattr(self._latest_position, "absolute_altitude_m", None),
            "relative_altitude_m": getattr(self._latest_position, "relative_altitude_m", None)
        }

    def latest_attitude(self) -> Optional[dict]:
        if self._latest_attitude is None:
            return None
        # EulerAngle fields are already in degrees (yaw_deg, pitch_deg, roll_deg)
        return {
            "yaw_deg": getattr(self._latest_attitude, "yaw_deg", None),
            "pitch_deg": getattr(self._latest_attitude, "pitch_deg", None),
            "roll_deg": getattr(self._latest_attitude, "roll_deg", None)
        }

# ---------- Main capture loop ----------
async def main_loop():
    # Create folder hierarchy: captures/YYYY-MM-DD/HH-MM-SS/
    start_time = datetime.now()
    today_str = start_time.strftime("%Y-%m-%d")
    session_str = start_time.strftime("%H-%M-%S")

    save_dir = os.path.join(SAVE_BASE_DIR, today_str, session_str)
    ensure_dir(save_dir)

    print(f"[INFO] Saving captures to: {save_dir}")


    mav = MAVDataFetcher(connection_url=MAVSDK_CONNECTION_URL)
    try:
        await mav.connect()
    except Exception as e:
        print(f"[WARN] Could not connect to MAVSDK system: {e}")
        print("[WARN] Continuing without telemetry. Images will be saved without GPS/attitude.")

    cap = cv2.VideoCapture(WEBCAM_INDEX, cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open webcam index {WEBCAM_INDEX}")

    print("[INFO] Webcam opened. Press Ctrl+C to stop.")
    try:
        while True:
            ret, frame = cap.read()
            print(frame)
            if not ret:
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
            alt = pos["absolute_altitude_m"] if pos and "absolute_altitude_m" in pos else None

            orientation_tuple = None
            if att:
                # att values are already degrees
                try:
                    yaw = att["yaw_deg"]
                    pitch = att["pitch_deg"]
                    roll = att["roll_deg"]
                    orientation_tuple = (float(yaw) if yaw is not None else None,
                                         float(pitch) if pitch is not None else None,
                                         float(roll) if roll is not None else None)
                except Exception:
                    orientation_tuple = None

            telemetry_raw = {"position": pos, "attitude": att} if (pos or att) else None
            exif_bytes = build_exif_bytes(timestamp_utc=ts, lat=lat, lon=lon, alt=alt,
                                          orientation=orientation_tuple, telemetry_raw=telemetry_raw)

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(rgb)
            pil_img.save(filepath, "JPEG", quality=JPEG_QUALITY, exif=exif_bytes)

            gps_str = f"{lat:.6f},{lon:.6f}" if (lat is not None and lon is not None) else "N/A"
            print(f"[SAVED] {filepath}  (gps={gps_str}, orient={orientation_tuple if orientation_tuple else 'N/A'})")

            await asyncio.sleep(CAPTURE_INTERVAL_SEC)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping capture (keyboard interrupt).")
    finally:
        cap.release()
        print("[INFO] Webcam released. Exiting.")

if __name__ == "__main__":
    try:
        asyncio.run(main_loop())
    except Exception as e:
        print(f"[ERROR] {e}")
