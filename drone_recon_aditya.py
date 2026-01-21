print("Code Started")

import asyncio
import json
import math
import time
import cv2
import websockets
import aiohttp
import numpy as np

from aiortc import RTCPeerConnection, VideoStreamTrack, RTCSessionDescription
from aiortc.sdp import candidate_from_sdp
from av import VideoFrame
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from test2 import compute_spacing_from_camera
from test2 import PreciseSurveyPlanner
from mavsdk.telemetry import FixType


# === Configuration ===
BACKEND_URL = "https://apidense.simhatel.link"   #http://100.88.6.111:8000"
WS_URL = "ws://apidense.simhatel.link"
DRONE_ID = "austin-recon-01"
SECRET_KEY = "drone_aus_123"


# Set to True to use dummy telemetry data (for testing without real drone)
USE_DUMMY_TELEMETRY = False

# Dummy telemetry starting position (Zurich, Switzerland - default MAVSDK SITL location)
DUMMY_START_LAT = 47.3977506
DUMMY_START_LNG = 8.5456067


def quaternion_to_euler(w, x, y, z):
    """Convert quaternion (w,x,y,z) to Euler roll, pitch, yaw in degrees."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.degrees(math.atan2(t3, t4))

    return roll_x, pitch_y, yaw_z


# class WebcamVideoTrack(VideoStreamTrack):
#     def __init__(self):
#         super().__init__()
#         self.cap = cv2.VideoCapture(0)
#         self.use_dummy = not self.cap.isOpened()
#         if self.use_dummy:
#             print(" Webcam not available — using dummy video")
#         else:
#             # Try to set resolution
#             self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#             self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

#     async def recv(self):
#         print("Checking camera connection")
#         pts, time_base = await self.next_timestamp()

#         if self.use_dummy:
#             # Create a dummy black frame with a message
#             frame = np.zeros((480, 640, 3), dtype=np.uint8)
#             cv2.putText(frame, "Camera not available", (120, 240),
#                         cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
#             frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         else:
#             ret, frame = self.cap.read()
#             if not ret:
#                 # fallback to dummy if frame read fails
#                 print(" Frame read failed, switching to dummy")
#                 self.use_dummy = False
#                 return await self.recv()
#             frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#         video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
#         video_frame.pts = pts
#         video_frame.time_base = time_base
#         return video_frame

#     def __del__(self):
#         try:
#             if hasattr(self, "cap") and self.cap.isOpened():
#                 self.cap.release()
#         except Exception:
#             pass


from siyi_cam import SIYICam  # Import your class
from time import sleep

class WebcamVideoTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()

        # Initialize the SIYI camera
        self.siyi = SIYICam()
        self.siyi.setup_cam()
        self.server_ip = self.siyi.server_ip

        # Use the same GStreamer pipeline as siyi_cam.py
        self.gst_pipeline = (
            f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency=10 ! "
            "rtph265depay ! h265parse ! nvv4l2decoder ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true sync=false"
        )

        print("Opening SIYI RTSP pipeline:\n", self.gst_pipeline)
        self.cap = cv2.VideoCapture(self.gst_pipeline, cv2.CAP_GSTREAMER)

        sleep(2)  # Give some time to establish the connection

        self.use_dummy = not self.cap.isOpened()
        if self.use_dummy:
            print("❌ Unable to open SIYI RTSP stream — using dummy video")
        else:
            print("✅ SIYI RTSP stream opened successfully")

    async def recv(self):
        """Return a frame from SIYI RTSP stream for WebRTC"""
        pts, time_base = await self.next_timestamp()

        if self.use_dummy:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "SIYI stream unavailable", (100, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            ret, frame = self.cap.read()
            if not ret:
                print("⚠️ Empty frame from SIYI camera, switching to dummy mode")
                self.use_dummy = True
                return await self.recv()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def __del__(self):
        """Clean up camera resources"""
        try:
            if hasattr(self, "cap") and self.cap.isOpened():
                self.cap.release()
        except Exception:
            pass




class DroneClient:
    def __init__(self):
        self.websocket = None
        self.pc = None
        self.video_track = None
        self.token = None
        self.drone = System()
        self.latest = {}
        self.telemetry_tasks = []

        # Dummy telemetry state
        self.dummy_lat = DUMMY_START_LAT
        self.dummy_lng = DUMMY_START_LNG
        self.dummy_alt = 0.0
        self.dummy_battery = 100.0
        self.dummy_speed = 0.0
        self.dummy_heading = 0.0
        self.dummy_time = 0

    async def authenticate(self):
        async with aiohttp.ClientSession() as session:
            async with session.post(f"{BACKEND_URL}/api/v1/auth/drone", json={
                "drone_id": DRONE_ID,
                "secret_key": SECRET_KEY
            }) as resp:
                data = await resp.json()
                self.token = data.get("access_token")
                print(" Authenticated, token:", self.token)

    async def connect_mavsdk(self):
        if USE_DUMMY_TELEMETRY:
            print("⏭️  Skipping MAVSDK connection (using dummy data)")
            return

        print("Connecting to MAVSDK...")
        # await self.drone.connect(system_address="udpin://0.0.0.0:14540")   #For real drone "serial:///dev/ttyACM0:57600" 
        await self.drone.connect(system_address="serial:///dev/ttyACM0:57600")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(" MAVSDK connected")
                break

    async def start_webrtc(self):
        self.video_track = WebcamVideoTrack()
        self.pc = RTCPeerConnection()
        self.pc.addTrack(self.video_track)

        @self.pc.on("icecandidate")
        async def on_ice(candidate):
            if candidate:
                msg = {
                    "type": "ice_candidate",
                    "drone_id": DRONE_ID,
                    "candidate": {
                        "candidate": candidate.candidate,
                        "sdpMid": candidate.sdpMid,
                        "sdpMLineIndex": candidate.sdpMLineIndex
                    }
                }
                await self.websocket.send(json.dumps(msg))

    def _generate_dummy_telemetry(self):
        """Generate realistic dummy telemetry data for testing"""
        self.dummy_time += 1

        # Simulate drone movement in a circular pattern
        radius = 0.0001  # ~11 meters
        angle = (self.dummy_time * 0.1) % (2 * math.pi)
        self.dummy_lat = DUMMY_START_LAT + radius * math.cos(angle)
        self.dummy_lng = DUMMY_START_LNG + radius * math.sin(angle)

        # Simulate altitude change
        self.dummy_alt = 50 + 10 * math.sin(self.dummy_time * 0.05)

        # Simulate speed
        self.dummy_speed = 5 + 2 * math.sin(self.dummy_time * 0.1)

        # Simulate heading
        self.dummy_heading = (angle * 180 / math.pi) % 360

        # Simulate battery drain
        self.dummy_battery = max(20, 100 - self.dummy_time * 0.1)

        # Build telemetry
        return {
            "lat": self.dummy_lat,
            "lng": self.dummy_lng,
            "alt": self.dummy_alt,
            "battery": self.dummy_battery,
            "speed": self.dummy_speed,
            "heading": self.dummy_heading,
            "gps": {
                "lat": self.dummy_lat,
                "lon": self.dummy_lng,
                "alt_abs_m": self.dummy_alt + 400,  # Zurich elevation ~400m
                "alt_rel_m": self.dummy_alt,
            },
            "orientation": {
                "roll": 2 * math.sin(self.dummy_time * 0.2),
                "pitch": 3 * math.cos(self.dummy_time * 0.15),
                "yaw": self.dummy_heading,
            },
            "speed_detail": {
                "hor_m_s": self.dummy_speed,
                "ver_m_s": 0.5 * math.sin(self.dummy_time * 0.1),
            },
            "battery_detail": {
                "voltage_v": 12.6 - (100 - self.dummy_battery) * 0.01,
                "remaining_percent": self.dummy_battery,
            },
            "status": {
                "flight_mode": "POSCTL" if self.dummy_alt > 5 else "MANUAL",
                "is_armed": self.dummy_alt > 1,
            },
            "gps_info": {
                "num_satellites": 10,
                "fix_type": 3  # 3D Fix
            },

        }

    async def send_telemetry(self):
        if USE_DUMMY_TELEMETRY:
            print("🤖 Using DUMMY telemetry data (no real drone connected)")
            print(f"📍 Starting position: {DUMMY_START_LAT}, {DUMMY_START_LNG}")
            print("💡 Set USE_DUMMY_TELEMETRY = False to use real MAVSDK data")
        else:
            # Launch telemetry reader tasks for real drone
            self.telemetry_tasks = [
                asyncio.create_task(self._position_reader()),
                asyncio.create_task(self._attitude_reader()),
                asyncio.create_task(self._velocity_reader()),
                asyncio.create_task(self._battery_reader()),
                asyncio.create_task(self._status_reader()),
                asyncio.create_task(self._imu_reader()),
                asyncio.create_task(self._gps_info_reader()),   
            ]

        while True:
            try:
                if USE_DUMMY_TELEMETRY:
                    # Generate dummy telemetry
                    dummy_data = self._generate_dummy_telemetry()

                    telemetry = {
                        "type": "telemetry",
                        "drone_id": DRONE_ID,
                        "timestamp": int(time.time() * 1000),
                        "lat": dummy_data["lat"],
                        "lng": dummy_data["lng"],
                        "alt": dummy_data["alt"],
                        "battery": dummy_data["battery"],
                        "speed": dummy_data["speed"],
                        "heading": dummy_data["heading"],
                        "gps": dummy_data["gps"],
                        "orientation": dummy_data["orientation"],
                        "speed_detail": dummy_data["speed_detail"],
                        "battery_detail": dummy_data["battery_detail"],
                        "status": dummy_data["status"],
                    }
                else:
                    # Build flat telemetry structure from real MAVSDK data
                    telemetry = {
                        "type": "telemetry",
                        "drone_id": DRONE_ID,
                        "timestamp": int(time.time() * 1000),  # milliseconds
                        # Basic fields (fallback to 0 if not available)
                        "lat": self.latest.get("gps", {}).get("lat", 0),
                        "lng": self.latest.get("gps", {}).get("lon", 0),
                        "alt": self.latest.get("gps", {}).get("alt_rel_m", 0),
                        "battery": self.latest.get("battery", {}).get("remaining_percent", 0),
                        "speed": self.latest.get("speed", {}).get("hor_m_s", 0),
                        "heading": self.latest.get("orientation", {}).get("yaw", 0),
                    }

                    # Add enhanced MAVSDK telemetry if available
                    if "gps" in self.latest:
                        telemetry["gps"] = self.latest["gps"]
                    if "orientation" in self.latest:
                        telemetry["orientation"] = self.latest["orientation"]
                    if "speed" in self.latest:
                        telemetry["speed_detail"] = self.latest["speed"]
                    if "battery" in self.latest:
                        telemetry["battery_detail"] = self.latest["battery"]
                    if "status" in self.latest:
                        telemetry["status"] = self.latest["status"]
                    if "imu" in self.latest:
                        telemetry["imu"] = self.latest["imu"]
                    if "gps_info" in self.latest:
                        telemetry["gps_info"] = self.latest["gps_info"]
                        telemetry["num_satellites"] = self.latest["gps_info"].get("num_satellites", 0)
                        telemetry["fix_type"] = self.latest["gps_info"].get("fix_type", 0)


                await self.websocket.send(json.dumps(telemetry))
            except Exception as e:
                print(" Telemetry send error:", e)
            await asyncio.sleep(1.0)  # adjust rate as needed

    # ------ Telemetry readers ------

    async def _position_reader(self):
        async for pos in self.drone.telemetry.position():
            self.latest.setdefault("gps", {})
            self.latest["gps"].update({
                "lat": pos.latitude_deg,
                "lon": pos.longitude_deg,
                "alt_abs_m": pos.absolute_altitude_m,
                "alt_rel_m": pos.relative_altitude_m,
            })

    async def _attitude_reader(self):
        try:
            async for e in self.drone.telemetry.attitude_euler():
                self.latest["orientation"] = {
                    "roll": e.roll_deg,
                    "pitch": e.pitch_deg,
                    "yaw": e.yaw_deg
                }
        except Exception:
            async for q in self.drone.telemetry.attitude_quaternion():
                r, p, y = quaternion_to_euler(q.w, q.x, q.y, q.z)
                self.latest["orientation"] = {"roll": r, "pitch": p, "yaw": y}

    from mavsdk.telemetry import FixType

    async def _gps_info_reader(self):
        """
        Store num_satellites and map MAVSDK FixType to integer:
        0 = NoGps, 1 = NoFix, 2 = Fix2D, 3 = Fix3D/DGPS/RTK
        """
        async for gps_info in self.drone.telemetry.gps_info():
            # default
            fix_type_value = 0

            try:
                # Enum comparisons (preferred)
                if gps_info.fix_type == FixType.NoGps:
                    fix_type_value = 0
                elif gps_info.fix_type == FixType.NoFix:
                    fix_type_value = 1
                elif gps_info.fix_type == FixType.Fix2D:
                    fix_type_value = 2
                elif gps_info.fix_type in (
                    FixType.Fix3D,
                    FixType.FixDgps,
                    FixType.RtkFloat,
                    FixType.RtkFixed,
                ):
                    fix_type_value = 3
                else:
                    fix_type_value = 0

            except Exception:
                # Fallback: string-based detection (covers wrapper/version differences)
                try:
                    fix_name = str(gps_info.fix_type).upper()
                    if "NOGPS" in fix_name or "NO_GPS" in fix_name:
                        fix_type_value = 0
                    elif "NOFIX" in fix_name or "NO_FIX" in fix_name:
                        fix_type_value = 1
                    elif "2D" in fix_name:
                        fix_type_value = 2
                    elif any(x in fix_name for x in ("3D", "DGPS", "RTK")):
                        fix_type_value = 3
                    else:
                        fix_type_value = 0
                except Exception:
                    fix_type_value = 0

            # update cache
            self.latest.setdefault("gps_info", {})
            self.latest["gps_info"].update({
                "num_satellites": int(getattr(gps_info, "num_satellites", 0)),
                "fix_type": fix_type_value,
            })



    async def _velocity_reader(self):
        try:
            async for v in self.drone.telemetry.velocity_ned():
                hor = math.hypot(v.velocity_north_m_s, v.velocity_east_m_s)
                ver = v.velocity_down_m_s
                self.latest["speed"] = {"hor_m_s": hor, "ver_m_s": ver}
        except Exception:
            pass

    async def _battery_reader(self):
        async for b in self.drone.telemetry.battery():
            self.latest["battery"] = {
                "voltage_v": b.voltage_v,
                "remaining_percent": b.remaining_percent
            }

    async def _status_reader(self):
        # Run both flight_mode and armed readers concurrently
        async def read_flight_mode():
            async for fm in self.drone.telemetry.flight_mode():
                self.latest.setdefault("status", {})
                self.latest["status"]["flight_mode"] = str(fm)

        async def read_armed():
            async for armed in self.drone.telemetry.armed():
                self.latest.setdefault("status", {})
                self.latest["status"]["is_armed"] = bool(armed)

        # Run both concurrently
        await asyncio.gather(read_flight_mode(), read_armed())

    async def _imu_reader(self):
        async for imu in self.drone.telemetry.imu():
            try:
                accel = [imu.acceleration_frd.x_m_s2,
                         imu.acceleration_frd.y_m_s2,
                         imu.acceleration_frd.z_m_s2]
            except Exception:
                accel = getattr(imu, "acceleration", None) or []
            try:
                gyro = [imu.angular_velocity_frd.x_rad_s,
                        imu.angular_velocity_frd.y_rad_s,
                        imu.angular_velocity_frd.z_rad_s]
            except Exception:
                gyro = getattr(imu, "angular_velocity", None) or []
            try:
                mag = [imu.magnetic_field_frd.x_ga,
                       imu.magnetic_field_frd.y_ga,
                       imu.magnetic_field_frd.z_ga]
            except Exception:
                mag = getattr(imu, "magnetic_field", None) or []
            self.latest["imu"] = {"accel": accel, "gyro": gyro, "mag": mag}

    # ------ Command handling ------

    async def handle_command(self, data):
        # Expect messages like:
        # { "type": "command", "cmd": "arm", "drone_id": "...", "height": ... }
        cmd = data.get("cmd")
        height = data.get("height")  # For takeoff command

        print(f"\n{'='*60}")
        print(f"⚡ COMMAND RECEIVED: {cmd}")
        print(f"Full command data: {json.dumps(data, indent=2)}")
        print(f"{'='*60}\n")

        try:
            if cmd == "arm":
                # Check if drone is healthy before arming
                print("🔍 Checking drone health before arming...")
                health = await self.drone.telemetry.health().__anext__()
                print(f"  GPS: {'✅' if health.is_gyrometer_calibration_ok else '❌'}")
                print(f"  Accelerometer: {'✅' if health.is_accelerometer_calibration_ok else '❌'}")
                print(f"  Magnetometer: {'✅' if health.is_magnetometer_calibration_ok else '❌'}")
                print(f"  Local position: {'✅' if health.is_local_position_ok else '❌'}")
                print(f"  Global position: {'✅' if health.is_global_position_ok else '❌'}")
                print(f"  Home position: {'✅' if health.is_home_position_ok else '❌'}")

                # Try to arm
                await self.drone.action.arm()
                print("✅ Arm executed successfully")

            if cmd == "arm":
                # Wait until important health checks are OK (with timeout)
                print("🔍 Checking drone health before arming...")
                # choose required flags here
                required_flags = [
                    "is_gyrometer_calibration_ok",
                    "is_accelerometer_calibration_ok",
                    "is_magnetometer_calibration_ok",
                    "is_local_position_ok",
                    "is_global_position_ok",
                    "is_home_position_ok",
                ]

                timeout_s = 20.0   # max wait time (seconds)
                poll_interval = 1.0
                start = time.time()
                healthy = False

                try:
                    # health() is an async iterator — iterate until all required flags are true or timeout
                    async for health in self.drone.telemetry.health():
                        # print a friendly status line
                        print(
                            f"  Gyro:{'✅' if health.is_gyrometer_calibration_ok else '❌'} "
                            f"Accel:{'✅' if health.is_accelerometer_calibration_ok else '❌'} "
                            f"Mag:{'✅' if health.is_magnetometer_calibration_ok else '❌'} "
                            f"LocalPos:{'✅' if health.is_local_position_ok else '❌'} "
                            f"GlobalPos:{'✅' if health.is_global_position_ok else '❌'} "
                            f"HomePos:{'✅' if health.is_home_position_ok else '❌'}"
                        )

                        all_ok = (
                            health.is_gyrometer_calibration_ok and
                            health.is_accelerometer_calibration_ok and
                            health.is_magnetometer_calibration_ok and
                            health.is_local_position_ok and
                            health.is_global_position_ok and
                            health.is_home_position_ok
                        )

                        if all_ok:
                            healthy = True
                            print("✅ All required health checks are OK")
                            break

                        # timeout handling
                        if (time.time() - start) >= timeout_s:
                            print(f"⛔ Health checks did not become OK within {timeout_s} seconds")
                            healthy = False
                            break

                        # wait a bit before reading the next health message
                        await asyncio.sleep(poll_interval)

                except Exception as e:
                    print(" Error while checking health:", e)
                    healthy = False

                if not healthy:
                    # refuse to arm — notify backend and do not call arm()
                    print("❌ Refusing to arm: health checks failed")
                    resp = {"type": "response", "drone_id": DRONE_ID, "cmd": "arm", "status": "error", "message": "health checks failed"}
                    try:
                        await self.websocket.send(json.dumps(resp))
                    except Exception:
                        pass
                else:
                    # safe to arm
                    try:
                        await self.drone.action.arm()
                        print("✅ Arm executed successfully")
                        resp = {"type": "response", "drone_id": DRONE_ID, "cmd": "arm", "status": "ok"}
                        await self.websocket.send(json.dumps(resp))
                    except Exception as e:
                        print(" Command execution failed: arm", e)
                        resp = {"type": "response", "drone_id": DRONE_ID, "cmd": "arm", "status": "error", "message": str(e)}
                        try:
                            await self.websocket.send(json.dumps(resp))
                        except Exception:
                            pass

            elif cmd == "disarm":
                await self.drone.action.disarm()
                print(" Disarm executed")

            elif cmd == "takeoff":
                alt = float(height) if height else 10.0
                await self.drone.action.set_takeoff_altitude(alt)
                await self.drone.action.takeoff()
                print(" Takeoff executed, altitude:", alt)

            elif cmd == "land":
                await self.drone.action.land()
                print(" Land executed")

            elif cmd in ("rtl", "return_to_launch"):
                await self.drone.action.return_to_launch()
                print(" RTL executed")

            

            else:
                print(" Unknown command:", cmd)

            # Send acknowledgment
            resp = {"type": "response", "drone_id": DRONE_ID, "cmd": cmd, "status": "ok"}
            await self.websocket.send(json.dumps(resp))

        except Exception as e:
            print(" Command execution failed:", cmd, e)
            resp = {"type": "response", "drone_id": DRONE_ID, "cmd": cmd, "status": "error", "message": str(e)}
            try:
                await self.websocket.send(json.dumps(resp))
            except Exception:
                pass


            
    async def _monitor_mission_progress(self, mission_id=None):
        """
        Background task that watches mission_progress and notifies websocket.
        This runs concurrently and will not block the ws receive loop.
        """
        try:
            async for mission_progress in self.drone.mission.mission_progress():
                # send progress to UI
                msg = {
                    "type": "mission_progress",
                    "drone_id": DRONE_ID,
                    "mission_id": mission_id or "unknown",
                    "current": mission_progress.current,
                    "total": mission_progress.total,
                }
                # send but don't let one send failure kill the loop
                try:
                    await self.websocket.send(json.dumps(msg))
                    print(f" Mission progress: {mission_progress.current}/"
                          f"{mission_progress.total}")
                except Exception:
                    # optional: log and continue
                    pass

                # stop condition
                if mission_progress.current == mission_progress.total:
                    # mission complete — notify and exit
                    try:
                        await self.websocket.send(json.dumps({
                            "type": "mission_response",
                            "drone_id": DRONE_ID,
                            "mission_id": mission_id or "unknown",
                            "status": "complete"
                        }))
                    except Exception:
                        pass
                    break

            # optional: take action on mission completion (e.g., RTL) — do it explicitly if desired
            await self.drone.action.return_to_launch()

        except Exception as e:
            try:
                await self.websocket.send(json.dumps({
                    "type": "mission_response",
                    "drone_id": DRONE_ID,
                    "mission_id": mission_id or "unknown",
                    "status": "error",
                    "message": str(e)
                }))
            except Exception:
                pass


    async def handle_mission_start(self, data):
        """Handle mission start command with waypoints"""
        try:
            waypoints = data.get("waypoints", [])
            mission_id = data.get("mission_id", "unknown")
            altitude = data.get("altitude", 50)
            speed = data.get("speed", 5)

            if not waypoints:
                print(" No waypoints provided in mission")
                return

            print(f" Mission {mission_id} received with {len(waypoints)} waypoints")

            # Upload mission to drone using MAVSDK
            from mavsdk import mission_raw

            mission_items = []
            for idx, wp in enumerate(waypoints):
                # Create mission item for each waypoint
                mission_item = mission_raw.MissionItem(
                    seq=idx,
                    frame=mission_raw.MavFrame.GLOBAL_RELATIVE_ALT_INT,
                    command=mission_raw.MavCmd.NAV_WAYPOINT,
                    current=1 if idx == 0 else 0,
                    autocontinue=1,
                    param1=0,  # Hold time at waypoint
                    param2=2,  # Acceptance radius
                    param3=0,  # Pass through waypoint
                    param4=float('nan'),  # Yaw angle
                    x=int(wp['lat'] * 1e7),  # Latitude in 1E7 degrees
                    y=int(wp['lng'] * 1e7),  # Longitude in 1E7 degrees
                    z=wp.get('alt', altitude),  # Altitude
                    mission_type=mission_raw.MissionType.MISSION
                )
                mission_items.append(mission_item)

            # Upload mission plan
            mission_plan = mission_raw.MissionPlan(mission_items)
            await self.drone.mission_raw.upload_mission(mission_plan)
            print(f" Mission uploaded: {len(mission_items)} waypoints")

            # Start mission
            await self.drone.mission.start_mission()
            print(" Mission started!")

            # Send response back to UI
            resp = {
                "type": "mission_response",
                "drone_id": DRONE_ID,
                "mission_id": mission_id,
                "status": "started",
                "waypoint_count": len(waypoints)
            }
            await self.websocket.send(json.dumps(resp))

        except Exception as e:
            print(f" Mission start failed: {e}")
            resp = {
                "type": "mission_response",
                "drone_id": DRONE_ID,
                "mission_id": data.get("mission_id", "unknown"),
                "status": "error",
                "message": str(e)
            }
            try:
                await self.websocket.send(json.dumps(resp))
            except Exception:
                pass
    

    async def clear_existing_mission(self, timeout: float = 5.0, retries: int = 2):
        """
        Clear mission on vehicle with retries and timeout.
        Works with different MAVSDK versions by supporting both:
         - await self.drone.mission.download_mission()  (returns a list)
         - async for items in self.drone.mission.download_mission(): (async iterator)
        Returns True if cleared (or believed cleared), False otherwise.
        """
        for attempt in range(1, retries + 1):
            try:
                print(f"Clearing mission (attempt {attempt}/{retries})...", flush=True)
                await self.drone.mission.clear_mission()
                await asyncio.sleep(0.5)

                # Try to verify by downloading mission (handle multiple MAVSDK styles)
                mission_items = []
                try:
                    # First try: some versions return an awaitable/list
                    maybe_items = await self.drone.mission.download_mission()
                    # if it returned a list-like object, use it
                    if maybe_items:
                        mission_items = maybe_items
                except TypeError:
                    # If download_mission is an async iterator, iterate it
                    try:
                        async for items in self.drone.mission.download_mission():
                            mission_items = items
                            break
                    except Exception:
                        # cannot verify, treat as unclear
                        mission_items = None
                except Exception:
                    # unknown error while attempting to download; fall back to assume cleared
                    mission_items = None

                # Evaluate verification result
                if mission_items is None:
                    print("Could not verify mission download; assuming clear succeeded.", flush=True)
                    return True
                elif not mission_items:
                    print("Mission cleared (verified).", flush=True)
                    return True
                else:
                    print("Mission still present after clear (download returned items).", flush=True)

            except Exception as e:
                print(f"clear_mission call failed: {e}", flush=True)
                await asyncio.sleep(0.5)

        print("Failed to clear mission after retries.", flush=True)
        return False


    async def start_survey_mission(self, area, altitude, speed, heading, mission_id=None):
        """Handle survey mission command with area and grid spacing"""
        try:
            # Provide a default mission_id if none given
            if mission_id is None:
                mission_id = f"survey-{int(time.time())}"

            horizontal_fov_deg = 30.0
            vertical_fov_deg = 20.0

            sidelap = 0.4
            frontlap = 0.7
            transect_spacing_m, photo_interval_m, fw, fh = compute_spacing_from_camera(
                altitude,
                horizontal_fov_deg=horizontal_fov_deg,
                vertical_fov_deg=vertical_fov_deg,
                sidelap_fraction=sidelap,
                frontlap_fraction=frontlap,
                sensor_width_is_across_track=True
            )

            planner = PreciseSurveyPlanner(
                polygon_coords=area,
                altitude_m=altitude,
                spacing_m=transect_spacing_m,
                angle_deg=0.0,
                speed_m_s=speed,
                camera_trigger_distance_m=photo_interval_m,
                turn_distance_m=0.1,
            )

            mission_items = planner.generate_mission_items()
            mission_plan = MissionPlan(mission_items)

            print("Clearing existing mission...")
            ok = await self.clear_existing_mission()
            if not ok:
                print("Warning: could not clear previous mission; proceeding to upload (may overwrite).")

            print("Uploading mission...")
            await self.drone.mission.upload_mission(mission_plan)

            print("Arming...")
            await self.drone.action.arm()

            print("Starting mission...")
            await self.drone.mission.start_mission()

            # monitor progress in background with a valid mission_id
            asyncio.create_task(self._monitor_mission_progress(mission_id=mission_id))

            return

        except Exception as e:
            # Use mission_id (guaranteed above) and avoid referencing any outer 'data'
            print(f" Survey mission failed: {e}")
            resp = {
                "type": "mission_response",
                "drone_id": DRONE_ID,
                "mission_id": mission_id or "unknown",
                "status": "error",
                "message": str(e)
            }
            try:
                await self.websocket.send(json.dumps(resp))
            except Exception:
                pass


    async def handle_survey_mission(self, data):
        """Handle survey mission with polygon area"""
        print(f"\n{'='*60}")
        print("📋 SURVEY MISSION RECEIVED!")
        print(f"{'='*60}")
        print(f"Full mission data: {json.dumps(data, indent=2)}")
        print(f"{'='*60}\n")

        try:
            # Extract and clean mission parameters
            drone_id = str(data.get("drone_id", "unknown"))
            altitude = float(data.get("altitude", 50))
            speed = float(data.get("speed", 5))
            heading = float(data.get("heading", 0))
            timestamp = data.get("timestamp")
            mission_id = data.get("mission_id", f"survey-{int(time.time())}")

            # Extract polygon coordinates as a list of (lat, lon) tuples
            polygon_raw = data.get("polygon", [])
            if not polygon_raw or len(polygon_raw) < 3:
                print("❌ Invalid polygon: Need at least 3 points")
                resp = {
                    "type": "survey_mission_response",
                    "drone_id": drone_id,
                    "status": "error",
                    "message": "invalid polygon (need >=3 points)"
                }
                try:
                    await self.websocket.send(json.dumps(resp))
                except Exception:
                    pass
                return

            polygon = []
            for point in polygon_raw:
                lat = float(point.get("latitude"))
                lon = float(point.get("longitude"))
                polygon.append((lat, lon))

            print(f"✅ Survey mission data extracted successfully!")

            # Pass mission_id through so background monitor can report it
            await self.start_survey_mission(polygon, altitude, speed, heading, mission_id=mission_id)

            # Send acknowledgment back to UI
            resp = {
                "type": "survey_mission_response",
                "drone_id": drone_id,
                "status": "received",
                "polygon_points": len(polygon),
                "timestamp": timestamp,
                "mission_id": mission_id
            }
            await self.websocket.send(json.dumps(resp))
            print(f"📤 Acknowledgment sent back to UI\n")

        except Exception as e:
            print(f"❌ Survey mission handling failed: {e}")
            resp = {
                "type": "survey_mission_response",
                "drone_id": data.get("drone_id", "unknown") if isinstance(data, dict) else DRONE_ID,
                "status": "error",
                "message": str(e)
            }
            try:
                await self.websocket.send(json.dumps(resp))
            except Exception:
                pass


    async def handle_signaling(self, data):
        typ = data.get("type")
        if typ == "webrtc_answer":
            # Add this check:
            if self.pc.signalingState == "stable":
                print("⏭️  Ignoring duplicate webrtc_answer (already in stable state)")
                return
            await self.pc.setRemoteDescription(RTCSessionDescription(sdp=data["sdp"], type="answer"))
        elif typ == "ice_candidate":
            cand = data.get("candidate", {})
            candidate_str = cand.get("candidate", "")
            if candidate_str:
                ice = candidate_from_sdp(candidate_str)
                ice.sdpMid = cand.get("sdpMid")
                ice.sdpMLineIndex = cand.get("sdpMLineIndex")
                await self.pc.addIceCandidate(ice)

    async def run(self):
        # 1. Authenticate
        await self.authenticate()

        # 2. Connect to MAVSDK
        await self.connect_mavsdk()

        # 3. Connect WebSocket
        ws_url = f"{WS_URL}/ws/drone/{DRONE_ID}?token={self.token}"
        print("Connecting to WebSocket:", ws_url)
        async with websockets.connect(ws_url) as ws:
            self.websocket = ws
            print(" WebSocket connected")

            # 4. Setup WebRTC
            await self.start_webrtc()

            offer = await self.pc.createOffer()
            await self.pc.setLocalDescription(offer)
            msg = {"type": "webrtc_offer", "drone_id": DRONE_ID, "sdp": self.pc.localDescription.sdp}
            await ws.send(json.dumps(msg))
            print("🎥 WebRTC offer sent")

            # 5. Start telemetry
            asyncio.create_task(self.send_telemetry())

            # 6. Message receive loop
            async for message in ws:
                try:
                    data = json.loads(message)
                except Exception as e:
                    print(" Invalid JSON from WebSocket:", e)
                    continue

                typ = data.get("type")
                print(f"📨 WS Message received: type='{typ}'")

                if typ in ("webrtc_answer", "ice_candidate"):
                    await self.handle_signaling(data)
                elif typ == "command":
                    await self.handle_command(data)
                elif typ == "mission_start":
                    await self.handle_mission_start(data)
                elif typ == "survey_mission":
                    await self.handle_survey_mission(data)
                else:
                    print(" Unknown WebSocket message type:", typ)


if __name__ == "__main__":
    client = DroneClient()
    try:
        asyncio.run(client.run())
    except KeyboardInterrupt:
        print(" Drone client exiting")