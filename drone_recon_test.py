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

# === Configuration ===
BACKEND_URL = "http://192.168.1.52:8000"
WS_URL = "ws://192.168.1.52:8000"
DRONE_ID = "austin-recon-01"
SECRET_KEY = "drone_aus_123"

# Set to True to use dummy telemetry data (for testing without real drone)
USE_DUMMY_TELEMETRY = True

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


class WebcamVideoTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)
        self.use_dummy = not self.cap.isOpened()
        if self.use_dummy:
            print(" Webcam not available — using dummy video")
        else:
            # Try to set resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        if self.use_dummy:
            # Create a dummy black frame with a message
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "Camera not available", (120, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            ret, frame = self.cap.read()
            if not ret:
                # fallback to dummy if frame read fails
                print(" Frame read failed, switching to dummy")
                self.use_dummy = True
                return await self.recv()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def __del__(self):
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
        await self.drone.connect(system_address="udpin://0.0.0.0:14540")
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
                "num_satellites": int(10 + 2 * math.sin(self.dummy_time * 0.05)),  # Simulate 8-12 satellites
                "fix_type": 3 if self.dummy_alt > 1 else 1,  # 3D fix when flying, No fix when on ground
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
                asyncio.create_task(self._gps_info_reader()),
                asyncio.create_task(self._attitude_reader()),
                asyncio.create_task(self._velocity_reader()),
                asyncio.create_task(self._battery_reader()),
                asyncio.create_task(self._status_reader()),
                asyncio.create_task(self._imu_reader()),
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
                        "gps_info": dummy_data["gps_info"],
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
                    if "gps_info" in self.latest:
                        telemetry["gps_info"] = self.latest["gps_info"]
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

    async def _gps_info_reader(self):
        """Read GPS satellite and fix information"""
        async for gps_info in self.drone.telemetry.gps_info():
            self.latest["gps_info"] = {
                "num_satellites": gps_info.num_satellites,
                "fix_type": gps_info.fix_type  # 0: No GPS, 1: No Fix, 2: 2D Fix, 3: 3D Fix
            }

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

                # Check current flight mode
                flight_mode = await self.drone.telemetry.flight_mode().__anext__()
                print(f"  Current flight mode: {flight_mode}")

                # Check if armed
                is_armed = await self.drone.telemetry.armed().__anext__()
                print(f"  Currently armed: {is_armed}")

                # Try to arm with force flag
                print("🔓 Attempting to arm (this may require safety switch or RC)...")
                try:
                    await self.drone.action.arm()
                    print("✅ Arm executed successfully")
                except Exception as arm_error:
                    print(f"❌ Arm failed: {arm_error}")
                    print("\n💡 Possible solutions:")
                    print("   1. Press the physical safety switch on the drone")
                    print("   2. Ensure RC transmitter is connected and in correct mode")
                    print("   3. Check MAVLink console for specific error messages")
                    print("   4. Try setting flight mode to STABILIZED or MANUAL first")
                    raise

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

    async def handle_survey_mission(self, data):
        """Handle survey mission with polygon area"""
        print(f"\n{'='*60}")
        print("📋 SURVEY MISSION RECEIVED!")
        print(f"{'='*60}")
        print(f"Full mission data: {json.dumps(data, indent=2)}")
        print(f"{'='*60}\n")

        try:
            drone_id = data.get("drone_id")
            polygon = data.get("polygon", [])
            altitude = data.get("altitude", 50)
            speed = data.get("speed", 5)
            heading = data.get("heading", 0)
            timestamp = data.get("timestamp")

            # Validate data
            if not polygon or len(polygon) < 3:
                print("❌ Invalid polygon: Need at least 3 points")
                return

            print(f"✅ Survey mission data validated:")
            print(f"   - Drone ID: {drone_id}")
            print(f"   - Polygon points: {len(polygon)}")
            print(f"   - Altitude: {altitude}m")
            print(f"   - Speed: {speed}m/s")
            print(f"   - Heading: {heading}°")
            print(f"   - Timestamp: {timestamp}")

            print(f"\n📍 Polygon coordinates:")
            for idx, point in enumerate(polygon):
                lat = point.get("latitude")
                lon = point.get("longitude")
                print(f"   Point {idx + 1}: Lat={lat:.6f}, Lon={lon:.6f}")

            # TODO: Here you would convert polygon to waypoints and upload to drone
            # For now, just acknowledge receipt
            print(f"\n✅ Survey mission data received successfully!")
            print(f"💡 Next steps: Convert polygon to waypoints and upload to drone")

            # Send acknowledgment back to UI
            resp = {
                "type": "survey_mission_response",
                "drone_id": DRONE_ID,
                "status": "received",
                "polygon_points": len(polygon),
                "timestamp": timestamp
            }
            await self.websocket.send(json.dumps(resp))
            print(f"📤 Acknowledgment sent back to UI\n")

        except Exception as e:
            print(f"❌ Survey mission handling failed: {e}")
            resp = {
                "type": "survey_mission_response",
                "drone_id": DRONE_ID,
                "status": "error",
                "message": str(e)
            }
            try:
                await self.websocket.send(json.dumps(resp))
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

    async def handle_signaling(self, data):
        typ = data.get("type")
        if typ == "webrtc_answer":
            await self.pc.setRemoteDescription(RTCSessionDescription(sdp=data["sdp"], type="answer"))
        elif typ == "ice_candidate":
            cand = data.get("candidate", {})
            candidate_str = cand.get("candidate", "")
            if candidate_str:  # Only process if candidate string is not empty
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
                elif typ == "survey_mission":
                    await self.handle_survey_mission(data)
                elif typ == "mission_start":
                    await self.handle_mission_start(data)
                else:
                    print(" Unknown WebSocket message type:", typ)


if __name__ == "__main__":
    client = DroneClient()
    try:
        asyncio.run(client.run())
    except KeyboardInterrupt:
        print(" Drone client exiting")