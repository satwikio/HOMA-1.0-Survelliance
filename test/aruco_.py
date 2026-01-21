print("Code Started")

import asyncio
import json
import math
import time
import sys
import cv2
import websockets
import aiohttp
import numpy as np
import base64

from aiortc import RTCPeerConnection, VideoStreamTrack, RTCSessionDescription
from aiortc.sdp import candidate_from_sdp
from av import VideoFrame
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from test2 import compute_spacing_from_camera
from test2 import PreciseSurveyPlanner
from mavsdk.telemetry import FixType
from fire_extinguish import FireExtinguisher
from go_align_drop3 import go_align_drop, DroneController
from siyi_cam2 import SIYICam


# === Exit Codes ===
EXIT_NETWORK_FAILURE = 10  # Network/video transmission issues
EXIT_BACKEND_UNAVAILABLE = 11  # Backend connection lost
EXIT_CRITICAL_ERROR = 12  # Critical error requiring attention


class VideoHealthMonitor:
    """
    Monitors video frame transmission health and detects network issues.
    Exits with specific error code if transmission fails persistently.
    """
    def __init__(self, max_consecutive_failures=10, frame_timeout=30, retry_delay=5):
        self.last_frame_sent = time.time()
        self.frames_sent_count = 0
        self.consecutive_send_failures = 0
        self.max_consecutive_failures = max_consecutive_failures
        self.frame_timeout = frame_timeout  # seconds without frame = issue
        self.retry_delay = retry_delay
        self.total_failures = 0
        self.last_health_check = time.time()

    def on_frame_sent_successfully(self):
        """Call this when a video frame is successfully sent."""
        self.last_frame_sent = time.time()
        self.frames_sent_count += 1
        self.consecutive_send_failures = 0  # Reset on success

    def on_frame_send_failed(self, error=None):
        """Call this when a video frame fails to send."""
        self.consecutive_send_failures += 1
        self.total_failures += 1

        print(f"⚠️  Frame send failure #{self.consecutive_send_failures} (total: {self.total_failures})")
        if error:
            print(f"   Error: {error}")

        # Too many consecutive failures = network issue
        if self.consecutive_send_failures >= self.max_consecutive_failures:
            print(f"\n{'='*60}")
            print(f"❌ NETWORK FAILURE DETECTED")
            print(f"{'='*60}")
            print(f"Consecutive frame send failures: {self.consecutive_send_failures}")
            print(f"Total failures: {self.total_failures}")
            print(f"Frames successfully sent: {self.frames_sent_count}")
            print(f"\n🔄 Waiting {self.retry_delay}s before restart...")
            print(f"{'='*60}\n")

            time.sleep(self.retry_delay)
            sys.exit(EXIT_NETWORK_FAILURE)

    def check_frame_timeout(self):
        """Check if we haven't sent frames in too long (indicates stall)."""
        time_since_last_frame = time.time() - self.last_frame_sent

        # Only check after initial startup (give 60s grace period)
        if self.frames_sent_count > 0 and time_since_last_frame > self.frame_timeout:
            print(f"\n{'='*60}")
            print(f"⚠️  VIDEO STREAM STALLED")
            print(f"{'='*60}")
            print(f"No frames sent in {time_since_last_frame:.1f}s (timeout: {self.frame_timeout}s)")
            print(f"Last successful frame: {time_since_last_frame:.1f}s ago")
            print(f"Total frames sent: {self.frames_sent_count}")
            print(f"\n🔄 Network may be too slow - waiting {self.retry_delay}s...")
            print(f"{'='*60}\n")

            time.sleep(self.retry_delay)

            # Check again after delay
            if time.time() - self.last_frame_sent > self.frame_timeout + self.retry_delay:
                print("❌ Frame transmission still stalled - exiting for restart")
                sys.exit(EXIT_NETWORK_FAILURE)

    def periodic_health_check(self):
        """Run periodic health checks (call this every few seconds)."""
        now = time.time()

        # Check every 10 seconds
        if now - self.last_health_check > 10:
            self.last_health_check = now
            self.check_frame_timeout()

            # Log stats
            if self.frames_sent_count > 0:
                uptime = now - self.last_frame_sent + (self.frames_sent_count * 0.033)  # rough estimate
                fps = self.frames_sent_count / uptime if uptime > 0 else 0
                failure_rate = (self.total_failures / (self.frames_sent_count + self.total_failures) * 100) if (self.frames_sent_count + self.total_failures) > 0 else 0

                print(f"📊 Video Health: {self.frames_sent_count} frames sent, "
                      f"{fps:.1f} fps, {failure_rate:.1f}% failure rate")


class TelemetryHealthMonitor:
    """
    Monitors telemetry transmission health.
    """
    def __init__(self, max_consecutive_failures=20):
        self.consecutive_send_failures = 0
        self.max_consecutive_failures = max_consecutive_failures
        self.total_sent = 0
        self.total_failures = 0

    def on_telemetry_sent_successfully(self):
        """Call when telemetry is successfully sent."""
        self.consecutive_send_failures = 0
        self.total_sent += 1

    def on_telemetry_send_failed(self, error=None):
        """Call when telemetry fails to send."""
        self.consecutive_send_failures += 1
        self.total_failures += 1

        print(f"⚠️  Telemetry send failure #{self.consecutive_send_failures}")
        if error:
            print(f"   Error: {error}")

        # Too many failures = connection issue
        if self.consecutive_send_failures >= self.max_consecutive_failures:
            print(f"\n❌ TELEMETRY TRANSMISSION FAILURE")
            print(f"Consecutive failures: {self.consecutive_send_failures}")
            print(f"Backend connection may be lost - exiting for reconnect\n")
            sys.exit(EXIT_BACKEND_UNAVAILABLE)


# === Configuration ===
import os

# Read from environment variables, fallback to defaults
BACKEND_URL = os.getenv("BACKEND_URL", "http://localhost:8000")
WS_URL = os.getenv("WS_URL", "ws://localhost:8000")
DRONE_ID = os.getenv("DRONE_ID", "austin-recon-01")
SECRET_KEY = os.getenv("SECRET_KEY", "drone_aus_123")
DRONE_TYPE = os.getenv("DRONE_TYPE", "recon")  # "recon" (surveillance) or "rescue" (payload)


# Set to True to use dummy telemetry data (for testing without real drone)
USE_DUMMY_TELEMETRY = False

# Dummy telemetry starting positions for different drones (Kolkata area)
# Each drone gets a unique position to show distinct locations on the map
DRONE_POSITIONS = {
    "austin-recon-01": (22.5726, 88.3639),    # Central Kolkata
    "austin-recon-02": (22.5800, 88.3700),    # North-East
    "austin-recon-03": (22.5650, 88.3580),    # South-West
}

# Default position if drone ID not in map (Kolkata area)
DEFAULT_START_LAT = 22.5726
DEFAULT_START_LNG = 88.3639

# Get starting position based on DRONE_ID
DUMMY_START_LAT, DUMMY_START_LNG = DRONE_POSITIONS.get(
    DRONE_ID,
    (DEFAULT_START_LAT, DEFAULT_START_LNG)
)


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
siyi_cam_ = SIYICam()
siyi_cam_.setup_cam()
from time import sleep
from aruco_detector import ArucoFireDetector
from video_quality import VideoQualityProfile

class WebcamVideoTrack(VideoStreamTrack):
    def __init__(self, detection_callback=None, quality_profile="720p_24", video_health_monitor=None):
        super().__init__()
        self.detection_callback = detection_callback
        self.current_quality = quality_profile
        self.video_health_monitor = video_health_monitor

        # Frame skipping for detection (only process every Nth frame)
        self.frame_counter = 0
        self.detection_skip_frames = 30  # Only detect on 1 out of 30 frames (~1 fps if streaming at 30fps)

        # Initialize the SIYI camera
        # self.siyi = SIYICam()
        # self.siyi.setup_cam()
        self.server_ip =  "192.168.144.25"  #self.siyi.server_ip

        # Build initial GStreamer pipeline based on quality profile
        self._build_pipeline()

        print(f"Opening SIYI RTSP pipeline ({self.current_quality}):\n", self.gst_pipeline)
        self.cap = cv2.VideoCapture(self.gst_pipeline, cv2.CAP_GSTREAMER)

        sleep(2)  # Give some time to establish the connection

        self.use_dummy = not self.cap.isOpened()
        #self.use_dummy = True

        if self.use_dummy:
            print("❌ Unable to open SIYI RTSP stream — using dummy video")
        else:
            print(f"✅ SIYI RTSP stream opened successfully ({self.current_quality})")

    def _build_pipeline(self):
        """Build GStreamer pipeline based on current quality profile"""
        profile = VideoQualityProfile.get_profile(self.current_quality)
        stream_name = profile.get("stream", "main.264")

        self.gst_pipeline = (
            f"rtspsrc location=rtsp://{self.server_ip}:8554/{stream_name} protocols=tcp latency=10 ! "
            "rtph265depay ! h265parse ! nvv4l2decoder ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true sync=false"
        )

    def change_quality(self, quality_profile):
        """
        Change video quality dynamically by switching stream.

        Args:
            quality_profile: Profile name (e.g., "1080p_30", "720p_24", "auto")
        """
        if quality_profile == self.current_quality:
            print(f"⏭️  Already using {quality_profile}, no change needed")
            return

        print(f"🔄 Changing video quality: {self.current_quality} → {quality_profile}")

        # Close existing stream
        if self.cap and self.cap.isOpened():
            self.cap.release()

        # Update quality and rebuild pipeline
        self.current_quality = quality_profile
        self._build_pipeline()

        # Reopen with new quality
        print(f"📹 Opening new stream: {self.gst_pipeline}")
        self.cap = cv2.VideoCapture(self.gst_pipeline, cv2.CAP_GSTREAMER)
        sleep(1)  # Give time to connect

        if self.cap.isOpened():
            print(f"✅ Quality changed to {quality_profile}")
        else:
            print(f"❌ Failed to open stream with {quality_profile}, reverting to dummy")
            self.use_dummy = True

    async def recv(self):
        """Return a frame from SIYI RTSP stream for WebRTC"""
        try:
            pts, time_base = await self.next_timestamp()

            if self.use_dummy:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "SIYI stream unavailable", (100, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                frame_bgr = frame  # Already BGR
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            else:
                ret, frame_bgr = self.cap.read()
                if not ret:
                    print("⚠️ Empty frame from SIYI camera, switching to dummy mode")
                    self.use_dummy = True
                    if self.video_health_monitor:
                        self.video_health_monitor.on_frame_send_failed("Empty frame from camera")
                    return await self.recv()
                frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            # Process frame for object detection ONLY every Nth frame (reduces CPU load)
            self.frame_counter += 1
            if self.detection_callback and (self.frame_counter % self.detection_skip_frames == 0):
                # Run detection in background, don't await (non-blocking)
                asyncio.create_task(self.detection_callback(frame_bgr))

            video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
            video_frame.pts = pts
            video_frame.time_base = time_base

            # Track successful frame send
            if self.video_health_monitor:
                self.video_health_monitor.on_frame_sent_successfully()

            return video_frame

        except Exception as e:
            # Track frame send failure
            if self.video_health_monitor:
                self.video_health_monitor.on_frame_send_failed(str(e))
            raise

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

        # Drone type and conditional features
        self.drone_type = DRONE_TYPE

        # Health monitors
        self.video_health = VideoHealthMonitor(
            max_consecutive_failures=10,
            frame_timeout=30,
            retry_delay=5
        )
        self.telemetry_health = TelemetryHealthMonitor(max_consecutive_failures=20)
        print("✅ Health monitoring initialized")

        # Video quality management
        self.current_quality = "720p_24"  # Default quality
        from video_quality import BandwidthMonitor
        self.bandwidth_monitor = BandwidthMonitor(check_interval=5.0)

        # Object detection - ONLY for surveillance (recon) drones
        if self.drone_type == "recon":
            self.detector = ArucoFireDetector(
                dict_type=cv2.aruco.DICT_4X4_50,
                marker_size=0.15,
                detection_interval=1.0  # 1 second between detections
            )
            self.frame_count = 0
            self.last_detection_time = 0  # Track last detection time (not frame-based)
            print(f"🔍 Surveillance drone - Object detection ENABLED (1 detection/second)")
        else:
            self.detector = None
            print(f"📦 Payload drone - Object detection DISABLED")

        # Extinguish mission queue - ONLY for payload (rescue) drones
        if self.drone_type == "rescue":
            self.fire_queue = []  # Queue of fire locations to extinguish
            self.current_mission = None  # Currently active extinguish mission (DroneController reference)
            self.is_airborne = False  # Track if drone has taken off
            self.mission_lock = asyncio.Lock()  # Prevent race conditions on mission start

            # Fire extinguisher module with position update callback
            self.fire_extinguisher = FireExtinguisher(
                drone=self.drone,
                use_dummy_telemetry=USE_DUMMY_TELEMETRY,
                position_update_callback=self.update_dummy_position
            )

            print(f"🔥 Payload drone - Extinguish mission handler ENABLED")

    def update_dummy_position(self, lat, lon, alt):
        """Callback to update dummy position from FireExtinguisher"""
        self.dummy_lat = lat
        self.dummy_lng = lon
        self.dummy_alt = alt

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
        await self.drone.connect(system_address="udpin://0.0.0.0:14540")   #For real drone "serial:///dev/ttyACM0:57600"
        # await self.drone.connect(system_address="serial:///dev/ttyACM0:57600")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(" MAVSDK connected")
                break

    async def start_webrtc(self):
        # Close existing peer connection if it exists
        if self.pc is not None:
            try:
                print("🧹 Closing existing RTCPeerConnection...")
                await self.pc.close()
            except Exception as e:
                print(f"⚠️  Error closing old peer connection: {e}")

        # Close existing video track if it exists
        if self.video_track is not None:
            try:
                self.video_track.stop()
            except Exception:
                pass

        # Create fresh connections with detection callback (only for recon drones)
        if self.drone_type == "recon":
            self.video_track = WebcamVideoTrack(
                detection_callback=self.process_frame_for_detection,
                quality_profile=self.current_quality,
                video_health_monitor=self.video_health
            )
        else:
            # Payload drones: video feed only, no detection
            self.video_track = WebcamVideoTrack(
                detection_callback=None,
                quality_profile=self.current_quality,
                video_health_monitor=self.video_health
            )
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
                self.telemetry_health.on_telemetry_sent_successfully()
            except Exception as e:
                print(" Telemetry send error:", e)
                self.telemetry_health.on_telemetry_send_failed(e)

            # Run periodic health checks
            self.video_health.periodic_health_check()

            await asyncio.sleep(1.0)  # adjust rate as needed

    async def send_heartbeat(self):
        """Send periodic ping to keep WebSocket connection alive"""
        while True:
            try:
                if self.websocket:
                    # Send ping every 10 seconds (well before 20s timeout)
                    await self.websocket.ping()
                    # Can also send a heartbeat message if backend expects it
                    # await self.websocket.send(json.dumps({"type": "ping"}))
            except Exception as e:
                print(f"⚠️ Heartbeat ping failed: {e}")
            await asyncio.sleep(10.0)

    async def process_frame_for_detection(self, frame):
        """Process a single frame for fire detection and send frame data to backend"""
        # Skip detection for payload drones
        if self.drone_type != "recon" or self.detector is None:
            return

        self.frame_count += 1

        # Time-based detection: 1 detection per second (FPS-independent)
        current_time = time.time()
        if current_time - self.last_detection_time < 1.0:
            return

        try:
            # Get current drone position
            if USE_DUMMY_TELEMETRY:
                drone_lat = self.dummy_lat
                drone_lon = self.dummy_lng
                drone_alt = self.dummy_alt
            else:
                drone_lat = self.latest.get("gps", {}).get("lat", 0)
                drone_lon = self.latest.get("gps", {}).get("lon", 0)
                drone_alt = self.latest.get("gps", {}).get("alt_rel_m", 0)

            # Run fire detection
            detection_result = self.detector.detect(
                frame,
                drone_lat=drone_lat,
                drone_lon=drone_lon,
                drone_alt=drone_alt
            )

            # Send detection result with frame data to backend if fire detected
            if detection_result and self.websocket:
                # Update last detection time
                self.last_detection_time = current_time

                # Encode frame as base64 JPEG (maximum quality as drone provides)
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                frame_b64 = base64.b64encode(buffer).decode('utf-8')

                detection_msg = {
                    "type": "object_detection",
                    "drone_id": DRONE_ID,
                    "timestamp": detection_result["timestamp"],
                    "frame_number": self.frame_count,
                    "bbox": detection_result["bbox"],  # [[p1_x, p1_y, p2_x, p2_y, lat, lon], ...]
                    "confidence": detection_result["confidence"],
                    "frame_data": frame_b64  # Base64 encoded frame
                }

                await self.websocket.send(json.dumps(detection_msg))
                print(f"🔥 Fire detected! Sent {len(detection_result['bbox'])} detection(s) with frame (frame {self.frame_count}, conf: {detection_result['confidence']})")

        except Exception as e:
            print(f"❌ Detection processing error: {e}")

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
                # If mission is running, signal it to RTL
                if self.drone_type == "rescue" and self.current_mission:
                    print("🏠 RTL during mission - signaling mission to abort and RTL")
                    self.current_mission.request_rtl()
                else:
                    await self.drone.action.return_to_launch()
                    print("✅ RTL executed")

            elif cmd == "cancel_mission":
                # Cancel current mission and hold position
                if self.drone_type == "rescue" and self.current_mission:
                    print("🛑 Cancel mission requested - signaling position hold")
                    self.current_mission.cancel_mission()
                else:
                    print("⚠️ No active mission to cancel")

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

    async def handle_gimbal_control(self, data):
        """Handle gimbal control commands from UI"""
        try:
            pitch = data.get("pitch", 0)
            roll = data.get("roll", 0)
            yaw = data.get("yaw", 0)
            zoom = data.get("zoom", 0)
            siyi_cam_.move(yaw_deg=yaw, pitch_deg=pitch)
            siyi_cam_.zoom(zoom_level=zoom)

            print(f"🎥 Gimbal Control Received - Pitch: {pitch}°, Roll: {roll}°, Yaw: {yaw}°, Zoom: {zoom}x")

            # TODO: Implement actual gimbal control
            # For now, this is a placeholder for future MAVSDK gimbal control implementation
            # If you have a physical gimbal, use MAVSDK:
            # await self.drone.gimbal.set_pitch_and_yaw(pitch, yaw)

            # For zoom control (if camera supports it):
            # If zoom > 0: zoom in, if zoom < 0: zoom out
            # await self.camera.set_zoom(abs(zoom))

            # Simulate gimbal movement for testing
            print(f"   Simulating camera orientation change...")
            print(f"   Pitch: {pitch}° (Down is negative, Up is positive)")
            print(f"   Roll: {roll}° (Left is negative, Right is positive)")
            print(f"   Yaw: {yaw}° (Pan left/right)")
            print(f"   Zoom: {zoom}x ({'IN' if zoom > 0 else 'OUT' if zoom < 0 else 'NORMAL'})")

            # Send acknowledgment back to UI
            resp = {
                "type": "gimbal_control_response",
                "drone_id": DRONE_ID,
                "status": "success",
                "pitch": pitch,
                "roll": roll,
                "yaw": yaw,
                "zoom": zoom
            }
            await self.websocket.send(json.dumps(resp))

        except Exception as e:
            print(f"❌ Gimbal control failed: {e}")
            resp = {
                "type": "gimbal_control_response",
                "drone_id": DRONE_ID,
                "status": "error",
                "message": str(e)
            }
            try:
                await self.websocket.send(json.dumps(resp))
            except Exception:
                pass

    async def handle_video_quality_change(self, data):
        """Handle video quality change requests from UI"""
        try:
            quality_profile = data.get("quality", "720p_24")
            auto_mode = data.get("auto", False)

            print(f"\n{'='*60}")
            print(f"🎬 VIDEO QUALITY CHANGE REQUESTED")
            print(f"{'='*60}")
            print(f"Profile: {quality_profile}")
            print(f"Auto Mode: {auto_mode}")
            print(f"{'='*60}\n")

            # Handle auto mode
            if auto_mode or quality_profile == "auto":
                current_bandwidth = self.bandwidth_monitor.get_current_bandwidth()
                quality_profile = VideoQualityProfile.get_profile_for_bandwidth(current_bandwidth)
                print(f"🤖 Auto mode: Selected {quality_profile} based on {current_bandwidth:.0f} kbps bandwidth")

            # Validate profile exists
            profile_data = VideoQualityProfile.get_profile(quality_profile)
            if not profile_data:
                raise ValueError(f"Invalid quality profile: {quality_profile}")

            # Update current quality
            self.current_quality = quality_profile

            # Change video track quality if video track exists
            if self.video_track and hasattr(self.video_track, 'change_quality'):
                self.video_track.change_quality(quality_profile)

            # Send acknowledgment back to UI
            resp = {
                "type": "video_quality_response",
                "drone_id": DRONE_ID,
                "status": "success",
                "quality": quality_profile,
                "profile": profile_data["label"],
                "auto_mode": auto_mode
            }
            await self.websocket.send(json.dumps(resp))

            print(f"✅ Video quality changed to: {profile_data['label']}")

        except Exception as e:
            print(f"❌ Video quality change failed: {e}")
            resp = {
                "type": "video_quality_response",
                "drone_id": DRONE_ID,
                "status": "error",
                "message": str(e)
            }
            try:
                await self.websocket.send(json.dumps(resp))
            except Exception:
                pass

    async def handle_extinguish_fire(self, data):
        """Handle extinguish fire command from UI - supports single fire or queue of fires"""
        try:
            # Check if this is a payload (rescue) drone
            if self.drone_type != "rescue":
                print(f"⚠️  This is a surveillance drone - cannot handle extinguish commands")
                resp = {
                    "type": "extinguish_fire_response",
                    "drone_id": DRONE_ID,
                    "status": "error",
                    "message": "This is a surveillance drone, not a payload drone"
                }
                await self.websocket.send(json.dumps(resp))
                return

            # Extract fire locations - supports both single fire and array of fires
            fire_locations = data.get("fire_locations", [])

            # Backward compatibility: if single fire provided with target_lat/target_lon
            if not fire_locations and data.get("target_lat") and data.get("target_lon"):
                fire_locations = [{
                    "target_lat": data.get("target_lat"),
                    "target_lon": data.get("target_lon"),
                    "detection_id": data.get("detection_id"),
                    "mission_id": data.get("mission_id")
                }]

            if not fire_locations:
                print(f"❌ No fire locations provided")
                return

            print(f"\n{'='*60}")
            print(f"🔥 EXTINGUISH FIRE COMMAND RECEIVED")
            print(f"{'='*60}")
            print(f"Number of fires in queue: {len(fire_locations)}")
            print(f"Drone Type: PAYLOAD (rescue)")
            print(f"{'='*60}\n")

            # Add fires to queue
            for fire in fire_locations:
                self.fire_queue.append(fire)
                print(f"📍 Added to queue: ({fire['target_lat']}, {fire['target_lon']})")

            print(f"\n✅ Total fires in queue: {len(self.fire_queue)}")

            # Send acknowledgment back to UI
            resp = {
                "type": "extinguish_fire_response",
                "drone_id": DRONE_ID,
                "status": "queued",
                "fires_queued": len(fire_locations),
                "total_queue_size": len(self.fire_queue),
                "message": f"Added {len(fire_locations)} fire(s) to extinguish queue"
            }
            await self.websocket.send(json.dumps(resp))

            # Start processing queue if not already processing (with lock to prevent race)
            async with self.mission_lock:
                if not self.current_mission:
                    asyncio.create_task(self.process_fire_queue())
                else:
                    print(f"⚠️ Mission already running - fires added to queue")

        except Exception as e:
            print(f"❌ Extinguish fire command failed: {e}")
            resp = {
                "type": "extinguish_fire_response",
                "drone_id": DRONE_ID,
                "status": "error",
                "message": str(e)
            }
            try:
                await self.websocket.send(json.dumps(resp))
            except Exception:
                pass

    async def process_fire_queue(self):
        """Process fire queue in BATCH mode - FOR PAYLOAD DRONES ONLY"""
        if self.drone_type != "rescue":
            return

        # Acquire lock to prevent race conditions
        async with self.mission_lock:
            if not self.fire_queue:
                print("⚠️ Fire queue is empty - nothing to process")
                return

            if self.current_mission:
                print("⚠️ Mission already in progress - cannot start another")
                return

            print(f"\n🔥 Starting BATCH fire queue processor...")
            print(f"📋 Total fires in queue: {len(self.fire_queue)}")

        # Arm and takeoff on FIRST mission (not on startup)
        if not self.is_airborne and not USE_DUMMY_TELEMETRY:
            try:
                print("🚁 Arming payload drone for first mission...")
                await self.drone.action.arm()
                print("✅ Drone armed")
            except Exception as e:
                print(f"⚠️ Arm failed (may already be armed): {e}")

            try:
                print(f"🚁 Taking off to 15m...")
                await self.drone.action.set_takeoff_altitude(15.0)
                await self.drone.action.takeoff()
                await asyncio.sleep(6)  # Wait for takeoff
                self.is_airborne = True
                print("✅ Drone airborne and ready for missions")
            except Exception as e:
                print(f"❌ Takeoff failed: {e}")
                print(f"⚠️ Continuing anyway (may already be airborne)")
                self.is_airborne = True  # Assume airborne to proceed

        # Get all fires from queue (DON'T clear yet - clear after success)
        fire_list = self.fire_queue.copy()
        print(f"📝 Processing {len(fire_list)} fires from queue (keeping in queue until completion)")

        # Send initial status for all fires
        for idx, fire in enumerate(fire_list):
            try:
                await self.websocket.send(json.dumps({
                    "type": "extinguish_mission_status",
                    "drone_id": DRONE_ID,
                    "status": "en_route",
                    "target_lat": fire["target_lat"],
                    "target_lon": fire["target_lon"],
                    "detection_id": fire.get("detection_id"),
                    "mission_id": fire.get("mission_id"),
                    "queue_remaining": len(fire_list) - idx - 1
                }))
            except Exception as e:
                print(f"Failed to send status update: {e}")

        # Convert fire_list to GPS coordinates format for go_align_drop
        # go_align_drop expects list of tuples: (lat, lon, alt)
        gps_list = [
            (fire["target_lat"], fire["target_lon"], 15.0)  # Use 15m altitude
            for fire in fire_list
        ]

        # Execute batch fire extinguishing using proven go_align_drop
        try:
            # Create DroneController wrapper around our MAVSDK drone
            # IMPORTANT: We reuse the existing connected drone to avoid breaking telemetry
            drone_controller = DroneController(connection_string="")
            drone_controller.drone = self.drone  # Use our already-connected drone

            # Share telemetry data from main script instead of creating duplicate subscriptions
            # This prevents MAVSDK connection from breaking due to multiple telemetry streams
            print("🔧 Sharing telemetry data with mission controller...")

            # Create a task to continuously update DroneController with our telemetry
            async def share_telemetry():
                while drone_controller == self.current_mission:
                    # Share position from main script's telemetry
                    if "gps" in self.latest:
                        if "alt_rel_m" in self.latest["gps"]:
                            drone_controller.altitude = self.latest["gps"]["alt_rel_m"]
                        if "lat" in self.latest["gps"]:
                            drone_controller.last_lat = self.latest["gps"]["lat"]
                        if "lon" in self.latest["gps"]:
                            drone_controller.last_lon = self.latest["gps"]["lon"]

                    # Share flight mode
                    if "status" in self.latest and "flight_mode" in self.latest["status"]:
                        drone_controller.last_flight_mode = str(self.latest["status"]["flight_mode"])

                    await asyncio.sleep(0.1)  # Update 10 times per second

            asyncio.create_task(share_telemetry())

            # Store reference to DroneController for cancel/RTL commands
            self.current_mission = drone_controller

            # Create camera object - go_align_drop will handle setup/open
            print("📷 Creating camera interface...")
            from siyi_cam2 import SIYICam
            cam = SIYICam()

            print(f"🚁 Executing go_align_drop for {len(gps_list)} fire locations...")

            # Execute the proven go_align_drop function (handles camera init)
            await go_align_drop(drone_controller, cam, gps_list, approach_alt=15.0)

            # Mark all as successful (go_align_drop handles each internally)
            results = [
                {
                    "success": True,
                    "detection_id": fire.get("detection_id"),
                    "mission_id": fire.get("mission_id"),
                    "failure_reason": None
                }
                for fire in fire_list
            ]

        except Exception as e:
            print(f"❌ go_align_drop failed: {e}")
            import traceback
            traceback.print_exc()
            # Mark all as failed
            results = [
                {
                    "success": False,
                    "detection_id": fire.get("detection_id"),
                    "mission_id": fire.get("mission_id"),
                    "failure_reason": str(e)
                }
                for fire in fire_list
            ]

        # Send completion status for each fire
        for idx, (fire, result) in enumerate(zip(fire_list, results)):
            try:
                status_data = {
                    "type": "extinguish_mission_status",
                    "drone_id": DRONE_ID,
                    "status": "completed" if result["success"] else "failed",
                    "target_lat": fire["target_lat"],
                    "target_lon": fire["target_lon"],
                    "detection_id": result["detection_id"],
                    "mission_id": result["mission_id"],
                    "queue_remaining": 0
                }
                if result["failure_reason"]:
                    status_data["failure_reason"] = result["failure_reason"]

                await self.websocket.send(json.dumps(status_data))

                if result["success"]:
                    print(f"✅ Fire {idx + 1} extinguished successfully!")
                else:
                    print(f"❌ Fire {idx + 1} extinguishing failed: {result['failure_reason']}")

            except Exception as e:
                print(f"Failed to send completion status: {e}")

        # Clear processed fires from queue and cleanup
        async with self.mission_lock:
            # Remove processed fires from queue
            for fire in fire_list:
                try:
                    self.fire_queue.remove(fire)
                except ValueError:
                    pass  # Already removed

            self.current_mission = None
            print(f"🧹 Cleared {len(fire_list)} fires from queue, {len(self.fire_queue)} remaining")

        successful_count = sum(1 for r in results if r["success"])
        print(f"\n✅ Batch queue completed! {successful_count}/{len(fire_list)} fires extinguished.\n")

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

    async def send_webrtc_offer(self):
        """Create and send a fresh WebRTC offer"""
        try:
            print(f"🎥 Creating new WebRTC offer... (current state: {self.pc.connectionState})")

            # Check if peer connection is not in a good state and recreate if needed
            # On refresh, connection goes to "disconnected" not "closed"
            if self.pc.connectionState in ("closed", "failed", "disconnected"):
                print(f"♻️  RTCPeerConnection is {self.pc.connectionState}, recreating...")
                await self.start_webrtc()

            offer = await self.pc.createOffer()
            await self.pc.setLocalDescription(offer)
            msg = {"type": "webrtc_offer", "drone_id": DRONE_ID, "sdp": self.pc.localDescription.sdp}
            await self.websocket.send(json.dumps(msg))
            print("✅ WebRTC offer sent")
        except Exception as e:
            print(f"❌ Failed to send WebRTC offer: {e}")
            # Try to recover by recreating the peer connection
            try:
                print("🔄 Attempting to recover by recreating peer connection...")
                await self.start_webrtc()
                offer = await self.pc.createOffer()
                await self.pc.setLocalDescription(offer)
                msg = {"type": "webrtc_offer", "drone_id": DRONE_ID, "sdp": self.pc.localDescription.sdp}
                await self.websocket.send(json.dumps(msg))
                print("✅ WebRTC offer sent after recovery")
            except Exception as e2:
                print(f"❌ Recovery failed: {e2}")

    async def run(self):
        try:
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

                # 4. Setup WebRTC for video feed
                await self.start_webrtc()

                if self.pc:
                    offer = await self.pc.createOffer()
                    await self.pc.setLocalDescription(offer)
                    msg = {"type": "webrtc_offer", "drone_id": DRONE_ID, "sdp": self.pc.localDescription.sdp}
                    await ws.send(json.dumps(msg))
                    print("🎥 WebRTC offer sent")

                # 5. Start telemetry
                asyncio.create_task(self.send_telemetry())

                # 5.5. Start altitude monitoring for payload drones
                if self.drone_type == "rescue":
                    asyncio.create_task(self.fire_extinguisher.monitor_altitude())

                # 5.6. Start WebSocket heartbeat to prevent timeout
                asyncio.create_task(self.send_heartbeat())

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
                    elif typ == "request_offer":
                        print("📞 UI requesting WebRTC offer")
                        await self.send_webrtc_offer()
                    elif typ == "command":
                        await self.handle_command(data)
                    elif typ == "mission_start":
                        await self.handle_mission_start(data)
                    elif typ == "survey_mission":
                        await self.handle_survey_mission(data)
                    elif typ == "gimbal_control":
                        await self.handle_gimbal_control(data)
                    elif typ == "video_quality":
                        await self.handle_video_quality_change(data)
                    elif typ == "extinguish_fire":
                        await self.handle_extinguish_fire(data)
                    else:
                        print("❌ Unknown WebSocket message type:", typ)

        except websockets.exceptions.ConnectionClosed as e:
            # WebSocket connection closed (backend down/restarted)
            print(f"\n{'='*60}", flush=True)
            print(f"🔌 WebSocket connection closed", flush=True)
            print(f"{'='*60}", flush=True)
            print(f"Reason: {e.rcvd.reason if e.rcvd else 'Unknown'}", flush=True)
            print(f"Code: {e.rcvd.code if e.rcvd else 'Unknown'}", flush=True)
            print(f"\n⚠️  Backend may have restarted or connection was lost", flush=True)
            print(f"Exiting with code {EXIT_BACKEND_UNAVAILABLE} for reconnection...", flush=True)
            print(f"{'='*60}\n", flush=True)

            # Force cleanup before exit
            try:
                asyncio.run(self.cleanup())
            except:
                pass

            # Ensure we actually exit
            import os
            os._exit(EXIT_BACKEND_UNAVAILABLE)

        except aiohttp.ClientError as e:
            # HTTP connection error (authentication failed, backend unreachable)
            print(f"\n{'='*60}", flush=True)
            print(f"❌ HTTP Connection Error", flush=True)
            print(f"{'='*60}", flush=True)
            print(f"Error: {e}", flush=True)
            print(f"\n⚠️  Cannot connect to backend", flush=True)
            print(f"Exiting with code {EXIT_BACKEND_UNAVAILABLE}...", flush=True)
            print(f"{'='*60}\n", flush=True)

            import os
            os._exit(EXIT_BACKEND_UNAVAILABLE)

        except Exception as e:
            # Unknown error
            print(f"\n{'='*60}", flush=True)
            print(f"❌ UNEXPECTED ERROR", flush=True)
            print(f"{'='*60}", flush=True)
            print(f"Error: {e}", flush=True)
            print(f"Type: {type(e).__name__}", flush=True)
            import traceback
            traceback.print_exc()
            print(f"\n⚠️  Critical error occurred", flush=True)
            print(f"Exiting with code {EXIT_CRITICAL_ERROR}...", flush=True)
            print(f"{'='*60}\n", flush=True)

            import os
            os._exit(EXIT_CRITICAL_ERROR)

    async def cleanup(self):
        """Cleanup resources properly"""
        print("🧹 Cleaning up resources...")

        # Close peer connection
        if self.pc is not None:
            try:
                await self.pc.close()
                print("✅ RTCPeerConnection closed")
            except Exception as e:
                print(f"⚠️  Error closing peer connection: {e}")

        # Stop video track
        if self.video_track is not None:
            try:
                self.video_track.stop()
                print("✅ Video track stopped")
            except Exception:
                pass

        # Close WebSocket
        if self.websocket is not None:
            try:
                await self.websocket.close()
                print("✅ WebSocket closed")
            except Exception:
                pass

        # Cancel telemetry tasks
        for task in self.telemetry_tasks:
            try:
                task.cancel()
            except Exception:
                pass

        print("✅ Cleanup complete")


if __name__ == "__main__":
    client = DroneClient()
    try:
        asyncio.run(client.run())
    except KeyboardInterrupt:
        print("\n⚠️  Keyboard interrupt received")
        # Run cleanup
        try:
            asyncio.run(client.cleanup())
        except Exception as e:
            print(f"Cleanup error: {e}")
        print("👋 Drone client exited")

























# print("Code Started")

# import asyncio
# import json
# import math
# import time
# import sys
# import cv2
# import websockets
# import aiohttp
# import numpy as np
# import base64
# import os

# from aiortc import RTCPeerConnection, VideoStreamTrack, RTCSessionDescription
# from aiortc.sdp import candidate_from_sdp
# from av import VideoFrame
# from mavsdk import System
# from mavsdk.mission import MissionItem, MissionPlan
# from mavsdk.telemetry import FixType

# # Custom imports
# from test2 import compute_spacing_from_camera
# from test2 import PreciseSurveyPlanner
# from fire_extinguish import FireExtinguisher
# from go_align_drop3 import go_align_drop, DroneController
# from siyi_cam2 import SIYICam
# from aruco_detector import ArucoFireDetector
# from video_quality import VideoQualityProfile, BandwidthMonitor


# # === Exit Codes ===
# EXIT_NETWORK_FAILURE = 10  # Network/video transmission issues
# EXIT_BACKEND_UNAVAILABLE = 11  # Backend connection lost
# EXIT_CRITICAL_ERROR = 12  # Critical error requiring attention


# class VideoHealthMonitor:
#     """
#     Monitors video frame transmission health and detects network issues.
#     Exits with specific error code if transmission fails persistently.
#     """
#     def __init__(self, max_consecutive_failures=10, frame_timeout=30, retry_delay=5):
#         self.last_frame_sent = time.time()
#         self.frames_sent_count = 0
#         self.consecutive_send_failures = 0
#         self.max_consecutive_failures = max_consecutive_failures
#         self.frame_timeout = frame_timeout  # seconds without frame = issue
#         self.retry_delay = retry_delay
#         self.total_failures = 0
#         self.last_health_check = time.time()

#     def on_frame_sent_successfully(self):
#         """Call this when a video frame is successfully sent."""
#         self.last_frame_sent = time.time()
#         self.frames_sent_count += 1
#         self.consecutive_send_failures = 0  # Reset on success

#     def on_frame_send_failed(self, error=None):
#         """Call this when a video frame fails to send."""
#         self.consecutive_send_failures += 1
#         self.total_failures += 1

#         print(f"⚠️  Frame send failure #{self.consecutive_send_failures} (total: {self.total_failures})")
#         if error:
#             print(f"   Error: {error}")

#         # Too many consecutive failures = network issue
#         if self.consecutive_send_failures >= self.max_consecutive_failures:
#             print(f"\n{'='*60}")
#             print(f"❌ NETWORK FAILURE DETECTED")
#             print(f"{'='*60}")
#             print(f"Consecutive frame send failures: {self.consecutive_send_failures}")
#             print(f"Total failures: {self.total_failures}")
#             print(f"Frames successfully sent: {self.frames_sent_count}")
#             print(f"\n🔄 Waiting {self.retry_delay}s before restart...")
#             print(f"{'='*60}\n")

#             time.sleep(self.retry_delay)
#             sys.exit(EXIT_NETWORK_FAILURE)

#     def check_frame_timeout(self):
#         """Check if we haven't sent frames in too long (indicates stall)."""
#         time_since_last_frame = time.time() - self.last_frame_sent

#         # Only check after initial startup (give 60s grace period)
#         if self.frames_sent_count > 0 and time_since_last_frame > self.frame_timeout:
#             print(f"\n{'='*60}")
#             print(f"⚠️  VIDEO STREAM STALLED")
#             print(f"{'='*60}")
#             print(f"No frames sent in {time_since_last_frame:.1f}s (timeout: {self.frame_timeout}s)")
#             print(f"Last successful frame: {time_since_last_frame:.1f}s ago")
#             print(f"Total frames sent: {self.frames_sent_count}")
#             print(f"\n🔄 Network may be too slow - waiting {self.retry_delay}s...")
#             print(f"{'='*60}\n")

#             time.sleep(self.retry_delay)

#             # Check again after delay
#             if time.time() - self.last_frame_sent > self.frame_timeout + self.retry_delay:
#                 print("❌ Frame transmission still stalled - exiting for restart")
#                 sys.exit(EXIT_NETWORK_FAILURE)

#     def periodic_health_check(self):
#         """Run periodic health checks (call this every few seconds)."""
#         now = time.time()

#         # Check every 10 seconds
#         if now - self.last_health_check > 10:
#             self.last_health_check = now
#             self.check_frame_timeout()

#             # Log stats
#             if self.frames_sent_count > 0:
#                 uptime = now - self.last_frame_sent + (self.frames_sent_count * 0.033)  # rough estimate
#                 fps = self.frames_sent_count / uptime if uptime > 0 else 0
#                 failure_rate = (self.total_failures / (self.frames_sent_count + self.total_failures) * 100) if (self.frames_sent_count + self.total_failures) > 0 else 0

#                 print(f"📊 Video Health: {self.frames_sent_count} frames sent, "
#                       f"{fps:.1f} fps, {failure_rate:.1f}% failure rate")


# class TelemetryHealthMonitor:
#     """
#     Monitors telemetry transmission health.
#     """
#     def __init__(self, max_consecutive_failures=20):
#         self.consecutive_send_failures = 0
#         self.max_consecutive_failures = max_consecutive_failures
#         self.total_sent = 0
#         self.total_failures = 0

#     def on_telemetry_sent_successfully(self):
#         """Call when telemetry is successfully sent."""
#         self.consecutive_send_failures = 0
#         self.total_sent += 1

#     def on_telemetry_send_failed(self, error=None):
#         """Call when telemetry fails to send."""
#         self.consecutive_send_failures += 1
#         self.total_failures += 1

#         print(f"⚠️  Telemetry send failure #{self.consecutive_send_failures}")
#         if error:
#             print(f"   Error: {error}")

#         # Too many failures = connection issue
#         if self.consecutive_send_failures >= self.max_consecutive_failures:
#             print(f"\n❌ TELEMETRY TRANSMISSION FAILURE")
#             print(f"Consecutive failures: {self.consecutive_send_failures}")
#             print(f"Backend connection may be lost - exiting for reconnect\n")
#             sys.exit(EXIT_BACKEND_UNAVAILABLE)


# # === Configuration ===

# # Read from environment variables, fallback to defaults
# BACKEND_URL = os.getenv("BACKEND_URL", "http://localhost:8000")
# WS_URL = os.getenv("WS_URL", "ws://localhost:8000")
# DRONE_ID = os.getenv("DRONE_ID", "austin-recon-01")
# SECRET_KEY = os.getenv("SECRET_KEY", "drone_aus_123")
# DRONE_TYPE = os.getenv("DRONE_TYPE", "recon")  # "recon" (surveillance) or "rescue" (payload)


# # Set to True to use dummy telemetry data (for testing without real drone)
# USE_DUMMY_TELEMETRY = False

# # Dummy telemetry starting positions for different drones (Kolkata area)
# DRONE_POSITIONS = {
#     "austin-recon-01": (22.5726, 88.3639),    # Central Kolkata
#     "austin-recon-02": (22.5800, 88.3700),    # North-East
#     "austin-recon-03": (22.5650, 88.3580),    # South-West
# }

# # Default position if drone ID not in map (Kolkata area)
# DEFAULT_START_LAT = 22.5726
# DEFAULT_START_LNG = 88.3639

# # Get starting position based on DRONE_ID
# DUMMY_START_LAT, DUMMY_START_LNG = DRONE_POSITIONS.get(
#     DRONE_ID,
#     (DEFAULT_START_LAT, DEFAULT_START_LNG)
# )


# def quaternion_to_euler(w, x, y, z):
#     """Convert quaternion (w,x,y,z) to Euler roll, pitch, yaw in degrees."""
#     t0 = +2.0 * (w * x + y * z)
#     t1 = +1.0 - 2.0 * (x * x + y * y)
#     roll_x = math.degrees(math.atan2(t0, t1))

#     t2 = +2.0 * (w * y - z * x)
#     t2 = +1.0 if t2 > +1.0 else t2
#     t2 = -1.0 if t2 < -1.0 else t2
#     pitch_y = math.degrees(math.asin(t2))

#     t3 = +2.0 * (w * z + x * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     yaw_z = math.degrees(math.atan2(t3, t4))

#     return roll_x, pitch_y, yaw_z


# class WebcamVideoTrack(VideoStreamTrack):
#     """
#     Video track that polls frames from the threaded SIYICam instance.
#     """
#     def __init__(self, camera_instance, detection_callback=None, video_health_monitor=None):
#         super().__init__()
#         self.camera = camera_instance  # Instance of SIYICam (threaded)
#         self.detection_callback = detection_callback
#         self.video_health_monitor = video_health_monitor

#         # Frame skipping for detection (only process every Nth frame)
#         self.frame_counter = 0
#         self.detection_skip_frames = 30  # Only detect on 1 out of 30 frames

#         # Use dummy mode if camera has no frame yet
#         self.use_dummy = False
#         print(f"✅ WebcamVideoTrack initialized with SIYI Camera Thread")

#     def change_quality(self, quality_profile):
#         """
#         Placeholder for quality change. 
#         Note: The threaded SIYICam currently uses a fixed pipeline.
#         To support dynamic quality, SIYICam would need to support pipeline restart.
#         """
#         print(f"⚠️ Change quality to {quality_profile} requested, but SIYICam is in threaded mode (fixed pipeline).")

#     async def recv(self):
#         """Return a frame from SIYI Camera thread for WebRTC"""
#         try:
#             pts, time_base = await self.next_timestamp()

#             # Retrieve latest frame from the background thread
#             frame_bgr = self.camera.get_frame()

#             if frame_bgr is None:
#                 # Frame not ready yet (startup or stalled)
#                 # Wait briefly to avoid busy loop
#                 await asyncio.sleep(0.01)
                
#                 # Check if we should fallback to dummy or return blank
#                 if self.use_dummy:
#                      # Create dummy frame
#                     frame = np.zeros((480, 640, 3), dtype=np.uint8)
#                     cv2.putText(frame, "Waiting for SIYI Stream...", (100, 240),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
#                     frame_bgr = frame 
#                     frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#                 else:
#                     # Recursive call to try again next tick
#                     return await self.recv()
#             else:
#                 self.use_dummy = False
#                 frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

#             # Process frame for object detection ONLY every Nth frame
#             self.frame_counter += 1
#             if self.detection_callback and (self.frame_counter % self.detection_skip_frames == 0):
#                 asyncio.create_task(self.detection_callback(frame_bgr))

#             video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
#             video_frame.pts = pts
#             video_frame.time_base = time_base

#             # Track successful frame send
#             if self.video_health_monitor:
#                 self.video_health_monitor.on_frame_sent_successfully()

#             return video_frame

#         except Exception as e:
#             # Track frame send failure
#             if self.video_health_monitor:
#                 self.video_health_monitor.on_frame_send_failed(str(e))
#             raise


# class DroneClient:
#     def __init__(self):
#         self.websocket = None
#         self.pc = None
#         self.video_track = None
#         self.token = None
#         self.drone = System()
#         self.latest = {}
#         self.telemetry_tasks = []

#         # Initialize SIYI Camera (Threaded)
#         self.siyi_cam = SIYICam(server_ip="192.168.144.25")
        
#         # Dummy telemetry state
#         self.dummy_lat = DUMMY_START_LAT
#         self.dummy_lng = DUMMY_START_LNG
#         self.dummy_alt = 0.0
#         self.dummy_battery = 100.0
#         self.dummy_speed = 0.0
#         self.dummy_heading = 0.0
#         self.dummy_time = 0

#         # Drone type and conditional features
#         self.drone_type = DRONE_TYPE

#         # Health monitors
#         self.video_health = VideoHealthMonitor(
#             max_consecutive_failures=10,
#             frame_timeout=30,
#             retry_delay=5
#         )
#         self.telemetry_health = TelemetryHealthMonitor(max_consecutive_failures=20)
#         print("✅ Health monitoring initialized")

#         # Video quality management
#         self.current_quality = "720p_24"  # Default quality
#         self.bandwidth_monitor = BandwidthMonitor(check_interval=5.0)

#         # Object detection - ONLY for surveillance (recon) drones
#         if self.drone_type == "recon":
#             self.detector = ArucoFireDetector(
#                 dict_type=cv2.aruco.DICT_4X4_50,
#                 marker_size=0.15,
#                 detection_interval=1.0  # 1 second between detections
#             )
#             self.frame_count = 0
#             self.last_detection_time = 0  # Track last detection time (not frame-based)
#             print(f"🔍 Surveillance drone - Object detection ENABLED (1 detection/second)")
#         else:
#             self.detector = None
#             print(f"📦 Payload drone - Object detection DISABLED")

#         # Extinguish mission queue - ONLY for payload (rescue) drones
#         if self.drone_type == "rescue":
#             self.fire_queue = []  # Queue of fire locations to extinguish
#             self.current_mission = None  # Currently active extinguish mission (DroneController reference)
#             self.is_airborne = False  # Track if drone has taken off
#             self.mission_lock = asyncio.Lock()  # Prevent race conditions on mission start

#             # Fire extinguisher module with position update callback
#             self.fire_extinguisher = FireExtinguisher(
#                 drone=self.drone,
#                 use_dummy_telemetry=USE_DUMMY_TELEMETRY,
#                 position_update_callback=self.update_dummy_position
#             )

#             print(f"🔥 Payload drone - Extinguish mission handler ENABLED")

#     def update_dummy_position(self, lat, lon, alt):
#         """Callback to update dummy position from FireExtinguisher"""
#         self.dummy_lat = lat
#         self.dummy_lng = lon
#         self.dummy_alt = alt

#     async def authenticate(self):
#         async with aiohttp.ClientSession() as session:
#             async with session.post(f"{BACKEND_URL}/api/v1/auth/drone", json={
#                 "drone_id": DRONE_ID,
#                 "secret_key": SECRET_KEY
#             }) as resp:
#                 data = await resp.json()
#                 self.token = data.get("access_token")
#                 print(" Authenticated, token:", self.token)

#     async def connect_mavsdk(self):
#         if USE_DUMMY_TELEMETRY:
#             print("⏭️  Skipping MAVSDK connection (using dummy data)")
#             return

#         print("Connecting to MAVSDK...")
#         # await self.drone.connect(system_address="udpin://0.0.0.0:14540")   #For real drone "serial:///dev/ttyACM0:57600"
#         await self.drone.connect(system_address="serial:///dev/ttyACM0:57600")
#         async for state in self.drone.core.connection_state():
#             if state.is_connected:
#                 print(" MAVSDK connected")
#                 break

#     async def start_webrtc(self):
#         # Close existing peer connection if it exists
#         if self.pc is not None:
#             try:
#                 print("🧹 Closing existing RTCPeerConnection...")
#                 await self.pc.close()
#             except Exception as e:
#                 print(f"⚠️  Error closing old peer connection: {e}")

#         # Close existing video track if it exists
#         if self.video_track is not None:
#             try:
#                 self.video_track.stop()
#             except Exception:
#                 pass

#         # Create fresh connections with detection callback (only for recon drones)
#         # Pass the SHARED threaded camera instance
#         if self.drone_type == "recon":
#             self.video_track = WebcamVideoTrack(
#                 camera_instance=self.siyi_cam,
#                 detection_callback=self.process_frame_for_detection,
#                 video_health_monitor=self.video_health
#             )
#         else:
#             # Payload drones: video feed only, no detection
#             self.video_track = WebcamVideoTrack(
#                 camera_instance=self.siyi_cam,
#                 detection_callback=None,
#                 video_health_monitor=self.video_health
#             )
            
#         self.pc = RTCPeerConnection()
#         self.pc.addTrack(self.video_track)

#         @self.pc.on("icecandidate")
#         async def on_ice(candidate):
#             if candidate:
#                 msg = {
#                     "type": "ice_candidate",
#                     "drone_id": DRONE_ID,
#                     "candidate": {
#                         "candidate": candidate.candidate,
#                         "sdpMid": candidate.sdpMid,
#                         "sdpMLineIndex": candidate.sdpMLineIndex
#                     }
#                 }
#                 await self.websocket.send(json.dumps(msg))

#     def _generate_dummy_telemetry(self):
#         """Generate realistic dummy telemetry data for testing"""
#         self.dummy_time += 1

#         # Simulate drone movement in a circular pattern
#         radius = 0.0001  # ~11 meters
#         angle = (self.dummy_time * 0.1) % (2 * math.pi)
#         self.dummy_lat = DUMMY_START_LAT + radius * math.cos(angle)
#         self.dummy_lng = DUMMY_START_LNG + radius * math.sin(angle)

#         # Simulate altitude change
#         self.dummy_alt = 50 + 10 * math.sin(self.dummy_time * 0.05)

#         # Simulate speed
#         self.dummy_speed = 5 + 2 * math.sin(self.dummy_time * 0.1)

#         # Simulate heading
#         self.dummy_heading = (angle * 180 / math.pi) % 360

#         # Simulate battery drain
#         self.dummy_battery = max(20, 100 - self.dummy_time * 0.1)

#         # Build telemetry
#         return {
#             "lat": self.dummy_lat,
#             "lng": self.dummy_lng,
#             "alt": self.dummy_alt,
#             "battery": self.dummy_battery,
#             "speed": self.dummy_speed,
#             "heading": self.dummy_heading,
#             "gps": {
#                 "lat": self.dummy_lat,
#                 "lon": self.dummy_lng,
#                 "alt_abs_m": self.dummy_alt + 400,  # Zurich elevation ~400m
#                 "alt_rel_m": self.dummy_alt,
#             },
#             "orientation": {
#                 "roll": 2 * math.sin(self.dummy_time * 0.2),
#                 "pitch": 3 * math.cos(self.dummy_time * 0.15),
#                 "yaw": self.dummy_heading,
#             },
#             "speed_detail": {
#                 "hor_m_s": self.dummy_speed,
#                 "ver_m_s": 0.5 * math.sin(self.dummy_time * 0.1),
#             },
#             "battery_detail": {
#                 "voltage_v": 12.6 - (100 - self.dummy_battery) * 0.01,
#                 "remaining_percent": self.dummy_battery,
#             },
#             "status": {
#                 "flight_mode": "POSCTL" if self.dummy_alt > 5 else "MANUAL",
#                 "is_armed": self.dummy_alt > 1,
#             },
#             "gps_info": {
#                 "num_satellites": 10,
#                 "fix_type": 3  # 3D Fix
#             },

#         }

#     async def send_telemetry(self):
#         if USE_DUMMY_TELEMETRY:
#             print("🤖 Using DUMMY telemetry data (no real drone connected)")
#             print(f"📍 Starting position: {DUMMY_START_LAT}, {DUMMY_START_LNG}")
#             print("💡 Set USE_DUMMY_TELEMETRY = False to use real MAVSDK data")
#         else:
#             # Launch telemetry reader tasks for real drone
#             self.telemetry_tasks = [
#                 asyncio.create_task(self._position_reader()),
#                 asyncio.create_task(self._attitude_reader()),
#                 asyncio.create_task(self._velocity_reader()),
#                 asyncio.create_task(self._battery_reader()),
#                 asyncio.create_task(self._status_reader()),
#                 asyncio.create_task(self._imu_reader()),
#                 asyncio.create_task(self._gps_info_reader()),   
#             ]

#         while True:
#             try:
#                 if USE_DUMMY_TELEMETRY:
#                     # Generate dummy telemetry
#                     dummy_data = self._generate_dummy_telemetry()

#                     telemetry = {
#                         "type": "telemetry",
#                         "drone_id": DRONE_ID,
#                         "timestamp": int(time.time() * 1000),
#                         "lat": dummy_data["lat"],
#                         "lng": dummy_data["lng"],
#                         "alt": dummy_data["alt"],
#                         "battery": dummy_data["battery"],
#                         "speed": dummy_data["speed"],
#                         "heading": dummy_data["heading"],
#                         "gps": dummy_data["gps"],
#                         "orientation": dummy_data["orientation"],
#                         "speed_detail": dummy_data["speed_detail"],
#                         "battery_detail": dummy_data["battery_detail"],
#                         "status": dummy_data["status"],
#                     }
#                 else:
#                     # Build flat telemetry structure from real MAVSDK data
#                     telemetry = {
#                         "type": "telemetry",
#                         "drone_id": DRONE_ID,
#                         "timestamp": int(time.time() * 1000),  # milliseconds
#                         # Basic fields (fallback to 0 if not available)
#                         "lat": self.latest.get("gps", {}).get("lat", 0),
#                         "lng": self.latest.get("gps", {}).get("lon", 0),
#                         "alt": self.latest.get("gps", {}).get("alt_rel_m", 0),
#                         "battery": self.latest.get("battery", {}).get("remaining_percent", 0),
#                         "speed": self.latest.get("speed", {}).get("hor_m_s", 0),
#                         "heading": self.latest.get("orientation", {}).get("yaw", 0),
#                     }

#                     # Add enhanced MAVSDK telemetry if available
#                     if "gps" in self.latest:
#                         telemetry["gps"] = self.latest["gps"]
#                     if "orientation" in self.latest:
#                         telemetry["orientation"] = self.latest["orientation"]
#                     if "speed" in self.latest:
#                         telemetry["speed_detail"] = self.latest["speed"]
#                     if "battery" in self.latest:
#                         telemetry["battery_detail"] = self.latest["battery"]
#                     if "status" in self.latest:
#                         telemetry["status"] = self.latest["status"]
#                     if "imu" in self.latest:
#                         telemetry["imu"] = self.latest["imu"]
#                     if "gps_info" in self.latest:
#                         telemetry["gps_info"] = self.latest["gps_info"]
#                         telemetry["num_satellites"] = self.latest["gps_info"].get("num_satellites", 0)
#                         telemetry["fix_type"] = self.latest["gps_info"].get("fix_type", 0)


#                 await self.websocket.send(json.dumps(telemetry))
#                 self.telemetry_health.on_telemetry_sent_successfully()
#             except Exception as e:
#                 print(" Telemetry send error:", e)
#                 self.telemetry_health.on_telemetry_send_failed(e)

#             # Run periodic health checks
#             self.video_health.periodic_health_check()

#             await asyncio.sleep(1.0)  # adjust rate as needed

#     async def send_heartbeat(self):
#         """Send periodic ping to keep WebSocket connection alive"""
#         while True:
#             try:
#                 if self.websocket:
#                     # Send ping every 10 seconds (well before 20s timeout)
#                     await self.websocket.ping()
#             except Exception as e:
#                 print(f"⚠️ Heartbeat ping failed: {e}")
#             await asyncio.sleep(10.0)

#     async def process_frame_for_detection(self, frame):
#         """Process a single frame for fire detection and send frame data to backend"""
#         # Skip detection for payload drones
#         if self.drone_type != "recon" or self.detector is None:
#             return

#         self.frame_count += 1

#         # Time-based detection: 1 detection per second (FPS-independent)
#         current_time = time.time()
#         if current_time - self.last_detection_time < 1.0:
#             return

#         try:
#             # Get current drone position
#             if USE_DUMMY_TELEMETRY:
#                 drone_lat = self.dummy_lat
#                 drone_lon = self.dummy_lng
#                 drone_alt = self.dummy_alt
#             else:
#                 drone_lat = self.latest.get("gps", {}).get("lat", 0)
#                 drone_lon = self.latest.get("gps", {}).get("lon", 0)
#                 drone_alt = self.latest.get("gps", {}).get("alt_rel_m", 0)

#             # Run fire detection
#             detection_result = self.detector.detect(
#                 frame,
#                 drone_lat=drone_lat,
#                 drone_lon=drone_lon,
#                 drone_alt=drone_alt
#             )

#             # Send detection result with frame data to backend if fire detected
#             if detection_result and self.websocket:
#                 # Update last detection time
#                 self.last_detection_time = current_time

#                 # Encode frame as base64 JPEG (maximum quality as drone provides)
#                 _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
#                 frame_b64 = base64.b64encode(buffer).decode('utf-8')

#                 detection_msg = {
#                     "type": "object_detection",
#                     "drone_id": DRONE_ID,
#                     "timestamp": detection_result["timestamp"],
#                     "frame_number": self.frame_count,
#                     "bbox": detection_result["bbox"],  # [[p1_x, p1_y, p2_x, p2_y, lat, lon], ...]
#                     "confidence": detection_result["confidence"],
#                     "frame_data": frame_b64  # Base64 encoded frame
#                 }

#                 await self.websocket.send(json.dumps(detection_msg))
#                 print(f"🔥 Fire detected! Sent {len(detection_result['bbox'])} detection(s) with frame (frame {self.frame_count}, conf: {detection_result['confidence']})")

#         except Exception as e:
#             print(f"❌ Detection processing error: {e}")

#     # ------ Telemetry readers ------

#     async def _position_reader(self):
#         async for pos in self.drone.telemetry.position():
#             self.latest.setdefault("gps", {})
#             self.latest["gps"].update({
#                 "lat": pos.latitude_deg,
#                 "lon": pos.longitude_deg,
#                 "alt_abs_m": pos.absolute_altitude_m,
#                 "alt_rel_m": pos.relative_altitude_m,
#             })

#     async def _attitude_reader(self):
#         try:
#             async for e in self.drone.telemetry.attitude_euler():
#                 self.latest["orientation"] = {
#                     "roll": e.roll_deg,
#                     "pitch": e.pitch_deg,
#                     "yaw": e.yaw_deg
#                 }
#         except Exception:
#             async for q in self.drone.telemetry.attitude_quaternion():
#                 r, p, y = quaternion_to_euler(q.w, q.x, q.y, q.z)
#                 self.latest["orientation"] = {"roll": r, "pitch": p, "yaw": y}

#     async def _gps_info_reader(self):
#         """
#         Store num_satellites and map MAVSDK FixType to integer:
#         0 = NoGps, 1 = NoFix, 2 = Fix2D, 3 = Fix3D/DGPS/RTK
#         """
#         async for gps_info in self.drone.telemetry.gps_info():
#             # default
#             fix_type_value = 0

#             try:
#                 # Enum comparisons (preferred)
#                 if gps_info.fix_type == FixType.NoGps:
#                     fix_type_value = 0
#                 elif gps_info.fix_type == FixType.NoFix:
#                     fix_type_value = 1
#                 elif gps_info.fix_type == FixType.Fix2D:
#                     fix_type_value = 2
#                 elif gps_info.fix_type in (
#                     FixType.Fix3D,
#                     FixType.FixDgps,
#                     FixType.RtkFloat,
#                     FixType.RtkFixed,
#                 ):
#                     fix_type_value = 3
#                 else:
#                     fix_type_value = 0

#             except Exception:
#                 # Fallback: string-based detection (covers wrapper/version differences)
#                 try:
#                     fix_name = str(gps_info.fix_type).upper()
#                     if "NOGPS" in fix_name or "NO_GPS" in fix_name:
#                         fix_type_value = 0
#                     elif "NOFIX" in fix_name or "NO_FIX" in fix_name:
#                         fix_type_value = 1
#                     elif "2D" in fix_name:
#                         fix_type_value = 2
#                     elif any(x in fix_name for x in ("3D", "DGPS", "RTK")):
#                         fix_type_value = 3
#                     else:
#                         fix_type_value = 0
#                 except Exception:
#                     fix_type_value = 0

#             # update cache
#             self.latest.setdefault("gps_info", {})
#             self.latest["gps_info"].update({
#                 "num_satellites": int(getattr(gps_info, "num_satellites", 0)),
#                 "fix_type": fix_type_value,
#             })

#     async def _velocity_reader(self):
#         try:
#             async for v in self.drone.telemetry.velocity_ned():
#                 hor = math.hypot(v.velocity_north_m_s, v.velocity_east_m_s)
#                 ver = v.velocity_down_m_s
#                 self.latest["speed"] = {"hor_m_s": hor, "ver_m_s": ver}
#         except Exception:
#             pass

#     async def _battery_reader(self):
#         async for b in self.drone.telemetry.battery():
#             self.latest["battery"] = {
#                 "voltage_v": b.voltage_v,
#                 "remaining_percent": b.remaining_percent
#             }

#     async def _status_reader(self):
#         # Run both flight_mode and armed readers concurrently
#         async def read_flight_mode():
#             async for fm in self.drone.telemetry.flight_mode():
#                 self.latest.setdefault("status", {})
#                 self.latest["status"]["flight_mode"] = str(fm)

#         async def read_armed():
#             async for armed in self.drone.telemetry.armed():
#                 self.latest.setdefault("status", {})
#                 self.latest["status"]["is_armed"] = bool(armed)

#         # Run both concurrently
#         await asyncio.gather(read_flight_mode(), read_armed())

#     async def _imu_reader(self):
#         async for imu in self.drone.telemetry.imu():
#             try:
#                 accel = [imu.acceleration_frd.x_m_s2,
#                          imu.acceleration_frd.y_m_s2,
#                          imu.acceleration_frd.z_m_s2]
#             except Exception:
#                 accel = getattr(imu, "acceleration", None) or []
#             try:
#                 gyro = [imu.angular_velocity_frd.x_rad_s,
#                         imu.angular_velocity_frd.y_rad_s,
#                         imu.angular_velocity_frd.z_rad_s]
#             except Exception:
#                 gyro = getattr(imu, "angular_velocity", None) or []
#             try:
#                 mag = [imu.magnetic_field_frd.x_ga,
#                        imu.magnetic_field_frd.y_ga,
#                        imu.magnetic_field_frd.z_ga]
#             except Exception:
#                 mag = getattr(imu, "magnetic_field", None) or []
#             self.latest["imu"] = {"accel": accel, "gyro": gyro, "mag": mag}

#     # ------ Command handling ------

#     async def handle_command(self, data):
#         # Expect messages like:
#         # { "type": "command", "cmd": "arm", "drone_id": "...", "height": ... }
#         cmd = data.get("cmd")
#         height = data.get("height")  # For takeoff command

#         print(f"\n{'='*60}")
#         print(f"⚡ COMMAND RECEIVED: {cmd}")
#         print(f"Full command data: {json.dumps(data, indent=2)}")
#         print(f"{'='*60}\n")

#         try:
#             if cmd == "arm":
#                 # Wait until important health checks are OK (with timeout)
#                 print("🔍 Checking drone health before arming...")
#                 # choose required flags here
#                 required_flags = [
#                     "is_gyrometer_calibration_ok",
#                     "is_accelerometer_calibration_ok",
#                     "is_magnetometer_calibration_ok",
#                     "is_local_position_ok",
#                     "is_global_position_ok",
#                     "is_home_position_ok",
#                 ]

#                 timeout_s = 20.0   # max wait time (seconds)
#                 poll_interval = 1.0
#                 start = time.time()
#                 healthy = False

#                 try:
#                     # health() is an async iterator — iterate until all required flags are true or timeout
#                     async for health in self.drone.telemetry.health():
#                         # print a friendly status line
#                         print(
#                             f"  Gyro:{'✅' if health.is_gyrometer_calibration_ok else '❌'} "
#                             f"Accel:{'✅' if health.is_accelerometer_calibration_ok else '❌'} "
#                             f"Mag:{'✅' if health.is_magnetometer_calibration_ok else '❌'} "
#                             f"LocalPos:{'✅' if health.is_local_position_ok else '❌'} "
#                             f"GlobalPos:{'✅' if health.is_global_position_ok else '❌'} "
#                             f"HomePos:{'✅' if health.is_home_position_ok else '❌'}"
#                         )

#                         all_ok = (
#                             health.is_gyrometer_calibration_ok and
#                             health.is_accelerometer_calibration_ok and
#                             health.is_magnetometer_calibration_ok and
#                             health.is_local_position_ok and
#                             health.is_global_position_ok and
#                             health.is_home_position_ok
#                         )

#                         if all_ok:
#                             healthy = True
#                             print("✅ All required health checks are OK")
#                             break

#                         # timeout handling
#                         if (time.time() - start) >= timeout_s:
#                             print(f"⛔ Health checks did not become OK within {timeout_s} seconds")
#                             healthy = False
#                             break

#                         # wait a bit before reading the next health message
#                         await asyncio.sleep(poll_interval)

#                 except Exception as e:
#                     print(" Error while checking health:", e)
#                     healthy = False

#                 if not healthy:
#                     # refuse to arm — notify backend and do not call arm()
#                     print("❌ Refusing to arm: health checks failed")
#                     resp = {"type": "response", "drone_id": DRONE_ID, "cmd": "arm", "status": "error", "message": "health checks failed"}
#                     try:
#                         await self.websocket.send(json.dumps(resp))
#                     except Exception:
#                         pass
#                 else:
#                     # safe to arm
#                     try:
#                         await self.drone.action.arm()
#                         print("✅ Arm executed successfully")
#                         resp = {"type": "response", "drone_id": DRONE_ID, "cmd": "arm", "status": "ok"}
#                         await self.websocket.send(json.dumps(resp))
#                     except Exception as e:
#                         print(" Command execution failed: arm", e)
#                         resp = {"type": "response", "drone_id": DRONE_ID, "cmd": "arm", "status": "error", "message": str(e)}
#                         try:
#                             await self.websocket.send(json.dumps(resp))
#                         except Exception:
#                             pass

#             elif cmd == "disarm":
#                 await self.drone.action.disarm()
#                 print(" Disarm executed")

#             elif cmd == "takeoff":
#                 alt = float(height) if height else 10.0
#                 await self.drone.action.set_takeoff_altitude(alt)
#                 await self.drone.action.takeoff()
#                 print(" Takeoff executed, altitude:", alt)

#             elif cmd == "land":
#                 await self.drone.action.land()
#                 print(" Land executed")

#             elif cmd in ("rtl", "return_to_launch"):
#                 # If mission is running, signal it to RTL
#                 if self.drone_type == "rescue" and self.current_mission:
#                     print("🏠 RTL during mission - signaling mission to abort and RTL")
#                     self.current_mission.request_rtl()
#                 else:
#                     await self.drone.action.return_to_launch()
#                     print("✅ RTL executed")

#             elif cmd == "cancel_mission":
#                 # Cancel current mission and hold position
#                 if self.drone_type == "rescue" and self.current_mission:
#                     print("🛑 Cancel mission requested - signaling position hold")
#                     self.current_mission.cancel_mission()
#                 else:
#                     print("⚠️ No active mission to cancel")

#             else:
#                 print(" Unknown command:", cmd)

#             # Send acknowledgment
#             resp = {"type": "response", "drone_id": DRONE_ID, "cmd": cmd, "status": "ok"}
#             await self.websocket.send(json.dumps(resp))

#         except Exception as e:
#             print(" Command execution failed:", cmd, e)
#             resp = {"type": "response", "drone_id": DRONE_ID, "cmd": cmd, "status": "error", "message": str(e)}
#             try:
#                 await self.websocket.send(json.dumps(resp))
#             except Exception:
#                 pass


            
#     async def _monitor_mission_progress(self, mission_id=None):
#         """
#         Background task that watches mission_progress and notifies websocket.
#         This runs concurrently and will not block the ws receive loop.
#         """
#         try:
#             async for mission_progress in self.drone.mission.mission_progress():
#                 # send progress to UI
#                 msg = {
#                     "type": "mission_progress",
#                     "drone_id": DRONE_ID,
#                     "mission_id": mission_id or "unknown",
#                     "current": mission_progress.current,
#                     "total": mission_progress.total,
#                 }
#                 # send but don't let one send failure kill the loop
#                 try:
#                     await self.websocket.send(json.dumps(msg))
#                     print(f" Mission progress: {mission_progress.current}/"
#                           f"{mission_progress.total}")
#                 except Exception:
#                     # optional: log and continue
#                     pass

#                 # stop condition
#                 if mission_progress.current == mission_progress.total:
#                     # mission complete — notify and exit
#                     try:
#                         await self.websocket.send(json.dumps({
#                             "type": "mission_response",
#                             "drone_id": DRONE_ID,
#                             "mission_id": mission_id or "unknown",
#                             "status": "complete"
#                         }))
#                     except Exception:
#                         pass
#                     break

#             # optional: take action on mission completion (e.g., RTL) — do it explicitly if desired
#             await self.drone.action.return_to_launch()

#         except Exception as e:
#             try:
#                 await self.websocket.send(json.dumps({
#                     "type": "mission_response",
#                     "drone_id": DRONE_ID,
#                     "mission_id": mission_id or "unknown",
#                     "status": "error",
#                     "message": str(e)
#                 }))
#             except Exception:
#                 pass


#     async def handle_mission_start(self, data):
#         """Handle mission start command with waypoints"""
#         try:
#             waypoints = data.get("waypoints", [])
#             mission_id = data.get("mission_id", "unknown")
#             altitude = data.get("altitude", 50)
#             speed = data.get("speed", 5)

#             if not waypoints:
#                 print(" No waypoints provided in mission")
#                 return

#             print(f" Mission {mission_id} received with {len(waypoints)} waypoints")

#             # Upload mission to drone using MAVSDK
#             from mavsdk import mission_raw

#             mission_items = []
#             for idx, wp in enumerate(waypoints):
#                 # Create mission item for each waypoint
#                 mission_item = mission_raw.MissionItem(
#                     seq=idx,
#                     frame=mission_raw.MavFrame.GLOBAL_RELATIVE_ALT_INT,
#                     command=mission_raw.MavCmd.NAV_WAYPOINT,
#                     current=1 if idx == 0 else 0,
#                     autocontinue=1,
#                     param1=0,  # Hold time at waypoint
#                     param2=2,  # Acceptance radius
#                     param3=0,  # Pass through waypoint
#                     param4=float('nan'),  # Yaw angle
#                     x=int(wp['lat'] * 1e7),  # Latitude in 1E7 degrees
#                     y=int(wp['lng'] * 1e7),  # Longitude in 1E7 degrees
#                     z=wp.get('alt', altitude),  # Altitude
#                     mission_type=mission_raw.MissionType.MISSION
#                 )
#                 mission_items.append(mission_item)

#             # Upload mission plan
#             mission_plan = mission_raw.MissionPlan(mission_items)
#             await self.drone.mission_raw.upload_mission(mission_plan)
#             print(f" Mission uploaded: {len(mission_items)} waypoints")

#             # Start mission
#             await self.drone.mission.start_mission()
#             print(" Mission started!")

#             # Send response back to UI
#             resp = {
#                 "type": "mission_response",
#                 "drone_id": DRONE_ID,
#                 "mission_id": mission_id,
#                 "status": "started",
#                 "waypoint_count": len(waypoints)
#             }
#             await self.websocket.send(json.dumps(resp))

#         except Exception as e:
#             print(f" Mission start failed: {e}")
#             resp = {
#                 "type": "mission_response",
#                 "drone_id": DRONE_ID,
#                 "mission_id": data.get("mission_id", "unknown"),
#                 "status": "error",
#                 "message": str(e)
#             }
#             try:
#                 await self.websocket.send(json.dumps(resp))
#             except Exception:
#                 pass
    

#     async def clear_existing_mission(self, timeout: float = 5.0, retries: int = 2):
#         """
#         Clear mission on vehicle with retries and timeout.
#         Works with different MAVSDK versions by supporting both:
#          - await self.drone.mission.download_mission()  (returns a list)
#          - async for items in self.drone.mission.download_mission(): (async iterator)
#         Returns True if cleared (or believed cleared), False otherwise.
#         """
#         for attempt in range(1, retries + 1):
#             try:
#                 print(f"Clearing mission (attempt {attempt}/{retries})...", flush=True)
#                 await self.drone.mission.clear_mission()
#                 await asyncio.sleep(0.5)

#                 # Try to verify by downloading mission (handle multiple MAVSDK styles)
#                 mission_items = []
#                 try:
#                     # First try: some versions return an awaitable/list
#                     maybe_items = await self.drone.mission.download_mission()
#                     # if it returned a list-like object, use it
#                     if maybe_items:
#                         mission_items = maybe_items
#                 except TypeError:
#                     # If download_mission is an async iterator, iterate it
#                     try:
#                         async for items in self.drone.mission.download_mission():
#                             mission_items = items
#                             break
#                     except Exception:
#                         # cannot verify, treat as unclear
#                         mission_items = None
#                 except Exception:
#                     # unknown error while attempting to download; fall back to assume cleared
#                     mission_items = None

#                 # Evaluate verification result
#                 if mission_items is None:
#                     print("Could not verify mission download; assuming clear succeeded.", flush=True)
#                     return True
#                 elif not mission_items:
#                     print("Mission cleared (verified).", flush=True)
#                     return True
#                 else:
#                     print("Mission still present after clear (download returned items).", flush=True)

#             except Exception as e:
#                 print(f"clear_mission call failed: {e}", flush=True)
#                 await asyncio.sleep(0.5)

#         print("Failed to clear mission after retries.", flush=True)
#         return False


#     async def start_survey_mission(self, area, altitude, speed, heading, mission_id=None):
#         """Handle survey mission command with area and grid spacing"""
#         try:
#             # Provide a default mission_id if none given
#             if mission_id is None:
#                 mission_id = f"survey-{int(time.time())}"

#             horizontal_fov_deg = 30.0
#             vertical_fov_deg = 20.0

#             sidelap = 0.4
#             frontlap = 0.7
#             transect_spacing_m, photo_interval_m, fw, fh = compute_spacing_from_camera(
#                 altitude,
#                 horizontal_fov_deg=horizontal_fov_deg,
#                 vertical_fov_deg=vertical_fov_deg,
#                 sidelap_fraction=sidelap,
#                 frontlap_fraction=frontlap,
#                 sensor_width_is_across_track=True
#             )

#             planner = PreciseSurveyPlanner(
#                 polygon_coords=area,
#                 altitude_m=altitude,
#                 spacing_m=transect_spacing_m,
#                 angle_deg=0.0,
#                 speed_m_s=speed,
#                 camera_trigger_distance_m=photo_interval_m,
#                 turn_distance_m=0.1,
#             )

#             mission_items = planner.generate_mission_items()
#             mission_plan = MissionPlan(mission_items)

#             print("Clearing existing mission...")
#             ok = await self.clear_existing_mission()
#             if not ok:
#                 print("Warning: could not clear previous mission; proceeding to upload (may overwrite).")

#             print("Uploading mission...")
#             await self.drone.mission.upload_mission(mission_plan)

#             print("Arming...")
#             await self.drone.action.arm()

#             print("Starting mission...")
#             await self.drone.mission.start_mission()

#             # monitor progress in background with a valid mission_id
#             asyncio.create_task(self._monitor_mission_progress(mission_id=mission_id))

#             return

#         except Exception as e:
#             # Use mission_id (guaranteed above) and avoid referencing any outer 'data'
#             print(f" Survey mission failed: {e}")
#             resp = {
#                 "type": "mission_response",
#                 "drone_id": DRONE_ID,
#                 "mission_id": mission_id or "unknown",
#                 "status": "error",
#                 "message": str(e)
#             }
#             try:
#                 await self.websocket.send(json.dumps(resp))
#             except Exception:
#                 pass


#     async def handle_survey_mission(self, data):
#         """Handle survey mission with polygon area"""
#         print(f"\n{'='*60}")
#         print("📋 SURVEY MISSION RECEIVED!")
#         print(f"{'='*60}")
#         print(f"Full mission data: {json.dumps(data, indent=2)}")
#         print(f"{'='*60}\n")

#         try:
#             # Extract and clean mission parameters
#             drone_id = str(data.get("drone_id", "unknown"))
#             altitude = float(data.get("altitude", 50))
#             speed = float(data.get("speed", 5))
#             heading = float(data.get("heading", 0))
#             timestamp = data.get("timestamp")
#             mission_id = data.get("mission_id", f"survey-{int(time.time())}")

#             # Extract polygon coordinates as a list of (lat, lon) tuples
#             polygon_raw = data.get("polygon", [])
#             if not polygon_raw or len(polygon_raw) < 3:
#                 print("❌ Invalid polygon: Need at least 3 points")
#                 resp = {
#                     "type": "survey_mission_response",
#                     "drone_id": drone_id,
#                     "status": "error",
#                     "message": "invalid polygon (need >=3 points)"
#                 }
#                 try:
#                     await self.websocket.send(json.dumps(resp))
#                 except Exception:
#                     pass
#                 return

#             polygon = []
#             for point in polygon_raw:
#                 lat = float(point.get("latitude"))
#                 lon = float(point.get("longitude"))
#                 polygon.append((lat, lon))

#             print(f"✅ Survey mission data extracted successfully!")

#             # Pass mission_id through so background monitor can report it
#             await self.start_survey_mission(polygon, altitude, speed, heading, mission_id=mission_id)

#             # Send acknowledgment back to UI
#             resp = {
#                 "type": "survey_mission_response",
#                 "drone_id": drone_id,
#                 "status": "received",
#                 "polygon_points": len(polygon),
#                 "timestamp": timestamp,
#                 "mission_id": mission_id
#             }
#             await self.websocket.send(json.dumps(resp))
#             print(f"📤 Acknowledgment sent back to UI\n")

#         except Exception as e:
#             print(f"❌ Survey mission handling failed: {e}")
#             resp = {
#                 "type": "survey_mission_response",
#                 "drone_id": data.get("drone_id", "unknown") if isinstance(data, dict) else DRONE_ID,
#                 "status": "error",
#                 "message": str(e)
#             }
#             try:
#                 await self.websocket.send(json.dumps(resp))
#             except Exception:
#                 pass

#     async def handle_gimbal_control(self, data):
#         """Handle gimbal control commands from UI"""
#         try:
#             pitch = data.get("pitch", 0)
#             roll = data.get("roll", 0)
#             yaw = data.get("yaw", 0)
#             zoom = data.get("zoom", 0)
            
#             # Use the local SIYICam instance
#             self.siyi_cam.move(yaw_deg=yaw, pitch_deg=pitch)
#             self.siyi_cam.zoom(zoom_level=zoom)

#             print(f"🎥 Gimbal Control Received - Pitch: {pitch}°, Roll: {roll}°, Yaw: {yaw}°, Zoom: {zoom}x")

#             # Send acknowledgment back to UI
#             resp = {
#                 "type": "gimbal_control_response",
#                 "drone_id": DRONE_ID,
#                 "status": "success",
#                 "pitch": pitch,
#                 "roll": roll,
#                 "yaw": yaw,
#                 "zoom": zoom
#             }
#             await self.websocket.send(json.dumps(resp))

#         except Exception as e:
#             print(f"❌ Gimbal control failed: {e}")
#             resp = {
#                 "type": "gimbal_control_response",
#                 "drone_id": DRONE_ID,
#                 "status": "error",
#                 "message": str(e)
#             }
#             try:
#                 await self.websocket.send(json.dumps(resp))
#             except Exception:
#                 pass

#     async def handle_video_quality_change(self, data):
#         """Handle video quality change requests from UI"""
#         try:
#             quality_profile = data.get("quality", "720p_24")
#             auto_mode = data.get("auto", False)

#             print(f"\n{'='*60}")
#             print(f"🎬 VIDEO QUALITY CHANGE REQUESTED")
#             print(f"{'='*60}")
#             print(f"Profile: {quality_profile}")
#             print(f"Auto Mode: {auto_mode}")
#             print(f"Note: Using threaded camera mode - quality changes may be ignored")
#             print(f"{'='*60}\n")

#             # Validate profile exists
#             profile_data = VideoQualityProfile.get_profile(quality_profile)
#             if not profile_data:
#                 raise ValueError(f"Invalid quality profile: {quality_profile}")

#             # Update current quality
#             self.current_quality = quality_profile

#             # Change video track quality if video track exists
#             # Note: This is now a no-op or placeholder in the threaded version
#             if self.video_track and hasattr(self.video_track, 'change_quality'):
#                 self.video_track.change_quality(quality_profile)

#             # Send acknowledgment back to UI
#             resp = {
#                 "type": "video_quality_response",
#                 "drone_id": DRONE_ID,
#                 "status": "success",
#                 "quality": quality_profile,
#                 "profile": profile_data["label"],
#                 "auto_mode": auto_mode
#             }
#             await self.websocket.send(json.dumps(resp))

#             print(f"✅ Video quality changed to: {profile_data['label']}")

#         except Exception as e:
#             print(f"❌ Video quality change failed: {e}")
#             resp = {
#                 "type": "video_quality_response",
#                 "drone_id": DRONE_ID,
#                 "status": "error",
#                 "message": str(e)
#             }
#             try:
#                 await self.websocket.send(json.dumps(resp))
#             except Exception:
#                 pass

#     async def handle_extinguish_fire(self, data):
#         """Handle extinguish fire command from UI - supports single fire or queue of fires"""
#         try:
#             # Check if this is a payload (rescue) drone
#             if self.drone_type != "rescue":
#                 print(f"⚠️  This is a surveillance drone - cannot handle extinguish commands")
#                 resp = {
#                     "type": "extinguish_fire_response",
#                     "drone_id": DRONE_ID,
#                     "status": "error",
#                     "message": "This is a surveillance drone, not a payload drone"
#                 }
#                 await self.websocket.send(json.dumps(resp))
#                 return

#             # Extract fire locations - supports both single fire and array of fires
#             fire_locations = data.get("fire_locations", [])

#             # Backward compatibility: if single fire provided with target_lat/target_lon
#             if not fire_locations and data.get("target_lat") and data.get("target_lon"):
#                 fire_locations = [{
#                     "target_lat": data.get("target_lat"),
#                     "target_lon": data.get("target_lon"),
#                     "detection_id": data.get("detection_id"),
#                     "mission_id": data.get("mission_id")
#                 }]

#             if not fire_locations:
#                 print(f"❌ No fire locations provided")
#                 return

#             print(f"\n{'='*60}")
#             print(f"🔥 EXTINGUISH FIRE COMMAND RECEIVED")
#             print(f"{'='*60}")
#             print(f"Number of fires in queue: {len(fire_locations)}")
#             print(f"Drone Type: PAYLOAD (rescue)")
#             print(f"{'='*60}\n")

#             # Add fires to queue
#             for fire in fire_locations:
#                 self.fire_queue.append(fire)
#                 print(f"📍 Added to queue: ({fire['target_lat']}, {fire['target_lon']})")

#             print(f"\n✅ Total fires in queue: {len(self.fire_queue)}")

#             # Send acknowledgment back to UI
#             resp = {
#                 "type": "extinguish_fire_response",
#                 "drone_id": DRONE_ID,
#                 "status": "queued",
#                 "fires_queued": len(fire_locations),
#                 "total_queue_size": len(self.fire_queue),
#                 "message": f"Added {len(fire_locations)} fire(s) to extinguish queue"
#             }
#             await self.websocket.send(json.dumps(resp))

#             # Start processing queue if not already processing (with lock to prevent race)
#             async with self.mission_lock:
#                 if not self.current_mission:
#                     asyncio.create_task(self.process_fire_queue())
#                 else:
#                     print(f"⚠️ Mission already running - fires added to queue")

#         except Exception as e:
#             print(f"❌ Extinguish fire command failed: {e}")
#             resp = {
#                 "type": "extinguish_fire_response",
#                 "drone_id": DRONE_ID,
#                 "status": "error",
#                 "message": str(e)
#             }
#             try:
#                 await self.websocket.send(json.dumps(resp))
#             except Exception:
#                 pass

#     async def process_fire_queue(self):
#         """Process fire queue in BATCH mode - FOR PAYLOAD DRONES ONLY"""
#         if self.drone_type != "rescue":
#             return

#         # Acquire lock to prevent race conditions
#         async with self.mission_lock:
#             if not self.fire_queue:
#                 print("⚠️ Fire queue is empty - nothing to process")
#                 return

#             if self.current_mission:
#                 print("⚠️ Mission already in progress - cannot start another")
#                 return

#             print(f"\n🔥 Starting BATCH fire queue processor...")
#             print(f"📋 Total fires in queue: {len(self.fire_queue)}")

#         # Arm and takeoff on FIRST mission (not on startup)
#         if not self.is_airborne and not USE_DUMMY_TELEMETRY:
#             try:
#                 print("🚁 Arming payload drone for first mission...")
#                 await self.drone.action.arm()
#                 print("✅ Drone armed")
#             except Exception as e:
#                 print(f"⚠️ Arm failed (may already be armed): {e}")

#             try:
#                 print(f"🚁 Taking off to 15m...")
#                 await self.drone.action.set_takeoff_altitude(15.0)
#                 await self.drone.action.takeoff()
#                 await asyncio.sleep(6)  # Wait for takeoff
#                 self.is_airborne = True
#                 print("✅ Drone airborne and ready for missions")
#             except Exception as e:
#                 print(f"❌ Takeoff failed: {e}")
#                 print(f"⚠️ Continuing anyway (may already be airborne)")
#                 self.is_airborne = True  # Assume airborne to proceed

#         # Get all fires from queue (DON'T clear yet - clear after success)
#         fire_list = self.fire_queue.copy()
#         print(f"📝 Processing {len(fire_list)} fires from queue (keeping in queue until completion)")

#         # Send initial status for all fires
#         for idx, fire in enumerate(fire_list):
#             try:
#                 await self.websocket.send(json.dumps({
#                     "type": "extinguish_mission_status",
#                     "drone_id": DRONE_ID,
#                     "status": "en_route",
#                     "target_lat": fire["target_lat"],
#                     "target_lon": fire["target_lon"],
#                     "detection_id": fire.get("detection_id"),
#                     "mission_id": fire.get("mission_id"),
#                     "queue_remaining": len(fire_list) - idx - 1
#                 }))
#             except Exception as e:
#                 print(f"Failed to send status update: {e}")

#         # Convert fire_list to GPS coordinates format for go_align_drop
#         # go_align_drop expects list of tuples: (lat, lon, alt)
#         gps_list = [
#             (fire["target_lat"], fire["target_lon"], 15.0)  # Use 15m altitude
#             for fire in fire_list
#         ]

#         # Execute batch fire extinguishing using proven go_align_drop
#         try:
#             # Create DroneController wrapper around our MAVSDK drone
#             # IMPORTANT: We reuse the existing connected drone to avoid breaking telemetry
#             drone_controller = DroneController(connection_string="")
#             drone_controller.drone = self.drone  # Use our already-connected drone

#             # Share telemetry data from main script instead of creating duplicate subscriptions
#             # This prevents MAVSDK connection from breaking due to multiple telemetry streams
#             print("🔧 Sharing telemetry data with mission controller...")

#             # Create a task to continuously update DroneController with our telemetry
#             async def share_telemetry():
#                 while drone_controller == self.current_mission:
#                     # Share position from main script's telemetry
#                     if "gps" in self.latest:
#                         if "alt_rel_m" in self.latest["gps"]:
#                             drone_controller.altitude = self.latest["gps"]["alt_rel_m"]
#                         if "lat" in self.latest["gps"]:
#                             drone_controller.last_lat = self.latest["gps"]["lat"]
#                         if "lon" in self.latest["gps"]:
#                             drone_controller.last_lon = self.latest["gps"]["lon"]

#                     # Share flight mode
#                     if "status" in self.latest and "flight_mode" in self.latest["status"]:
#                         drone_controller.last_flight_mode = str(self.latest["status"]["flight_mode"])

#                     await asyncio.sleep(0.1)  # Update 10 times per second

#             asyncio.create_task(share_telemetry())

#             # Store reference to DroneController for cancel/RTL commands
#             self.current_mission = drone_controller

#             # REUSE existing SIYICam instance instead of creating new one
#             print("📷 Using shared camera interface...")
            
#             # Make sure camera is started (it should be)
#             if not self.siyi_cam.running:
#                  print("⚠️ Camera thread not running, starting it...")
#                  self.siyi_cam.start()

#             print(f"🚁 Executing go_align_drop for {len(gps_list)} fire locations...")

#             # Execute the proven go_align_drop function (handles camera init)
#             # Pass our existing threaded camera instance
#             await go_align_drop(drone_controller, self.siyi_cam, gps_list, approach_alt=15.0)

#             # Mark all as successful (go_align_drop handles each internally)
#             results = [
#                 {
#                     "success": True,
#                     "detection_id": fire.get("detection_id"),
#                     "mission_id": fire.get("mission_id"),
#                     "failure_reason": None
#                 }
#                 for fire in fire_list
#             ]

#         except Exception as e:
#             print(f"❌ go_align_drop failed: {e}")
#             import traceback
#             traceback.print_exc()
#             # Mark all as failed
#             results = [
#                 {
#                     "success": False,
#                     "detection_id": fire.get("detection_id"),
#                     "mission_id": fire.get("mission_id"),
#                     "failure_reason": str(e)
#                 }
#                 for fire in fire_list
#             ]

#         # Send completion status for each fire
#         for idx, (fire, result) in enumerate(zip(fire_list, results)):
#             try:
#                 status_data = {
#                     "type": "extinguish_mission_status",
#                     "drone_id": DRONE_ID,
#                     "status": "completed" if result["success"] else "failed",
#                     "target_lat": fire["target_lat"],
#                     "target_lon": fire["target_lon"],
#                     "detection_id": result["detection_id"],
#                     "mission_id": result["mission_id"],
#                     "queue_remaining": 0
#                 }
#                 if result["failure_reason"]:
#                     status_data["failure_reason"] = result["failure_reason"]

#                 await self.websocket.send(json.dumps(status_data))

#                 if result["success"]:
#                     print(f"✅ Fire {idx + 1} extinguished successfully!")
#                 else:
#                     print(f"❌ Fire {idx + 1} extinguishing failed: {result['failure_reason']}")

#             except Exception as e:
#                 print(f"Failed to send completion status: {e}")

#         # Clear processed fires from queue and cleanup
#         async with self.mission_lock:
#             # Remove processed fires from queue
#             for fire in fire_list:
#                 try:
#                     self.fire_queue.remove(fire)
#                 except ValueError:
#                     pass  # Already removed

#             self.current_mission = None
#             print(f"🧹 Cleared {len(fire_list)} fires from queue, {len(self.fire_queue)} remaining")

#         successful_count = sum(1 for r in results if r["success"])
#         print(f"\n✅ Batch queue completed! {successful_count}/{len(fire_list)} fires extinguished.\n")

#     async def handle_signaling(self, data):
#         typ = data.get("type")
#         if typ == "webrtc_answer":
#             # Add this check:
#             if self.pc.signalingState == "stable":
#                 print("⏭️  Ignoring duplicate webrtc_answer (already in stable state)")
#                 return
#             await self.pc.setRemoteDescription(RTCSessionDescription(sdp=data["sdp"], type="answer"))
#         elif typ == "ice_candidate":
#             cand = data.get("candidate", {})
#             candidate_str = cand.get("candidate", "")
#             if candidate_str:
#                 ice = candidate_from_sdp(candidate_str)
#                 ice.sdpMid = cand.get("sdpMid")
#                 ice.sdpMLineIndex = cand.get("sdpMLineIndex")
#                 await self.pc.addIceCandidate(ice)

#     async def send_webrtc_offer(self):
#         """Create and send a fresh WebRTC offer"""
#         try:
#             print(f"🎥 Creating new WebRTC offer... (current state: {self.pc.connectionState})")

#             # Check if peer connection is not in a good state and recreate if needed
#             # On refresh, connection goes to "disconnected" not "closed"
#             if self.pc.connectionState in ("closed", "failed", "disconnected"):
#                 print(f"♻️  RTCPeerConnection is {self.pc.connectionState}, recreating...")
#                 await self.start_webrtc()

#             offer = await self.pc.createOffer()
#             await self.pc.setLocalDescription(offer)
#             msg = {"type": "webrtc_offer", "drone_id": DRONE_ID, "sdp": self.pc.localDescription.sdp}
#             await self.websocket.send(json.dumps(msg))
#             print("✅ WebRTC offer sent")
#         except Exception as e:
#             print(f"❌ Failed to send WebRTC offer: {e}")
#             # Try to recover by recreating the peer connection
#             try:
#                 print("🔄 Attempting to recover by recreating peer connection...")
#                 await self.start_webrtc()
#                 offer = await self.pc.createOffer()
#                 await self.pc.setLocalDescription(offer)
#                 msg = {"type": "webrtc_offer", "drone_id": DRONE_ID, "sdp": self.pc.localDescription.sdp}
#                 await self.websocket.send(json.dumps(msg))
#                 print("✅ WebRTC offer sent after recovery")
#             except Exception as e2:
#                 print(f"❌ Recovery failed: {e2}")

#     async def run(self):
#         try:
#             # 1. Authenticate
#             await self.authenticate()

#             # 2. Connect to MAVSDK
#             await self.connect_mavsdk()
            
#             # 3. Setup Camera (Before WebRTC)
#             try:
#                 print("🎥 Setting up SIYI camera...")
#                 self.siyi_cam.setup_cam()  # This might block for 1-2 seconds
#                 self.siyi_cam.start()      # Start the background thread
#                 print("✅ Camera thread started")
#             except Exception as e:
#                 print(f"❌ Failed to start camera: {e}")

#             # 4. Connect WebSocket
#             ws_url = f"{WS_URL}/ws/drone/{DRONE_ID}?token={self.token}"
#             print("Connecting to WebSocket:", ws_url)
#             async with websockets.connect(ws_url) as ws:
#                 self.websocket = ws
#                 print(" WebSocket connected")

#                 # 5. Setup WebRTC for video feed
#                 await self.start_webrtc()

#                 if self.pc:
#                     offer = await self.pc.createOffer()
#                     await self.pc.setLocalDescription(offer)
#                     msg = {"type": "webrtc_offer", "drone_id": DRONE_ID, "sdp": self.pc.localDescription.sdp}
#                     await ws.send(json.dumps(msg))
#                     print("🎥 WebRTC offer sent")

#                 # 6. Start telemetry
#                 asyncio.create_task(self.send_telemetry())

#                 # 6.5. Start altitude monitoring for payload drones
#                 if self.drone_type == "rescue":
#                     asyncio.create_task(self.fire_extinguisher.monitor_altitude())

#                 # 6.6. Start WebSocket heartbeat to prevent timeout
#                 asyncio.create_task(self.send_heartbeat())

#                 # 7. Message receive loop
#                 async for message in ws:
#                     try:
#                         data = json.loads(message)
#                     except Exception as e:
#                         print(" Invalid JSON from WebSocket:", e)
#                         continue

#                     typ = data.get("type")
#                     print(f"📨 WS Message received: type='{typ}'")

#                     if typ in ("webrtc_answer", "ice_candidate"):
#                         await self.handle_signaling(data)
#                     elif typ == "request_offer":
#                         print("📞 UI requesting WebRTC offer")
#                         await self.send_webrtc_offer()
#                     elif typ == "command":
#                         await self.handle_command(data)
#                     elif typ == "mission_start":
#                         await self.handle_mission_start(data)
#                     elif typ == "survey_mission":
#                         await self.handle_survey_mission(data)
#                     elif typ == "gimbal_control":
#                         await self.handle_gimbal_control(data)
#                     elif typ == "video_quality":
#                         await self.handle_video_quality_change(data)
#                     elif typ == "extinguish_fire":
#                         await self.handle_extinguish_fire(data)
#                     else:
#                         print("❌ Unknown WebSocket message type:", typ)

#         except websockets.exceptions.ConnectionClosed as e:
#             # WebSocket connection closed (backend down/restarted)
#             print(f"\n{'='*60}", flush=True)
#             print(f"🔌 WebSocket connection closed", flush=True)
#             print(f"{'='*60}", flush=True)
#             print(f"Reason: {e.rcvd.reason if e.rcvd else 'Unknown'}", flush=True)
#             print(f"Code: {e.rcvd.code if e.rcvd else 'Unknown'}", flush=True)
#             print(f"\n⚠️  Backend may have restarted or connection was lost", flush=True)
#             print(f"Exiting with code {EXIT_BACKEND_UNAVAILABLE} for reconnection...", flush=True)
#             print(f"{'='*60}\n", flush=True)

#             # Force cleanup before exit
#             try:
#                 asyncio.run(self.cleanup())
#             except:
#                 pass

#             # Ensure we actually exit
#             import os
#             os._exit(EXIT_BACKEND_UNAVAILABLE)

#         except aiohttp.ClientError as e:
#             # HTTP connection error (authentication failed, backend unreachable)
#             print(f"\n{'='*60}", flush=True)
#             print(f"❌ HTTP Connection Error", flush=True)
#             print(f"{'='*60}", flush=True)
#             print(f"Error: {e}", flush=True)
#             print(f"\n⚠️  Cannot connect to backend", flush=True)
#             print(f"Exiting with code {EXIT_BACKEND_UNAVAILABLE}...", flush=True)
#             print(f"{'='*60}\n", flush=True)

#             import os
#             os._exit(EXIT_BACKEND_UNAVAILABLE)

#         except Exception as e:
#             # Unknown error
#             print(f"\n{'='*60}", flush=True)
#             print(f"❌ UNEXPECTED ERROR", flush=True)
#             print(f"{'='*60}", flush=True)
#             print(f"Error: {e}", flush=True)
#             print(f"Type: {type(e).__name__}", flush=True)
#             import traceback
#             traceback.print_exc()
#             print(f"\n⚠️  Critical error occurred", flush=True)
#             print(f"Exiting with code {EXIT_CRITICAL_ERROR}...", flush=True)
#             print(f"{'='*60}\n", flush=True)

#             import os
#             os._exit(EXIT_CRITICAL_ERROR)

#     async def cleanup(self):
#         """Cleanup resources properly"""
#         print("🧹 Cleaning up resources...")
        
#         # Stop camera thread
#         if hasattr(self, 'siyi_cam'):
#             print("🛑 Stopping camera thread...")
#             self.siyi_cam.stop()

#         # Close peer connection
#         if self.pc is not None:
#             try:
#                 await self.pc.close()
#                 print("✅ RTCPeerConnection closed")
#             except Exception as e:
#                 print(f"⚠️  Error closing peer connection: {e}")

#         # Stop video track
#         if self.video_track is not None:
#             try:
#                 self.video_track.stop()
#                 print("✅ Video track stopped")
#             except Exception:
#                 pass

#         # Close WebSocket
#         if self.websocket is not None:
#             try:
#                 await self.websocket.close()
#                 print("✅ WebSocket closed")
#             except Exception:
#                 pass

#         # Cancel telemetry tasks
#         for task in self.telemetry_tasks:
#             try:
#                 task.cancel()
#             except Exception:
#                 pass

#         print("✅ Cleanup complete")


# if __name__ == "__main__":
#     client = DroneClient()
#     try:
#         asyncio.run(client.run())
#     except KeyboardInterrupt:
#         print("\n⚠️  Keyboard interrupt received")
#         # Run cleanup
#         try:
#             asyncio.run(client.cleanup())
#         except Exception as e:
#             print(f"Cleanup error: {e}")
#         print("👋 Drone client exited")