print("Code Started")

import asyncio
import json
import math
import time
import sys
import websockets
import aiohttp
import numpy as np
import base64
import os
import threading
import grpc  # CRITICAL: Needed to catch MAVSDK crashes
import psutil
import logging
from pathlib import Path
from collections import defaultdict, deque

sys.path.insert(0, "/home/nanosuper/opencv_build/build/lib/python3/")
import cv2

from aiortc import RTCPeerConnection, VideoStreamTrack, RTCSessionDescription
from aiortc.sdp import candidate_from_sdp
from av import VideoFrame
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.telemetry import FixType

# === Custom Imports ===
from test2 import compute_spacing_from_camera
from test2 import PreciseSurveyPlanner
from fire_extinguish import FireExtinguisher
from go_align_drop_robust import go_align_drop, DroneController
from siyi_cam2 import SIYICam
from aruco_detector import ArucoFireDetector
from fire_detection_test_model import FireDetector
# from fire_detector_minimal import FireDetector
from video_quality import VideoQualityProfile

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=["fire", "aruco"], default="aruco")
args = parser.parse_args()

DETECTION_MODE = args.mode
print(f"🔥 Detection Mode Selected: {DETECTION_MODE}")


# === Exit Codes ===
EXIT_NETWORK_FAILURE = 10
EXIT_BACKEND_UNAVAILABLE = 11
EXIT_CRITICAL_ERROR = 12

# === Global Camera Instance ===
# Initialize globally to share between threads
# siyi_cam_ = SIYICam(telemetry_provider=lambda: TelemetrySnapshot().get())
siyi_cam_ = SIYICam()
siyi_cam_.setup_cam()

# === Logging Configuration ===
# Suppress verbose logs from libraries caused by SIYI SDK's root logger config
logging.getLogger("aiortc").setLevel(logging.WARNING)
logging.getLogger("aioice").setLevel(logging.WARNING)
logging.getLogger("av").setLevel(logging.WARNING)


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
#         self.frame_timeout = frame_timeout
#         self.retry_delay = retry_delay
#         self.total_failures = 0
#         self.last_health_check = time.time()

#     def on_frame_sent_successfully(self):
#         self.last_frame_sent = time.time()
#         self.frames_sent_count += 1
#         self.consecutive_send_failures = 0

#     def on_frame_send_failed(self, error=None):
#         self.consecutive_send_failures += 1
#         self.total_failures += 1
        
#         # Only log every 10th failure to avoid console spam
#         if self.consecutive_send_failures % 10 == 0:
#             print(f"⚠️  Frame send failure #{self.consecutive_send_failures} (total: {self.total_failures})")
#             if error: print(f"    Error: {error}")

#         if self.consecutive_send_failures >= self.max_consecutive_failures:
#             print(f"\n{'='*60}")
#             print(f"❌ NETWORK FAILURE DETECTED")
#             print(f"{'='*60}")
#             print(f"Consecutive frame send failures: {self.consecutive_send_failures}")
#             print(f"\n🔄 Waiting {self.retry_delay}s before restart...")
#             time.sleep(self.retry_delay)
#             sys.exit(EXIT_NETWORK_FAILURE)

#     def check_frame_timeout(self):
#         time_since_last_frame = time.time() - self.last_frame_sent
#         if self.frames_sent_count > 0 and time_since_last_frame > self.frame_timeout:
#             print(f"\n⚠️  VIDEO STREAM STALLED: No frames sent in {time_since_last_frame:.1f}s")

#     def periodic_health_check(self):
#         now = time.time()
#         if now - self.last_health_check > 10:
#             self.last_health_check = now
#             self.check_frame_timeout()


class VideoHealthMonitor:
    """
    Monitors video frame transmission health.
    Keep-alive strategy: Just log health, don't restart connections.
    """
    def __init__(self, camera=None):
        self.last_frame_sent = time.time()
        self.frames_sent_count = 0
        self.consecutive_send_failures = 0
        self.total_failures = 0
        self.last_health_check = time.time()
        self.camera = camera  # Camera reference for health status

    def on_frame_sent_successfully(self):
        self.last_frame_sent = time.time()
        self.frames_sent_count += 1
        self.consecutive_send_failures = 0

    def on_frame_send_failed(self, error=None):
        self.consecutive_send_failures += 1
        self.total_failures += 1
        
        # Only log every 20th failure to avoid console spam
        if self.consecutive_send_failures % 20 == 0:
            camera_status = "connected" if (self.camera and self.camera.is_connected) else "disconnected"
            print(f"⚠️  Frame send failure #{self.consecutive_send_failures} (camera: {camera_status})")
            if error: print(f"    Error: {error}")

    def check_frame_timeout(self):
        """Check if video stream has stalled - just log, don't restart."""
        time_since_last_frame = time.time() - self.last_frame_sent
        if self.frames_sent_count > 0 and time_since_last_frame > 30:  # 30s timeout
            camera_status = "connected" if (self.camera and self.camera.is_connected) else "disconnected"
            print(f"\n📹 VIDEO STREAM STALLED: {time_since_last_frame:.0f}s (camera: {camera_status})")
            self.last_frame_sent = time.time()  # Reset timer to avoid spam

    def periodic_health_check(self):
        now = time.time()
        if now - self.last_health_check > 10:
            self.last_health_check = now
            self.check_frame_timeout()

class TelemetryHealthMonitor:
    """Monitors telemetry transmission health."""
    def __init__(self, max_consecutive_failures=20):
        self.consecutive_send_failures = 0
        self.max_consecutive_failures = max_consecutive_failures
        self.total_sent = 0
        self.total_failures = 0

    def on_telemetry_sent_successfully(self):
        self.consecutive_send_failures = 0
        self.total_sent += 1

    def on_telemetry_send_failed(self, error=None):
        self.consecutive_send_failures += 1
        self.total_failures += 1
        # Reduced failures before warning/reset
        if self.consecutive_send_failures >= 5: 
            print(f"\n❌ TELEMETRY TRANSMISSION FAILURE: Connection lost (Will auto-reconnect via Main Loop)")
            # REMOVED: sys.exit(EXIT_BACKEND_UNAVAILABLE) - Let the main loop handle reconnection
            self.consecutive_send_failures = 0 # Reset to avoid log spam

# === Configuration ===
BACKEND_URL = os.getenv("BACKEND_URL", "http://localhost:8000")
WS_URL = os.getenv("WS_URL", "ws://localhost:8000")
DRONE_ID = os.getenv("DRONE_ID", "austin-recon-01")
STREAM_TYPE = os.getenv("STREAM_TYPE", "direct")  # "direct" or "rf_relay"
SECRET_KEY = os.getenv("SECRET_KEY", "drone_aus_123")
DRONE_TYPE = os.getenv("DRONE_TYPE", "recon")
USE_DUMMY_TELEMETRY = False

DRONE_POSITIONS = {
    "austin-recon-01": (22.5726, 88.3639),
    "austin-recon-02": (22.5800, 88.3700),
    "austin-recon-03": (22.5650, 88.3580),
}
DEFAULT_START_LAT = 22.5726
DEFAULT_START_LNG = 88.3639
DUMMY_START_LAT, DUMMY_START_LNG = DRONE_POSITIONS.get(DRONE_ID, (DEFAULT_START_LAT, DEFAULT_START_LNG))

def quaternion_to_euler(w, x, y, z):
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
    """
    THREADED WebcamVideoTrack. 
    Instead of running cv2.VideoCapture itself, it pulls the latest frame 
    from the global `siyi_cam` instance which runs in a background thread.
    """
    def __init__(self, siyi_cam_instance, detection_callback=None, quality_profile="720p_24", video_health_monitor=None):
        super().__init__()
        self.detection_callback = detection_callback
        self.current_quality = quality_profile
        self.video_health_monitor = video_health_monitor
        self.siyi_cam = siyi_cam_instance
        
        self.frame_counter = 0
        self.detection_skip_frames = 0
        self.use_dummy = False
        print(f"✅ WebcamVideoTrack initialized (Profile: {quality_profile})")

    def change_quality(self, quality_profile):
        # NOTE: Dynamic pipeline changing requires stopping/starting the thread in siyi_cam2.
        # For now we just acknowledge the request.
        if quality_profile == self.current_quality:
            print(f"⏭️  Already using {quality_profile}")
            return
        print(f"⚠️  Quality change to {quality_profile} requested (Pipeline update required)")
        self.current_quality = quality_profile

    async def recv(self):
        """Return a frame from the threaded camera buffer."""
        try:
            pts, time_base = await self.next_timestamp()

            # 1. Non-blocking frame fetch from thread
            frame_bgr = self.siyi_cam.get_frame()

            # 2. Handle missing frames (initialization or camera disconnect)
            if frame_bgr is None:
                self.use_dummy = True
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "Waiting for SIYI...", (100, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            else:
                self.use_dummy = False
                frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            # 3. Object Detection (only periodically)
            self.frame_counter += 1
            if self.detection_callback and not self.use_dummy:
                if (self.detection_skip_frames == 0):
                    # Run in background to avoid blocking video stream
                    asyncio.create_task(self.detection_callback(frame_bgr))
                elif (self.frame_counter % self.detection_skip_frames == 0):
                    # Run in background to avoid blocking video stream
                    asyncio.create_task(self.detection_callback(frame_bgr))

            # 4. Create VideoFrame
            video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
            video_frame.pts = pts
            video_frame.time_base = time_base

            if self.video_health_monitor:
                self.video_health_monitor.on_frame_sent_successfully()

            return video_frame

        except Exception as e:
            if self.video_health_monitor:
                self.video_health_monitor.on_frame_send_failed(str(e))
            # Keep stream alive with black frame
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
            pts, time_base = await self.next_timestamp()
            video_frame.pts = pts
            video_frame.time_base = time_base
            return video_frame


class TelemetrySnapshot:
    """
    Thread-safe telemetry cache that updates at configurable rate.
    Prevents redundant telemetry fetches for high-FPS frame processing.
    """
    def __init__(self, update_rate_hz=5):
        self.update_interval = 1.0 / update_rate_hz
        self.last_update = 0
        self.snapshot = {}
        self.lock = threading.Lock()
        
    def update(self, telemetry_dict):
        """Called by telemetry loop to refresh snapshot"""
        with self.lock:
            self.snapshot = telemetry_dict.copy()
            self.snapshot['captured_at'] = time.time()
            self.last_update = time.time()
    
    def get(self):
        """Returns latest telemetry snapshot (thread-safe)"""
        with self.lock:
            return self.snapshot.copy()
    
    def get_age_ms(self):
        """Returns age of current snapshot in milliseconds"""
        with self.lock:
            return (time.time() - self.last_update) * 1000


class DroneClient:
    def __init__(self):
        self.websocket = None
        self.pc = None
        self.video_track = None
        self.token = None
        self.drone = System()
        self.latest = {}
        self.telemetry_tasks = []
        self.mavsdk_connected = False

        # --- Camera Setup ---
        self.cam = siyi_cam_
        print("📸 Starting SIYI Camera background thread...")
        try:
            self.cam.start()
            # Safety delay to ensure camera thread is ready before any gimbal calls
            time.sleep(2.0)
        except Exception as e:
            print(f"❌ Failed to start SIYI Camera: {e}")
        self.record_mode = False

        # Dummy telemetry state
        self.dummy_lat = DUMMY_START_LAT
        self.dummy_lng = DUMMY_START_LNG
        self.dummy_alt = 0.0
        self.dummy_battery = 100.0
        self.dummy_speed = 0.0
        self.dummy_heading = 0.0
        self.dummy_time = 0

        self.drone_type = DRONE_TYPE

        # Health monitors
        # self.video_health = VideoHealthMonitor()
        # Health monitors (keep-alive strategy - no restarts)
        self.video_health = VideoHealthMonitor(camera=self.cam)
        self.telemetry_health = TelemetryHealthMonitor()
        
        from video_quality import BandwidthMonitor
        self.bandwidth_monitor = BandwidthMonitor(check_interval=5.0)
        self.telemetry_snapshot = TelemetrySnapshot(update_rate_hz=5)
        self.cam.set_telemetry_provider(lambda: self.telemetry_snapshot.get())
        # Note: Camera state callback removed - using keep-alive strategy
    
        # Object detection
        if self.drone_type == "recon":
            if DETECTION_MODE == "fire":
                self.detector = FireDetector(
                    detection_interval=0.05
                    )
            else:            
                self.detector = ArucoFireDetector(
                    dict_type=cv2.aruco.DICT_4X4_50,
                    marker_size=0.15,
                    detection_interval=.05
                    )
            print(f"🔍 Surveillance drone - Object detection ENABLED")
        else:
            self.detector = None
            print(f"📦 Payload drone - Object detection DISABLED")

        # Performance tracking for frame processing
        self.process = psutil.Process(os.getpid())
        self.detection_perf_stats = {
            "frames_received": 0,
            "detections_sent": 0,
            "frame_encode_times": deque(maxlen=200),  # Bounded deque prevents memory leak
            "websocket_send_times": deque(maxlen=200),
            "total_process_times": deque(maxlen=200),
            "last_stats_log_time": time.time()
        }

        # Setup file logger for detection performance
        logs_dir = Path("logs")
        logs_dir.mkdir(exist_ok=True)

        self.detection_stats_logger = logging.getLogger("drone_detection_stats")
        self.detection_stats_logger.setLevel(logging.INFO)
        self.detection_stats_logger.handlers = []

        file_handler = logging.FileHandler(logs_dir / "drone_detection_performance.log")
        file_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
        self.detection_stats_logger.addHandler(file_handler)

        if self.drone_type == "recon":
            print(f"📊 Drone Detection Performance Logging Enabled → logs/drone_detection_performance.log")

        # Restart lock to prevent multiple simultaneous restart attempts
        self.restart_lock = asyncio.Lock()
        self.is_restarting = False

        # Rescue mode logic
        if self.drone_type == "rescue":
            self.fire_queue = []
            self.current_mission = None
            self.is_airborne = False
            self.mission_lock = asyncio.Lock()
            self.fire_extinguisher = FireExtinguisher(
                drone=self.drone,
                use_dummy_telemetry=USE_DUMMY_TELEMETRY,
                position_update_callback=self.update_dummy_position
            )
            print(f"🔥 Payload drone - Extinguish mission handler ENABLED")

    def update_dummy_position(self, lat, lon, alt):
        self.dummy_lat = lat
        self.dummy_lng = lon
        self.dummy_alt = alt
    

    async def authenticate(self):
        async with aiohttp.ClientSession() as session:
            try:
                async with session.post(f"{BACKEND_URL}/api/v1/auth/drone", json={
                    "drone_id": DRONE_ID,
                    "secret_key": SECRET_KEY
                }) as resp:
                    if resp.status != 200:
                        print(f"❌ Auth failed: {resp.status}")
                        sys.exit(EXIT_BACKEND_UNAVAILABLE)
                    data = await resp.json()
                    self.token = data.get("access_token")
                    print("✅ Authenticated, token received")
            except Exception as e:
                print(f"❌ Auth error: {e}")
                sys.exit(EXIT_BACKEND_UNAVAILABLE)

    async def connect_mavsdk(self):
        if USE_DUMMY_TELEMETRY:
            print("⏭️  Skipping MAVSDK connection (using dummy data)")
            self.mavsdk_connected = True
            return

        print("🔌 Connecting to MAVSDK...")
        try:
            # Connect to localhost UDP for MAVSDK Server
            # await self.drone.connect(system_address="udpin://0.0.0.0:14540")
            await self.drone.connect(system_address="serial:///dev/ttyACM0:57600")
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    print("✅ MAVSDK connected")
                    self.mavsdk_connected = True
                    break
        except Exception as e:
            print(f"❌ Initial MAVSDK connection failed: {e}")
            self.mavsdk_connected = False

    async def restart_mavsdk(self):
        """
        CRITICAL: Re-initializes the drone system if the server crashes.
        Uses a lock to prevent multiple simultaneous restart attempts.
        """
        # Check if restart is already in progress
        if self.is_restarting:
            print("⚠️  MAVSDK restart already in progress, skipping duplicate restart")
            return

        # Acquire lock to ensure only one restart happens at a time
        async with self.restart_lock:
            # Double-check after acquiring lock
            if self.is_restarting:
                return

            self.is_restarting = True

            try:
                print("\n" + "="*50)
                print("🚨 DETECTED MAVSDK CRASH (Socket Closed)")
                print("🔄 RESTARTING MAVSDK CONNECTION...")
                print("="*50 + "\n")

                self.mavsdk_connected = False

                # 1. Cancel existing telemetry tasks
                for task in self.telemetry_tasks:
                    task.cancel()
                self.telemetry_tasks = []

                # 2. Destroy the old drone object (which holds the broken channel)
                #    and create a new one.
                del self.drone
                self.drone = System()

                # 3. Update references in sub-modules
                if self.drone_type == "rescue":
                    self.fire_extinguisher.drone = self.drone

                # 4. Reconnect
                await self.connect_mavsdk()

                # 5. Restart Telemetry
                if self.mavsdk_connected:
                    asyncio.create_task(self.send_telemetry())
                    print("✅ MAVSDK restart completed successfully")
                else:
                    print("⚠️  MAVSDK restart completed but connection not established")

            finally:
                self.is_restarting = False

    async def start_webrtc(self):
        if self.pc is not None:
            try: await self.pc.close()
            except: pass
        if self.video_track is not None:
            try: self.video_track.stop()
            except: pass

        # Create fresh track using the SHARED camera instance
        self.video_track = WebcamVideoTrack(
            siyi_cam_instance=self.cam,
            detection_callback=self.process_frame_for_detection if self.drone_type == "recon" else None,
            quality_profile="720p_24",
            video_health_monitor=self.video_health
        )
        self.pc = RTCPeerConnection()
        self.pc.addTrack(self.video_track)

        @self.pc.on("icecandidate")
        async def on_ice(candidate):
            if candidate and self.websocket:
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
                
        @self.pc.on("connectionstatechange")
        async def on_connection_state_change():
            """Monitor WebRTC connection state and auto-restart on failure"""
            state = self.pc.connectionState
            print(f"🔗 WebRTC Connection State: {state}")
            
            if state == "connected":
                print("✅ WebRTC connected successfully")
                # Reset restart counter on successful connection
                self.video_health.restart_count = 0
                
                    
            elif state == "failed":
                print("❌ WebRTC connection failed")
                # Trigger restart - Network failure (ICE failed)
                asyncio.create_task(self.restart_webrtc())
                    
            elif state == "disconnected":
                print("⚠️  WebRTC disconnected - waiting to reconnect...")

        @self.pc.on("iceconnectionstatechange")
        async def on_ice_state_change():
            """Monitor ICE connection state"""
            ice_state = self.pc.iceConnectionState
            
            if ice_state == "failed":
                print(f"❌ ICE connection failed - triggering restart")
                # Trigger restart - Network failure
                asyncio.create_task(self.restart_webrtc())
            
            elif ice_state == "disconnected":
                print(f"⚠️  ICE disconnected - may reconnect automatically")
            
            elif ice_state == "connected":
                print(f"✅ ICE connected")


    async def restart_webrtc(self):
        """
        Restart WebRTC connection (ONLY for network failures).
        Does not affect camera stream.
        """
        if self.is_restarting: return
        self.is_restarting = True
        
        print("\n" + "="*60)
        print("🔄 RESTARTING WEBRTC (Network Failure)")
        print("="*60)
        
        try:
            if self.pc:
                try: await self.pc.close()
                except: pass
            
            # Small delay
            await asyncio.sleep(1)
            
            # Recreate connection
            print("1️⃣  Creating new WebRTC connection...")
            await self.start_webrtc()
            
            print("2️⃣  Sending new WebRTC offer...")
            await self.send_webrtc_offer()
            
            print("✅ WebRTC network restart complete")
            print("="*60 + "\n")
            
        except Exception as e:
            print(f"❌ WebRTC restart failed: {e}")
        finally:
            self.is_restarting = False

    async def monitor_bandwidth(self):
        """
        Monitor bandwidth usage and adapt video quality.
        Switch to sub-stream (480p) on low bandwidth.
        """
        print("📉 Bandwidth adaptation loop started")
        current_profile = "720p_24"  # Start assumption
        
        while True:
            try:
                # 1. Get current estimated bandwidth
                bw_kbps = self.bandwidth_monitor.get_current_bandwidth()
                
                # 2. Determine best profile
                best_profile = VideoQualityProfile.get_profile_for_bandwidth(bw_kbps)
                
                # 3. Apply change if needed (with hysteresis)
                if best_profile != current_profile:
                    # Only downgrade if significantly lower, or upgrade if significantly higher
                    # Simple logic: just apply it for now, can add hysteresis later
                    print(f"📉 Bandwidth: {bw_kbps:.0f} kbps -> Switching to {best_profile}")
                    
                    self.cam.set_quality(best_profile)
                    current_profile = best_profile
                
                # Check status periodically
                if self.websocket and self.websocket.open:
                    # Update bandwidth monitor with fake data for now since we don't have real transport stats hooked up easily
                    # In a real impl, we'd read RTCP stats. For now, we assume bandwidth is sufficient unless specified.
                    pass 

            except Exception as e:
                print(f"⚠️ Bandwidth monitor error: {e}")
            
            await asyncio.sleep(5.0)  # Check every 5 seconds

    def _generate_dummy_telemetry(self):
        self.dummy_time += 1
        radius = 0.0001
        angle = (self.dummy_time * 0.1) % (2 * math.pi)
        
        # Only move dummy pos if airborne or we want it to move
        if self.is_airborne or self.drone_type == "recon":
            self.dummy_lat = DUMMY_START_LAT + radius * math.cos(angle)
            self.dummy_lng = DUMMY_START_LNG + radius * math.sin(angle)
            self.dummy_alt = 50 + 10 * math.sin(self.dummy_time * 0.05)
            self.dummy_speed = 5 + 2 * math.sin(self.dummy_time * 0.1)
            self.dummy_heading = (angle * 180 / math.pi) % 360
        
        return {
            "lat": self.dummy_lat, "lng": self.dummy_lng,
            "alt": self.dummy_alt, "battery": self.dummy_battery,
            "speed": self.dummy_speed, "heading": self.dummy_heading,
            "gps": {
                "lat": self.dummy_lat, "lon": self.dummy_lng, 
                "alt_abs_m": self.dummy_alt + 400, "alt_rel_m": self.dummy_alt
            },
            "orientation": {"roll": 2*math.sin(self.dummy_time*0.2), "pitch": 3*math.cos(self.dummy_time*0.15), "yaw": self.dummy_heading},
            "speed_detail": {"hor_m_s": self.dummy_speed, "ver_m_s": 0.5},
            "battery_detail": {"voltage_v": 12.6, "remaining_percent": self.dummy_battery},
            "status": {"flight_mode": "POSCTL" if self.dummy_alt > 5 else "MANUAL", "is_armed": self.dummy_alt > 1},
            "gps_info": {"num_satellites": 10, "fix_type": 3}
        }

    # --- Robust Telemetry Readers ---
    # These wrap the MAVSDK streams. If a gRPC error occurs (Socket Closed), 
    # they trigger restart_mavsdk().
    
    async def _reader_wrapper(self, iterator, key, processor_func):
        try:
            async for item in iterator:
                processor_func(key, item)
        except grpc.aio.AioRpcError:
            print(f"⚠️ MAVSDK Stream Error on {key}: Connection refused/closed.")
            # Only trigger restart if not already restarting and was previously connected
            if self.mavsdk_connected and not self.is_restarting:
                asyncio.create_task(self.restart_mavsdk())
        except Exception:
            pass

    async def _position_reader(self):
        def process(k, pos):
            self.latest.setdefault("gps", {})
            self.latest["gps"].update({
                "lat": pos.latitude_deg, "lon": pos.longitude_deg,
                "alt_abs_m": pos.absolute_altitude_m, "alt_rel_m": pos.relative_altitude_m,
            })
        await self._reader_wrapper(self.drone.telemetry.position(), "gps", process)

    async def _attitude_reader(self):
        def process(k, e):
            self.latest["orientation"] = {"roll": e.roll_deg, "pitch": e.pitch_deg, "yaw": e.yaw_deg}
        await self._reader_wrapper(self.drone.telemetry.attitude_euler(), "att", process)

    async def _velocity_reader(self):
        def process(k, v):
            hor = math.hypot(v.velocity_north_m_s, v.velocity_east_m_s)
            self.latest["speed"] = {"hor_m_s": hor, "ver_m_s": v.velocity_down_m_s}
        await self._reader_wrapper(self.drone.telemetry.velocity_ned(), "vel", process)

    async def _battery_reader(self):
        def process(k, b):
            self.latest["battery"] = {"voltage_v": b.voltage_v, "remaining_percent": b.remaining_percent}
        await self._reader_wrapper(self.drone.telemetry.battery(), "bat", process)

    async def _status_reader(self):
        # Wrappers for flight mode and armed status
        async def f_mode():
            try:
                async for fm in self.drone.telemetry.flight_mode():
                    self.latest.setdefault("status", {})["flight_mode"] = str(fm)
            except grpc.aio.AioRpcError:
                # Only trigger restart if not already restarting and was previously connected
                if self.mavsdk_connected and not self.is_restarting:
                    asyncio.create_task(self.restart_mavsdk())
        async def f_armed():
            try:
                async for armed in self.drone.telemetry.armed():
                    self.latest.setdefault("status", {})["is_armed"] = bool(armed)
            except grpc.aio.AioRpcError:
                pass
        await asyncio.gather(f_mode(), f_armed())

    async def _imu_reader(self):
        def process(k, imu):
            self.latest["imu"] = {
                "accel": [imu.acceleration_frd.x_m_s2, imu.acceleration_frd.y_m_s2, imu.acceleration_frd.z_m_s2],
                "gyro": [imu.angular_velocity_frd.x_rad_s, imu.angular_velocity_frd.y_rad_s, imu.angular_velocity_frd.z_rad_s],
                "mag": [imu.magnetic_field_frd.x_ga, imu.magnetic_field_frd.y_ga, imu.magnetic_field_frd.z_ga]
            }
        await self._reader_wrapper(self.drone.telemetry.imu(), "imu", process)

    async def _gps_info_reader(self):
        def process(k, info):
            fix_map = {FixType.NO_GPS: 0, FixType.NO_FIX: 1, FixType.FIX_2D: 2, FixType.FIX_3D: 3}
            ft = getattr(info, "fix_type", 0)
            val = fix_map.get(str(ft), 3) # default to 3 if enum mismatch
            self.latest["gps_info"] = {"num_satellites": info.num_satellites, "fix_type": val}
        await self._reader_wrapper(self.drone.telemetry.gps_info(), "gps_info", process)

    async def send_telemetry(self):
        if not USE_DUMMY_TELEMETRY:
            # Wait for initial connection
            while not self.mavsdk_connected:
                await asyncio.sleep(1)
            
            print("📡 Starting Telemetry Streams...")
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
                if not USE_DUMMY_TELEMETRY and not self.mavsdk_connected:
                    await asyncio.sleep(1)
                    continue

                if USE_DUMMY_TELEMETRY:
                    dummy_data = self._generate_dummy_telemetry()
                    telemetry = {
                        "type": "telemetry", "drone_id": DRONE_ID,
                        "timestamp": int(time.time() * 1000),
                        **dummy_data
                    }
                else:
                    telemetry = {
                        "type": "telemetry", "drone_id": DRONE_ID,
                        "timestamp": int(time.time() * 1000),
                        "lat": self.latest.get("gps", {}).get("lat", 0),
                        "lng": self.latest.get("gps", {}).get("lon", 0),
                        "alt": self.latest.get("gps", {}).get("alt_rel_m", 0),
                        "battery": self.latest.get("battery", {}).get("remaining_percent", 0),
                        "speed": self.latest.get("speed", {}).get("hor_m_s", 0),
                        "heading": self.latest.get("orientation", {}).get("yaw", 0),
                        "gps": self.latest.get("gps", {}),
                        "orientation": self.latest.get("orientation", {}),
                        "speed_detail": self.latest.get("speed", {}),
                        "battery_detail": self.latest.get("battery", {}),
                        "status": self.latest.get("status", {}),
                        "imu": self.latest.get("imu", {}),
                        "gps_info": self.latest.get("gps_info", {})
                    }
                    if "num_satellites" in self.latest.get("gps_info", {}):
                         telemetry["num_satellites"] = self.latest["gps_info"]["num_satellites"]
                         telemetry["fix_type"] = self.latest["gps_info"]["fix_type"]

                self.telemetry_snapshot.update(telemetry)

                if self.websocket:
                    await self.websocket.send(json.dumps(telemetry))
                    self.telemetry_health.on_telemetry_sent_successfully()
                
                self.video_health.periodic_health_check()
                
                # Check bandwidth health
                if self.bandwidth_monitor and self.video_track:
                    # Update bandwidth monitor with recently sent bytes (approximate from video/telemetry)
                    # Note: We rely on the periodic update within bandwidth_monitor if it had deep integration,
                    # but here we check its estimated state if it was being fed data.
                    # Since we don't have per-packet byte counts here easily without hooking send,
                    # we will rely on video_health_monitor detecting stalls.
                    pass 

            except Exception as e:
                self.telemetry_health.on_telemetry_send_failed(e)
            
            await asyncio.sleep(1.0) 

    async def send_heartbeat(self):
        while True:
            try:
                if self.websocket: await self.websocket.ping()
            except: pass
            await asyncio.sleep(10.0)

    async def process_frame_for_detection(self, frame_bgr):
        if self.detector is None: return

        # Start timing
        process_start = time.time()

        # Track resource usage
        mem_before = self.process.memory_info().rss / 1024 / 1024  # MB
        cpu_before = self.process.cpu_percent(interval=None)

        try:
            # Track frames received
            self.detection_perf_stats["frames_received"] += 1

            # Get GPS coordinates
            gps = self.latest.get("gps", {})
            lat = gps.get("lat", self.dummy_lat)
            lon = gps.get("lon", self.dummy_lng)
            alt = gps.get("alt_rel_m", self.dummy_alt)

            # Run detection (timed internally in detector)
            result = self.detector.detect(frame_bgr, drone_lat=lat, drone_lon=lon, drone_alt=alt)


            if result and self.websocket:
                # Step 1: Frame encoding
                encode_start = time.time()
                _, buffer = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])  # Reduced from 95 to 85
                frame_b64 = base64.b64encode(buffer).decode('utf-8')
                del buffer  # Free JPEG buffer immediately
                encode_time = time.time() - encode_start
                self.detection_perf_stats["frame_encode_times"].append(encode_time)

                # Step 2: WebSocket send
                send_start = time.time()
                msg = {
                    "type": "object_detection",
                    "drone_id": DRONE_ID,
                    "timestamp": result["timestamp"],
                    "bbox": result["bbox"],
                    "confidence": result["confidence"],
                    "frame_data": frame_b64
                }
                await self.websocket.send(json.dumps(msg))
                send_time = time.time() - send_start
                self.detection_perf_stats["websocket_send_times"].append(send_time)

                # Explicitly free memory after send
                del frame_b64
                del msg

                # Track successful sends
                self.detection_perf_stats["detections_sent"] += 1

                # Get resource usage after
                mem_after = self.process.memory_info().rss / 1024 / 1024  # MB
                cpu_after = self.process.cpu_percent(interval=None)

                # Total time
                process_total = time.time() - process_start
                self.detection_perf_stats["total_process_times"].append(process_total)

                # Log individual detection
                print(f"📤 Detection Sent: {len(result['bbox'])} bbox | "
                      f"Encode: {encode_time*1000:.1f}ms | "
                      f"Send: {send_time*1000:.1f}ms | "
                      f"Total: {process_total*1000:.1f}ms | "
                      f"Mem: {mem_after:.1f}MB | "
                      f"CPU: {cpu_after:.1f}%")

                # Periodic stats logging
                self._log_detection_periodic_stats()

        except Exception as e:
            print(f"❌ Detection processing error: {e}")
            # Clean up any allocated memory even on error
            import gc
            gc.collect()


    def _log_detection_periodic_stats(self):
        """Log comprehensive detection processing stats every 10 seconds"""
        current_time = time.time()
        elapsed = current_time - self.detection_perf_stats["last_stats_log_time"]

        if elapsed >= 3.0:  # Log every 3 seconds
            # Calculate averages
            avg_encode_time = np.mean(self.detection_perf_stats["frame_encode_times"]) if self.detection_perf_stats["frame_encode_times"] else 0
            avg_send_time = np.mean(self.detection_perf_stats["websocket_send_times"]) if self.detection_perf_stats["websocket_send_times"] else 0
            avg_total_time = np.mean(self.detection_perf_stats["total_process_times"]) if self.detection_perf_stats["total_process_times"] else 0

            # Calculate rates
            frames_per_sec = self.detection_perf_stats["frames_received"] / elapsed if elapsed > 0 else 0
            detections_per_sec = self.detection_perf_stats["detections_sent"] / elapsed if elapsed > 0 else 0

            # Get current resource usage
            mem_current = self.process.memory_info().rss / 1024 / 1024  # MB
            cpu_percent = self.process.cpu_percent(interval=0.1)

            # Log to file
            self.detection_stats_logger.info(f"\n{'='*80}")
            self.detection_stats_logger.info(f"Drone Detection Processing Stats (last {elapsed:.1f}s):")
            self.detection_stats_logger.info(f"  Frames Received: {self.detection_perf_stats['frames_received']} ({frames_per_sec:.1f} FPS)")
            self.detection_stats_logger.info(f"  Detections Sent: {self.detection_perf_stats['detections_sent']} ({detections_per_sec:.1f}/sec)")
            self.detection_stats_logger.info(f"  Avg Frame Encode Time: {avg_encode_time*1000:.2f}ms")
            self.detection_stats_logger.info(f"  Avg WebSocket Send Time: {avg_send_time*1000:.2f}ms")
            self.detection_stats_logger.info(f"  Avg Total Processing Time: {avg_total_time*1000:.2f}ms")
            self.detection_stats_logger.info(f"  Memory Usage: {mem_current:.1f} MB")
            self.detection_stats_logger.info(f"  CPU Usage: {cpu_percent:.1f}%")

            # Console summary
            print(f"📊 Drone Stats: {frames_per_sec:.1f} FPS | {detections_per_sec:.1f} det/s | "
                  f"Encode: {avg_encode_time*1000:.1f}ms | Send: {avg_send_time*1000:.1f}ms | "
                  f"{mem_current:.1f}MB | {cpu_percent:.1f}% CPU")

            # Reset counters (deques auto-evict, no need to clear)
            self.detection_perf_stats["frames_received"] = 0
            self.detection_perf_stats["detections_sent"] = 0
            self.detection_perf_stats["last_stats_log_time"] = current_time


    # --- Command Handling ---
    async def handle_command(self, data):
        cmd = data.get("cmd")
        print(f"⚡ Command Received: {cmd}")

        # 1. Check Connection explicitly and RESPOND if missing
        if not self.mavsdk_connected and not USE_DUMMY_TELEMETRY:
            print(f"❌ Command '{cmd}' Failed: MAVSDK Disconnected")
            # CRITICAL FIX: Send error response instead of silent return
            error_msg = {
                "type": "response", 
                "drone_id": DRONE_ID,
                "cmd": cmd, 
                "status": "error", 
                "message": "Drone not connected to MAVSDK"
            }
            if self.websocket:
                await self.websocket.send(json.dumps(error_msg))
            return

        # 2. Execute Command
        try:
            if cmd == "arm":
                print("   Attempting to arm...")
                await self.drone.action.arm()
            elif cmd == "disarm":
                await self.drone.action.disarm()
            elif cmd == "takeoff":
                alt = float(data.get("height", 10.0))
                print(f"   Taking off to {alt}m...")
                await self.drone.action.set_takeoff_altitude(alt)
                await self.drone.action.takeoff()
            elif cmd == "land":
                await self.drone.action.land()
            elif cmd == "rtl":
                print("   Returning to launch...")
                if self.drone_type == "rescue" and self.current_mission:
                    self.current_mission.request_rtl()
                else:
                    await self.drone.action.return_to_launch()
            elif cmd == "cancel_mission":
                if self.drone_type == "rescue" and self.current_mission:
                    self.current_mission.cancel_mission()
            elif cmd == "start":
                print("Started Local Recording")
                self.record_mode = cmd
                siyi_cam_.start_recording()
            elif cmd == "stop":
                print("Stopped Recording.")
                self.record_mode = cmd
                siyi_cam_.stop_recording()
    
            # 3. Send Success Acknowledgement
            print(f"✅ Command '{cmd}' executed successfully")
            success_msg = {
                "type": "response", 
                "drone_id": DRONE_ID,
                "cmd": cmd, 
                "status": "ok"
            }
            await self.websocket.send(json.dumps(success_msg))

        except Exception as e:
            print(f"❌ Command '{cmd}' Error: {e}")
            # 4. Send Failure Acknowledgement
            error_msg = {
                "type": "response", 
                "drone_id": DRONE_ID,
                "cmd": cmd, 
                "status": "error", 
                "message": str(e)
            }
            await self.websocket.send(json.dumps(error_msg))

    # --- Mission Progress ---
    async def _monitor_mission_progress(self, mission_id=None):
        try:
            async for progress in self.drone.mission.mission_progress():
                if self.websocket:
                    await self.websocket.send(json.dumps({
                        "type": "mission_progress", "drone_id": DRONE_ID,
                        "mission_id": mission_id, "current": progress.current, "total": progress.total
                    }))
                if progress.current == progress.total:
                    await self.drone.action.return_to_launch()
                    break
        except Exception: pass

    async def clear_existing_mission(self):
        try:
            await self.drone.mission.clear_mission()
            return True
        except: return False

    async def start_survey_mission(self, area, altitude, speed, heading, mission_id=None):
        try:
            # (Restored full logic using PreciseSurveyPlanner)
            horizontal_fov_deg = 30.0
            vertical_fov_deg = 20.0
            transect_spacing_m, photo_interval_m, _, _ = transect_spacing_m, photo_interval_m, fw, fh = compute_spacing_from_camera(
                    altitude,
                    horizontal_fov_deg=horizontal_fov_deg,  # Must name the argument
                    vertical_fov_deg=vertical_fov_deg,      # Must name the argument
                    sidelap_fraction=0.04,               # Must name the argument
                    frontlap_fraction=0.7,             # Must name the argument
                    sensor_width_is_across_track=True       # Must name the argument
                )
            # compute_spacing_from_camera(
            #     altitude, horizontal_fov_deg, vertical_fov_deg, 0.4, 0.7, True
            # )
            
            planner = PreciseSurveyPlanner(area, altitude, transect_spacing_m, 0.0, speed, photo_interval_m, 0.1)
            mission_items = planner.generate_mission_items()
            
            await self.clear_existing_mission()
            await self.drone.mission.upload_mission(MissionPlan(mission_items))
            await self.drone.action.arm()
            await self.drone.mission.start_mission()
            
            asyncio.create_task(self._monitor_mission_progress(mission_id=mission_id))

        except Exception as e:
            print(f"Survey Error: {e}")
            await self.websocket.send(json.dumps({"type": "mission_response", "status": "error", "message": str(e)}))

    async def handle_survey_mission(self, data):
        try:
            poly_raw = data.get("polygon", [])
            if len(poly_raw) < 3: return
            polygon = [(float(p["latitude"]), float(p["longitude"])) for p in poly_raw]
            
            await self.start_survey_mission(
                polygon, float(data.get("altitude", 50)), float(data.get("speed", 5)),
                float(data.get("heading", 0)), data.get("mission_id")
            )
            
            await self.websocket.send(json.dumps({"type": "survey_mission_response", "status": "received"}))
        except Exception as e:
            print(f"Survey Handler Error: {e}")

    async def handle_mission_start(self, data):
        # Basic waypoint mission
        try:
            waypoints = data.get("waypoints", [])
            mission_items = []
            for idx, wp in enumerate(waypoints):
                mission_items.append(MissionItem(
                    idx, 3, 16, 1 if idx==0 else 0, 1, 0, 2, 0, float('nan'),
                    int(wp['lat']*1e7), int(wp['lng']*1e7), wp.get('alt', 50), 0
                ))
            await self.drone.mission_raw.upload_mission(MissionPlan(mission_items))
            await self.drone.mission.start_mission()
            await self.websocket.send(json.dumps({"type": "mission_response", "status": "started"}))
        except Exception as e:
            await self.websocket.send(json.dumps({"type": "mission_response", "status": "error", "message": str(e)}))

    # --- Gimbal & Video ---
    async def handle_gimbal_control(self, data):
        try:
            self.cam.move(yaw_deg=data.get("yaw", 0), pitch_deg=data.get("pitch", 0))
            self.cam.zoom(zoom_level=data.get("zoom", 0))
            await self.websocket.send(json.dumps({"type": "gimbal_control_response", "status": "success"}))
        except Exception as e:
            pass

    async def handle_video_quality_change(self, data):
        # Notify UI that quality change was received, even if thread limits immediate update
        await self.websocket.send(json.dumps({
            "type": "video_quality_response", "status": "success", "quality": data.get("quality")
        }))

    # --- Fire Extinguish Logic (Rescue) ---
    async def handle_extinguish_fire(self, data):
        if self.drone_type != "rescue": return
        locations = data.get("fire_locations", [])
        if not locations: return

        print(f"🔥 Queuing {len(locations)} fires")
        self.fire_queue.extend(locations)
        await self.websocket.send(json.dumps({"type": "extinguish_fire_response", "status": "queued", "count": len(locations)}))

        async with self.mission_lock:
            if not self.current_mission:
                asyncio.create_task(self.process_fire_queue())

    async def process_fire_queue(self):
        if self.drone_type != "rescue": return
        
        async with self.mission_lock:
            if not self.fire_queue: return
            
            # Check connection
            if not self.mavsdk_connected and not USE_DUMMY_TELEMETRY:
                print("❌ Cannot start mission: MAVSDK Disconnected")
                return

            print("🚀 Starting Batch Fire Mission")
            fire_list = self.fire_queue.copy()
            gps_list = [(f["target_lat"], f["target_lon"], 10.0) for f in fire_list]

            try:
                if not self.is_airborne and not USE_DUMMY_TELEMETRY:
                    print("🚁 Auto-Takeoff...")
                    await self.drone.action.arm()
                    await self.drone.action.set_takeoff_altitude(10.0)
                    await self.drone.action.takeoff()
                    await asyncio.sleep(8)
                    self.is_airborne = True

                # CRITICAL: Wrapped in Try/Except for gRPC errors
                await go_align_drop(self.drone, self.cam, gps_list)

                for f in fire_list:
                    await self.websocket.send(json.dumps({
                        "type": "extinguish_mission_status", "status": "completed", "mission_id": f.get("mission_id")
                    }))
            
            except grpc.aio.AioRpcError:
                print("❌ MISSION FAILED: Connection Lost during mission.")
                # Only trigger restart if not already restarting and was previously connected
                if self.mavsdk_connected and not self.is_restarting:
                    asyncio.create_task(self.restart_mavsdk())

            except Exception as e:
                print(f"❌ Mission Failed (General Error): {e}")

            for f in fire_list:
                if f in self.fire_queue: self.fire_queue.remove(f)
        
        self.is_airborne = False
        self.current_mission = None

    

    # --- Signaling ---
    async def handle_signaling(self, data):
        if data["type"] == "webrtc_answer":
            if self.pc.signalingState != "stable":
                desc = RTCSessionDescription(sdp=data["sdp"], type="answer")
                await self.pc.setRemoteDescription(desc)
        elif data["type"] == "ice_candidate":
            c = data["candidate"]
            if c.get("candidate"):
                ice = candidate_from_sdp(c["candidate"])
                ice.sdpMid = c.get("sdpMid")
                ice.sdpMLineIndex = c.get("sdpMLineIndex")
                await self.pc.addIceCandidate(ice)

    async def send_webrtc_offer(self):
        if self.pc.connectionState in ["closed", "failed"]:
            await self.start_webrtc()
        
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)
        await self.websocket.send(json.dumps({
            "type": "webrtc_offer", 
            "drone_id": DRONE_ID, 
            "sdp": self.pc.localDescription.sdp
        }))

    # --- Main Loop ---
    async def run(self):
        try:
            await self.authenticate()
            await self.connect_mavsdk()
            
            # Helper: Check connectivity to backend before attempting WebSocket
            # REMOVED: Relying on websockets.connect timeout instead for better switching support
            # async def check_backend_connection(): ...

            # Helper to stop tasks when connection drops
            async def stop_tasks():
                print("🛑 Stopping background tasks...")
                for t in self.telemetry_tasks: t.cancel()
                self.telemetry_tasks = []
                
                if self.video_track: 
                    try: self.video_track.stop()
                    except: pass
                    
                if self.pc: 
                    try: await self.pc.close()
                    except: pass
                
                # Close WebSocket explicitly if open to unblock any reads
                if self.websocket:
                    try: await self.websocket.close()
                    except: pass
            
            # Helper to restart tasks
            def start_tasks():
                asyncio.create_task(self.send_webrtc_offer())
                asyncio.create_task(self.send_telemetry())
                asyncio.create_task(self.send_heartbeat())
                asyncio.create_task(self.monitor_bandwidth())
                if self.drone_type == "rescue":
                    asyncio.create_task(self.fire_extinguisher.monitor_altitude())

            while True:
                try:
                    # 1. REMOVED: Fast Backend Connectivity Check
                    # Rely on websockets.connect timeout (2s) instead
                    
                    url = f"{WS_URL}/ws/drone/{DRONE_ID}?token={self.token}&stream_type={STREAM_TYPE}"
                    print(f"🌍 Connecting to {url} (stream: {STREAM_TYPE})")
                    
                    # 2. Tightened WebSocket Settings for Fast Failure Detection
                    async with websockets.connect(
                        url, 
                        ping_interval=5,   # Send ping every 5s
                        ping_timeout=3,    # Wait only 3s for pong (fail fast)
                        close_timeout=2    # Fast close
                    ) as ws:
                        self.websocket = ws
                        print("✅ WebSocket Connected")
                        
                        # Reset monitors
                        self.telemetry_health.consecutive_send_failures = 0
                        self.video_health.consecutive_send_failures = 0

                        await self.start_webrtc()
                        start_tasks() # using helper

                        async for msg in ws:
                            try:
                                data = json.loads(msg)
                                t = data.get("type")
                                if t in ["webrtc_answer", "ice_candidate"]: await self.handle_signaling(data)
                                elif t == "request_offer": await self.send_webrtc_offer()
                                elif t == "command": await self.handle_command(data)
                                elif t == "gimbal_control": await self.handle_gimbal_control(data)
                                elif t == "extinguish_fire": await self.handle_extinguish_fire(data)
                                elif t == "mission_start": await self.handle_mission_start(data)
                                elif t == "survey_mission": await self.handle_survey_mission(data)
                                elif t == "video_quality": await self.handle_video_quality_change(data)
                            except Exception as e:
                                print(f"Msg Error: {e}")
                                
                except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError, 
                        websockets.exceptions.InvalidURI, OSError, asyncio.TimeoutError) as e:
                     print(f"❌ WebSocket Connection Failed: {e}")
                     print(f"🔄 Reconnecting in 3 seconds...")
                     await stop_tasks()
                     await asyncio.sleep(3)
                     
                except Exception as e:
                    print(f"❌ Unexpected Main Loop Error: {e}")
                    import traceback
                    traceback.print_exc()
                    print(f"🔄 Reconnecting in 3 seconds...")
                    await stop_tasks()
                    await asyncio.sleep(3)

        except Exception as e:
            print(f"❌ Critical Startup Error: {e}")
            sys.exit(EXIT_BACKEND_UNAVAILABLE)

    async def cleanup(self):
        print("🛑 Shutting down...")
        try: self.cam.stop()
        except: pass
        if self.pc: await self.pc.close()
        if self.websocket: await self.websocket.close()
        for t in self.telemetry_tasks: t.cancel()

if __name__ == "__main__":
    client = DroneClient()
    try:
        asyncio.run(client.run())
    except KeyboardInterrupt:
        print("\n👋 User interrupted")
    finally:
        asyncio.run(client.cleanup())
