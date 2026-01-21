"""
Fire Extinguishing Module
Handles precision payload drop using ArUco marker alignment and visual servoing.
"""

import asyncio
import time
import cv2
import numpy as np
from collections import deque
from time import monotonic

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# Import SIYICam - must be in same directory
try:
    from siyi_cam2 import SIYICam
except ImportError:
    print("⚠️ Warning: siyi_cam module not found. Camera features will be disabled.")
    SIYICam = None


# === Configuration ===
# PD controller gains for vision-based alignment
KP_X = 2.5
KD_X = 0.5
KP_Y = 2.5
KD_Y = 0.5
KP_CLIMB = 0.6
KP_YAW = 0.8
KD_YAW = 0.1

# Velocity limits
MAX_VEL_XY = 1.0
MAX_VEL_Z = 0.5
MAX_YAW_RATE = 30.0  # deg/s

# Alignment thresholds
PIX_ERR_LIMIT = 20  # pixels - vertical motion disabled until error below this
OFFBOARD_HEIGHT = 7.0  # meters - target altitude for payload drop
X_THRESHOLD = 20  # pixels
Y_THRESHOLD = 20  # pixels
TARGET_AREA = 10000
AREA_THRESHOLD = 2000

# Stability and timing
ALIGN_STABLE_COUNT = 8  # consecutive aligned frames required
ALIGN_CHECK_HZ = 10.0  # alignment loop frequency
ALIGN_TIMEOUT = 45.0  # seconds to try aligning before giving up
APPROACH_THRESHOLD_METERS = 4.0  # when to start ArUco alignment
ARRIVAL_THRESHOLD_METERS = 1.5  # considered "arrived"

# Error averaging
AVG_ERR_TIME = 10.0
MAX_AVG_ERR = 20.0


# === Helper Functions ===
def haversine_meters(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates in meters"""
    R = 6371000.0  # Earth radius in meters
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    dphi = np.radians(lat2 - lat1)
    dlambda = np.radians(lon2 - lon1)
    a = np.sin(dphi/2.0)**2 + np.cos(phi1)*np.cos(phi2)*(np.sin(dlambda/2.0)**2)
    c = 2*np.arctan2(np.sqrt(a), np.sqrt(1-a))
    return R * c


def drop_payload():
    """Execute payload drop mechanism"""
    print("🔴 PAYLOAD DROPPED - Extinguishing fire!")
    # TODO: Add actual servo/mechanism control here
    # e.g., GPIO trigger, MAVLink command, etc.


class ArucoAlignmentDetector:
    """ArUco marker detector for precision alignment during payload drop"""
    def __init__(self, dict_type=cv2.aruco.DICT_4X4_50, marker_size=0.15):
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(dict_type)

        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            try:
                self.aruco_params = cv2.aruco.DetectorParameters()
            except Exception:
                self.aruco_params = None

        self.use_aruco_detector = False
        try:
            if hasattr(cv2.aruco, "ArucoDetector") and self.aruco_params is not None:
                self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
                self.use_aruco_detector = True
        except Exception:
            self.detector = None
            self.use_aruco_detector = False

        self.marker_size = marker_size

    def detect(self, frame):
        """Detect ArUco markers in frame"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.use_aruco_detector and self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
            return corners, ids, rejected
        else:
            if self.aruco_params is None:
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.aruco_params
                )
            return corners, ids, rejected

    def get_marker_center(self, corners):
        """Get center point and area of first detected marker"""
        if corners is None or len(corners) == 0:
            return None
        corner = corners[0][0]
        center_x = int(np.mean(corner[:, 0]))
        center_y = int(np.mean(corner[:, 1]))
        area = cv2.contourArea(corner)
        return center_x, center_y, area


class FireExtinguisher:
    """
    Handles fire extinguishing missions with GPS navigation and ArUco alignment.
    """
    def __init__(self, drone: System, use_dummy_telemetry=False, position_update_callback=None):
        self.drone = drone
        self.use_dummy_telemetry = use_dummy_telemetry
        self.position_update_callback = position_update_callback  # Callback to update parent's position
        print(f"🔧 FireExtinguisher initialized with use_dummy_telemetry={use_dummy_telemetry}")

        # Camera and alignment
        self.payload_cam = None
        self.alignment_detector = ArucoAlignmentDetector()

        # PD controller state
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_area = 0.0
        self.prev_time = None
        self.err_window = deque()

        # Offboard mode state
        self.offboard_started = False
        self.current_altitude = None

        # Dummy telemetry state (for simulation)
        self.dummy_lat = None
        self.dummy_lng = None
        self.dummy_alt = 0.0

    async def distance_to_target(self, target_lat, target_lon):
        """Calculate horizontal distance to target GPS coordinates"""
        try:
            if self.use_dummy_telemetry:
                if self.dummy_lat is None or self.dummy_lng is None:
                    return float('inf')
                return haversine_meters(self.dummy_lat, self.dummy_lng, target_lat, target_lon)
            else:
                async for pos in self.drone.telemetry.position():
                    lat = pos.latitude_deg
                    lon = pos.longitude_deg
                    return haversine_meters(lat, lon, target_lat, target_lon)
        except Exception as e:
            print(f"⚠️ Error calculating distance: {e}")
            return float('inf')

    async def start_offboard(self):
        """Start offboard mode for direct velocity control"""
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            self.offboard_started = True
            print("✅ Offboard mode started")
            return True
        except OffboardError as e:
            print(f"❌ Offboard start failed: {e}")
            self.offboard_started = False
            return False
        except Exception as e:
            print(f"❌ Offboard error: {e}")
            self.offboard_started = False
            return False

    async def stop_offboard(self):
        """Stop offboard mode"""
        try:
            await self.drone.offboard.stop()
        except Exception as e:
            print(f"⚠️ Error stopping offboard: {e}")
        self.offboard_started = False

    async def send_velocity_command(self, vx, vy, vz, yaw_rate):
        """Send velocity command in body frame"""
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(float(vx), float(vy), float(vz), float(yaw_rate))
        )

    def calculate_velocity_commands(self, frame_center, marker_center, frame_shape):
        """
        Calculate velocity commands using PD controller for visual servoing.
        Returns: vx, vy, vz, yaw_rate, is_aligned
        """
        now = monotonic()
        if self.prev_time is None:
            dt = 0.05
        else:
            dt = max(1e-3, now - self.prev_time)
        self.prev_time = now

        if marker_center is None:
            # Reset derivatives to avoid spikes on reacquisition
            self.prev_error_x = 0.0
            self.prev_error_y = 0.0
            self.prev_error_area = 0.0
            return 0.0, 0.0, 0.0, 0.0, False

        frame_cx, frame_cy = frame_center
        marker_x, marker_y, marker_area = marker_center
        frame_h, frame_w = frame_shape

        error_x = marker_x - frame_cx
        error_y = marker_y - frame_cy
        error_area = TARGET_AREA - marker_area

        norm_error_x = error_x / frame_w
        norm_error_y = error_y / frame_h

        der_x = (error_x - self.prev_error_x) / dt
        der_y = (error_y - self.prev_error_y) / dt

        # Body frame velocities (right is +Y, forward is +X)
        vy = KP_X * norm_error_x + KD_X * (der_x / frame_w)
        vy = float(np.clip(vy, -MAX_VEL_XY, MAX_VEL_XY))

        vx = -KP_Y * norm_error_y - KD_Y * (der_y / frame_h)
        vx = float(np.clip(vx, -MAX_VEL_XY, MAX_VEL_XY))

        # Yaw rate
        yaw_rate = KP_YAW * (norm_error_x * 100.0) + KD_YAW * (der_x / frame_w * 100.0)
        yaw_rate = float(np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE))

        # Vertical velocity logic
        pixel_err_mag = float(np.hypot(error_x, error_y))
        vz = 0.0

        if pixel_err_mag > PIX_ERR_LIMIT:
            # Too much horizontal error - don't climb yet
            vz = 0.0
        else:
            # Small pixel error - climb to target altitude if needed
            current_alt = self.current_altitude or self.dummy_alt
            if current_alt is not None and abs(current_alt - OFFBOARD_HEIGHT) > 0.1:
                alt_err = OFFBOARD_HEIGHT - current_alt
                vz_candidate = -KP_CLIMB * alt_err
                vz = float(np.clip(vz_candidate, -MAX_VEL_Z, MAX_VEL_Z))
            else:
                vz = 0.0

        # Check if aligned
        is_aligned = (
            abs(error_x) < X_THRESHOLD and
            abs(error_y) < Y_THRESHOLD and
            abs(error_area) < AREA_THRESHOLD
        )

        # Update previous errors
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_area = error_area

        # Update error window
        self.err_window.append((monotonic(), pixel_err_mag))
        cutoff = monotonic() - AVG_ERR_TIME
        while self.err_window and self.err_window[0][0] < cutoff:
            self.err_window.popleft()

        return vx, vy, vz, yaw_rate, is_aligned

    def get_avg_pixel_error(self):
        """Get average pixel error over recent window"""
        if not self.err_window:
            return float('inf')
        vals = [v for (_t, v) in self.err_window]
        return sum(vals) / len(vals)

    async def monitor_altitude(self):
        """Continuously monitor and update current altitude"""
        if self.use_dummy_telemetry:
            # In dummy mode, altitude is already tracked in dummy_alt
            while True:
                self.current_altitude = self.dummy_alt
                await asyncio.sleep(0.1)
        else:
            try:
                async for pos in self.drone.telemetry.position():
                    # Try different possible altitude attribute names
                    alt = None
                    if hasattr(pos, "relative_altitude_m"):
                        alt = getattr(pos, "relative_altitude_m")
                    elif hasattr(pos, "relative_altitude_meters"):
                        alt = getattr(pos, "relative_altitude_meters")
                    elif hasattr(pos, "absolute_altitude_m"):
                        alt = getattr(pos, "absolute_altitude_m")
                    elif hasattr(pos, "absolute_altitude_meters"):
                        alt = getattr(pos, "absolute_altitude_meters")

                    if alt is not None:
                        try:
                            self.current_altitude = float(alt)
                        except Exception:
                            pass
            except Exception as e:
                print(f"⚠️ Altitude monitoring error: {e}")

    def set_dummy_position(self, lat, lng, alt):
        """Set dummy GPS position for simulation mode"""
        self.dummy_lat = lat
        self.dummy_lng = lng
        self.dummy_alt = alt

    async def extinguish_fire(self, target_lat, target_lon, approach_altitude=15.0):
        """
        Execute fire extinguishing mission at given GPS coordinates.

        Args:
            target_lat: Target latitude
            target_lon: Target longitude
            approach_altitude: Altitude to approach target (meters)

        Returns:
            (success: bool, failure_reason: str or None)
        """
        success = False
        failure_reason = None

        try:
            # Step 1: Navigate to GPS coordinates
            print(f"📍 Navigating to ({target_lat}, {target_lon})...")

            if not self.use_dummy_telemetry:
                # Command drone to go to location using mission API
                # NOTE: Drone should already be armed and in air before calling this
                try:
                    item = MissionItem(
                        latitude_deg=float(target_lat),
                        longitude_deg=float(target_lon),
                        relative_altitude_m=approach_altitude,
                        speed_m_s=5.0,
                        is_fly_through=True,
                        gimbal_pitch_deg=0.0,
                        gimbal_yaw_deg=0.0,
                        camera_action=MissionItem.CameraAction.NONE,
                        loiter_time_s=0.0,
                        camera_photo_interval_s=0.0,
                        acceptance_radius_m=2.0,
                        yaw_deg=float("nan"),
                        camera_photo_distance_m=1.0,
                        vehicle_action=MissionItem.VehicleAction.NONE,
                    )
                    plan = MissionPlan([item])
                    await self.drone.mission.clear_mission()
                    await self.drone.mission.upload_mission(plan)
                    await self.drone.mission.start_mission()
                    print("✅ Mission uploaded - drone en route")
                except Exception as e:
                    print(f"⚠️ Mission API failed, using action.goto_location: {e}")
                    await self.drone.action.goto_location(target_lat, target_lon, approach_altitude, 0.0)

                # Wait until within approach threshold
                approach_deadline = time.time() + 60.0
                while True:
                    dist = await self.distance_to_target(target_lat, target_lon)
                    print(f"   Distance to target: {dist:.1f} m", end="\r")
                    if dist <= APPROACH_THRESHOLD_METERS:
                        print(f"\n✅ Reached approach radius ({APPROACH_THRESHOLD_METERS} m)")
                        break
                    if time.time() > approach_deadline:
                        print("\n⚠️ Timeout approaching target - proceeding anyway")
                        break
                    await asyncio.sleep(1.0)
            else:
                # Dummy mode - simulate navigation
                print("🎮 DUMMY MODE: Simulating navigation...")
                await asyncio.sleep(3)
                self.dummy_lat = target_lat
                self.dummy_lng = target_lon
                self.dummy_alt = approach_altitude

                # Update parent's position via callback
                if self.position_update_callback:
                    self.position_update_callback(target_lat, target_lon, approach_altitude)

            # Step 2: ArUco alignment and payload drop
            print(f"\n🎯 Starting precision alignment...")

            if not self.use_dummy_telemetry:
                # Initialize camera if not already done
                if self.payload_cam is None:
                    try:
                        if SIYICam is None:
                            raise ImportError("SIYICam module not available")
                        self.payload_cam = SIYICam()
                        self.payload_cam.setup_cam()
                        self.payload_cam.open_stream()
                        print("✅ Payload camera initialized")
                    except Exception as e:
                        print(f"⚠️ Camera initialization failed: {e}")
                        failure_reason = f"Camera init failed: {e}"
                        raise

                # Start offboard mode for direct velocity control
                started = await self.start_offboard()
                if not started:
                    print("⚠️ Offboard not started - alignment may fail")

                aligned_count = 0
                start_time = time.time()

                # Get frame generator (create once, reuse)
                frames_gen = self.payload_cam.frames()

                # Alignment loop
                while True:
                    # Get frame from camera
                    try:
                        frame = next(frames_gen)
                    except Exception as e:
                        print(f"⚠️ Frame capture failed: {e}")
                        await asyncio.sleep(0.05)
                        continue

                    if frame is None:
                        await asyncio.sleep(0.05)
                        continue

                    frame_h, frame_w = frame.shape[:2]
                    frame_center = (frame_w // 2, frame_h // 2)

                    # Detect ArUco marker
                    corners, ids, _ = self.alignment_detector.detect(frame)

                    if ids is not None and len(ids) > 0:
                        marker_center = self.alignment_detector.get_marker_center(corners)
                        vx, vy, vz, yaw_rate, is_aligned = self.calculate_velocity_commands(
                            frame_center, marker_center, (frame_h, frame_w)
                        )

                        # Send velocity command
                        if self.offboard_started:
                            try:
                                await self.send_velocity_command(vx, vy, vz, yaw_rate)
                            except Exception as e:
                                print(f"⚠️ Velocity command failed: {e}")

                        # Check alignment
                        if is_aligned:
                            aligned_count += 1
                            print(f"   Aligned: {aligned_count}/{ALIGN_STABLE_COUNT}", end="\r")
                        else:
                            aligned_count = 0

                    else:
                        # No marker found
                        aligned_count = 0
                        if self.offboard_started:
                            try:
                                await self.send_velocity_command(0.0, 0.0, 0.0, 0.0)
                            except Exception:
                                pass

                    # Check if stable alignment achieved
                    if aligned_count >= ALIGN_STABLE_COUNT:
                        print(f"\n✅ Aligned for {aligned_count} cycles - dropping payload!")
                        drop_payload()
                        success = True
                        break

                    # Check alignment timeout
                    if time.time() - start_time > ALIGN_TIMEOUT:
                        print(f"\n⚠️ Alignment timeout ({ALIGN_TIMEOUT}s) - aborting")
                        failure_reason = "Alignment timeout"
                        break

                    # Check average error as additional criteria
                    avg_err = self.get_avg_pixel_error()
                    if avg_err != float('inf') and avg_err < MAX_AVG_ERR:
                        current_alt = self.current_altitude or self.dummy_alt
                        if abs(current_alt - OFFBOARD_HEIGHT) < 0.1:
                            print(f"\n✅ Average error {avg_err:.1f} < {MAX_AVG_ERR} - dropping!")
                            drop_payload()
                            success = True
                            break

                    await asyncio.sleep(1.0 / ALIGN_CHECK_HZ)

                # Stop offboard mode
                if self.offboard_started:
                    await self.stop_offboard()

            else:
                # Dummy mode - simulate alignment and drop
                print("🎮 DUMMY MODE: Simulating alignment...")
                await asyncio.sleep(2)
                print("🎮 DUMMY MODE: Dropping payload...")
                drop_payload()
                success = True

        except Exception as e:
            print(f"❌ Mission failed: {e}")
            import traceback
            traceback.print_exc()
            failure_reason = str(e)
            success = False

        return success, failure_reason

    async def extinguish_fires_batch(self, fire_locations: list, approach_altitude=15.0):
        """
        Execute fire extinguishing mission for multiple fires in batch (like go_align_drop.py).
        Process all fires in one continuous mission.

        Args:
            fire_locations: list of dicts with 'target_lat', 'target_lon', 'detection_id', 'mission_id'
            approach_altitude: Altitude to approach targets (meters)

        Returns:
            (overall_success: bool, results: list of dicts with per-fire results)
        """
        results = []
        overall_success = True

        print(f"\n🔥 Starting BATCH fire extinguishing mission for {len(fire_locations)} fires")

        # Initialize camera once for entire batch (if not in dummy mode)
        # NOTE: Drone should already be armed and airborne before calling this method
        if not self.use_dummy_telemetry and self.payload_cam is None:
                try:
                    if SIYICam is None:
                        raise ImportError("SIYICam module not available")
                    self.payload_cam = SIYICam()
                    self.payload_cam.setup_cam()
                    self.payload_cam.open_stream()
                    print("✅ Payload camera initialized for batch mission")
                except Exception as e:
                    print(f"⚠️ Camera initialization failed: {e}")
                    # Return failure for all fires
                    for fire in fire_locations:
                        results.append({
                            "detection_id": fire.get("detection_id"),
                            "mission_id": fire.get("mission_id"),
                            "success": False,
                            "failure_reason": f"Camera init failed: {e}"
                        })
                    return False, results

        # Process each fire in sequence
        for idx, fire in enumerate(fire_locations):
            target_lat = fire["target_lat"]
            target_lon = fire["target_lon"]
            detection_id = fire.get("detection_id")
            mission_id = fire.get("mission_id")

            print(f"\n{'='*60}")
            print(f"🎯 Fire {idx + 1}/{len(fire_locations)}")
            print(f"{'='*60}")
            print(f"Target: ({target_lat}, {target_lon})")
            print(f"{'='*60}\n")

            try:
                # Navigate to GPS coordinates
                print(f"📍 Navigating to ({target_lat}, {target_lon})...")

                if not self.use_dummy_telemetry:
                    # Command drone to go to location using mission API
                    try:
                        item = MissionItem(
                            latitude_deg=float(target_lat),
                            longitude_deg=float(target_lon),
                            relative_altitude_m=approach_altitude,
                            speed_m_s=5.0,
                            is_fly_through=True,
                            gimbal_pitch_deg=0.0,
                            gimbal_yaw_deg=0.0,
                            camera_action=MissionItem.CameraAction.NONE,
                            loiter_time_s=0.0,
                            camera_photo_interval_s=0.0,
                            acceptance_radius_m=2.0,
                            yaw_deg=float("nan"),
                            camera_photo_distance_m=1.0,
                            vehicle_action=MissionItem.VehicleAction.NONE,
                        )
                        plan = MissionPlan([item])

                        # Note: Drone already in air from batch takeoff, just navigate
                        await self.drone.mission.clear_mission()
                        await self.drone.mission.upload_mission(plan)
                        await self.drone.mission.start_mission()
                        print("✅ Mission uploaded - drone en route")
                    except Exception as e:
                        print(f"⚠️ Mission API failed, using action.goto_location: {e}")
                        await self.drone.action.goto_location(target_lat, target_lon, approach_altitude, 0.0)

                    # Wait until within approach threshold
                    approach_deadline = time.time() + 60.0
                    while True:
                        dist = await self.distance_to_target(target_lat, target_lon)
                        print(f"   Distance to target: {dist:.1f} m", end="\r")
                        if dist <= APPROACH_THRESHOLD_METERS:
                            print(f"\n✅ Reached approach radius ({APPROACH_THRESHOLD_METERS} m)")
                            break
                        if time.time() > approach_deadline:
                            print("\n⚠️ Timeout approaching target - proceeding anyway")
                            break
                        await asyncio.sleep(1.0)
                else:
                    # Dummy mode - simulate navigation
                    print("🎮 DUMMY MODE: Simulating navigation...")
                    await asyncio.sleep(2)
                    self.dummy_lat = target_lat
                    self.dummy_lng = target_lon
                    self.dummy_alt = approach_altitude

                    # Update parent's position via callback
                    if self.position_update_callback:
                        self.position_update_callback(target_lat, target_lon, approach_altitude)

                # ArUco alignment and payload drop
                print(f"\n🎯 Starting precision alignment...")

                if not self.use_dummy_telemetry:
                    # Start offboard mode for direct velocity control
                    started = await self.start_offboard()
                    if not started:
                        print("⚠️ Offboard not started - alignment may fail")

                    aligned_count = 0
                    start_time = time.time()

                    # Get frame generator (create once, reuse)
                    frames_gen = self.payload_cam.frames()

                    # Alignment loop
                    while True:
                        # Get frame from camera
                        try:
                            frame = next(frames_gen)
                        except Exception as e:
                            print(f"⚠️ Frame capture failed: {e}")
                            await asyncio.sleep(0.05)
                            continue

                        if frame is None:
                            await asyncio.sleep(0.05)
                            continue

                        frame_h, frame_w = frame.shape[:2]
                        frame_center = (frame_w // 2, frame_h // 2)

                        # Detect ArUco marker
                        corners, ids, _ = self.alignment_detector.detect(frame)

                        if ids is not None and len(ids) > 0:
                            marker_center = self.alignment_detector.get_marker_center(corners)
                            vx, vy, vz, yaw_rate, is_aligned = self.calculate_velocity_commands(
                                frame_center, marker_center, (frame_h, frame_w)
                            )

                            # Send velocity command
                            if self.offboard_started:
                                try:
                                    await self.send_velocity_command(vx, vy, vz, yaw_rate)
                                except Exception as e:
                                    print(f"⚠️ Velocity command failed: {e}")

                            # Check alignment
                            if is_aligned:
                                aligned_count += 1
                                print(f"   Aligned: {aligned_count}/{ALIGN_STABLE_COUNT}", end="\r")
                            else:
                                aligned_count = 0

                        else:
                            # No marker found
                            aligned_count = 0
                            if self.offboard_started:
                                try:
                                    await self.send_velocity_command(0.0, 0.0, 0.0, 0.0)
                                except Exception:
                                    pass

                        # Check if stable alignment achieved
                        if aligned_count >= ALIGN_STABLE_COUNT:
                            print(f"\n✅ Aligned for {aligned_count} cycles - dropping payload!")
                            drop_payload()
                            break

                        # Check alignment timeout
                        if time.time() - start_time > ALIGN_TIMEOUT:
                            print(f"\n⚠️ Alignment timeout ({ALIGN_TIMEOUT}s) - aborting")
                            raise Exception("Alignment timeout")

                        # Check average error as additional criteria
                        avg_err = self.get_avg_pixel_error()
                        if avg_err != float('inf') and avg_err < MAX_AVG_ERR:
                            current_alt = self.current_altitude or self.dummy_alt
                            if abs(current_alt - OFFBOARD_HEIGHT) < 0.1:
                                print(f"\n✅ Average error {avg_err:.1f} < {MAX_AVG_ERR} - dropping!")
                                drop_payload()
                                break

                        await asyncio.sleep(1.0 / ALIGN_CHECK_HZ)

                    # Stop offboard mode
                    if self.offboard_started:
                        await self.stop_offboard()

                else:
                    # Dummy mode - simulate alignment and drop
                    print("🎮 DUMMY MODE: Simulating alignment...")
                    await asyncio.sleep(2)
                    print("🎮 DUMMY MODE: Dropping payload...")
                    drop_payload()

                # Success for this fire
                results.append({
                    "detection_id": detection_id,
                    "mission_id": mission_id,
                    "success": True,
                    "failure_reason": None
                })
                print(f"✅ Fire {idx + 1}/{len(fire_locations)} extinguished successfully!")

            except Exception as e:
                # Failure for this fire
                print(f"❌ Fire {idx + 1}/{len(fire_locations)} failed: {e}")
                results.append({
                    "detection_id": detection_id,
                    "mission_id": mission_id,
                    "success": False,
                    "failure_reason": str(e)
                })
                overall_success = False
                # Continue to next fire instead of aborting entire batch

        print(f"\n🎉 Batch mission completed! {sum(1 for r in results if r['success'])}/{len(fire_locations)} fires extinguished")
        return overall_success, results

    def cleanup(self):
        """Clean up resources"""
        try:
            if self.payload_cam:
                self.payload_cam.release()
        except Exception:
            pass
