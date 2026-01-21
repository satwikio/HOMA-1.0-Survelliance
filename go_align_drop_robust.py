#!/usr/bin/env python3
"""
go_align_drop_v2.py

Robust implementation of Drone Delivery Logic.
Addresses:
1. Dynamic Approach Deadlines & Visual Verification before Offboard.
2. Robust Mission Mode switching with Hold/Retry logic.
3. Strict flow control (no zombie tasks) & Dependency Injection.

Usage:
    Import `go_align_drop` into your main script or run this file directly.
"""

import asyncio
import math
import cv2
import numpy as np
from time import time, monotonic
from collections import deque

# MAVSDK Imports
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.telemetry import FlightMode

# Camera Import
# Ensure siyi_cam2.py is in the same directory
from siyi_cam2 import SIYICam

# ------------------------------------------------------------------------------
# CONFIGURATION & TUNING
# ------------------------------------------------------------------------------
PREVIEW = False  # Set False for headless operation

# -- ArUco --
ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_SIZE = 0.15

# -- Control Gains (PD) --
KP_X = 2.5
KD_X = 0.5
KP_Y = 2.5
KD_Y = 0.5
KP_CLIMB = 0.6
KP_YAW = 0.8

# -- Velocity Limits --
MAX_VEL_XY = 1.0
MAX_VEL_Z = 0.5
MAX_YAW_RATE = 30.0

# -- Logic Thresholds --
PIX_ERR_LIMIT = 30            # Start vertical descent only if error < this
OFFBOARD_HEIGHT = 7.0         # Target align altitude
ALIGN_STABLE_COUNT = 8        # Frames required to be aligned
ALIGN_CHECK_HZ = 20.0
ALIGN_TIMEOUT = 40.0          # Max time allowed in offboard alignment

# -- Robustness Parameters (Problem 1 & 2 Solutions) --
VISUAL_VERIFY_TIME = 4.0      # Seconds to observe before starting offboard
VISUAL_VERIFY_THRESH = 0.7    # 70% of frames must have marker
MODE_CHANGE_DELAY = 2.0       # Seconds to wait between Hold and Mission
NUM_RETRY_MODE_CHANGE = 3     # Max retries for mode switching
CRUISE_SPEED = 5.0            # m/s

# ------------------------------------------------------------------------------
# UTILITIES
# ------------------------------------------------------------------------------

def drop_payload():
    """User-defined payload release."""
    print("🔴 ACTION: PAYLOAD DROPPED")

def haversine_meters(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    dphi = np.radians(lat2 - lat1)
    dlambda = np.radians(lon2 - lon1)
    a = np.sin(dphi/2.0)**2 + np.cos(phi1)*np.cos(phi2)*(np.sin(dlambda/2.0)**2)
    c = 2*np.arctan2(np.sqrt(a), np.sqrt(1-a))
    return R * c

# ------------------------------------------------------------------------------
# ARUCO DETECTOR
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# ARUCO DETECTOR (Version Compatible)
# ------------------------------------------------------------------------------

class ArucoDetector:
    def __init__(self, dict_type=ARUCO_DICT):
        # 1. Handle Dictionary Retrieval (Old vs New)
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(dict_type)

        # 2. Handle Detector Parameters (Old vs New)
        try:
            # OpenCV 4.7+
            self.aruco_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            # OpenCV < 4.7
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        # 3. Handle Detector Object (Old vs New)
        self.detector = None
        try:
            # OpenCV 4.7+ has a dedicated ArucoDetector class
            if hasattr(cv2.aruco, "ArucoDetector"):
                self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        except Exception:
            pass

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Use the class-based detector if available (Newer OpenCV)
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        else:
            # Fallback to function-based detection (Older OpenCV)
            return cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

    def get_marker_center(self, corners):
        if corners is None or len(corners) == 0:
            return None
        c = corners[0][0]
        cx = int(np.mean(c[:, 0]))
        cy = int(np.mean(c[:, 1]))
        area = cv2.contourArea(c)
        return cx, cy, area

    def draw_markers(self, frame, corners, ids):
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return frame
# ------------------------------------------------------------------------------
# ROBUST DRONE CONTROLLER
# ------------------------------------------------------------------------------

class DroneController:
    def __init__(self, drone_system: System):
        self.drone = drone_system
        self.offboard_started = False
        self.altitude = None
        self.prev_time = None
        self.prev_error_x = 0
        self.prev_error_y = 0
        
        # Start background tasks
        asyncio.create_task(self._monitor_position())

    async def _monitor_position(self):
        """Robust background telemetry monitor."""
        async for pos in self.drone.telemetry.position():
            # Prefer relative altitude, fallback to absolute
            if not math.isnan(pos.relative_altitude_m):
                self.altitude = pos.relative_altitude_m
            else:
                self.altitude = pos.absolute_altitude_m

    async def get_current_position(self):
        """One-shot position fetch."""
        async for pos in self.drone.telemetry.position():
            return pos.latitude_deg, pos.longitude_deg

    # --- PROBLEM 2 SOLUTION: Robust Mode Switching ---
    async def set_flight_mode_robust(self, mode_string: str):
        """
        Tries to set flight mode with retries.
        Sequence: Hold -> Wait -> Requested Mode -> Verify.
        """
        target_mode = mode_string.upper()
        
        for attempt in range(1, NUM_RETRY_MODE_CHANGE + 1):
            print(f"⚙️  Mode Switch Attempt {attempt}/{NUM_RETRY_MODE_CHANGE} -> {target_mode}")
            
            # 1. Command HOLD first to clear active commands
            try:
                await self.drone.action.hold()
                await asyncio.sleep(MODE_CHANGE_DELAY)
            except Exception as e:
                print(f"   ⚠️ Hold command warning: {e}")

            # 2. Command Target Mode
            try:
                if target_mode == "MISSION":
                    await self.drone.mission.start_mission()
                elif target_mode == "RETURN_TO_LAUNCH":
                    await self.drone.action.return_to_launch()
                elif target_mode == "OFFBOARD":
                    # Offboard requires a setpoint first
                    await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
                    await self.drone.offboard.start()
                    self.offboard_started = True
            except Exception as e:
                print(f"   ⚠️ Command failed: {e}")

            # 3. Verify
            await asyncio.sleep(1.0)
            current_mode = await self._get_flight_mode()
            if current_mode == target_mode:
                print(f"✅ Mode Confirmed: {target_mode}")
                return True
            else:
                print(f"   ❌ Mode Verification Failed. Current: {current_mode}")
        
        return False

    async def _get_flight_mode(self):
        async for mode in self.drone.telemetry.flight_mode():
            return str(mode).split(".")[-1].upper() # e.g. "MISSION"

    async def goto_location_mission_robust(self, lat, lon, alt):
        """Generates mission plan and attempts robust start."""
        try:
            item = MissionItem(
                latitude_deg=float(lat),
                longitude_deg=float(lon),
                relative_altitude_m=float(alt),
                speed_m_s=float(CRUISE_SPEED),
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
            
            # Clear & Upload
            await self.drone.mission.clear_mission()
            await self.drone.mission.upload_mission(plan)
            
            # Start via Robust Switcher
            return await self.set_flight_mode_robust("MISSION")
        except Exception as e:
            print(f"❌ Mission Upload Error: {e}")
            return False

    async def stop_offboard(self):
        """Safely stops offboard."""
        if self.offboard_started:
            try:
                await self.drone.offboard.stop()
            except:
                pass
            try:
                await self.drone.action.hold()
            except:
                pass
            self.offboard_started = False
            print("⏹️  Offboard Stopped.")

    # def calculate_velocity(self, frame_center, marker_center, frame_shape):
    #     """PD Controller Logic."""
    #     now = monotonic()
    #     dt = 0.05 if self.prev_time is None else max(1e-3, now - self.prev_time)
    #     self.prev_time = now

    #     if marker_center is None:
    #         return 0.0, 0.0, 0.0, 0.0, False

    #     frame_cx, frame_cy = frame_center
    #     mx, my, _ = marker_center
    #     h, w = frame_shape

    #     # Normalize Errors
    #     err_x = (mx - frame_cx) / w  # Image X -> Body Y
    #     err_y = (my - frame_cy) / h  # Image Y -> Body -X

    #     # Derivatives
    #     der_x = (err_x - self.prev_error_x) / dt
    #     der_y = (err_y - self.prev_error_y) / dt
    #     self.prev_error_x = err_x
    #     self.prev_error_y = err_y

    #     # PID Output (Mapping Image frame to Body NED frame)
    #     # Image X is Right -> Body Y is Right
    #     # Image Y is Down  -> Body X is Backwards
    #     vy = (KP_X * err_x) + (KD_X * der_x)
    #     vx = (-KP_Y * err_y) - (KD_Y * der_y)
        
    #     # Clamp
    #     vx = float(np.clip(vx, -MAX_VEL_XY, MAX_VEL_XY))
    #     vy = float(np.clip(vy, -MAX_VEL_XY, MAX_VEL_XY))

    #     # Yaw
    #     yaw_rate = float(np.clip(KP_YAW * err_x * 100, -MAX_YAW_RATE, MAX_YAW_RATE))

    #     # Vertical Logic
    #     vz = 0.0
    #     pix_err_mag = math.hypot(mx - frame_cx, my - frame_cy)
        
    #     if self.altitude is not None:
    #         # Only descend/climb if horizontally aligned close enough
    #         if pix_err_mag < PIX_ERR_LIMIT and abs(self.altitude - OFFBOARD_HEIGHT) > 0.2:
    #             # Proportional descent
    #             vz = -KP_CLIMB * (OFFBOARD_HEIGHT - self.altitude)
    #             vz = float(np.clip(vz, -MAX_VEL_Z, MAX_VEL_Z))
        
    #     is_aligned = (abs(mx - frame_cx) < 20 and abs(my - frame_cy) < 20)
    #     return vx, vy, vz, yaw_rate, is_aligned



    def calculate_velocity(self, frame_center, marker_center, frame_shape):
        """PD Controller Logic."""
        now = monotonic()
        dt = 0.05 if self.prev_time is None else max(1e-3, now - self.prev_time)
        self.prev_time = now

        if marker_center is None:
            return 0.0, 0.0, 0.0, 0.0, False

        frame_cx, frame_cy = frame_center
        mx, my, _ = marker_center
        h, w = frame_shape

        # Normalize Errors
        err_x = (mx - frame_cx) / w  # Image X -> Body Y
        err_y = (my - frame_cy) / h  # Image Y -> Body -X

        # Derivatives
        der_x = (err_x - self.prev_error_x) / dt
        der_y = (err_y - self.prev_error_y) / dt
        self.prev_error_x = err_x
        self.prev_error_y = err_y

        # PID Output (Mapping Image frame to Body NED frame)
        vy = (KP_X * err_x) + (KD_X * der_x)
        vx = (-KP_Y * err_y) - (KD_Y * der_y)
        
        # Clamp Horizontal Velocity
        vx = float(np.clip(vx, -MAX_VEL_XY, MAX_VEL_XY))
        vy = float(np.clip(vy, -MAX_VEL_XY, MAX_VEL_XY))

        # Yaw Control
        yaw_rate = float(np.clip(KP_YAW * err_x * 100, -MAX_YAW_RATE, MAX_YAW_RATE))

        # --- VERTICAL LOGIC (Corrected) ---
        vz = 0.0
        pix_err_mag = math.hypot(mx - frame_cx, my - frame_cy)
        altitude_error = 0.0
        
        if self.altitude is not None:
            # Calculate altitude error (Target - Current)
            # Positive diff means we are too low (need to climb, -vz)
            # Negative diff means we are too high (need to descend, +vz)
            # MAVSDK NED: +Z is Down. To descend (+Z), we need positive velocity.
            
            # Note: OFFBOARD_HEIGHT is positive (e.g. 7.0m). self.altitude is positive (e.g. 20.0m).
            # If at 20m, target 7m: 7 - 20 = -13. We need +ve velocity to go down.
            # So vz = -KP * (Target - Current) -> -0.6 * -13 = +7.8 (Down). Correct.
            
            altitude_diff = OFFBOARD_HEIGHT - self.altitude
            altitude_error = abs(altitude_diff)

            # Only descend/climb if horizontally aligned close enough (Safety)
            if pix_err_mag < PIX_ERR_LIMIT:
                if altitude_error > 0.2: # Deadband
                    vz = -KP_CLIMB * altitude_diff
                    vz = float(np.clip(vz, -MAX_VEL_Z, MAX_VEL_Z))
            else:
                # If horizontal error is large, stop vertical movement to prevent loss of target
                vz = 0.0

        # --- ALIGNMENT CHECK (Corrected) ---
        # Must be centered horizontally AND close to target altitude
        is_aligned = (
            abs(mx - frame_cx) < 40 and    # Horizontal X ok
            abs(my - frame_cy) < 40 and    # Horizontal Y ok
            altitude_error < 0.5           # Altitude within 0.5m of target
        )
        
        return vx, vy, vz, yaw_rate, is_aligned

    async def send_vel(self, vx, vy, vz, yaw):
        if self.offboard_started:
            try:
                await self.drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(vx, vy, vz, yaw)
                )
            except OffboardError:
                pass

# ------------------------------------------------------------------------------
# CORE LOGIC: GO -> VERIFY -> ALIGN -> DROP
# ------------------------------------------------------------------------------

async def verify_visual_target(cam, detector) -> bool:
    """
    Problem 1 Solution:
    Hovers and checks frames for DETECTOR_TIME. 
    Returns True only if valid detection ratio > 0.7.
    """
    print(f"👁️  Verifying Visual Target ({VISUAL_VERIFY_TIME}s)...")
    
    total_frames = 0
    detected_frames = 0
    start_t = time()
    
    while time() - start_t < VISUAL_VERIFY_TIME:
        frame = cam.get_frame()
        if frame is None:
            await asyncio.sleep(0.01)
            continue
        
        total_frames += 1
        corners, ids, _ = detector.detect(frame)
        
        has_marker = (ids is not None and len(ids) > 0)
        if has_marker:
            detected_frames += 1
            
        if PREVIEW:
            detector.draw_markers(frame, corners, ids)
            cv2.putText(frame, "VERIFYING TARGET", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
            cv2.imshow("DroneView", frame)
            cv2.waitKey(1)
            
        await asyncio.sleep(0.05) # ~20Hz check
        
    if total_frames == 0:
        print("⚠️ No frames received during verification.")
        return False
        
    ratio = detected_frames / total_frames
    print(f"📊 Visual Stats: {detected_frames}/{total_frames} ({ratio*100:.1f}%)")
    
    return ratio >= VISUAL_VERIFY_THRESH

def calculate_dynamic_timeout(dist_meters):
    """Problem 1 Solution: Dynamic Deadline."""
    # Time = Distance / Speed + Margin
    # Margin = 10s for acceleration/deceleration + 50% buffer
    return (dist_meters / CRUISE_SPEED) * 1.5 + 20.0

async def go_align_drop(drone_sys: System, cam: SIYICam, gps_list):
    """
    Main State Machine.
    Args:
        drone_sys: Connected MAVSDK System object
        cam: Initialized Camera object
        gps_list: List of (lat, lon, alt)
    """
    controller = DroneController(drone_sys)
    detector = ArucoDetector()
    
    print("🚀 Starting Mission Sequence")

    for idx, (lat, lon, alt) in enumerate(gps_list):
        print(f"\n📍 Waypoint {idx+1}/{len(gps_list)}: {lat}, {lon}")
        
        # --- STEP 1: CALCULATE DYNAMIC TIMEOUT ---
        curr_lat, curr_lon = await controller.get_current_position()
        dist = haversine_meters(curr_lat, curr_lon, lat, lon)
        timeout_s = calculate_dynamic_timeout(dist)
        print(f"   Dist: {dist:.1f}m | Timeout: {timeout_s:.1f}s")
        
        # --- STEP 2: ROBUST APPROACH (Mission Mode) ---
        success_goto = await controller.goto_location_mission_robust(lat, lon, alt)
        if not success_goto:
            print("❌ Failed to start mission/goto. Aborting mission sequence.")
            await controller.set_flight_mode_robust("RETURN_TO_LAUNCH")
            return

        # Monitor Arrival
        approach_start = time()
        arrived = False
        while time() - approach_start < timeout_s:
            curr_lat, curr_lon = await controller.get_current_position()
            d = haversine_meters(curr_lat, curr_lon, lat, lon)
            
            if d < 2.0: # 2 meter radius
                print("✅ Arrived at location.")
                arrived = True
                break
            
            if int(time()) % 5 == 0:
                print(f"   Approaching... {d:.1f}m", end='\r')
            await asyncio.sleep(0.5)
            
        if not arrived:
            print("⏱️ Approach Timed Out. Skipping this waypoint.")
            continue # Skip to next point (Problem 3)

        # --- STEP 3: VISUAL VERIFICATION (Problem 1) ---
        # Switch to Hold first to stabilize for camera check
        await controller.drone.action.hold()
        await asyncio.sleep(2.0) # Settle
        
        visual_ok = await verify_visual_target(cam, detector)
        
        if not visual_ok:
            print("⚠️ Visual Check Failed (Target not seen). Skipping Drop.")
            continue # Skip alignment (Problem 3)

        # --- STEP 4: ALIGNMENT & DROP (Offboard) ---
        # Only reached if visual_ok is True
        
        # Start Offboard
        if not await controller.set_flight_mode_robust("OFFBOARD"):
            print("❌ Failed to start Offboard. Skipping.")
            continue

        print("🎯 Starting Alignment Loop...")
        align_start = time()
        stable_frames = 0
        
        try:
            while time() - align_start < ALIGN_TIMEOUT:
                frame = cam.get_frame()
                if frame is None:
                    await asyncio.sleep(0.01)
                    continue

                h, w = frame.shape[:2]
                center = (w//2, h//2)
                
                corners, ids, _ = detector.detect(frame)
                
                # Check target
                marker_center = None
                if ids is not None:
                    marker_center = detector.get_marker_center(corners)
                
                # Calculate Velocities
                vx, vy, vz, yaw, is_aligned = controller.calculate_velocity(center, marker_center, (h,w))
                
                # Send Command
                await controller.send_vel(vx, vy, vz, yaw)
                
                # Logic
                if is_aligned:
                    stable_frames += 1
                else:
                    stable_frames = 0
                
                # Preview
                if PREVIEW:
                    detector.draw_markers(frame, corners, ids)
                    color = (0,255,0) if is_aligned else (0,0,255)
                    cv2.circle(frame, center, 5, color, -1)
                    cv2.putText(frame, f"Vel: {vx:.2f},{vy:.2f},{vz:.2f}", (10,60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                    cv2.imshow("DroneView", frame)
                    cv2.waitKey(1)

                # SUCCESS CONDITION
                if stable_frames >= ALIGN_STABLE_COUNT:
                    print("✅ Target Locked & Stable.")
                    drop_payload()
                    break
                
                await asyncio.sleep(1/ALIGN_CHECK_HZ)

        except Exception as e:
            print(f"❌ Error during alignment: {e}")
            
        finally:
            # PROBLEM 3 FIX: Always ensure cleanup
            await controller.stop_offboard()
            if PREVIEW:
                cv2.destroyAllWindows()
                
    print("\n🏁 Mission Sequence Finished. RTL.")
    await controller.set_flight_mode_robust("RETURN_TO_LAUNCH")
    return

# ------------------------------------------------------------------------------
# EXAMPLE ENTRY POINT
# ------------------------------------------------------------------------------
async def main():
    # Example Usage
    gps_targets = [
        (47.39803986, 8.54557254, 20),
        (47.39803622, 8.54501464, 20)
    ]
    
    # 1. Connect System (Dependency Injection)
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected.")
            break
            
    # 2. Wait for Global Position
    print("Waiting for GPS...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("GPS OK.")
            break
            
    # 3. Arm and Takeoff (Initial)
    print("Arming & Takeoff...")
    try:
        await drone.action.arm()
        await drone.action.set_takeoff_altitude(10.0)
        await drone.action.takeoff()
        await asyncio.sleep(10)
    except Exception as e:
        print(f"Takeoff Error: {e}")

    # 4. Initialize Camera
    cam = SIYICam()
    # cam.connect() # If needed
    
    # 5. Run Logic
    await go_align_drop(drone, cam, gps_targets)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass