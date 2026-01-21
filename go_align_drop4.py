# #!/usr/bin/env python3
# """
# go_align_drop.py

# Usage:
#     python3 go_align_drop.py

# Contains:
#     - ArucoDetector
#     - DroneController (MAVSDK offboard + PD mapping + position monitor + mission goto)
#     - go_align_drop(gps_list, ...) : go to each GPS, align to ArUco, drop payload
#     - Example main that runs a list of coordinates

# Notes:
#  - Vertical motion (vz) behavior changed per request:
#    * If pixel error magnitude > PIX_ERR_LIMIT -> vz = 0 (no vertical motion).
#    * Else if pixel error <= PIX_ERR_LIMIT AND current relative altitude < OFFBOARD_HEIGHT -> command climb vz
#      (proportional to altitude error, clipped by MAX_VEL_Z).
#    * Otherwise -> vz = 0. (No area-based vertical command anywhere.)
# """

# import asyncio
# from time import sleep, monotonic, time
# from collections import deque
# import sys
# import os
# import math

# import cv2
# import numpy as np

# # MAVSDK imports
# from mavsdk import System
# from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# # Import your camera wrapper (the new one you provided)
# # Make sure siyi_cam2 is in PYTHONPATH or same folder.
# from siyi_cam2 import SIYICam

# # -----------------------------
# # Configuration / Tuning
# # -----------------------------
# PREVIEW = 1

# # ArUco
# ARUCO_DICT = cv2.aruco.DICT_4X4_50
# MARKER_SIZE = 0.15  # meters (unused for image-only controller)

# # PD controller gains (image->body mapping)
# KP_X = 2.5
# KD_X = 0.5
# KP_Y = 2.5
# KD_Y = 0.5

# # Vertical climb proportional gain (used to climb toward OFFBOARD_HEIGHT)
# KP_CLIMB = 0.6   # tuned gain for altitude error -> climb speed

# # (KD_Z not used anymore since vertical commands are purely based on altitude error)
# # KD_Z = 0.1

# KP_YAW = 0.8
# KD_YAW = 0.1

# # Limits
# MAX_VEL_XY = 1.0
# MAX_VEL_Z = 0.5
# MAX_YAW_RATE = 30.0  # deg/s

# # Vertical error / offboard height behaviour (new)
# PIX_ERR_LIMIT = 20            # pixels - vertical motion disabled until error below this
# OFFBOARD_HEIGHT = 5.0         # meters - climb until this altitude when aligned

# # Alignment thresholds
# X_THRESHOLD = 20
# Y_THRESHOLD = 20
# TARGET_AREA = 10000
# AREA_THRESHOLD = 2000

# # Stability criteria (must be aligned for N consecutive cycles)
# ALIGN_STABLE_COUNT = 8
# ALIGN_CHECK_HZ = 10.0
# ALIGN_TIMEOUT = 40.0  # seconds to try aligning before giving up and continuing

# # Offboard and approach params
# APPROACH_THRESHOLD_METERS = 4.0  # when to start ArUco alignment (distance to target)
# ARRIVAL_THRESHOLD_METERS = 1.5   # considered "arrived" by position before attempting alignment
# ALTITUDE = 8.0                 # default altitude to travel to (m) if per-point altitude is not provided

# # Averaging error window (like your previous logic)
# AVG_ERR_TIME = 10.0
# MAX_AVG_ERR = 20.0
# err_window = deque()

# # -----------------------------
# # Helper: payload action (user provided)
# # -----------------------------
# def drop_payload():
#     """User-defined payload release. Customize as needed."""
#     print("🔴 PAYLOAD DROPPED (placeholder)")

# # -----------------------------
# # ArUco detector class (adapted)
# # -----------------------------
# class ArucoDetector:
#     def __init__(self, dict_type=ARUCO_DICT, marker_size=MARKER_SIZE):
#         try:
#             self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
#         except AttributeError:
#             self.aruco_dict = cv2.aruco.Dictionary_get(dict_type)

#         try:
#             self.aruco_params = cv2.aruco.DetectorParameters_create()
#         except AttributeError:
#             try:
#                 self.aruco_params = cv2.aruco.DetectorParameters()
#             except Exception:
#                 self.aruco_params = None

#         self.use_aruco_detector = False
#         try:
#             if hasattr(cv2.aruco, "ArucoDetector") and self.aruco_params is not None:
#                 self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
#                 self.use_aruco_detector = True
#         except Exception:
#             self.detector = None
#             self.use_aruco_detector = False

#         self.marker_size = marker_size

#     def detect(self, frame):
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         if self.use_aruco_detector and self.detector is not None:
#             corners, ids, rejected = self.detector.detectMarkers(gray)
#             return corners, ids, rejected
#         else:
#             if self.aruco_params is None:
#                 corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict)
#             else:
#                 corners, ids, rejected = cv2.aruco.detectMarkers(
#                     gray, self.aruco_dict, parameters=self.aruco_params
#                 )
#             return corners, ids, rejected

#     def draw_markers(self, frame, corners, ids):
#         if ids is not None:
#             cv2.aruco.drawDetectedMarkers(frame, corners, ids)
#         return frame

#     def get_marker_center(self, corners):
#         if corners is None or len(corners) == 0:
#             return None
#         corner = corners[0][0]  # first marker
#         center_x = int(np.mean(corner[:, 0]))
#         center_y = int(np.mean(corner[:, 1]))
#         area = cv2.contourArea(corner)
#         return center_x, center_y, area

# # -----------------------------
# # Small Haversine util
# # -----------------------------
# def haversine_meters(lat1, lon1, lat2, lon2):
#     # numeric stable haversine
#     R = 6371000.0
#     phi1 = np.radians(lat1)
#     phi2 = np.radians(lat2)
#     dphi = np.radians(lat2 - lat1)
#     dlambda = np.radians(lon2 - lon1)
#     a = np.sin(dphi/2.0)**2 + np.cos(phi1)*np.cos(phi2)*(np.sin(dlambda/2.0)**2)
#     c = 2*np.arctan2(np.sqrt(a), np.sqrt(1-a))
#     return R * c

# # -----------------------------
# # Drone controller (MAVSDK)
# # -----------------------------
# class DroneController:
#     def __init__(self, connection_string="udp://:14540"):
#         self.drone = System()
#         self.connection_string = connection_string

#         # PD state
#         self.prev_error_x = 0.0
#         self.prev_error_y = 0.0
#         self.prev_error_area = 0.0
#         self.prev_time = None

#         # offboard state
#         self.offboard_started = False
#         self.last_flight_mode = None

#         # last known telemetry (updated by monitor)
#         self.altitude = None  # in meters (relative altitude when available)

#     async def connect(self):
#         print(f"🔗 Connecting to drone at {self.connection_string} ...")
#         await self.drone.connect(system_address=self.connection_string)

#         # wait for connection
#         async for state in self.drone.core.connection_state():
#             if state.is_connected:
#                 print("✅ Drone connected")
#                 break

#         # start monitors
#         asyncio.create_task(self._monitor_flight_mode())
#         asyncio.create_task(self._monitor_position())

#     async def _monitor_flight_mode(self):
#         try:
#             async for mode in self.drone.telemetry.flight_mode():
#                 try:
#                     self.last_flight_mode = str(mode).upper()
#                 except Exception:
#                     self.last_flight_mode = None
#         except Exception:
#             # telemetry.flight_mode() might not be present
#             pass

#     async def _monitor_position(self):
#         """Continuously read telemetry.position() to keep latest altitude cached in self.altitude."""
#         try:
#             async for pos in self.drone.telemetry.position():
#                 alt = None
#                 # try to locate commonly named altitude attributes
#                 if hasattr(pos, "relative_altitude_m"):
#                     alt = getattr(pos, "relative_altitude_m")
#                 elif hasattr(pos, "relative_altitude_meters"):
#                     alt = getattr(pos, "relative_altitude_meters")
#                 elif hasattr(pos, "absolute_altitude_m"):
#                     alt = getattr(pos, "absolute_altitude_m")
#                 elif hasattr(pos, "absolute_altitude_meters"):
#                     alt = getattr(pos, "absolute_altitude_meters")

#                 if alt is not None:
#                     try:
#                         self.altitude = float(alt)
#                     except Exception:
#                         pass
#         except Exception:
#             # telemetry may not be available on all platforms
#             pass

#     async def start_offboard(self):
#         # send an initial setpoint (required by many autopilots)
#         try:
#             await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
#         except Exception:
#             pass

#         try:
#             await self.drone.offboard.start()
#             self.offboard_started = True
#             print("✅ Offboard started")
#             return True
#         except OffboardError as e:
#             print("❌ Offboard start failed:", e)
#             self.offboard_started = False
#             return False
#         except Exception as e:
#             print("❌ Offboard start error:", e)
#             self.offboard_started = False
#             return False

#     async def stop_offboard(self):
#         try:
#             await self.drone.offboard.stop()
#             await self.drone.action.hold()
#         except Exception as e:
#             print("⚠️ Error stopping offboard:", e)
#         self.offboard_started = False

#     def _should_publish(self):
#         if self.last_flight_mode is not None:
#             return "OFFBOARD" in self.last_flight_mode
#         else:
#             return self.offboard_started

#     async def send_velocity_command(self, vx, vy, vz, yaw_rate):
#         await self.drone.offboard.set_velocity_body(
#             VelocityBodyYawspeed(float(vx), float(vy), float(vz), float(yaw_rate))
#         )

#     async def goto_location_mission(self, lat, lon, alt, speed_m_s: float = 5.0, wait_for_arrival: bool = False):
#         """
#         Attempt to use mission APIs to go to a single GPS waypoint.
#         If mission API is not available or fails, falls back to action.goto_location.
#         """
#         try:
#             # Import here to handle installations that may not include mission
#             from mavsdk.mission import MissionItem, MissionPlan


#             item = MissionItem(
#                 latitude_deg=float(lat),
#                 longitude_deg=float(lon),
#                 relative_altitude_m= float(alt),
#                 speed_m_s= float(speed_m_s),
#                 is_fly_through=True,
#                 gimbal_pitch_deg=0.0,
#                 gimbal_yaw_deg=0.0,
#                 camera_action=MissionItem.CameraAction.NONE,
#                 loiter_time_s=0.0,
#                 camera_photo_interval_s=0.0,
#                 acceptance_radius_m=2.0,
#                 yaw_deg=float("nan"),
#                 camera_photo_distance_m=1.0,
#                 vehicle_action=MissionItem.VehicleAction.NONE,
#             )
#             plan = MissionPlan([item])
#             self.mission = plan
#             #await self.drone.action.set_takeoff_altitude(alt)
#             #await self.drone.action.takeoff()
#             await self.drone.mission.clear_mission()
#             await self.drone.mission.upload_mission(plan)
#             await self.drone.mission.start_mission()
#             print("✅ Mission uploaded & started to go to waypoint via mission API.")
#             if wait_for_arrival:
#                 start = time()
#                 while True:
#                     # dist = await self.distance_to_target(lat, lon)
#                     # if dist <= ARRIVAL_THRESHOLD_METERS:
#                     #     print(f"✅ Arrived at waypoint (mission). Distance {dist:.1f} m")
#                     #     break
#                     async for progress in self.drone.mission.mission_progress():
#                         if progress.current == progress.total:
#                             print(f"✅ Arrived at waypoint (mission).")
#                             break
#                     if time() - start > 120.0:
#                         print("⚠️ Wait for arrival timed out.")
#                         break
#                     await asyncio.sleep(1.0)
#             return True
#         except Exception as e:
#             print("⚠️ goto_location_mission: mission API failed or not available, falling back to action.goto_location. Error:", e)
#             try:
#                 await self.drone.action.goto_location(lat, lon, alt, 0.0)
#                 return True
#             except Exception as e2:
#                 print("⚠️ Fallback action.goto_location also failed:", e2)
#                 return False

#     def calculate_velocity_commands(self, frame_center, marker_center, frame_shape):
#         """
#         Modified vertical behavior per user's request:
#          - No area-based vertical command anywhere.
#          - If pixel error > PIX_ERR_LIMIT -> vz = 0.
#          - Else if pixel error <= PIX_ERR_LIMIT and relative altitude < OFFBOARD_HEIGHT -> climb vz (proportional).
#          - Else vz = 0.
#         Returns: vx, vy, vz, yaw_rate, is_aligned
#         """
#         now = monotonic()
#         if self.prev_time is None:
#             dt = 0.05
#         else:
#             dt = max(1e-3, now - self.prev_time)
#         self.prev_time = now

#         if marker_center is None:
#             # reset derivatives to avoid spikes on reacquisition
#             self.prev_error_x = 0.0
#             self.prev_error_y = 0.0
#             self.prev_error_area = 0.0
#             return 0.0, 0.0, 0.0, 0.0, False

#         frame_cx, frame_cy = frame_center
#         marker_x, marker_y, marker_area = marker_center
#         frame_h, frame_w = frame_shape

#         error_x = marker_x - frame_cx
#         error_y = marker_y - frame_cy
#         # note: error_area not used for vertical anymore
#         error_area = TARGET_AREA - marker_area

#         norm_error_x = error_x / frame_w
#         norm_error_y = error_y / frame_h

#         der_x = (error_x - self.prev_error_x) / dt
#         der_y = (error_y - self.prev_error_y) / dt

#         vy = KP_X * norm_error_x + KD_X * (der_x / frame_w)
#         vy = float(np.clip(vy, -MAX_VEL_XY, MAX_VEL_XY))

#         vx = -KP_Y * norm_error_y - KD_Y * (der_y / frame_h)
#         vx = float(np.clip(vx, -MAX_VEL_XY, MAX_VEL_XY))

#         # Compute yaw as before
#         yaw_rate = KP_YAW * (norm_error_x * 100.0) + KD_YAW * (der_x / frame_w * 100.0)
#         yaw_rate = float(np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE))

#         # Pixel error magnitude (image plane)
#         pixel_err_mag = float(np.hypot(error_x, error_y))

#         # Vertical velocity logic per request:
#         vz = 0.0
#         current_alt = self.altitude

#         if pixel_err_mag > PIX_ERR_LIMIT:
#             vz = 0.0
#         else:
#             # small pixel error -> consider climb to OFFBOARD_HEIGHT
#             if current_alt is not None and abs(current_alt - OFFBOARD_HEIGHT)> 0.1:
#                 alt_err = OFFBOARD_HEIGHT - current_alt
#                 vz_candidate = -KP_CLIMB * alt_err
#                 # limit climb speed
#                 vz = float(np.clip(vz_candidate, -MAX_VEL_Z, MAX_VEL_Z))
#             else:
#                 # either altitude unknown or already at/above OFFBOARD_HEIGHT -> no vertical motion
#                 vz = 0.0

#         # update alignment boolean
#         is_aligned = (
#             abs(error_x) < X_THRESHOLD and
#             abs(error_y) < Y_THRESHOLD and
#             abs(error_area) < AREA_THRESHOLD
#         )

#         # update previous errors
#         self.prev_error_x = error_x
#         self.prev_error_y = error_y
#         self.prev_error_area = error_area

#         # update error window
#         err_window.append((monotonic(), pixel_err_mag))
#         cutoff = monotonic() - AVG_ERR_TIME
#         while err_window and err_window[0][0] < cutoff:
#             err_window.popleft()

#         return vx, vy, vz, yaw_rate, is_aligned

#     def get_avg_pixel_error(self):
#         if not err_window:
#             return float('inf')
#         vals = [v for (_t, v) in err_window]
#         return sum(vals) / len(vals)

#     # Helper to check distance to a GPS target (uses telemetry)
#     async def distance_to_target(self, target_lat, target_lon):
#         # returns approximate horizontal distance (meters) to target from telemetry (Haversine)
#         try:
#             async for pos in self.drone.telemetry.position():
#                 lat = pos.latitude_deg
#                 lon = pos.longitude_deg
#                 return haversine_meters(lat, lon, target_lat, target_lon)
#         except Exception:
#             pass
#         return float('inf')

# # -----------------------------
# # Core function: go_align_drop
# # -----------------------------
# async def go_align_drop(drone: DroneController, cam: SIYICam, gps_list, approach_alt=ALTITUDE):
#     """
#     gps_list: list of tuples (lat, lon, alt) or (lat, lon)
#     For each GPS point:
#       - goto location (uses goto_location_mission)
#       - once within APPROACH_THRESHOLD_METERS start ArUco alignment loop
#       - start offboard and send velocity commands based on image PD
#       - when aligned for ALIGN_STABLE_COUNT cycles => drop_payload()
#       - stop offboard and continue next point
#     """
#     print(f"\n{'#'*60}")
#     print(f"🚁 [MISSION START] Fire Extinguishing Mission")
#     print(f"🚁 [CONFIG] Total waypoints: {len(gps_list)}")
#     print(f"🚁 [CONFIG] Default approach altitude: {approach_alt}m")
#     print(f"🚁 [CONFIG] Alignment altitude target: {OFFBOARD_HEIGHT}m")
#     print(f"🚁 [CONFIG] Alignment timeout: {ALIGN_TIMEOUT}s")
#     print(f"🚁 [CONFIG] Stable count required: {ALIGN_STABLE_COUNT}")
#     print(f"🚁 [CONFIG] Max velocities: XY={MAX_VEL_XY}m/s, Z={MAX_VEL_Z}m/s")
#     print(f"🚁 [CONFIG] Camera running: {cam.running if hasattr(cam, 'running') else 'Unknown'}")
#     print(f"{'#'*60}\n")

#     detector = ArucoDetector()

#     # NOTE: cam.open_stream() is removed here because the caller (DroneClient)
#     # now manages the camera thread lifecycle (start/stop) to allow sharing.

#     for idx, g in enumerate(gps_list):
#         if len(g) == 3:
#             lat, lon, alt = g
#         else:
#             lat, lon = g
#             alt = approach_alt

#         print(f"\n{'='*60}")
#         print(f"➡️  [WAYPOINT {idx+1}/{len(gps_list)}] Starting")
#         print(f"📍 Target: lat={lat:.6f}, lon={lon:.6f}, alt={alt}m")
#         print(f"{'='*60}")

#         # command goto using new mission-based helper
#         print("Waiting for global position estimate...")
#         try:
#             async for health in drone.drone.telemetry.health():
#                 if getattr(health, "is_global_position_ok", False) and getattr(health, "is_home_position_ok", False):
#                     print("✅ Global position OK")
#                     break
#                 await asyncio.sleep(0.5)
#         except Exception:
#             # telemetry.health may not be available; continue anyway
#             pass

#         try:
#             print("\n✈️  [MISSION] Commanding goto_location_mission ...")
#             print(f"✈️  [MISSION] Parameters: lat={lat:.6f}, lon={lon:.6f}, alt={alt}m, speed=5.0m/s")
#             await asyncio.sleep(1.0)  # brief pause before command
#             ok = await drone.goto_location_mission(lat, lon, alt, speed_m_s=5.0, wait_for_arrival=False)
#             if not ok:
#                 print("⚠️  [MISSION] goto_location_mission failed; continuing and will rely on telemetry.")
#             else:
#                 print("✅ [MISSION] Mission command sent successfully")
#         except Exception as e:
#             print(f"⚠️  [MISSION] Exception raised: {e}")

#         # wait until within APPROACH_THRESHOLD_METERS or timeout
#         print(f"\n📍 [APPROACH] Monitoring mission progress (timeout: 60s)")
#         approach_deadline = time() + 60.0
#         approach_start_time = time()
#         arrived = False
#         progress_check_count = 0
#         while time() < approach_deadline:
#             async for progress in drone.drone.mission.mission_progress():
#                 progress_check_count += 1
#                 elapsed = time() - approach_start_time
#                 if progress_check_count % 10 == 0:  # Log every 10 checks
#                     print(f"📍 [APPROACH] Check #{progress_check_count} | Elapsed: {elapsed:.1f}s | Progress: {progress.current}/{progress.total} | Alt: {drone.altitude}m")
#                 if progress.current == progress.total:
#                     print(f"\n✅ [APPROACH] Mission complete! Time: {elapsed:.1f}s, Altitude: {drone.altitude}m")
#                     arrived = True
#                     break
#                 break
#             if arrived:
#                 break
#         await asyncio.sleep(0.2)

#         if not arrived:
#             elapsed = time() - approach_start_time
#             print(f"\n⚠️  [APPROACH] TIMEOUT after {elapsed:.1f}s ({progress_check_count} checks)")
#             print(f"📍 [APPROACH] Current altitude: {drone.altitude}m (target: {alt}m)")

#         # async for progress in drone.drone.mission.mission_progress():
#         #     if progress.current == progress.total:
#         #         print("\n✅ Reached fire location. Starting ArUco alignment.")
#         #         arrived = True
#         #         break

#         #     if time() > approach_deadline:
#         #         print("\n⚠️ Timeout while approaching - proceeding to alignment attempt anyway.")
#         #         break

#         #     await asyncio.sleep(0.2)

#         # IMPORTANT: exit point
#         print(f"\n{'='*60}")
#         print("🎯 [ALIGNMENT] Starting alignment phase")
#         print(f"🎯 [ALIGNMENT] Current altitude: {drone.altitude}m, Target alignment altitude: {OFFBOARD_HEIGHT}m")
#         print(f"🎯 [ALIGNMENT] Timeout: {ALIGN_TIMEOUT}s, Stable count needed: {ALIGN_STABLE_COUNT}")
#         print(f"{'='*60}")

#         # Alignment loop
#         # Start offboard for direct velocity control
#         started = await drone.start_offboard()
#         if not started:
#             print("⚠️  [ALIGNMENT] Offboard not started; alignment may not publish velocities. You may enable OFFBOARD on RC.")
#         else:
#             print("✅ [ALIGNMENT] Offboard mode started successfully")
#         aligned_count = 0
#         start_time = time()
#         last_frame = None
#         frame_count = 0
#         no_frame_count = 0
#         marker_detected_count = 0
#         no_marker_count = 0

#         try:
#             # run alignment loop until stable alignment or timeout
#             while True:
#                 # --- NEW PATTERN: Retrieve latest frame from threaded camera ---
#                 frame = cam.get_frame()

#                 if frame is None:
#                     # Frame not ready yet, wait briefly and retry
#                     no_frame_count += 1
#                     if no_frame_count == 1 or no_frame_count % 100 == 0:  # Log first miss and every 100 misses
#                         elapsed = time() - start_time
#                         print(f"⚠️  [CAMERA] No frame available (count: {no_frame_count}, elapsed: {elapsed:.1f}s)")
#                     await asyncio.sleep(0.01)
#                     continue

#                 no_frame_count = 0  # Reset counter when frame received
#                 frame_count += 1
#                 last_frame = frame
#                 frame_h, frame_w = frame.shape[:2]
#                 frame_center = (frame_w // 2, frame_h // 2)

#                 corners, ids, _ = detector.detect(frame)

#                 if ids is not None and len(ids) > 0:
#                     marker_detected_count += 1
#                     no_marker_count = 0  # Reset no marker counter
#                     marker_center = detector.get_marker_center(corners)
#                     vx, vy, vz, yaw_rate, is_aligned = drone.calculate_velocity_commands(
#                         frame_center, marker_center, (frame_h, frame_w)
#                     )

#                     # Log marker detection every 1 second (at 10Hz = every 10 frames)
#                     if marker_detected_count % 10 == 0:
#                         elapsed = time() - start_time
#                         mx, my, area = marker_center if marker_center else (0, 0, 0)
#                         print(f"🎯 [MARKER] Detected | Time: {elapsed:.1f}s | Alt: {drone.altitude:.1f}m | "
#                               f"Pos:({mx},{my}) Area:{area:.0f} | Vel:(vx:{vx:.2f} vy:{vy:.2f} vz:{vz:.2f}) | "
#                               f"Aligned:{is_aligned} Count:{aligned_count}/{ALIGN_STABLE_COUNT}")

#                     # publish velocity if allowed
#                     if drone._should_publish():
#                         try:
#                             await drone.send_velocity_command(vx, vy, vz, yaw_rate)
#                         except Exception as e:
#                             print(f"⚠️  [VELOCITY] Failed to send velocity: {e}")

#                     # preview overlays
#                     if PREVIEW:
#                         frame = detector.draw_markers(frame, corners, ids)
#                         cv2.circle(frame, frame_center, 4, (0,255,0), -1)
#                         if marker_center:
#                             mx, my, area = marker_center
#                             cv2.circle(frame, (mx,my), 4, (0,0,255), -1)
#                             cv2.line(frame, frame_center, (mx,my), (255,0,0), 2)
#                             cv2.putText(frame, f"Area:{area:.0f}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
#                             cv2.putText(frame, f"vx:{vx:.2f} vy:{vy:.2f} vz:{vz:.2f} yaw:{yaw_rate:.1f}", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

#                     # alignment stable logic
#                     if is_aligned:
#                         aligned_count += 1
#                     else:
#                         aligned_count = 0

#                 else:
#                     # no marker found -> zero velocities (if publishing allowed)
#                     marker_detected_count = 0  # Reset marker counter
#                     no_marker_count += 1
#                     if no_marker_count == 1 or no_marker_count % 20 == 0:  # Log first miss and every 2 seconds
#                         elapsed = time() - start_time
#                         print(f"❌ [MARKER] Not detected (count: {no_marker_count}, elapsed: {elapsed:.1f}s, alt: {drone.altitude:.1f}m)")
#                     marker_center = None
#                     aligned_count = 0
#                     try:
#                         if drone._should_publish():
#                             await drone.send_velocity_command(0.0, 0.0, 0.0, 0.0)
#                     except Exception:
#                         pass

#                     if PREVIEW:
#                         cv2.putText(frame, "NO MARKER", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)

#                 if PREVIEW and frame is not None:
#                     cv2.imshow("Alignment View", frame)
#                     if cv2.waitKey(1) & 0xFF == ord('q'):
#                         print("🛑 Preview quit requested by user")
#                         break

#                 # check avg pixel error as additional check (optional)
#                 avg_err = drone.get_avg_pixel_error()
#                 if avg_err != float('inf') and avg_err < MAX_AVG_ERR and abs(drone.altitude - OFFBOARD_HEIGHT) < 0.1:
#                     elapsed = time() - start_time
#                     print(f"\n{'='*60}")
#                     print(f"✅ [SUCCESS] Average error check passed!")
#                     print(f"✅ [SUCCESS] Avg error: {avg_err:.1f} < {MAX_AVG_ERR}")
#                     print(f"✅ [SUCCESS] Altitude: {drone.altitude:.1f}m ≈ {OFFBOARD_HEIGHT}m")
#                     print(f"✅ [SUCCESS] Time to align: {elapsed:.1f}s")
#                     print(f"{'='*60}")
#                     # perform payload drop
#                     drop_payload()
#                     err_window.clear()
#                     break

#                 # if aligned for required consecutive frames -> drop
#                 if aligned_count >= ALIGN_STABLE_COUNT:
#                     elapsed = time() - start_time
#                     print(f"\n{'='*60}")
#                     print(f"✅ [SUCCESS] Stable alignment achieved!")
#                     print(f"✅ [SUCCESS] Aligned count: {aligned_count}/{ALIGN_STABLE_COUNT}")
#                     print(f"✅ [SUCCESS] Final altitude: {drone.altitude:.1f}m")
#                     print(f"✅ [SUCCESS] Time to align: {elapsed:.1f}s")
#                     print(f"{'='*60}")
#                     drop_payload()
#                     break

#                 # timeout for alignment
#                 elapsed = time() - start_time
#                 if elapsed > ALIGN_TIMEOUT:
#                     print(f"\n{'='*60}")
#                     print(f"⚠️  [TIMEOUT] Alignment timeout after {elapsed:.1f}s")
#                     print(f"⚠️  [TIMEOUT] Max aligned count reached: {aligned_count}/{ALIGN_STABLE_COUNT}")
#                     print(f"⚠️  [TIMEOUT] Current altitude: {drone.altitude:.1f}m")
#                     print(f"⚠️  [TIMEOUT] Frames processed: {frame_count}")
#                     print(f"⚠️  [TIMEOUT] Markers detected: {marker_detected_count}")
#                     print(f"⚠️  [TIMEOUT] Moving to next waypoint...")
#                     print(f"{'='*60}")
#                     break

#                 # small sleep to control loop rate
#                 await asyncio.sleep(1.0 / ALIGN_CHECK_HZ)

#         finally:
#             # stop offboard after aligning attempt
#             print(f"\n🧹 [CLEANUP] Starting cleanup for waypoint {idx+1}")
#             print(f"🧹 [CLEANUP] Offboard status: {drone.offboard_started}")
#             print(f"🧹 [CLEANUP] Current altitude: {drone.altitude}m")

#             # small settle
#             try:
#                 if drone._should_publish():
#                     print(f"🧹 [CLEANUP] Sending zero velocity command...")
#                     await drone.send_velocity_command(0.0, 0.0, 0.0, 0.0)
#                     await asyncio.sleep(0.5)
#                 else:
#                     print(f"🧹 [CLEANUP] Cannot publish (offboard not active)")
#             except Exception as e:
#                 print(f"⚠️  [CLEANUP] Error sending zero velocity: {e}")

#             if drone.offboard_started:
#                 print(f"🧹 [CLEANUP] Stopping offboard mode...")
#                 await drone.stop_offboard()
#                 await asyncio.sleep(1.0)
#                 print(f"✅ [CLEANUP] Offboard stopped")

#             print(f"\n{'='*60}")
#             print(f"✅ [WAYPOINT] Completed waypoint {idx+1}/{len(gps_list)}")
#             print(f"📊 [STATS] Frames: {frame_count}, Markers: {marker_detected_count}, Final Alt: {drone.altitude}m")
#             print(f"{'='*60}")

#     # NOTE: cam.release() removed. 
#     # The caller (DroneClient) owns the camera and will close it on shutdown.
    
#     if PREVIEW:
#         try:
#             cv2.destroyAllWindows()
#         except Exception:
#             pass

#     print(f"\n{'='*60}")
#     print("🎉 [MISSION] All waypoints processed!")
#     print(f"🎉 [MISSION] Total waypoints: {len(gps_list)}")
#     print(f"🎉 [MISSION] Executing Return to Launch (RTL)...")
#     print(f"{'='*60}")

#     try:
#         await drone.drone.action.return_to_launch()
#         print("✅ [MISSION] RTL command executed successfully")
#     except Exception as e:
#         print(f"⚠️  [MISSION] RTL execution failed: {e}")

#     return

# # -----------------------------
# # Example main
# # -----------------------------
# async def main():
#     # Example GPS list: (lat, lon [, alt])
#     print('abc')
#     gps_list = [
#         # (0.0,0.0,25.0),  # dummy point for testing
#         (47.398039859999997, 8.5455725400000002, 25),
#         (47.388030, 8.5455725400000002, 25),
#         (47.388039859999997, 8.555579, 25),
#         # add your own points
#     ]

#     # instantiate camera and drone controller
#     cam = SIYICam()  # uses defaults; pass args if your constructor requires (ip, port, ...)
#     drone = DroneController(connection_string="udp://:14540")  # change connection if needed

#     await drone.connect()
#     # ensure global position ok - wait briefly (optional)
#     print("Waiting for global position estimate...")
#     try:
#         async for health in drone.drone.telemetry.health():
#             if getattr(health, "is_global_position_ok", False) and getattr(health, "is_home_position_ok", False):
#                 print("✅ Global position OK")
#                 break
#             await asyncio.sleep(0.5)
#     except Exception:
#         # telemetry.health may not be available; continue anyway
#         pass

#     # Arm and climb to safe altitude before going to waypoints
#     try:
#         print("Arming...")
#         await drone.drone.action.arm()
#     except Exception as e:
#         print("⚠️ Arm failed (maybe already armed):", e)

#     # optional: takeoff to a safe altitude if supported
#     try:
#         print(f"Taking off to {ALTITUDE} m (action.takeoff may be autopilot-specific)")
#         await drone.drone.action.set_takeoff_altitude(ALTITUDE)
#         await drone.drone.action.takeoff()
#         # wait short period
#         await asyncio.sleep(6.0)
#     except Exception as e:
#         print("⚠️ takeoff not supported or failed:", e)

#     # call core function
#     await go_align_drop(drone, cam, gps_list)

#     # land at end (optional)
#     try:
#         print("Landing ...")
#         await drone.drone.action.land()
#     except Exception as e:
#         print("⚠️ Land failed or not supported:", e)

# if __name__ == "__main__":
#     try:
#         asyncio.run(main())
#     except KeyboardInterrupt:
#         print("Interrupted by user - exiting.")







#!/usr/bin/env python3
"""
go_align_drop.py  (MISSION-FREE FINAL VERSION)

PX4 / MAVSDK SAFE VERSION
- NO Mission mode
- NO mission_progress
- Action + Telemetry + Offboard ONLY
"""

import asyncio
from time import monotonic, time
from collections import deque
import math
import cv2
import numpy as np

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

from siyi_cam2 import SIYICam

# ============================================================
# CONFIG
# ============================================================
PREVIEW = 1

ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_SIZE = 0.15

KP_X = 2.5
KD_X = 0.5
KP_Y = 2.5
KD_Y = 0.5
KP_CLIMB = 0.6
KP_YAW = 0.8
KD_YAW = 0.1

MAX_VEL_XY = 1.0
MAX_VEL_Z = 0.5
MAX_YAW_RATE = 30.0

PIX_ERR_LIMIT = 20
OFFBOARD_HEIGHT = 5.0

X_THRESHOLD = 20
Y_THRESHOLD = 20
TARGET_AREA = 10000
AREA_THRESHOLD = 2000

ALIGN_STABLE_COUNT = 8
ALIGN_CHECK_HZ = 10.0
ALIGN_TIMEOUT = 40.0

APPROACH_THRESHOLD_METERS = 4.0
ARRIVAL_THRESHOLD_METERS = 1.5
ALTITUDE = 8.0

AVG_ERR_TIME = 10.0
MAX_AVG_ERR = 20.0
err_window = deque()

# ============================================================
# UTILS
# ============================================================
def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) \
        * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def drop_payload():
    print("🔴 PAYLOAD DROPPED")

# ============================================================
# ARUCO
# ============================================================
class ArucoDetector:
    def __init__(self):
        self.dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dict, self.params)

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return self.detector.detectMarkers(gray)

    def get_marker_center(self, corners):
        c = corners[0][0]
        x = int(np.mean(c[:, 0]))
        y = int(np.mean(c[:, 1]))
        area = cv2.contourArea(c)
        return x, y, area

# ============================================================
# DRONE CONTROLLER
# ============================================================
class DroneController:
    def __init__(self, connection="udp://:14540"):
        self.drone = System()
        self.conn = connection
        self.altitude = None
        self.prev_error_x = 0
        self.prev_error_y = 0
        self.prev_time = None
        self.offboard_started = False

    async def connect(self):
        await self.drone.connect(system_address=self.conn)
        async for s in self.drone.core.connection_state():
            if s.is_connected:
                print("✅ Drone connected")
                break
        asyncio.create_task(self._monitor_position())

    async def _monitor_position(self):
        async for pos in self.drone.telemetry.position():
            self.altitude = pos.relative_altitude_m

    async def goto_location(self, lat, lon, alt):
        await self.drone.action.goto_location(lat, lon, alt, 0.0)

    async def wait_until_arrived(self, lat, lon, timeout=60):
        start = time()
        async for pos in self.drone.telemetry.position():
            d = haversine_m(pos.latitude_deg, pos.longitude_deg, lat, lon)
            if d < ARRIVAL_THRESHOLD_METERS:
                print("✅ Arrived at waypoint")
                return True
            if time() - start > timeout:
                print("⚠️ Arrival timeout")
                return False

    async def start_offboard(self):
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0,0,0,0)
        )
        try:
            await self.drone.offboard.start()
            self.offboard_started = True
            return True
        except OffboardError:
            return False

    async def stop_offboard(self):
        try:
            await self.drone.offboard.stop()
            await self.drone.action.hold()
        except Exception:
            pass
        self.offboard_started = False

    async def send_vel(self, vx, vy, vz, yaw):
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(vx, vy, vz, yaw)
        )

    def compute_velocity(self, frame_center, marker_center, frame_shape):
        now = monotonic()
        dt = 0.05 if self.prev_time is None else max(1e-3, now-self.prev_time)
        self.prev_time = now

        fx, fy = frame_center
        mx, my, area = marker_center
        fh, fw = frame_shape

        ex = mx - fx
        ey = my - fy

        vx = -KP_Y * (ey / fh)
        vy = KP_X * (ex / fw)

        vx = float(np.clip(vx, -MAX_VEL_XY, MAX_VEL_XY))
        vy = float(np.clip(vy, -MAX_VEL_XY, MAX_VEL_XY))

        yaw = KP_YAW * (ex / fw * 100)
        yaw = float(np.clip(yaw, -MAX_YAW_RATE, MAX_YAW_RATE))

        pixel_err = math.hypot(ex, ey)
        vz = 0.0
        if pixel_err <= PIX_ERR_LIMIT and self.altitude is not None:
            dz = OFFBOARD_HEIGHT - self.altitude
            vz = float(np.clip(-KP_CLIMB * dz, -MAX_VEL_Z, MAX_VEL_Z))

        aligned = abs(ex) < X_THRESHOLD and abs(ey) < Y_THRESHOLD \
                  and abs(TARGET_AREA - area) < AREA_THRESHOLD

        err_window.append((monotonic(), pixel_err))
        while err_window and err_window[0][0] < monotonic() - AVG_ERR_TIME:
            err_window.popleft()

        return vx, vy, vz, yaw, aligned

# ============================================================
# CORE LOGIC
# ============================================================
async def go_align_drop(drone, cam, gps_list):
    detector = ArucoDetector()

    for i, (lat, lon, alt) in enumerate(gps_list):
        print(f"\n➡️ WAYPOINT {i+1}")
        await drone.goto_location(lat, lon, alt)
        await drone.wait_until_arrived(lat, lon)

        await drone.start_offboard()

        aligned_count = 0
        start = time()

        while True:
            frame = cam.get_frame()
            if frame is None:
                await asyncio.sleep(0.01)
                continue

            h, w = frame.shape[:2]
            corners, ids, _ = detector.detect(frame)

            if ids is not None:
                mc = detector.get_marker_center(corners)
                vx, vy, vz, yaw, aligned = drone.compute_velocity(
                    (w//2, h//2), mc, (h, w)
                )
                await drone.send_vel(vx, vy, vz, yaw)
                aligned_count = aligned_count+1 if aligned else 0
            else:
                await drone.send_vel(0,0,0,0)
                aligned_count = 0

            if aligned_count >= ALIGN_STABLE_COUNT:
                drop_payload()
                break

            if time() - start > ALIGN_TIMEOUT:
                print("⚠️ Alignment timeout")
                break

            await asyncio.sleep(1.0 / ALIGN_CHECK_HZ)

        await drone.stop_offboard()

    await drone.drone.action.return_to_launch()

# ============================================================
# MAIN
# ============================================================
async def main():
    gps_list = [
        (47.39803986, 8.54557254, 25),
        (47.38803000, 8.54557254, 25),
        (47.38803986, 8.55557900, 25),
    ]

    cam = SIYICam()
    drone = DroneController()

    await drone.connect()
    await drone.drone.action.arm()
    await drone.drone.action.set_takeoff_altitude(ALTITUDE)
    await drone.drone.action.takeoff()
    await asyncio.sleep(6)

    await go_align_drop(drone, cam, gps_list)

if __name__ == "__main__":
    asyncio.run(main())
