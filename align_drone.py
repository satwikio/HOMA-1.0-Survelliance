# import sys
# import os
# from time import sleep
# import cv2
# import numpy as np
# import asyncio
# from mavsdk import System
# from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# # ---- Add parent directory to path ----
# current = os.path.dirname(os.path.realpath(__file__))
# parent_directory = os.path.dirname(current)
# sys.path.append(parent_directory)

# from siyi_sdk import SIYISDK

# # ============ CONFIGURATION ============
# PREVIEW = 0  # Set to False to disable video preview
# ARUCO_DICT = cv2.aruco.DICT_4X4_50  # ArUco dictionary type
# MARKER_SIZE = 0.15  # Marker size in meters (adjust based on your marker)

# # P Controller Gains (tune these values)
# KP_X =2.5  # Proportional gain for lateral (left/right) movement
# KP_Y = 2.5  # Proportional gain for vertical (up/down) movement
# KP_Z = 0.5  # Proportional gain for forward/backward movement
# KP_YAW = 0.8  # Proportional gain for yaw rotation

# # Velocity limits (m/s and deg/s)
# MAX_VEL_XY = 1.0  # Maximum lateral velocity
# MAX_VEL_Z = 0.5   # Maximum vertical velocity
# MAX_YAW_RATE = 30.0  # Maximum yaw rate in deg/s

# # Alignment thresholds (pixels and area)
# X_THRESHOLD = 20  # Pixel tolerance for X alignment
# Y_THRESHOLD = 20  # Pixel tolerance for Y alignment
# TARGET_AREA = 10000  # Target marker area in pixels (adjust based on desired distance)
# AREA_THRESHOLD = 2000  # Area tolerance


# class SIYICam:
#     def __init__(self, server_ip="192.168.144.25", port=37260):
#         self.server_ip = server_ip
#         self.cam = SIYISDK(server_ip=server_ip, port=port)
#         self.cap = None

#     def setup_cam(self):
#         """Connect to the camera, set motion mode, and adjust angles"""
#         print("🔧 Setting up camera...")
#         if not self.cam.connect():
#             print("❌ No connection to camera.")
#             exit(1)

#         self.cam.requestFollowMode()
#         sleep(2)

#         print("✅ Current motion mode:", self.cam._motionMode_msg.mode)

#         target_yaw_deg = 0.0
#         target_pitch_deg = -90.0
#         self.cam.requestSetAngles(target_yaw_deg, target_pitch_deg)

#         print("🎯 Attitude (yaw, pitch, roll):", self.cam.getAttitude())
#         sleep(2)

#         self.cam.disconnect()
#         print("🔌 Camera disconnected from control interface.")

#     def open_stream(self):
#         """Initialize RTSP stream and return a generator"""
#         gst_pipeline = (
#             f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency=10 ! "
#             "rtph265depay ! h265parse ! nvv4l2decoder ! "
#             "nvvidconv ! video/x-raw, format=BGRx ! "
#             "videoconvert ! video/x-raw, format=BGR ! "
#             "appsink drop=true sync=false"
#         )

#         print("📡 Opening RTSP stream...")
#         print(gst_pipeline)
#         self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

#         if not self.cap.isOpened():
#             raise RuntimeError("❌ Unable to open RTSP stream")

#         print("✅ RTSP stream opened successfully.")
#         sleep(2)  # Give camera time to stabilize

#     def get_frame(self):
#         """Get a single frame from the stream"""
#         if self.cap is None:
#             self.open_stream()

#         ret, frame = self.cap.read()
#         if not ret:
#             print("⚠️ Empty frame received")
#             return None
#         return frame

#     def release(self):
#         if self.cap:
#             self.cap.release()


# class ArucoDetector:
#     def __init__(self, dict_type=ARUCO_DICT, marker_size=MARKER_SIZE):
#         # dictionary (support multiple OpenCV versions)
#         try:
#             self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
#         except AttributeError:
#             # older API
#             self.aruco_dict = cv2.aruco.Dictionary_get(dict_type)

#         # detector parameters (use create() factory)
#         try:
#             self.aruco_params = cv2.aruco.DetectorParameters_create()
#         except AttributeError:
#             # Some builds may have alternate name; try the constructor but catch failure
#             try:
#                 self.aruco_params = cv2.aruco.DetectorParameters()
#             except Exception:
#                 # final fallback to empty dict (used by detectMarkers below)
#                 self.aruco_params = None

#         # prefer the high-level ArucoDetector if available (OpenCV >= 4.7-ish)
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
#         """Detect ArUco markers in the frame. Returns (corners, ids, rejected)"""
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         if self.use_aruco_detector and self.detector is not None:
#             # New-style detector (returns corners, ids, rejected)
#             corners, ids, rejected = self.detector.detectMarkers(gray)
#             return corners, ids, rejected
#         else:
#             # Fallback to detectMarkers API
#             # If self.aruco_params is None, pass only dictionary
#             if self.aruco_params is None:
#                 corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict)
#             else:
#                 corners, ids, rejected = cv2.aruco.detectMarkers(
#                     gray, self.aruco_dict, parameters=self.aruco_params
#                 )
#             return corners, ids, rejected

#     def draw_markers(self, frame, corners, ids):
#         """Draw detected markers on the frame"""
#         if ids is not None:
#             cv2.aruco.drawDetectedMarkers(frame, corners, ids)
#         return frame

#     def get_marker_center(self, corners):
#         """Calculate the center point of the first detected marker"""
#         if corners is None or len(corners) == 0:
#             return None

#         # Get the first marker's corners
#         corner = corners[0][0]
#         center_x = int(np.mean(corner[:, 0]))
#         center_y = int(np.mean(corner[:, 1]))

#         # Calculate marker area (for distance estimation)
#         area = cv2.contourArea(corner)

#         return center_x, center_y, area



# class DroneController:
#     def __init__(self):
#         self.drone = System()
#         self.is_aligned = False

#     async def connect(self, connection_string="serial:///dev/ttyACM0:57600"):
#         """Connect to the drone"""
#         print(f"🔗 Connecting to drone on {connection_string}...")
#         await self.drone.connect(system_address=connection_string)

#         print("⏳ Waiting for drone to connect...")
#         async for state in self.drone.core.connection_state():
#             if state.is_connected:
#                 print("✅ Drone connected!")
#                 break

#         # Wait for drone to be ready
#         # print("⏳ Waiting for drone to have a global position estimate...")
#         # async for health in self.drone.telemetry.health():
#         #     if health.is_global_position_ok and health.is_home_position_ok:
#         #         print("✅ Global position estimate OK")
#         #         break

#     async def start_offboard(self):
#         """Start offboard mode"""
#         print("🚁 Starting offboard mode...")
        
#         # Send initial setpoint before starting offboard
#         await self.drone.offboard.set_velocity_body(
#             VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
#         )
        
#         try:
#             await self.drone.offboard.start()
#             print("✅ Offboard mode started")
#         except OffboardError as e:
#             print(f"❌ Starting offboard mode failed: {e}")
#             return False
        
#         return True

#     async def stop_offboard(self):
#         """Stop offboard mode"""
#         try:
#             await self.drone.offboard.stop()
#             print("🛑 Offboard mode stopped")
#         except OffboardError as e:
#             print(f"⚠️ Stopping offboard mode failed: {e}")

    
#     async def send_velocity_command(self, vx, vy, vz, yaw_rate):
#         """Send velocity command to the drone"""
#         await self.drone.offboard.set_velocity_body(
#             VelocityBodyYawspeed(vx, vy, vz, yaw_rate)
#         )


#     def calculate_velocity_commands(self, frame_center, marker_center, frame_shape):
#         """
#         Map image errors to body-frame velocities for a downward-facing camera whose
#         'top' points to the drone +X (forward). Returns vx, vy, vz, yaw_rate.
#         Assumes VelocityBodyYawspeed expects vz positive DOWN (NED convention).
#         """

#         if marker_center is None:
#             # no marker -> zero motion
#             return 0.0, 0.0, 0.0, 0.0

#         frame_cx, frame_cy = frame_center
#         marker_x, marker_y, marker_area = marker_center
#         frame_h, frame_w = frame_shape

#         # pixel errors (image coords)
#         error_x = marker_x - frame_cx    # positive => marker is to the RIGHT in image
#         error_y = marker_y - frame_cy    # positive => marker is DOWN in image
#         error_area = TARGET_AREA - marker_area  # positive => marker smaller (too far/high)

#         # normalized errors
#         norm_error_x = error_x / frame_w    # right = +, left = -
#         norm_error_y = error_y / frame_h    # down = +, up = -
#         norm_error_area = error_area / TARGET_AREA

#         # Map normalized image errors -> body velocities:
#         # - vy (right) should follow image-right
#         vy = np.clip(KP_X * norm_error_x, -MAX_VEL_XY, MAX_VEL_XY)

#         # - vx (forward) maps to image-up (which is -norm_error_y)
#         #   So if marker is above center (norm_error_y negative) vx should be positive (move forward).
#         vx = np.clip(-KP_X * norm_error_y, -MAX_VEL_XY, MAX_VEL_XY)

#         # - vz (down positive): move down when marker area is smaller than target (norm_error_area > 0)
#         vz = np.clip(KP_Z * norm_error_area, -MAX_VEL_Z, MAX_VEL_Z) * 0.0  # disable vertical for now

#         # - yaw: rotate to reduce horizontal offset (image x). Sign may need flip depending on yaw sign convention.
#         #   Here we turn proportionally to image horizontal error: positive -> yaw right (clockwise).
#         yaw_rate = np.clip(KP_YAW * norm_error_x * 100.0, -MAX_YAW_RATE, MAX_YAW_RATE) * 0.0  # disable yaw for now

#         # aligned criteria (pixel and area thresholds)
#         self.is_aligned = (
#             abs(error_x) < X_THRESHOLD and
#             abs(error_y) < Y_THRESHOLD and
#             abs(error_area) < AREA_THRESHOLD
#         )

#         return vx, vy, vz, yaw_rate


# async def main():
#     # Initialize camera
#     cam = SIYICam()
#     cam.setup_cam()
#     sleep(1)
#     cam.open_stream()
#     sleep(2)  # Give camera more time to stabilize

#     # Initialize ArUco detector
#     detector = ArucoDetector()

#     # Initialize drone controller
#     drone = DroneController()
#     await drone.connect()  # Change connection string if needed
    
#     # Start offboard mode
#     if not await drone.start_offboard():
#         print("❌ Failed to start offboard mode. Exiting...")
#         cam.release()
#         cv2.destroyAllWindows()
#         return

#     print("\n" + "="*50)
#     print("🎯 ARUCO MARKER ALIGNMENT ACTIVE")
#     print("="*50)
#     print("📹 Press 'q' to quit")
#     print("="*50 + "\n")

#     try:
#         while True:
#             # Get frame from camera
#             frame = cam.get_frame()
#             if frame is None:
#                 continue

#             # Detect ArUco markers
#             corners, ids, _ = detector.detect(frame)
            
#             # Get frame dimensions
#             frame_h, frame_w = frame.shape[:2]
#             frame_center = (frame_w // 2, frame_h // 2)

#             # Calculate velocity commands
#             if ids is not None and len(ids) > 0:
#                 marker_center = detector.get_marker_center(corners)
#                 vx, vy, vz, yaw_rate = drone.calculate_velocity_commands(
#                     frame_center, marker_center, (frame_h, frame_w)
#                 )
                
#                 # Send velocity command
#                 await drone.send_velocity_command(vx, vy, vz, yaw_rate)

#                 # Display info
#                 if PREVIEW:
#                     frame = detector.draw_markers(frame, corners, ids)
                    
#                     # Draw frame center
#                     cv2.circle(frame, frame_center, 5, (0, 255, 0), -1)
                    
#                     # Draw marker center
#                     if marker_center:
#                         mx, my, area = marker_center
#                         cv2.circle(frame, (mx, my), 5, (0, 0, 255), -1)
#                         cv2.line(frame, frame_center, (mx, my), (255, 0, 0), 2)
                        
#                         # Display velocity commands
#                         info_text = [
#                             f"ID: {ids[0][0]}",
#                             f"vx: {vx:.2f} m/s",
#                             f"vy: {vy:.2f} m/s",
#                             f"vz: {vz:.2f} m/s",
#                             f"yaw: {yaw_rate:.2f} deg/s",
#                             f"Area: {area:.0f}",
#                             f"Aligned: {drone.is_aligned}"
#                         ]
                        
#                         y_offset = 30
#                         for i, text in enumerate(info_text):
#                             color = (0, 255, 0) if drone.is_aligned and i == len(info_text)-1 else (255, 255, 255)
#                             cv2.putText(frame, text, (10, y_offset + i*25),
#                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
#             else:
#                 # No marker detected, stop the drone
#                 await drone.send_velocity_command(0.0, 0.0, 0.0, 0.0)
                
#                 if PREVIEW:
#                     cv2.putText(frame, "NO MARKER DETECTED", (10, 30),
#                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

#             # Display frame
#             if PREVIEW:
#                 cv2.imshow("ArUco Detection", frame)
#                 if cv2.waitKey(1) & 0xFF == ord('q'):
#                     print("\n🛑 Quitting...")
#                     break

#             # Small delay to prevent overwhelming the system
#             await asyncio.sleep(0.05)  # 20 Hz update rate

#     except KeyboardInterrupt:
#         print("\n⚠️ Interrupted by user")
    
#     finally:
#         # Stop the drone
#         print("🛑 Stopping drone...")
#         await drone.send_velocity_command(0.0, 0.0, 0.0, 0.0)
#         await asyncio.sleep(0.5)
        
#         # Stop offboard mode
#         await drone.stop_offboard()
        
#         # Release resources
#         cam.release()
#         if PREVIEW:
#             cv2.destroyAllWindows()
        
#         print("✅ Cleanup complete")


# if __name__ == "__main__":
#     asyncio.run(main())











import sys
import os
from time import sleep
import cv2
import numpy as np
import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.telemetry import FlightMode

# ---- Add parent directory to path ----
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK

# ============ CONFIGURATION ============
PREVIEW = 0
ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_SIZE = 0.15

# P Controller Gains
KP_X = 4.5
KP_Y = 4.5
KP_Z = 0.5
KP_YAW = 0.8

# D Controller Gains (NEW)
KD_X = 0.5
KD_Y = 0.5

# Velocity limits
MAX_VEL_XY = 2.0
MAX_VEL_Z = 0.5
MAX_YAW_RATE = 30.0

# Alignment thresholds
X_THRESHOLD = 20
Y_THRESHOLD = 20
TARGET_AREA = 10000
AREA_THRESHOLD = 2000


class SIYICam:
    def __init__(self, server_ip="192.168.144.25", port=37260):
        self.server_ip = server_ip
        self.cam = SIYISDK(server_ip=server_ip, port=port)
        self.cap = None

    def setup_cam(self):
        print("🔧 Setting up camera...")
        if not self.cam.connect():
            print("❌ No connection to camera.")
            exit(1)

        self.cam.requestFollowMode()
        sleep(2)
        self.cam.requestSetAngles(0.0, -90.0)
        sleep(2)
        self.cam.disconnect()

    def open_stream(self):
        gst_pipeline = (
            f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency=10 ! "
            "rtph265depay ! h265parse ! nvv4l2decoder ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true sync=false"
        )
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("❌ Unable to open RTSP stream")

    def get_frame(self):
        ret, frame = self.cap.read()
        return frame if ret else None

    def release(self):
        if self.cap:
            self.cap.release()


class ArucoDetector:
    def __init__(self, dict_type=ARUCO_DICT):
        # 1. Get dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
        
        # 2. FIX: Use 'DetectorParameters_create()' for older OpenCV versions
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            # Fallback for very new versions just in case
            self.aruco_params = cv2.aruco.DetectorParameters()

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 3. FIX: Use the direct function 'detectMarkers' instead of the 'ArucoDetector' object
        # The 'cv2.aruco.ArucoDetector' class does not exist in older versions.
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, 
            self.aruco_dict, 
            parameters=self.aruco_params
        )
        return corners, ids, rejected

    def get_marker_center(self, corners):
        if corners is None or len(corners) == 0:
            return None
        corner = corners[0][0]
        center_x = int(np.mean(corner[:, 0]))
        center_y = int(np.mean(corner[:, 1]))
        area = cv2.contourArea(corner)
        return center_x, center_y, area


class DroneController:
    def __init__(self):
        self.drone = System()
        self.is_aligned = False
        self.current_mode = None

        # --- Previous error storage for D-term ---
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

    async def connect(self, connection_string="serial:///dev/ttyACM0:57600"):
        print(f"🔗 Connecting to drone on {connection_string}...")
        await self.drone.connect(system_address=connection_string)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("✅ Drone connected!")
                break

        asyncio.create_task(self.observe_flight_mode())

    async def observe_flight_mode(self):
        async for mode in self.drone.telemetry.flight_mode():
            if mode != self.current_mode:
                print(f"🕹️ Mode Switch: {mode}")
                self.current_mode = mode

    async def send_velocity_command(self, vx, vy, vz, yaw_rate):
        try:
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(vx, vy, vz, yaw_rate)
            )
        except OffboardError:
            pass

    def calculate_velocity_commands(self, frame_center, marker_center, frame_shape):
        if marker_center is None:
            return 0.0, 0.0, 0.0, 0.0

        frame_cx, frame_cy = frame_center
        marker_x, marker_y, marker_area = marker_center
        frame_h, frame_w = frame_shape

        # --- Normalized errors ---
        error_x = (marker_x - frame_cx) / frame_w
        error_y = (marker_y - frame_cy) / frame_h

        # --- Derivative terms ---
        d_error_x = error_x - self.prev_error_x
        d_error_y = error_y - self.prev_error_y

        self.prev_error_x = error_x
        self.prev_error_y = error_y

        # --- PD Control ---
        vy = KP_X * error_x + KD_X * d_error_x
        vx = -(KP_Y * error_y + KD_Y * d_error_y)

        vy = np.clip(vy, -MAX_VEL_XY, MAX_VEL_XY)
        vx = np.clip(vx, -MAX_VEL_XY, MAX_VEL_XY)

        vz = 0.0
        yaw_rate = 0.0

        self.is_aligned = (
            abs(marker_x - frame_cx) < X_THRESHOLD and
            abs(marker_y - frame_cy) < Y_THRESHOLD
        )
        print(f"vx: {vx:.2f}, vy: {vy:.2f}, Aligned: {self.is_aligned}")
        return vx, vy, vz, yaw_rate


async def main():
    cam = SIYICam()
    cam.setup_cam()
    cam.open_stream()

    detector = ArucoDetector()
    drone = DroneController()
    await drone.connect()

    print("\n" + "=" * 50)
    print("📡 STANDBY: Switch RC to OFFBOARD to begin tracking")
    print("=" * 50 + "\n")

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                continue

            corners, ids, _ = detector.detect(frame)
            frame_h, frame_w = frame.shape[:2]
            frame_center = (frame_w // 2, frame_h // 2)

            if drone.current_mode == FlightMode.OFFBOARD:
                if ids is not None and len(ids) > 0:
                    m_center = detector.get_marker_center(corners)
                    vx, vy, vz, yaw = drone.calculate_velocity_commands(
                        frame_center, m_center, (frame_h, frame_w)
                    )
                    await drone.send_velocity_command(vx, vy, vz, yaw)
                else:
                    await drone.send_velocity_command(0.0, 0.0, 0.0, 0.0)
            else:
                await drone.send_velocity_command(0.0, 0.0, 0.0, 0.0)

            if PREVIEW:
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                status_color = (0, 255, 0) if drone.current_mode == FlightMode.OFFBOARD else (0, 0, 255)
                cv2.putText(frame, f"MODE: {drone.current_mode}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

                cv2.imshow("Drone Tracker", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            await asyncio.sleep(0.05)

    except KeyboardInterrupt:
        print("\n⚠️ Interrupted")
    finally:
        cam.release()
        cv2.destroyAllWindows()
        print("✅ Cleanup complete")


if __name__ == "__main__":
    asyncio.run(main())
