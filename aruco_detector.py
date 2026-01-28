# """
# ArUco Marker Detector for Fire Detection Testing
# Detects ArUco markers in video frames and converts them to fire detection format.
# Extracted from align_drone.py for use in the drone detection pipeline.
# """
# import cv2
# import numpy as np
# import time


# class ArucoFireDetector:
#     """
#     Adapter for ArUco marker detection to work with fire detection pipeline.
#     Treats detected ArUco markers as "fire" detections for real drone testing.
#     """

#     def __init__(self, dict_type=cv2.aruco.DICT_4X4_50, marker_size=0.15):
#         """
#         Initialize ArUco fire detector.

#         Args:
#             dict_type: ArUco dictionary type (default: DICT_4X4_50)
#             marker_size: Physical marker size in meters (default: 0.15)
#         """
#         # Initialize ArUco dictionary (support multiple OpenCV versions)
#         try:
#             self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
#         except AttributeError:
#             self.aruco_dict = cv2.aruco.Dictionary_get(dict_type)

#         # Initialize detector parameters
#         try:
#             self.aruco_params = cv2.aruco.DetectorParameters_create()
#         except AttributeError:
#             try:
#                 self.aruco_params = cv2.aruco.DetectorParameters()
#             except Exception:
#                 self.aruco_params = None

#         # Try to use new-style ArucoDetector if available (OpenCV >= 4.7)
#         self.use_aruco_detector = False
#         try:
#             if hasattr(cv2.aruco, "ArucoDetector") and self.aruco_params is not None:
#                 self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
#                 self.use_aruco_detector = True
#         except Exception:
#             self.detector = None
#             self.use_aruco_detector = False

#         self.marker_size = marker_size
#         self.last_detection_time = time.time()
#         self.min_detection_interval = 4.0  # Minimum 4 seconds between detections

#     def detect(self, frame, drone_lat=None, drone_lon=None, drone_alt=None):
#         """
#         Detect ArUco markers and return them as fire detections.

#         Args:
#             frame: OpenCV image (numpy array)
#             drone_lat: Current drone latitude
#             drone_lon: Current drone longitude
#             drone_alt: Current drone altitude in meters

#         Returns:
#             Detection dict with format:
#             {
#                 'bbox': [[p1_x, p1_y, p2_x, p2_y, lat, lon], ...],
#                 'confidence': float,
#                 'timestamp': float
#             }
#             Returns None if no markers detected.
#         """
#         timestamp = time.time()

#         # Check if enough time has passed since last detection
#         time_since_last_detection = timestamp - self.last_detection_time
#         if time_since_last_detection < self.min_detection_interval:
#             return None

#         # Detect ArUco markers
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         if self.use_aruco_detector and self.detector is not None:
#             corners, ids, _ = self.detector.detectMarkers(gray)
#         else:
#             if self.aruco_params is None:
#                 corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)
#             else:
#                 corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

#         # No markers detected
#         if ids is None or len(ids) == 0:
#             return None

#         # Update last detection time
#         self.last_detection_time = timestamp

#         # Convert markers to detection format
#         height, width = frame.shape[:2]
#         bbox_list = []
#         confidences = []

#         for i, corner in enumerate(corners):
#             # Get bounding box from marker corners
#             corner_points = corner[0]  # Shape: (4, 2)

#             # Calculate bounding box
#             x_coords = corner_points[:, 0]
#             y_coords = corner_points[:, 1]
#             p1_x = int(np.min(x_coords))
#             p1_y = int(np.min(y_coords))
#             p2_x = int(np.max(x_coords))
#             p2_y = int(np.max(y_coords))

#             # Calculate center of marker
#             center_x = int(np.mean(x_coords))
#             center_y = int(np.mean(y_coords))

#             # Estimate world coordinates based on marker position in frame
#             if drone_lat and drone_lon and drone_alt:
#                 # Offset from drone position based on pixel position
#                 lat_offset = (center_x - width / 2) / width * 0.001
#                 lon_offset = (center_y - height / 2) / height * 0.001
#                 world_lat = drone_lat + lat_offset
#                 world_lon = drone_lon + lon_offset
#             else:
#                 world_lat = drone_lat or 0.0
#                 world_lon = drone_lon or 0.0

#             # ArUco detection is very reliable, so high confidence
#             confidence = 0.95
#             confidences.append(confidence)

#             # Format: [p1_x, p1_y, p2_x, p2_y, lat, lon]
#             bbox_list.append([p1_x, p1_y, p2_x, p2_y, world_lat, world_lon])

#         # Return None if no valid detections
#         if not bbox_list:
#             return None

#         # Calculate average confidence
#         avg_confidence = sum(confidences) / len(confidences)

#         return {
#             'bbox': bbox_list,
#             'confidence': round(avg_confidence, 2),
#             'timestamp': timestamp
#         }

#     def draw_detections(self, frame, detection_result):
#         """
#         Draw bounding boxes and labels on frame.

#         Args:
#             frame: OpenCV image
#             detection_result: Detection dict

#         Returns:
#             Frame with drawn detections
#         """
#         if not detection_result or not detection_result.get('bbox'):
#             return frame

#         frame_copy = frame.copy()
#         bbox_list = detection_result['bbox']
#         confidence = detection_result['confidence']

#         for idx, bbox in enumerate(bbox_list):
#             p1_x, p1_y, p2_x, p2_y, world_lat, world_lon = bbox

#             # Draw box (red for "fire")
#             cv2.rectangle(frame_copy, (p1_x, p1_y), (p2_x, p2_y), (0, 0, 255), 2)

#             # Draw label
#             label = f"ArUco Fire {idx+1} ({confidence:.2f})"
#             cv2.putText(frame_copy, label, (p1_x, p1_y - 10),
#                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

#             # Draw GPS coordinates
#             gps_text = f"({world_lat:.6f}, {world_lon:.6f})"
#             cv2.putText(frame_copy, gps_text, (p1_x, p2_y + 20),
#                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

#         return frame_copy


# # Example usage for testing
# if __name__ == "__main__":
#     import sys

#     print("ArUco Fire Detector Test")
#     print("=" * 50)

#     # Initialize detector
#     detector = ArucoFireDetector()
#     print("✅ Detector initialized")

#     # Try to open webcam
#     cap = cv2.VideoCapture(0)

#     if not cap.isOpened():
#         print("❌ Could not open webcam. Creating dummy test frame...")
#         frame = np.zeros((480, 640, 3), dtype=np.uint8)
#         cv2.putText(frame, "No camera - Print ArUco marker to test", (50, 240),
#                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
#         cv2.imshow("ArUco Fire Detection Test", frame)
#         cv2.waitKey(3000)
#         sys.exit(0)

#     print("✅ Webcam opened successfully")
#     print("\n📷 Instructions:")
#     print("   - Show an ArUco marker (DICT_4X4_50) to the camera")
#     print("   - Detection will trigger every 4 seconds")
#     print("   - Press 'q' to quit\n")

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("⚠️ Failed to read frame")
#             break

#         # Simulate detection with dummy GPS coords (Kolkata)
#         detection_result = detector.detect(
#             frame,
#             drone_lat=22.5726,
#             drone_lon=88.3639,
#             drone_alt=50.0
#         )

#         # Draw detections if found
#         if detection_result:
#             bbox_list = detection_result['bbox']
#             print(f"🔥 Detected {len(bbox_list)} ArUco marker(s):")
#             print(f"   Confidence: {detection_result['confidence']}")
#             for idx, bbox in enumerate(bbox_list):
#                 p1_x, p1_y, p2_x, p2_y, lat, lon = bbox
#                 print(f"   Marker {idx+1}: Box=({p1_x},{p1_y})->({p2_x},{p2_y}), GPS=({lat:.6f}, {lon:.6f})")

#             frame = detector.draw_detections(frame, detection_result)

#         cv2.imshow("ArUco Fire Detection Test", frame)

#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()
#     print("\n✅ Test complete")




"""
ArUco Marker Detector for Fire Detection Testing
Detects ArUco markers in video frames and converts them to fire detection format.
Extracted from align_drone.py for use in the drone detection pipeline.
"""
import cv2
import numpy as np
import time
import psutil
import os
import logging
from pathlib import Path
from collections import defaultdict, deque


class ArucoFireDetector:
    """
    Adapter for ArUco marker detection to work with fire detection pipeline.
    Treats detected ArUco markers as "fire" detections for real drone testing.
    """

    def __init__(self, dict_type=cv2.aruco.DICT_4X4_50, marker_size=0.15, detection_interval=0.05):
        """
        Initialize ArUco fire detector.

        Args:
            dict_type: ArUco dictionary type (default: DICT_4X4_50)
            marker_size: Physical marker size in meters (default: 0.15)
            detection_interval: Minimum seconds between detections (default: 1.0)
        """
        # Initialize ArUco dictionary (support multiple OpenCV versions)
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(dict_type)

        # Initialize detector parameters
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            try:
                self.aruco_params = cv2.aruco.DetectorParameters()
            except Exception:
                self.aruco_params = None

        # Try to use new-style ArucoDetector if available (OpenCV >= 4.7)
        self.use_aruco_detector = False
        try:
            if hasattr(cv2.aruco, "ArucoDetector") and self.aruco_params is not None:
                self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
                self.use_aruco_detector = True
        except Exception:
            self.detector = None
            self.use_aruco_detector = False

        self.marker_size = marker_size
        self.last_detection_time = time.time()  # Initialize to current time for proper interval calculation
        self.min_detection_interval = detection_interval  # Configurable detection interval

        # Performance tracking
        self.process = psutil.Process(os.getpid())
        self.perf_stats = {
            "total_frames_processed": 0,
            "total_detections": 0,
            "frames_skipped": 0,
            "detect_times": deque(maxlen=200),  # Bounded deque prevents memory leak
            "gray_conversion_times": deque(maxlen=200),
            "marker_detection_times": deque(maxlen=200),
            "bbox_calculation_times": deque(maxlen=200),
            "last_stats_log_time": time.time()
        }

        # Setup file logger for performance stats
        logs_dir = Path("logs")
        logs_dir.mkdir(exist_ok=True)

        self.stats_logger = logging.getLogger("aruco_detector_stats")
        self.stats_logger.setLevel(logging.INFO)
        self.stats_logger.handlers = []  # Clear existing handlers

        file_handler = logging.FileHandler(logs_dir / "aruco_detector_performance.log")
        file_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
        self.stats_logger.addHandler(file_handler)

        print(f"📊 ArUco Detector Performance Logging Enabled → logs/aruco_detector_performance.log")

    def detect(self, frame, drone_lat=None, drone_lon=None, drone_alt=None):
        """
        Detect ArUco markers and return them as fire detections.

        Args:
            frame: OpenCV image (numpy array)
            drone_lat: Current drone latitude
            drone_lon: Current drone longitude
            drone_alt: Current drone altitude in meters

        Returns:
            Detection dict with format:
            {
                'bbox': [[p1_x, p1_y, p2_x, p2_y, lat, lon], ...],
                'confidence': float,
                'timestamp': float
            }
            Returns None if no markers detected.
        """
        # Start overall timing
        detect_start = time.time()
        timestamp = time.time()

        # Track frames processed
        self.perf_stats["total_frames_processed"] += 1

        # Get resource usage at start
        mem_before = self.process.memory_info().rss / 1024 / 1024  # MB
        cpu_percent_before = self.process.cpu_percent(interval=None)

        # Time-based rate limiting (handled by caller now, but keep as safety)
        # This is now just a backup check - main timing is in script.py
        time_since_last_detection = timestamp - self.last_detection_time
        if time_since_last_detection < self.min_detection_interval:
            self.perf_stats["frames_skipped"] += 1
            self._log_periodic_stats()
            return None

        # Step 1: Gray conversion
        gray_start = time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_time = time.time() - gray_start
        self.perf_stats["gray_conversion_times"].append(gray_time)

        # Step 2: Marker detection
        marker_start = time.time()
        if self.use_aruco_detector and self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            if self.aruco_params is None:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        marker_time = time.time() - marker_start
        self.perf_stats["marker_detection_times"].append(marker_time)

        # No markers detected
        if ids is None or len(ids) == 0:
            detect_total = time.time() - detect_start
            self.perf_stats["detect_times"].append(detect_total)
            self._log_periodic_stats()
            return None

        # Update last detection time
        self.last_detection_time = timestamp
        self.perf_stats["total_detections"] += len(ids)

        # Step 3: Bounding box calculation
        bbox_start = time.time()
        height, width = frame.shape[:2]
        bbox_list = []
        confidences = []

        for i, corner in enumerate(corners):
            # Get bounding box from marker corners
            corner_points = corner[0]  # Shape: (4, 2)

            # Calculate bounding box
            x_coords = corner_points[:, 0]
            y_coords = corner_points[:, 1]
            p1_x = int(np.min(x_coords))
            p1_y = int(np.min(y_coords))
            p2_x = int(np.max(x_coords))
            p2_y = int(np.max(y_coords))

            # Calculate center of marker
            center_x = int(np.mean(x_coords))
            center_y = int(np.mean(y_coords))

            # For testing: always return drone GPS coordinates
            world_lat = drone_lat or 0.0
            world_lon = drone_lon or 0.0

            # ArUco detection is very reliable, so high confidence
            confidence = 0.95
            confidences.append(confidence)

            # Format: [p1_x, p1_y, p2_x, p2_y, lat, lon]
            bbox_list.append([p1_x, p1_y, p2_x, p2_y, world_lat, world_lon])

        bbox_time = time.time() - bbox_start
        self.perf_stats["bbox_calculation_times"].append(bbox_time)

        # Total detection time
        detect_total = time.time() - detect_start
        self.perf_stats["detect_times"].append(detect_total)

        # Get resource usage after
        mem_after = self.process.memory_info().rss / 1024 / 1024  # MB
        cpu_percent_after = self.process.cpu_percent(interval=None)

        # Log this detection
        print(f"🔍 ArUco Detection: {len(bbox_list)} marker(s) | "
              f"Total: {detect_total*1000:.1f}ms | "
              f"Gray: {gray_time*1000:.1f}ms | "
              f"Detect: {marker_time*1000:.1f}ms | "
              f"BBox: {bbox_time*1000:.1f}ms | "
              f"Mem: {mem_after:.1f}MB | "
              f"CPU: {cpu_percent_after:.1f}%")

        # Periodic stats logging
        self._log_periodic_stats()

        # Return None if no valid detections
        if not bbox_list:
            return None

        # Calculate average confidence
        avg_confidence = sum(confidences) / len(confidences)

        return {
            'bbox': bbox_list,
            'confidence': round(avg_confidence, 2),
            'timestamp': timestamp
        }

    def _log_periodic_stats(self):
        """Log comprehensive stats every 10 seconds"""
        current_time = time.time()
        elapsed = current_time - self.perf_stats["last_stats_log_time"]

        if elapsed >= 3.0:  # Log every 3 seconds
            # Calculate averages
            avg_detect_time = np.mean(self.perf_stats["detect_times"]) if self.perf_stats["detect_times"] else 0
            avg_gray_time = np.mean(self.perf_stats["gray_conversion_times"]) if self.perf_stats["gray_conversion_times"] else 0
            avg_marker_time = np.mean(self.perf_stats["marker_detection_times"]) if self.perf_stats["marker_detection_times"] else 0
            avg_bbox_time = np.mean(self.perf_stats["bbox_calculation_times"]) if self.perf_stats["bbox_calculation_times"] else 0

            # Calculate FPS
            fps = self.perf_stats["total_frames_processed"] / elapsed if elapsed > 0 else 0
            detections_per_sec = self.perf_stats["total_detections"] / elapsed if elapsed > 0 else 0

            # Get current resource usage
            mem_current = self.process.memory_info().rss / 1024 / 1024  # MB
            cpu_percent = self.process.cpu_percent(interval=0.1)

            # Log to file
            self.stats_logger.info(f"\n{'='*80}")
            self.stats_logger.info(f"ArUco Detector Performance Stats (last {elapsed:.1f}s):")
            self.stats_logger.info(f"  Frames Processed: {self.perf_stats['total_frames_processed']} ({fps:.1f} FPS)")
            self.stats_logger.info(f"  Frames Skipped (rate limit): {self.perf_stats['frames_skipped']}")
            self.stats_logger.info(f"  Total Detections: {self.perf_stats['total_detections']} ({detections_per_sec:.1f}/sec)")
            self.stats_logger.info(f"  Avg Detection Time: {avg_detect_time*1000:.2f}ms")
            self.stats_logger.info(f"    - Gray Conversion: {avg_gray_time*1000:.2f}ms")
            self.stats_logger.info(f"    - Marker Detection: {avg_marker_time*1000:.2f}ms")
            self.stats_logger.info(f"    - BBox Calculation: {avg_bbox_time*1000:.2f}ms")
            self.stats_logger.info(f"  Memory Usage: {mem_current:.1f} MB")
            self.stats_logger.info(f"  CPU Usage: {cpu_percent:.1f}%")

            # Console summary
            print(f"📊 ArUco Stats: {fps:.1f} FPS | {detections_per_sec:.1f} det/s | "
                  f"{avg_detect_time*1000:.1f}ms avg | {mem_current:.1f}MB | {cpu_percent:.1f}% CPU")

            # Reset counters (deques auto-evict, no need to clear)
            self.perf_stats["total_frames_processed"] = 0
            self.perf_stats["total_detections"] = 0
            self.perf_stats["frames_skipped"] = 0
            self.perf_stats["last_stats_log_time"] = current_time

    def draw_detections(self, frame, detection_result):
        """
        Draw bounding boxes and labels on frame.

        Args:
            frame: OpenCV image
            detection_result: Detection dict

        Returns:
            Frame with drawn detections
        """
        if not detection_result or not detection_result.get('bbox'):
            return frame

        frame_copy = frame.copy()
        bbox_list = detection_result['bbox']
        confidence = detection_result['confidence']

        for idx, bbox in enumerate(bbox_list):
            p1_x, p1_y, p2_x, p2_y, world_lat, world_lon = bbox

            # Draw box (red for "fire")
            cv2.rectangle(frame_copy, (p1_x, p1_y), (p2_x, p2_y), (0, 0, 255), 2)

            # Draw label
            label = f"ArUco Fire {idx+1} ({confidence:.2f})"
            cv2.putText(frame_copy, label, (p1_x, p1_y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Draw GPS coordinates
            gps_text = f"({world_lat:.6f}, {world_lon:.6f})"
            cv2.putText(frame_copy, gps_text, (p1_x, p2_y + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        return frame_copy


# Example usage for testing
if __name__ == "__main__":
    import sys

    print("ArUco Fire Detector Test")
    print("=" * 50)

    # Initialize detector
    detector = ArucoFireDetector()
    print("✅ Detector initialized")

    # Try to open webcam
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ Could not open webcam. Creating dummy test frame...")
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, "No camera - Print ArUco marker to test", (50, 240),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("ArUco Fire Detection Test", frame)
        cv2.waitKey(3000)
        sys.exit(0)

    print("✅ Webcam opened successfully")
    print("\n📷 Instructions:")
    print("   - Show an ArUco marker (DICT_4X4_50) to the camera")
    print("   - Detection will trigger every 4 seconds")
    print("   - Press 'q' to quit\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Failed to read frame")
            break

        # Simulate detection with dummy GPS coords (Kolkata)
        detection_result = detector.detect(
            frame,
            drone_lat=22.5726,
            drone_lon=88.3639,
            drone_alt=50.0
        )

        # Draw detections if found
        if detection_result:
            bbox_list = detection_result['bbox']
            print(f"🔥 Detected {len(bbox_list)} ArUco marker(s):")
            print(f"   Confidence: {detection_result['confidence']}")
            for idx, bbox in enumerate(bbox_list):
                p1_x, p1_y, p2_x, p2_y, lat, lon = bbox
                print(f"   Marker {idx+1}: Box=({p1_x},{p1_y})->({p2_x},{p2_y}), GPS=({lat:.6f}, {lon:.6f})")

            frame = detector.draw_detections(frame, detection_result)

        cv2.imshow("ArUco Fire Detection Test", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("\n✅ Test complete")
