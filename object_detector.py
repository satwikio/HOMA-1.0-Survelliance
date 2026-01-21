"""
ArUco Marker Detector for Fire Detection Testing
Detects ArUco markers in video frames and converts them to fire detection format.
Extracted from align_drone.py for use in the drone detection pipeline.
"""
import cv2
import numpy as np
import time


class ArucoFireDetector:
    """
    Adapter for ArUco marker detection to work with fire detection pipeline.
    Treats detected ArUco markers as "fire" detections for real drone testing.
    """

    def __init__(self, dict_type=cv2.aruco.DICT_4X4_50, marker_size=0.15):
        """
        Initialize ArUco fire detector.

        Args:
            dict_type: ArUco dictionary type (default: DICT_4X4_50)
            marker_size: Physical marker size in meters (default: 0.15)
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
        self.last_detection_time = time.time()
        self.min_detection_interval = 4.0  # Minimum 4 seconds between detections

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
        timestamp = time.time()

        # Check if enough time has passed since last detection
        time_since_last_detection = timestamp - self.last_detection_time
        if time_since_last_detection < self.min_detection_interval:
            return None

        # Detect ArUco markers
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.use_aruco_detector and self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            if self.aruco_params is None:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # No markers detected
        if ids is None or len(ids) == 0:
            return None

        # Update last detection time
        self.last_detection_time = timestamp

        # Convert markers to detection format
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

            # Estimate world coordinates based on marker position in frame
            if drone_lat and drone_lon and drone_alt:
                # Offset from drone position based on pixel position
                lat_offset = (center_x - width / 2) / width * 0.001
                lon_offset = (center_y - height / 2) / height * 0.001
                world_lat = drone_lat + lat_offset
                world_lon = drone_lon + lon_offset
            else:
                world_lat = drone_lat or 0.0
                world_lon = drone_lon or 0.0

            # ArUco detection is very reliable, so high confidence
            confidence = 0.95
            confidences.append(confidence)

            # Format: [p1_x, p1_y, p2_x, p2_y, lat, lon]
            bbox_list.append([p1_x, p1_y, p2_x, p2_y, world_lat, world_lon])

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
