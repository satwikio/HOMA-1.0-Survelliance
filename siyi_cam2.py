









# import os
# import sys
# import threading
# import time
# import cv2
# from time import sleep

# # ---- Add parent directory to path ----
# current = os.path.dirname(os.path.realpath(__file__))
# parent_directory = os.path.dirname(current)
# sys.path.append(parent_directory)

# from siyi_sdk import SIYISDK


# class SIYICam:
#     def __init__(self, server_ip="192.168.144.25", port=37260):
#         self.server_ip = server_ip
#         self.cam = SIYISDK(server_ip=server_ip, port=port)

#         self.cap = None
#         self.running = False
#         self.latest_frame = None
#         self.frame_lock = threading.Lock()
#         self.reader_thread = None

#     def connect(self):
#         if not self.cam.connect():
#             print("❌ No connection to camera.")
#             exit(1)

#     def move(self, yaw_deg, pitch_deg):
#         print("Moving camera to yaw:", yaw_deg, "pitch:", pitch_deg)
#         self.cam.requestSetAngles(yaw_deg, pitch_deg)

#     def zoom(self, zoom_level):
#         self.cam.requestAbsoluteZoom(zoom_level)


#     # ---------------- CAMERA CONTROL ----------------
#     def setup_cam(self):
#         """Connect once to control interface and set angles."""
#         if not self.cam.connect():
#             raise RuntimeError("❌ Cannot connect to SIYI camera")

#         self.cam.requestFollowMode()
#         sleep(1)

#         print("✅ Motion mode:", self.cam._motionMode_msg.mode)

#         target_yaw_deg = 0.0
#         target_pitch_deg = -90.0
#         self.cam.requestSetAngles(target_yaw_deg, target_pitch_deg)

#         print("🎯 Camera attitude:", self.cam.getAttitude())
#         sleep(1)

#         # self.cam.disconnect()
#         print("🔌 Control interface disconnected")



#     # ---------------- STREAM ----------------
#     def open_stream(self):
#         if self.cap is not None:
#             return

#         gst_pipeline = (
#             f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency=10 ! "
#             "rtph265depay ! h265parse ! nvv4l2decoder ! "
#             "nvvidconv ! video/x-raw, format=BGRx ! "
#             "videoconvert ! video/x-raw, format=BGR ! "
#             "appsink drop=true sync=false"
#         )

#         print("📡 Opening GStreamer pipeline:\n", gst_pipeline)

#         self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

#         if not self.cap.isOpened():
#             raise RuntimeError("❌ Failed to open RTSP stream")

#         print("✅ RTSP stream opened")

#     # ---------------- THREAD ----------------
#     def _reader_loop(self):
#         """Blocking frame reader (runs in background thread)."""
#         while self.running:
#             ret, frame = self.cap.read()
#             if not ret:
#                 time.sleep(0.01)
#                 continue

#             with self.frame_lock:
#                 self.latest_frame = frame

#         print("🛑 Frame reader thread stopped")

#     def start(self):
#         """Start background frame capture."""
#         self.open_stream()
#         self.running = True
#         self.reader_thread = threading.Thread(
#             target=self._reader_loop,
#             daemon=True
#         )
#         self.reader_thread.start()
#         print("▶️ Frame capture started")

#     def get_frame(self):
#         """Non-blocking latest-frame access (async-safe)."""
#         with self.frame_lock:
#             return self.latest_frame

#     # ---------------- CLEANUP ----------------
#     def stop(self):
#         self.running = False
#         if self.reader_thread:
#             self.reader_thread.join(timeout=2)

#         if self.cap:
#             self.cap.release()

#         print("🧹 Camera resources released")
















import os
import sys
import threading
import time
import cv2
import queue
from datetime import datetime
from time import sleep
import json
import piexif

# ---- Add parent directory to path ----
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK
try:
    from siyi_sdk.siyi_message import COMMAND
except ImportError:
    # Fallback if specific path fails (e.g. running from source)
    try:
        from siyi_message import COMMAND
    except ImportError:
        print("⚠️  Could not import COMMAND from siyi_message. Using headers.")

class SafeSIYISDK(SIYISDK):
    """
    Wrapper around SIYISDK to suppress infinite warning loops when camera is disconnected.
    Also adds robustness to the receive loop.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._last_error_log_time = 0
        self._error_log_interval = 10.0  # Log errors at most once every 10 seconds
        self._consecutive_errors = 0

    def bufferCallback(self):
        """
        Overridden to catch 'Bad file descriptor' errors preventing log spam.
        And preventing CPU spin on persistent errors.
        """
        try:
            buff, addr = self._socket.recvfrom(self._BUFF_SIZE)
        except Exception as e:
            # Handle Bad file descriptor specifically to avoid infinite spam
            err_str = str(e)
            
            # Check if it's the specific error we want to suppress
            is_bad_fd = "[Errno 9] Bad file descriptor" in err_str
            
            now = time.time()
            if now - self._last_error_log_time > self._error_log_interval:
                if is_bad_fd:
                    # Log fully if interval passed
                    self._logger.warning("%s. Did not receive message within %s second(s) (suppressed partial logs)", e, self._rcv_wait_t)
                else:
                    self._logger.error(f"[bufferCallback] {e}")
                
                self._last_error_log_time = now
                self._consecutive_errors = 0
            else:
                self._consecutive_errors += 1
            
            # Prevent CPU spin if we are hitting immediate errors
            if self._consecutive_errors > 5:
                time.sleep(0.1)
            
            return

        # --- Re-implementation of message parsing/dispatching from base class ---
        
        buff_str = buff.hex()
        # self._logger.debug("Buffer: %s", buff_str) 

        # 10 bytes: STX+CTRL+Data_len+SEQ+CMD_ID+CRC16
        #            2 + 1  +    2   + 2 +   1  + 2
        MINIMUM_DATA_LENGTH = 10*2
        HEADER = '5566'

        # Go through the buffer
        while(len(buff_str) >= MINIMUM_DATA_LENGTH):
            if buff_str[0:4] != HEADER:
                # Remove the 1st element and continue 
                tmp = buff_str[1:]
                buff_str = tmp
                continue

            # Now we got minimum amount of data. Check if we have enough
            low_b = buff_str[6:8] # low byte
            high_b = buff_str[8:10] # high byte
            data_len_hex = high_b + low_b
            data_len = int('0x' + data_len_hex, base=16)
            char_len = data_len * 2

            # Check if there is enough data (including payload)
            if(len(buff_str) < (MINIMUM_DATA_LENGTH + char_len)):
                # No useful data
                buff_str = ''
                break
            
            packet = buff_str[0:MINIMUM_DATA_LENGTH + char_len]
            buff_str = buff_str[MINIMUM_DATA_LENGTH + char_len:]

            # Finally decode the packet!
            val = self._in_msg.decodeMsg(packet)
            if val is None:
                continue
            
            data, data_len, cmd_id, seq = val[0], val[1], val[2], val[3]

            # Use COMMAND class for ID matching
            if cmd_id == COMMAND.ACQUIRE_FW_VER:
                self.parseFirmwareMsg(data, seq)
            elif cmd_id == COMMAND.ACQUIRE_HW_ID:
                self.parseHardwareIDMsg(data, seq)
            elif cmd_id == COMMAND.ACQUIRE_GIMBAL_INFO:
                self.parseGimbalInfoMsg(data, seq)
            elif cmd_id == COMMAND.ACQUIRE_GIMBAL_ATT:
                self.parseAttitudeMsg(data, seq)
            elif cmd_id == COMMAND.FUNC_FEEDBACK_INFO:
                self.parseFunctionFeedbackMsg(data, seq)
            elif cmd_id == COMMAND.GIMBAL_SPEED:
                self.parseGimbalSpeedMsg(data, seq)
            elif cmd_id == COMMAND.AUTO_FOCUS:
                self.parseAutoFocusMsg(data, seq)
            elif cmd_id == COMMAND.MANUAL_FOCUS:
                self.parseManualFocusMsg(data, seq)
            elif cmd_id == COMMAND.MANUAL_ZOOM:
                self.parseZoomMsg(data, seq)
            elif cmd_id == COMMAND.CENTER:
                self.parseGimbalCenterMsg(data, seq)
            elif cmd_id == COMMAND.SET_GIMBAL_ATTITUDE:
                self.parseSetGimbalAnglesMsg(data, seq)
            elif cmd_id == COMMAND.SET_DATA_STREAM:
                self.parseRequestStreamMsg()
            elif cmd_id == COMMAND.CURRENT_ZOOM_VALUE:
                self.parseCurrentZoomLevelMsg(data, seq)
            else:
                pass
                # self._logger.warning("CMD ID is not recognized")

class SIYICam:
    def __init__(self, server_ip="192.168.144.25", port=37260, telemetry_provider=None, camera_state_callback=None):
        self.server_ip = server_ip
        # Use SafeSIYISDK instead of standard SIYISDK
        self.cam = SafeSIYISDK(server_ip=server_ip, port=port)

        self.cap = None
        self.running = False
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.reader_thread = None
        self.telemetry_provider = telemetry_provider
        
        # --- Camera connection state tracking ---
        self.is_connected = False
        self.camera_state_callback = camera_state_callback  # Callback to notify drone.py
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 15  # Max attempts before cooldown (increased)
        self.reconnect_backoff_base = 2.0  # Base delay for exponential backoff (reduced from 3.0)
        self.cooldown_period = 30.0  # Cooldown after max attempts
        self.last_reconnect_attempt = 0
        
        # --- Frame health tracking ---
        self.last_frame_time = 0
        self.frame_timeout = 1.0  # 1 second timeout for stale frames
        self.reconnect_delay = 3.0  # Wait 3s before reconnecting (deprecated, using backoff)

        # --- Recording variables ---
        self.is_recording = False
        self.record_thread = None
        self.is_recording = False
        self.record_thread = None
        self.save_queue = queue.Queue()
        self.recording_path = ""
        
        # --- Stream Configuration ---
        self.stream_name = "main.264"  # Default to main stream (HD)

    # ... [connect, move, zoom, setup_cam functions remain the same] ...

    def set_telemetry_provider(self, provider):
        """Set or update the telemetry provider callback"""
        self.telemetry_provider = provider
        print(f"📊 Telemetry provider {'updated' if provider else 'cleared'}")
    
    def set_camera_state_callback(self, callback):
        """Set callback to notify when camera connection state changes"""
        self.camera_state_callback = callback
        print(f"📸 Camera state callback {'set' if callback else 'cleared'}")
    
    def _notify_camera_state_change(self, is_connected):
        """Internal method to notify state changes"""
        if self.is_connected != is_connected:
            self.is_connected = is_connected
            if self.camera_state_callback:
                try:
                    self.camera_state_callback(is_connected)
                except Exception as e:
                    print(f"⚠️ Camera state callback error: {e}")

    def connect(self):
        if not self.cam.connect():
            print("❌ No connection to camera.")
            exit(1)

    def move(self, yaw_deg, pitch_deg):
        print("Moving camera to yaw:", yaw_deg, "pitch:", pitch_deg)
        self.cam.requestSetAngles(yaw_deg, pitch_deg)

    def zoom(self, zoom_level):
        self.cam.requestAbsoluteZoom(zoom_level)

    def setup_cam(self):
        print("🔌 Connecting to SIYI camera...")
        if not self.cam.connect():
            raise RuntimeError("❌ Cannot connect to SIYI camera")
        self.cam.requestFollowMode()
        sleep(1)
        target_yaw_deg = 0.0
        target_pitch_deg = -0.0
        self.cam.requestSetAngles(target_yaw_deg, target_pitch_deg)
        sleep(1)

    # ---------------- RECORDING LOGIC ----------------

    def start_recording(self):
        """Initializes the folder structure and starts the background saving thread."""
        if self.is_recording:
            print("⚠️ Already recording.")
            return

        # Generate paths: captures/YYYY-MM-DD/HH_MM_SS
        now = datetime.now()
        date_folder = now.strftime("%Y-%m-%d")
        time_folder = now.strftime("%H_%M_%S")
        
        self.recording_path = os.path.join("captures", date_folder, time_folder)
        os.makedirs(self.recording_path, exist_ok=True)

        self.is_recording = True
        self.record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self.record_thread.start()
        print(f"📸 Recording started: {self.recording_path}")

    def stop_recording(self):
        """Stops the recording loop."""
        self.is_recording = False
        if self.record_thread:
            self.record_thread.join()
        print("🛑 Recording stopped and saved.")

    # def _record_loop(self):
    #     """Background thread that pulls frames from the queue and saves them to disk."""
    #     frame_count = 0
    #     while self.is_recording or not self.save_queue.empty():
    #         try:
    #             # Get frame from queue with a timeout so it can exit loop if recording stops
    #             frame = self.save_queue.get(timeout=1)
                
    #             # Generate filename: HH_MM_SS_frame#.jpg
    #             timestamp = datetime.now().strftime("%H_%M_%S")
    #             filename = f"{timestamp}_{frame_count:04d}.jpg"
    #             filepath = os.path.join(self.recording_path, filename)

    #             cv2.imwrite(filepath, frame)
    #             frame_count += 1
    #             self.save_queue.task_done()
    #         except queue.Empty:
    #             continue

    def _record_loop(self):
        """
        Background thread with non-blocking async I/O for frame saving.
        Uses ThreadPoolExecutor for parallel disk writes.
        """
        from concurrent.futures import ThreadPoolExecutor
        
        frame_count_per_second = {}  # Track frames per second
        executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="frame_saver")
        
        def save_frame_with_metadata(frame, filepath, telemetry, frame_num_in_second):
            try:
                cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])

                exif_dict = {"0th": {}}

                exif_dict["0th"][piexif.ImageIFD.ImageDescription] = json.dumps({
                    "latitude": telemetry.get("lat"),
                    "longitude": telemetry.get("lng"),
                    "battery": telemetry.get("battery", 0),
                    "speed": telemetry.get("speed", 0),
                    "heading": telemetry.get("heading", 0),
                    "frame_num": frame_num_in_second,
                    "telemetry_age_ms": telemetry.get("age_ms", 0)
                }).encode("utf-8")

                exif_bytes = piexif.dump(exif_dict)
                piexif.insert(exif_bytes, filepath)

                return True
            except Exception as e:
                print(f"⚠️ Frame save failed: {e}")
                return False

        
        @staticmethod
        def _deg_to_dms(self, deg):
            d = int(deg)
            m = int((deg - d) * 60)
            s = int((((deg - d) * 60) - m) * 60 * 100)
            return ((d, 1), (m, 1), (s, 100))
        
        while self.is_recording or not self.save_queue.empty():
            try:
                frame_data = self.save_queue.get(timeout=1)
                frame = frame_data['frame']
                telemetry = frame_data['telemetry']
                # Generate timestamp-based filename
                now = datetime.now()
                timestamp_key = now.strftime("%Y%m%d_%H%M%S")
                
                # Track frame count per second
                if timestamp_key not in frame_count_per_second:
                    frame_count_per_second = {timestamp_key: 0}  # Clear old keys
                
                frame_count_per_second[timestamp_key] += 1
                frame_num = frame_count_per_second[timestamp_key]
                
                # Filename: captures/YYYY-MM-DD/HH_MM_SS/capture_YYYYMMDD_HHMMSS_01.jpg
                date_folder = now.strftime("%Y-%m-%d")
                time_folder = now.strftime("%H_%M_%S")
                folder_path = os.path.join("captures", date_folder, time_folder)
                os.makedirs(folder_path, exist_ok=True)
                
                filename = f"capture_{timestamp_key}_{frame_num:02d}.jpg"
                filepath = os.path.join(folder_path, filename)
                
                # Submit to thread pool (non-blocking)
                executor.submit(save_frame_with_metadata, frame, filepath, telemetry, frame_num)
                
                self.save_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"❌ Recording loop error: {e}")
        
        executor.shutdown(wait=True)
        print("✅ All frames saved to disk")

    def set_quality(self, profile_name):
        """
        Switch stream quality dynamically.
        Args:
            profile_name: Profile name from VideoQualityProfile (e.g. "480p_24")
        """
        new_stream = "sub.264" if "480p" in profile_name else "main.264"
        
        if new_stream != self.stream_name:
            print(f"🔄 Switching stream quality: {self.stream_name} -> {new_stream} ({profile_name})")
            self.stream_name = new_stream
            
            # Trigger immediate reconnection with new stream
            # We use a separate thread/task to avoid blocking usage
            threading.Thread(target=self._reconnect_stream, daemon=True).start()
        else:
            print(f"ℹ️  Already using stream {self.stream_name} for profile {profile_name}")


    # ---------------- STREAM & CAPTURE ----------------

    def open_stream(self):
        """Open RTSP stream."""
        if self.cap is not None and self.cap.isOpened():
            return True

        gst_pipeline = (
            f"rtspsrc location=rtsp://{self.server_ip}:8554/{self.stream_name} protocols=tcp latency=100 timeout=3000000 ! "
            "rtph265depay ! h265parse ! avdec_h265 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=true sync=false"
        )

        print("📡 Opening GStreamer pipeline...")
        try:
            self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            if not self.cap.isOpened():
                print("❌ Failed to open RTSP stream")
                self._notify_camera_state_change(False)
                return False
            print("✅ RTSP stream opened")
            self._notify_camera_state_change(True)
            return True
        except Exception as e:
            print(f"❌ Error opening stream: {e}")
            self._notify_camera_state_change(False)
            return False

    def _reconnect_stream(self):
        """Silent internal stream reconnection."""
        if self.cap:
            try:
                self.cap.release()
            except: pass
            self.cap = None
        
        # Try to re-open
        return self.open_stream()
    


    def _reader_loop(self):
        """
        Keep-alive frame reader - stream stays open permanently.
        GStreamer will auto-recover when camera reconnects.
        """
        consecutive_failures = 0
        last_state_logged = None  # Track last logged state to avoid spam
        
        while self.running:
            try:
                # Check if stream was never opened (startup failure)
                if not self.cap or not self.cap.isOpened():
                    if last_state_logged != "never_opened":
                        print("⚠️  Camera stream not opened at startup")
                        self._notify_camera_state_change(False)
                        last_state_logged = "never_opened"
                    time.sleep(1.0)
                    continue
                
                # Try to read frame
                ret, frame = self.cap.read()
                
                if not ret:
                    # Read failed - camera likely disconnected
                    consecutive_failures += 1
                    
                    # Log state change only once when transitioning to disconnected
                    if consecutive_failures == 5 and last_state_logged != "disconnected":
                        print(f"📸 Camera disconnected - keeping stream alive for auto-recovery")
                        self._notify_camera_state_change(False)
                        last_state_logged = "disconnected"
                    
                    # Clear stale frame
                    with self.frame_lock:
                        self.latest_frame = None
                    
                    # If we've failed too many times, trigger a silent internal reconnection
                    # This is needed because OpenCV often can't recover from a broken pipe without re-initializing
                    if consecutive_failures >= 15: # Wait ~1.5s before forcing reconnect
                        time.sleep(1.0)
                        self._reconnect_stream()
                        # Don't reset failures yet, wait for successful read
                    else:
                         # Brief delay before next read attempt
                        time.sleep(0.1)
                    
                    continue
                
                # Successful read - camera is working
                if consecutive_failures > 0:
                    # Log reconnection only if we were previously disconnected
                    if last_state_logged == "disconnected":
                        print(f"✅ Camera reconnected automatically (was down for {consecutive_failures} attempts)")
                        self._notify_camera_state_change(True)
                        last_state_logged = "connected"
                    consecutive_failures = 0
                
                # Update frame
                current_time = time.time()
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                    self.last_frame_time = current_time
                
                # If recording is active, push a copy to the save queue
                if self.is_recording:
                    try:
                        telemetry = self.telemetry_provider() if self.telemetry_provider else {}
                        self.save_queue.put_nowait({
                            'frame': frame.copy(),
                            'telemetry': telemetry
                        })
                    except queue.Full:
                        pass  # Skip frame if disk I/O is too slow
                        
            except Exception as e:
                # Unexpected error - log but keep stream alive
                if last_state_logged != "error":
                    print(f"⚠️  Camera read error: {e} (keeping stream alive)")
                    last_state_logged = "error"
                consecutive_failures += 1
                with self.frame_lock:
                    self.latest_frame = None
                time.sleep(0.5)
        
        print("🛑 Frame reader thread stopped")

    def start(self):
        self.open_stream()
        self.running = True
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        print("▶️ Frame capture started")

    def get_frame(self):
        """Get the latest frame, returns None if stale or unavailable."""
        with self.frame_lock:
            # Check if frame is stale
            if self.latest_frame is not None:
                time_since_frame = time.time() - self.last_frame_time
                if time_since_frame > self.frame_timeout:
                    # Frame is stale, clear it
                    self.latest_frame = None
            return self.latest_frame

    def stop(self):
        self.stop_recording()
        self.running = False
        if self.reader_thread:
            self.reader_thread.join(timeout=2)
        if self.cap:
            self.cap.release()
        print("🧹 Camera resources released")