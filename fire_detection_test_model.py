# import cv2
# import numpy as np
# import time
# import psutil
# import os
# import logging
# import torch
# from pathlib import Path
# from collections import deque
# from ultralytics import YOLO
# from threading import Thread, Lock
# import queue
# import torch

# # Jetson-specific optimizations
# os.environ['CUDA_LAUNCH_BLOCKING'] = '0'  # Async CUDA operations
# os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128'  # Prevent fragmentation

# # If using Jetson Orin/Xavier
# torch.backends.cuda.matmul.allow_tf32 = True
# torch.backends.cudnn.allow_tf32 = True

# class FireDetector:
#     def __init__(
#         self,
#         model_path="yolov8n.pt",
#         detection_interval=0.5,
#         conf_threshold=0.4,
#         area_threshold=100,
#         temporal_window=3,
#         resize=416
#     ):
#         # ============ GPU OPTIMIZATION ============
#         # Force CUDA if available (Jetson)
#         self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
#         print(f"🔥 Fire Detector using device: {self.device}")
        
#         if self.device == 'cuda':
#             print(f"   GPU: {torch.cuda.get_device_name(0)}")
#             print(f"   CUDA Version: {torch.version.cuda}")
#             # Optimize CUDA settings for Jetson
#             torch.backends.cudnn.benchmark = True
#             torch.backends.cudnn.enabled = True
        
#         # Load model with explicit device setting
#         self.model = YOLO(model_path)
#         self.model.to(self.device)
        
#         # Warm up the model (critical for accurate timing)
#         print("🔥 Warming up GPU...")
#         dummy = np.zeros((resize, resize, 3), dtype=np.uint8)
#         for _ in range(3):
#             _ = self.model(dummy, device=self.device, verbose=False)
#         print("✅ GPU warm-up complete")
        
#         # ============ CONFIGURATION ============
#         self.detection_interval = detection_interval
#         self.conf_threshold = conf_threshold
#         self.area_threshold = area_threshold
#         self.temporal_window = temporal_window
#         self.resize = resize
#         self.last_detection_time = time.time()
        
#         # ============ TEMPORAL FILTERING ============
#         self.temporal_history = deque(maxlen=10)
#         self.cluster_memory = []
        
#         # ============ PERFORMANCE MONITORING ============
#         self.process = psutil.Process(os.getpid())
#         self.perf_stats = {
#             "frames": 0,
#             "detections": 0,
#             "skipped": 0,
#             "detect_times": deque(maxlen=200),
#             "quality_rejects": 0,
#             "last_log": time.time(),
#             "gpu_infer_times": deque(maxlen=200),  # NEW: Track GPU inference separately
#             "preprocess_times": deque(maxlen=200),
#             "postprocess_times": deque(maxlen=200),
#         }
        
#         # ============ LOGGING ============
#         logs_dir = Path("logs")
#         logs_dir.mkdir(exist_ok=True)
#         self.logger = logging.getLogger("fire_detector_stats")
#         self.logger.setLevel(logging.INFO)
#         self.logger.handlers = []
#         fh = logging.FileHandler(logs_dir / "fire_detector.log")
#         fh.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
#         self.logger.addHandler(fh)
        
#         print("🔥 Fire Detector Initialized")

#     # ---------------- Frame Quality Check (OPTIMIZED) ----------------
#     def _is_frame_valid(self, frame):
#         """Fast frame quality check using GPU if available"""
#         # Downscale for faster quality check
#         small = cv2.resize(frame, (frame.shape[1]//4, frame.shape[0]//4))
#         gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        
#         # Fast blur detection (Laplacian variance)
#         blur_score = cv2.Laplacian(gray, cv2.CV_64F).var()
        
#         # Fast brightness check
#         brightness = np.mean(gray)
        
#         if blur_score < 50 or brightness < 30:
#             self.perf_stats["quality_rejects"] += 1
#             return False
#         return True

#     # ---------------- Color Filter (OPTIMIZED) ----------------
#     def _color_fire_filter(self, roi):
#         """Vectorized color filtering for fire detection"""
#         # Resize ROI if too large (faster processing)
#         if roi.shape[0] > 100 or roi.shape[1] > 100:
#             roi = cv2.resize(roi, (min(100, roi.shape[1]), min(100, roi.shape[0])))
        
#         hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
#         lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        
#         # Vectorized mean calculations
#         hue_mean = np.mean(hsv[:, :, 0])
#         sat_mean = np.mean(hsv[:, :, 1])
#         lum_mean = np.mean(lab[:, :, 0])
        
#         # Fire color heuristic (orange/red/yellow)
#         if 5 < hue_mean < 50 and sat_mean > 50 and lum_mean > 80:
#             return True
#         return False

#     # ---------------- Clustering (UNCHANGED) ----------------
#     def _cluster_detection(self, lat, lon):
#         """Check if detection is near a previous cluster"""
#         if lat is None or lon is None:
#             return False
            
#         for cluster in self.cluster_memory:
#             if abs(cluster[0] - lat) < 0.0001 and abs(cluster[1] - lon) < 0.0001:
#                 return True
        
#         self.cluster_memory.append((lat, lon))
        
#         # Prevent memory growth - keep only recent 100 clusters
#         if len(self.cluster_memory) > 100:
#             self.cluster_memory = self.cluster_memory[-100:]
        
#         return False

#     # ---------------- MAIN DETECTION (HEAVILY OPTIMIZED) ----------------
#     def detect(self, frame, drone_lat=None, drone_lon=None, drone_alt=None):
#         """
#         GPU-accelerated fire detection with same input/output format.
        
#         Args:
#             frame: BGR image (numpy array)
#             drone_lat: Current drone latitude
#             drone_lon: Current drone longitude
#             drone_alt: Current drone altitude
            
#         Returns:
#             dict or None: {
#                 "bbox": [[x1, y1, x2, y2, lat, lon], ...],
#                 "confidence": float,
#                 "timestamp": float
#             }
#         """
#         start_time = time.time()
#         timestamp = start_time
#         self.perf_stats["frames"] += 1
        
#         # ============ RATE LIMITING ============
#         if timestamp - self.last_detection_time < self.detection_interval:
#             self.perf_stats["skipped"] += 1
#             return None
        
#         # ============ QUALITY CHECK ============
#         if not self._is_frame_valid(frame):
#             return None
        
#         self.last_detection_time = timestamp
        
#         # ============ PREPROCESSING (OPTIMIZED) ============
#         preprocess_start = time.time()
#         h, w = frame.shape[:2]
        
#         # Use cv2.resize with INTER_LINEAR (faster than INTER_CUBIC)
#         resized = cv2.resize(frame, (self.resize, self.resize), interpolation=cv2.INTER_LINEAR)
        
#         preprocess_time = time.time() - preprocess_start
#         self.perf_stats["preprocess_times"].append(preprocess_time)
        
#         # ============ GPU INFERENCE ============
#         gpu_start = time.time()
        
#         # Run inference with explicit device and optimizations
#         results = self.model(
#             resized,
#             device=self.device,
#             conf=self.conf_threshold,
#             verbose=False,
#             half=True if self.device == 'cuda' else False,  # FP16 on GPU
#             imgsz=self.resize
#         )
        
#         gpu_time = time.time() - gpu_start
#         self.perf_stats["gpu_infer_times"].append(gpu_time)
        
#         # ============ POSTPROCESSING ============
#         postprocess_start = time.time()
        
#         detections = []
#         confidences = []
        
#         # Scale factors (pre-compute once)
#         x_scale = w / self.resize
#         y_scale = h / self.resize
        
#         for r in results:
#             if r.boxes is None or len(r.boxes) == 0:
#                 continue
                
#             for box in r.boxes:
#                 conf = float(box.conf)
#                 if conf < self.conf_threshold:
#                     continue
                
#                 # Extract coordinates (already on CPU from ultralytics)
#                 xyxy = box.xyxy[0].cpu().numpy()
#                 x1, y1, x2, y2 = map(int, xyxy)
                
#                 # Scale back to original resolution
#                 x1 = int(x1 * x_scale)
#                 x2 = int(x2 * x_scale)
#                 y1 = int(y1 * y_scale)
#                 y2 = int(y2 * y_scale)
                
#                 # Clamp to frame boundaries
#                 x1, x2 = max(0, x1), min(w, x2)
#                 y1, y2 = max(0, y1), min(h, y2)
                
#                 # Area filtering
#                 area = (x2 - x1) * (y2 - y1)
#                 if area < self.area_threshold:
#                     continue
                
#                 # Color filter on ROI
#                 roi = frame[y1:y2, x1:x2]
#                 if roi.size == 0 or not self._color_fire_filter(roi):
#                     continue
                
#                 # Cluster filtering
#                 if self._cluster_detection(drone_lat, drone_lon):
#                     continue
                
#                 # Valid detection
#                 detections.append([x1, y1, x2, y2, drone_lat, drone_lon])
#                 confidences.append(conf)
        
#         postprocess_time = time.time() - postprocess_start
#         self.perf_stats["postprocess_times"].append(postprocess_time)
        
#         # ============ NO DETECTIONS ============
#         if not detections:
#             return None
        
#         # ============ TEMPORAL FILTERING ============
#         self.temporal_history.append(len(detections))
#         if sum(self.temporal_history) < self.temporal_window:
#             return None
        
#         # ============ RESULT CONSTRUCTION ============
#         avg_conf = sum(confidences) / len(confidences)
#         total_time = time.time() - start_time
        
#         self.perf_stats["detect_times"].append(total_time)
#         self.perf_stats["detections"] += len(detections)
        
#         # Log stats periodically
#         self._log_stats()
        
#         # Return in EXACT same format as original
#         return {
#             "bbox": detections,
#             "confidence": round(avg_conf, 2),
#             "timestamp": timestamp
#         }

#     # ---------------- ENHANCED LOGGING ----------------
#     def _log_stats(self):
#         """Enhanced logging with GPU metrics"""
#         now = time.time()
#         elapsed = now - self.perf_stats["last_log"]
        
#         if elapsed < 3:
#             return
        
#         # Calculate averages
#         avg_total = np.mean(self.perf_stats["detect_times"]) if self.perf_stats["detect_times"] else 0
#         avg_gpu = np.mean(self.perf_stats["gpu_infer_times"]) if self.perf_stats["gpu_infer_times"] else 0
#         avg_preprocess = np.mean(self.perf_stats["preprocess_times"]) if self.perf_stats["preprocess_times"] else 0
#         avg_postprocess = np.mean(self.perf_stats["postprocess_times"]) if self.perf_stats["postprocess_times"] else 0
        
#         fps = self.perf_stats["frames"] / elapsed
#         mem = self.process.memory_info().rss / 1024 / 1024
#         cpu = self.process.cpu_percent(interval=0.1)
        
#         # GPU memory usage (Jetson specific)
#         gpu_mem = 0
#         if self.device == 'cuda':
#             gpu_mem = torch.cuda.memory_allocated() / 1024 / 1024  # MB
        
#         log_msg = (
#             f"🔥 FPS:{fps:.1f} | Total:{avg_total*1000:.1f}ms "
#             f"(Pre:{avg_preprocess*1000:.1f}ms + GPU:{avg_gpu*1000:.1f}ms + Post:{avg_postprocess*1000:.1f}ms) | "
#             f"Detections:{self.perf_stats['detections']} | "
#             f"Skipped:{self.perf_stats['skipped']} | "
#             f"QualityRejects:{self.perf_stats['quality_rejects']} | "
#             f"Mem:{mem:.1f}MB | GPU:{gpu_mem:.1f}MB | CPU:{cpu:.1f}%"
#         )
        
#         print(log_msg)
#         self.logger.info(log_msg)
        
#         # Reset counters
#         self.perf_stats["frames"] = 0
#         self.perf_stats["detections"] = 0
#         self.perf_stats["skipped"] = 0
#         self.perf_stats["quality_rejects"] = 0
#         self.perf_stats["last_log"] = now

#     # ---------------- Drawing (UNCHANGED) ----------------
#     def draw_detections(self, frame, result):
#         """Draw detection bounding boxes on frame"""
#         if not result:
#             return frame
        
#         for bbox in result["bbox"]:
#             x1, y1, x2, y2, lat, lon = bbox
#             cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
#             cv2.putText(
#                 frame, 
#                 f"Fire {result['confidence']:.2f}",
#                 (x1, y1 - 10), 
#                 cv2.FONT_HERSHEY_SIMPLEX, 
#                 0.5,
#                 (0, 0, 255), 
#                 2
#             )
        
#         return frame

# import cv2
# import numpy as np
# import time
# import psutil
# import os
# import logging
# import torch
# from pathlib import Path
# from collections import deque
# from ultralytics import YOLO
# from threading import Thread, Lock
# import queue

# # ============================================================================
# # JETSON-SPECIFIC OPTIMIZATIONS FOR NVIDIA CUDA GPU
# # ============================================================================
# # Async CUDA operations for better GPU utilization
# os.environ['CUDA_LAUNCH_BLOCKING'] = '0'
# # Prevent memory fragmentation on limited Jetson memory
# os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128'

# # Enable TensorFloat-32 for faster matrix operations on Jetson Orin/Xavier/Nano Super
# torch.backends.cuda.matmul.allow_tf32 = True
# torch.backends.cudnn.allow_tf32 = True


# class FireDetector:
#     """
#     GPU-Accelerated Fire Detection for UAV Platform on Jetson Nano Super
    
#     Optimizations implemented:
#     1. CUDA GPU prioritization with FP16 inference
#     2. Dynamic frame quality assessment (blur + brightness)
#     3. Multi-scale color analysis (HSV + LAB space)
#     4. Temporal consistency filtering (reduces false positives)
#     5. Spatial clustering with memory-efficient tracking
#     6. Comprehensive performance monitoring every 3 seconds
#     """
    
#     def __init__(
#         self,
#         model_path="yolov8n.pt",
#         detection_interval=0.5,
#         conf_threshold=0.4,
#         area_threshold=100,
#         temporal_window=3,
#         resize=416,
#         log_interval=3.0  # NEW: Configurable logging interval (default 3 seconds)
#     ):
#         # ============ GPU OPTIMIZATION & INITIALIZATION ============
#         # Prioritize CUDA GPU (Jetson Nano Super has NVIDIA GPU)
#         self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
#         print(f"🔥 Fire Detector using device: {self.device}")
        
#         if self.device == 'cuda':
#             # Display GPU information for monitoring
#             print(f"   GPU: {torch.cuda.get_device_name(0)}")
#             print(f"   CUDA Version: {torch.version.cuda}")
#             print(f"   CUDA Memory: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.2f} GB")
            
#             # Enable cuDNN autotuner - finds optimal convolution algorithms
#             # Critical for repetitive operations like video processing
#             torch.backends.cudnn.benchmark = True
#             torch.backends.cudnn.enabled = True
            
#             # Set GPU to performance mode if possible
#             try:
#                 torch.cuda.set_per_process_memory_fraction(0.8)  # Reserve 80% GPU memory
#             except:
#                 pass
        
#         # ============ MODEL LOADING WITH GPU OPTIMIZATION ============
#         print(f"🔥 Loading YOLO model: {model_path}")
#         self.model = YOLO(model_path)
#         self.model.to(self.device)
        
#         # Warm up the model - critical for accurate timing and cuDNN optimization
#         # First inference is always slower due to kernel compilation
#         print("🔥 Warming up GPU (compiling CUDA kernels)...")
#         dummy = np.zeros((resize, resize, 3), dtype=np.uint8)
#         for i in range(5):  # Increased warm-up iterations
#             _ = self.model(
#                 dummy, 
#                 device=self.device, 
#                 verbose=False,
#                 half=True if self.device == 'cuda' else False  # FP16 on GPU
#             )
#             print(f"   Warm-up {i+1}/5 complete")
#         print("✅ GPU warm-up complete - ready for real-time detection")
        
#         # ============ DETECTION CONFIGURATION ============
#         self.detection_interval = detection_interval  # Minimum time between detections
#         self.conf_threshold = conf_threshold          # Confidence threshold for YOLO
#         self.area_threshold = area_threshold          # Minimum bbox area (pixels)
#         self.temporal_window = temporal_window        # Frames needed for confirmation
#         self.resize = resize                          # Input size for YOLO (416 is optimal for speed/accuracy)
#         self.last_detection_time = time.time()        # Track last detection for rate limiting
#         self.log_interval = log_interval              # NEW: Logging interval
        
#         # ============ TEMPORAL FILTERING (RESEARCH-BASED) ============
#         # Research shows temporal consistency reduces false positives by 60-80%
#         # We track detection counts over last N frames
#         self.temporal_history = deque(maxlen=10)
        
#         # ============ SPATIAL CLUSTERING (MEMORY EFFICIENT) ============
#         # Prevents duplicate alerts for same fire location
#         # Uses lat/lon coordinates with tolerance threshold
#         self.cluster_memory = []
        
#         # ============ ENHANCED PERFORMANCE MONITORING ============
#         self.process = psutil.Process(os.getpid())
#         self.perf_stats = {
#             # Frame processing stats
#             "frames_processed": 0,
#             "frames_total": 0,
#             "detections": 0,
#             "skipped_rate_limit": 0,       # NEW: Track rate limit skips
#             "skipped_quality": 0,          # NEW: Track quality check skips
#             "skipped_temporal": 0,         # NEW: Track temporal filter skips
#             "skipped_color": 0,            # NEW: Track color filter skips
#             "skipped_cluster": 0,          # NEW: Track cluster filter skips
            
#             # Timing breakdown (deques auto-discard old values)
#             "total_times": deque(maxlen=300),
#             "gpu_infer_times": deque(maxlen=300),
#             "preprocess_times": deque(maxlen=300),
#             "postprocess_times": deque(maxlen=300),
#             "quality_check_times": deque(maxlen=300),
            
#             # Logging control
#             "last_log": time.time(),
            
#             # NEW: GPU-specific metrics
#             "gpu_mem_peak": 0,
#             "gpu_util_samples": deque(maxlen=100),
#         }
        
#         # ============ LOGGING SETUP ============
#         logs_dir = Path("logs")
#         logs_dir.mkdir(exist_ok=True)
#         self.logger = logging.getLogger("fire_detector_stats")
#         self.logger.setLevel(logging.INFO)
#         self.logger.handlers = []  # Clear existing handlers
        
#         # File handler for persistent logs
#         fh = logging.FileHandler(logs_dir / "fire_detector.log")
#         fh.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
#         self.logger.addHandler(fh)
        
#         # Console handler for terminal output
#         ch = logging.StreamHandler()
#         ch.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
#         self.logger.addHandler(ch)
        
#         print("✅ Fire Detector Initialized Successfully")
#         print(f"   Detection interval: {detection_interval}s")
#         print(f"   Confidence threshold: {conf_threshold}")
#         print(f"   Logging interval: {log_interval}s")
#         print("=" * 80)

#     # ============================================================================
#     # FRAME QUALITY ASSESSMENT (RESEARCH-BASED)
#     # ============================================================================
#     def _is_frame_valid(self, frame):
#         """
#         Fast frame quality check to skip low-quality frames
        
#         Research basis: Poor quality frames increase false positives
#         - Blurry frames from motion/vibration (common in UAVs)
#         - Under/over-exposed frames from lighting changes
        
#         Method:
#         1. Downscale 4x for speed (16x fewer pixels to process)
#         2. Laplacian variance for blur detection (edge sharpness)
#         3. Mean brightness for exposure check
        
#         Returns: True if frame quality is acceptable
#         """
#         quality_start = time.time()
        
#         # Downscale to 1/16th size for 16x faster processing
#         small = cv2.resize(frame, (frame.shape[1]//4, frame.shape[0]//4))
#         gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        
#         # Blur detection using Laplacian variance
#         # Higher variance = sharper edges = less blur
#         # Threshold of 50 derived from empirical testing
#         blur_score = cv2.Laplacian(gray, cv2.CV_64F).var()
        
#         # Brightness check (avoid under/over-exposed frames)
#         brightness = np.mean(gray)
        
#         quality_time = time.time() - quality_start
#         self.perf_stats["quality_check_times"].append(quality_time)
        
#         # Reject low quality frames
#         if blur_score < 50:
#             self.perf_stats["skipped_quality"] += 1
#             return False
#         if brightness < 30 or brightness > 225:
#             self.perf_stats["skipped_quality"] += 1
#             return False
            
#         return True

#     # ============================================================================
#     # FIRE COLOR SIGNATURE ANALYSIS (MULTI-SPACE)
#     # ============================================================================
#     def _color_fire_filter(self, roi):
#         """
#         Advanced color-based fire verification using dual color spaces
        
#         Research basis: Fire has distinctive color signature across multiple spaces
#         - HSV: Hue (color), Saturation (purity), Value (brightness)
#         - LAB: Luminance and color opponents (perceptually uniform)
        
#         Fire characteristics:
#         - Hue: 5-50° (orange-red-yellow range)
#         - Saturation: >50 (pure colors, not washed out)
#         - Luminance: >80 (bright, self-luminous)
#         - a* channel: >0 (red component in LAB)
        
#         This reduces false positives from red objects that aren't fire
#         """
#         # Resize large ROIs for consistent processing time
#         if roi.shape[0] > 100 or roi.shape[1] > 100:
#             roi = cv2.resize(roi, (min(100, roi.shape[1]), min(100, roi.shape[0])))
        
#         # Convert to HSV (Hue, Saturation, Value)
#         hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
#         # Convert to LAB (Luminance, a*, b*)
#         lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        
#         # Vectorized mean calculations (much faster than loops)
#         hue_mean = np.mean(hsv[:, :, 0])
#         sat_mean = np.mean(hsv[:, :, 1])
#         val_mean = np.mean(hsv[:, :, 2])
#         lum_mean = np.mean(lab[:, :, 0])
#         a_mean = np.mean(lab[:, :, 1])  # Red-green opponent
        
#         # Fire color heuristic (empirically validated)
#         # Hue: 5-50 = orange/red/yellow
#         # Saturation: >50 = vibrant colors
#         # Luminance: >80 = self-luminous
#         # a*: >128 = red component present
#         is_fire_color = (
#             5 < hue_mean < 50 and 
#             sat_mean > 50 and 
#             lum_mean > 80 and
#             a_mean > 128  # NEW: Additional LAB space check
#         )
        
#         if not is_fire_color:
#             self.perf_stats["skipped_color"] += 1
            
#         return is_fire_color

#     # ============================================================================
#     # SPATIAL CLUSTERING (DUPLICATE PREVENTION)
#     # ============================================================================
#     def _cluster_detection(self, lat, lon):
#         """
#         Check if detection is near a previously reported fire location
        
#         Purpose: Prevent multiple alerts for the same fire as drone moves
        
#         Method:
#         - Maintain memory of recent fire locations (lat/lon)
#         - Check if new detection is within threshold distance
#         - Threshold: 0.0001° ≈ 11 meters (appropriate for UAV altitude)
        
#         Returns: True if this is a duplicate (near existing cluster)
#         """
#         if lat is None or lon is None:
#             return False
            
#         # Check against all known clusters
#         for cluster in self.cluster_memory:
#             # Simple Euclidean distance in lat/lon space
#             # 0.0001° ≈ 11m at equator (good for typical UAV ops)
#             if abs(cluster[0] - lat) < 0.0001 and abs(cluster[1] - lon) < 0.0001:
#                 self.perf_stats["skipped_cluster"] += 1
#                 return True  # Duplicate detection
        
#         # New unique location - add to memory
#         self.cluster_memory.append((lat, lon))
        
#         # Memory management - keep only recent 100 clusters
#         # Prevents unbounded memory growth during long missions
#         if len(self.cluster_memory) > 100:
#             self.cluster_memory = self.cluster_memory[-100:]
        
#         return False

#     # ============================================================================
#     # MAIN DETECTION PIPELINE (HEAVILY OPTIMIZED)
#     # ============================================================================
#     def detect(self, frame, drone_lat=None, drone_lon=None, drone_alt=None):
#         """
#         GPU-accelerated fire detection with comprehensive filtering
        
#         Pipeline stages:
#         1. Rate limiting (prevent GPU overload)
#         2. Quality check (skip bad frames)
#         3. Preprocessing (resize, optimize)
#         4. GPU inference (YOLO detection)
#         5. Postprocessing (filtering, validation)
#         6. Temporal filtering (consistency check)
        
#         Args:
#             frame: BGR image (numpy array, HxWx3)
#             drone_lat: Current drone latitude (float or None)
#             drone_lon: Current drone longitude (float or None)
#             drone_alt: Current drone altitude (float or None)
            
#         Returns:
#             dict or None: {
#                 "bbox": [[x1, y1, x2, y2, lat, lon], ...],  # List of detections
#                 "confidence": float,                          # Average confidence
#                 "timestamp": float                            # Detection timestamp
#             }
#         """
#         start_time = time.time()
#         timestamp = start_time
#         self.perf_stats["frames_total"] += 1
        
#         # ============ STAGE 1: RATE LIMITING ============
#         # Prevent GPU overload by enforcing minimum interval between detections
#         # Typical: 0.5s interval = max 2 FPS detection rate
#         if timestamp - self.last_detection_time < self.detection_interval:
#             self.perf_stats["skipped_rate_limit"] += 1
#             return None
        
#         # ============ STAGE 2: QUALITY CHECK ============
#         # Skip blurry or poorly lit frames (common in UAV footage)
#         if not self._is_frame_valid(frame):
#             # Stats already updated in _is_frame_valid
#             return None
        
#         self.last_detection_time = timestamp
#         self.perf_stats["frames_processed"] += 1
        
#         # ============ STAGE 3: PREPROCESSING ============
#         preprocess_start = time.time()
#         h, w = frame.shape[:2]
        
#         # Resize to YOLO input size using fast INTER_LINEAR
#         # INTER_LINEAR is 2-3x faster than INTER_CUBIC with minimal quality loss
#         resized = cv2.resize(frame, (self.resize, self.resize), interpolation=cv2.INTER_LINEAR)
        
#         preprocess_time = time.time() - preprocess_start
#         self.perf_stats["preprocess_times"].append(preprocess_time)
        
#         # ============ STAGE 4: GPU INFERENCE ============
#         gpu_start = time.time()
        
#         # Track GPU memory before inference
#         if self.device == 'cuda':
#             gpu_mem_before = torch.cuda.memory_allocated()
        
#         # Run YOLO inference with optimizations:
#         # - device=self.device: Force GPU execution
#         # - conf=threshold: Filter low-confidence detections early
#         # - verbose=False: Suppress output for speed
#         # - half=True: Use FP16 (2x faster on modern GPUs, minimal accuracy loss)
#         # - imgsz=resize: Explicit input size
#         results = self.model(
#             resized,
#             device=self.device,
#             conf=self.conf_threshold,
#             verbose=False,
#             half=True if self.device == 'cuda' else False,
#             imgsz=self.resize
#         )
        
#         gpu_time = time.time() - gpu_start
#         self.perf_stats["gpu_infer_times"].append(gpu_time)
        
#         # Track GPU memory usage
#         if self.device == 'cuda':
#             gpu_mem_after = torch.cuda.memory_allocated()
#             gpu_mem_used = (gpu_mem_after - gpu_mem_before) / 1024 / 1024  # MB
#             self.perf_stats["gpu_mem_peak"] = max(self.perf_stats["gpu_mem_peak"], gpu_mem_used)
        
#         # ============ STAGE 5: POSTPROCESSING & FILTERING ============
#         postprocess_start = time.time()
        
#         detections = []
#         confidences = []
        
#         # Pre-compute scale factors (avoid repeated division)
#         x_scale = w / self.resize
#         y_scale = h / self.resize
        
#         # Process all detections from YOLO
#         for r in results:
#             if r.boxes is None or len(r.boxes) == 0:
#                 continue
                
#             for box in r.boxes:
#                 conf = float(box.conf)
                
#                 # Double-check confidence (YOLO may return slightly lower values)
#                 if conf < self.conf_threshold:
#                     continue
                
#                 # Extract bounding box coordinates
#                 # xyxy format: [x1, y1, x2, y2] in resized image space
#                 xyxy = box.xyxy[0].cpu().numpy()
#                 x1, y1, x2, y2 = map(int, xyxy)
                
#                 # Scale coordinates back to original resolution
#                 x1 = int(x1 * x_scale)
#                 x2 = int(x2 * x_scale)
#                 y1 = int(y1 * y_scale)
#                 y2 = int(y2 * y_scale)
                
#                 # Clamp to frame boundaries (prevent out-of-bounds access)
#                 x1, x2 = max(0, x1), min(w, x2)
#                 y1, y2 = max(0, y1), min(h, y2)
                
#                 # FILTER 1: Area threshold (reject tiny detections)
#                 area = (x2 - x1) * (y2 - y1)
#                 if area < self.area_threshold:
#                     continue
                
#                 # FILTER 2: Color-based fire verification
#                 # Extract ROI and check if it has fire-like colors
#                 roi = frame[y1:y2, x1:x2]
#                 if roi.size == 0 or not self._color_fire_filter(roi):
#                     # Stats updated in _color_fire_filter
#                     continue
                
#                 # FILTER 3: Spatial clustering (duplicate prevention)
#                 # Skip if fire already detected at this location
#                 if self._cluster_detection(drone_lat, drone_lon):
#                     # Stats updated in _cluster_detection
#                     continue
                
#                 # ✅ VALID DETECTION - passed all filters
#                 detections.append([x1, y1, x2, y2, drone_lat, drone_lon])
#                 confidences.append(conf)
        
#         postprocess_time = time.time() - postprocess_start
#         self.perf_stats["postprocess_times"].append(postprocess_time)
        
#         # ============ NO VALID DETECTIONS ============
#         if not detections:
#             return None
        
#         # ============ STAGE 6: TEMPORAL FILTERING ============
#         # Research shows temporal consistency dramatically reduces false positives
#         # Require fire to be detected in multiple consecutive frames
#         self.temporal_history.append(len(detections))
        
#         # Check if we have enough detections over recent frames
#         if sum(self.temporal_history) < self.temporal_window:
#             self.perf_stats["skipped_temporal"] += 1
#             return None
        
#         # ============ FINAL RESULT CONSTRUCTION ============
#         avg_conf = sum(confidences) / len(confidences)
#         total_time = time.time() - start_time
        
#         self.perf_stats["total_times"].append(total_time)
#         self.perf_stats["detections"] += len(detections)
        
#         # Periodic logging (every N seconds)
#         self._log_stats()
        
#         # Return in EXACT same format as original (maintains compatibility)
#         return {
#             "bbox": detections,
#             "confidence": round(avg_conf, 2),
#             "timestamp": timestamp
#         }

#     # ============================================================================
#     # COMPREHENSIVE PERFORMANCE LOGGING (EVERY 3 SECONDS)
#     # ============================================================================
#     def _log_stats(self):
#         """
#         Detailed performance monitoring logged every 3 seconds
        
#         Metrics tracked:
#         - Frame processing rate (FPS)
#         - Detection statistics
#         - Skip reasons breakdown
#         - Timing breakdown (preprocess, GPU, postprocess)
#         - Resource usage (CPU, GPU, memory)
#         - GPU utilization
#         """
#         now = time.time()
#         elapsed = now - self.perf_stats["last_log"]
        
#         # Only log every N seconds (default 3)
#         if elapsed < self.log_interval:
#             return
        
#         # ============ CALCULATE TIMING STATISTICS ============
#         # Use numpy for efficient stats calculation
#         avg_total = np.mean(self.perf_stats["total_times"]) if self.perf_stats["total_times"] else 0
#         avg_gpu = np.mean(self.perf_stats["gpu_infer_times"]) if self.perf_stats["gpu_infer_times"] else 0
#         avg_preprocess = np.mean(self.perf_stats["preprocess_times"]) if self.perf_stats["preprocess_times"] else 0
#         avg_postprocess = np.mean(self.perf_stats["postprocess_times"]) if self.perf_stats["postprocess_times"] else 0
#         avg_quality = np.mean(self.perf_stats["quality_check_times"]) if self.perf_stats["quality_check_times"] else 0
        
#         # ============ CALCULATE THROUGHPUT ============
#         fps_total = self.perf_stats["frames_total"] / elapsed  # All frames received
#         fps_processed = self.perf_stats["frames_processed"] / elapsed  # Frames actually processed
        
#         # Model inference FPS (actual GPU throughput)
#         if avg_gpu > 0:
#             model_fps = 1.0 / avg_gpu
#         else:
#             model_fps = 0
        
#         # ============ SYSTEM RESOURCE USAGE ============
#         # CPU and memory monitoring
#         mem_mb = self.process.memory_info().rss / 1024 / 1024
#         cpu_percent = self.process.cpu_percent(interval=0.1)
        
#         # GPU metrics (Jetson-specific)
#         gpu_mem_mb = 0
#         gpu_util_percent = 0
#         if self.device == 'cuda':
#             gpu_mem_mb = torch.cuda.memory_allocated() / 1024 / 1024
#             # Try to get GPU utilization (may not work on all systems)
#             try:
#                 import pynvml
#                 pynvml.nvmlInit()
#                 handle = pynvml.nvmlDeviceGetHandleByIndex(0)
#                 gpu_util_percent = pynvml.nvmlDeviceGetUtilizationRates(handle).gpu
#                 pynvml.nvmlShutdown()
#             except:
#                 gpu_util_percent = -1  # Not available
        
#         # ============ SKIP REASONS BREAKDOWN ============
#         total_skipped = (
#             self.perf_stats["skipped_rate_limit"] +
#             self.perf_stats["skipped_quality"] +
#             self.perf_stats["skipped_temporal"] +
#             self.perf_stats["skipped_color"] +
#             self.perf_stats["skipped_cluster"]
#         )
        
#         # ============ FORMAT LOG MESSAGE ============
#         log_msg = (
#             f"\n{'='*80}\n"
#             f"🔥 FIRE DETECTOR PERFORMANCE STATS ({elapsed:.1f}s interval)\n"
#             f"{'='*80}\n"
#             f"📊 FRAME PROCESSING:\n"
#             f"   Total Frames:      {self.perf_stats['frames_total']:>6} ({fps_total:>5.1f} FPS)\n"
#             f"   Processed:         {self.perf_stats['frames_processed']:>6} ({fps_processed:>5.1f} FPS)\n"
#             f"   Detections:        {self.perf_stats['detections']:>6}\n"
#             f"\n"
#             f"⏭️  FRAMES SKIPPED ({total_skipped} total):\n"
#             f"   Rate Limit:        {self.perf_stats['skipped_rate_limit']:>6}\n"
#             f"   Quality Check:     {self.perf_stats['skipped_quality']:>6}\n"
#             f"   Temporal Filter:   {self.perf_stats['skipped_temporal']:>6}\n"
#             f"   Color Filter:      {self.perf_stats['skipped_color']:>6}\n"
#             f"   Cluster Filter:    {self.perf_stats['skipped_cluster']:>6}\n"
#             f"\n"
#             f"⏱️  TIMING BREAKDOWN (avg per processed frame):\n"
#             f"   Total Pipeline:    {avg_total*1000:>6.1f} ms\n"
#             f"   ├─ Quality Check:  {avg_quality*1000:>6.1f} ms\n"
#             f"   ├─ Preprocessing:  {avg_preprocess*1000:>6.1f} ms\n"
#             f"   ├─ GPU Inference:  {avg_gpu*1000:>6.1f} ms ({model_fps:>5.1f} FPS)\n"
#             f"   └─ Postprocessing: {avg_postprocess*1000:>6.1f} ms\n"
#             f"\n"
#             f"💻 SYSTEM RESOURCES:\n"
#             f"   CPU Usage:         {cpu_percent:>5.1f}%\n"
#             f"   RAM Usage:         {mem_mb:>6.1f} MB\n"
#         )
        
#         # Add GPU stats if available
#         if self.device == 'cuda':
#             log_msg += (
#                 f"   GPU Memory:        {gpu_mem_mb:>6.1f} MB (peak: {self.perf_stats['gpu_mem_peak']:.1f} MB)\n"
#             )
#             if gpu_util_percent >= 0:
#                 log_msg += f"   GPU Utilization:   {gpu_util_percent:>5.1f}%\n"
        
#         log_msg += f"{'='*80}"
        
#         # Output to both console and log file
#         print(log_msg)
#         self.logger.info(log_msg)
        
#         # ============ RESET COUNTERS ============
#         self.perf_stats["frames_total"] = 0
#         self.perf_stats["frames_processed"] = 0
#         self.perf_stats["detections"] = 0
#         self.perf_stats["skipped_rate_limit"] = 0
#         self.perf_stats["skipped_quality"] = 0
#         self.perf_stats["skipped_temporal"] = 0
#         self.perf_stats["skipped_color"] = 0
#         self.perf_stats["skipped_cluster"] = 0
#         self.perf_stats["last_log"] = now

#     # ============================================================================
#     # VISUALIZATION (UNCHANGED - MAINTAINS ORIGINAL FUNCTIONALITY)
#     # ============================================================================
#     def draw_detections(self, frame, result):
#         """
#         Draw detection bounding boxes on frame
        
#         Args:
#             frame: BGR image to draw on
#             result: Detection result dict from detect()
            
#         Returns:
#             frame: Image with bounding boxes drawn
#         """
#         if not result:
#             return frame
        
#         for bbox in result["bbox"]:
#             x1, y1, x2, y2, lat, lon = bbox
            
#             # Draw red rectangle around detection
#             cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            
#             # Add confidence label
#             cv2.putText(
#                 frame, 
#                 f"Fire {result['confidence']:.2f}",
#                 (x1, y1 - 10), 
#                 cv2.FONT_HERSHEY_SIMPLEX, 
#                 0.5,
#                 (0, 0, 255), 
#                 2
#             )
        
#         return frame


# # ============================================================================
# # EXAMPLE USAGE
# # ============================================================================
# if __name__ == "__main__":
#     """
#     Example usage demonstrating the fire detector
#     """
#     # Initialize detector
#     detector = FireDetector(
#         model_path="yolov8n.pt",
#         detection_interval=0.5,
#         conf_threshold=0.4,
#         resize=416,
#         log_interval=3.0  # Log stats every 3 seconds
#     )
    
#     # Example: Process video stream
#     cap = cv2.VideoCapture(0)  # Use 0 for webcam, or path to video file
    
#     print("\n🔥 Starting fire detection...")
#     print("Press 'q' to quit\n")
    
#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break
        
#         # Run detection (with simulated GPS coords)
#         result = detector.detect(
#             frame,
#             drone_lat=37.7749,  # Example: San Francisco
#             drone_lon=-122.4194,
#             drone_alt=100.0
#         )
        
#         # Draw detections if any
#         if result:
#             frame = detector.draw_detections(frame, result)
#             print(f"🔥 FIRE DETECTED! Confidence: {result['confidence']}, Boxes: {len(result['bbox'])}")
        
#         # Display frame
#         cv2.imshow('Fire Detection', frame)
        
#         # Exit on 'q' key
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
    
#     cap.release()
#     cv2.destroyAllWindows()


import cv2
import numpy as np
import time
import psutil
import os
import logging
import sys
import torch
from pathlib import Path
from collections import deque
from ultralytics import YOLO
from threading import Thread, Lock
import queue

# ============================================================================
# JETSON-SPECIFIC OPTIMIZATIONS FOR NVIDIA CUDA GPU
# ============================================================================
# Async CUDA operations for better GPU utilization
os.environ['CUDA_LAUNCH_BLOCKING'] = '0'
# Prevent memory fragmentation on limited Jetson memory
os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128'

# Enable TensorFloat-32 for faster matrix operations on Jetson Orin/Xavier/Nano Super
torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True


class FireDetector:
    """
    GPU-Accelerated Fire Detection for UAV Platform on Jetson Nano Super
    
    DEBUG VERSION: Adds immediate console output and error handling
    """
    
    def __init__(
        self,
        model_path="yolov8n.pt",
        detection_interval=0.5,
        conf_threshold=0.4,
        area_threshold=100,
        temporal_window=3,
        resize=416,
        log_interval=3.0
    ):
        # ============ FORCE IMMEDIATE CONSOLE OUTPUT ============
        # Ensure print() statements appear immediately (no buffering)
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
        
        print("\n" + "="*80, flush=True)
        print("🔥 FIRE DETECTOR INITIALIZATION STARTING...", flush=True)
        print("="*80 + "\n", flush=True)
        
        # ============ GPU OPTIMIZATION & INITIALIZATION ============
        try:
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
            print(f"✅ Device selection: {self.device}", flush=True)
            
            if self.device == 'cuda':
                print(f"   GPU: {torch.cuda.get_device_name(0)}", flush=True)
                print(f"   CUDA Version: {torch.version.cuda}", flush=True)
                print(f"   CUDA Memory: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.2f} GB", flush=True)
                
                torch.backends.cudnn.benchmark = True
                torch.backends.cudnn.enabled = True
                
                try:
                    torch.cuda.set_per_process_memory_fraction(0.8)
                    print("   ✅ GPU memory allocation: 80%", flush=True)
                except Exception as e:
                    print(f"   ⚠️  Could not set GPU memory fraction: {e}", flush=True)
            else:
                print("   ⚠️  WARNING: Running on CPU - will be VERY slow!", flush=True)
                
        except Exception as e:
            print(f"❌ ERROR during GPU initialization: {e}", flush=True)
            raise
        
        # ============ MODEL LOADING WITH ERROR HANDLING ============
        try:
            print(f"\n🔥 Loading YOLO model: {model_path}", flush=True)
            
            if not os.path.exists(model_path):
                print(f"❌ ERROR: Model file not found: {model_path}", flush=True)
                print(f"   Current directory: {os.getcwd()}", flush=True)
                print(f"   Please ensure model file exists!", flush=True)
                raise FileNotFoundError(f"Model not found: {model_path}")
            
            self.model = YOLO(model_path)
            self.model.to(self.device)
            print("✅ Model loaded successfully", flush=True)
            
        except Exception as e:
            print(f"❌ ERROR loading model: {e}", flush=True)
            raise
        
        # ============ GPU WARM-UP ============
        try:
            print("\n🔥 Warming up GPU (this takes ~5 seconds)...", flush=True)
            dummy = np.zeros((resize, resize, 3), dtype=np.uint8)
            
            for i in range(5):
                _ = self.model(
                    dummy, 
                    device=self.device, 
                    verbose=False,
                    half=True if self.device == 'cuda' else False
                )
                print(f"   Warm-up iteration {i+1}/5 complete", flush=True)
            
            print("✅ GPU warm-up complete\n", flush=True)
            
        except Exception as e:
            print(f"❌ ERROR during warm-up: {e}", flush=True)
            raise
        
        # ============ DETECTION CONFIGURATION ============
        self.detection_interval = detection_interval
        self.conf_threshold = conf_threshold
        self.area_threshold = area_threshold
        self.temporal_window = temporal_window
        self.resize = resize
        self.last_detection_time = time.time()
        self.log_interval = log_interval
        
        print(f"⚙️  Configuration:", flush=True)
        print(f"   Detection interval: {detection_interval}s", flush=True)
        print(f"   Confidence threshold: {conf_threshold}", flush=True)
        print(f"   Area threshold: {area_threshold} px", flush=True)
        print(f"   Temporal window: {temporal_window} frames", flush=True)
        print(f"   Input size: {resize}x{resize}", flush=True)
        print(f"   Log interval: {log_interval}s\n", flush=True)
        
        # ============ TEMPORAL FILTERING ============
        self.temporal_history = deque(maxlen=10)
        
        # ============ SPATIAL CLUSTERING ============
        self.cluster_memory = []
        
        # ============ ENHANCED PERFORMANCE MONITORING ============
        self.process = psutil.Process(os.getpid())
        self.perf_stats = {
            "frames_processed": 0,
            "frames_total": 0,
            "detections": 0,
            "skipped_rate_limit": 0,
            "skipped_quality": 0,
            "skipped_temporal": 0,
            "skipped_color": 0,
            "skipped_cluster": 0,
            
            "total_times": deque(maxlen=300),
            "gpu_infer_times": deque(maxlen=300),
            "preprocess_times": deque(maxlen=300),
            "postprocess_times": deque(maxlen=300),
            "quality_check_times": deque(maxlen=300),
            
            "last_log": time.time(),
            "gpu_mem_peak": 0,
            "gpu_util_samples": deque(maxlen=100),
        }
        
        # ============ LOGGING SETUP WITH IMMEDIATE OUTPUT ============
        try:
            logs_dir = Path("logs")
            logs_dir.mkdir(exist_ok=True)
            
            self.logger = logging.getLogger("fire_detector_stats")
            self.logger.setLevel(logging.INFO)
            self.logger.handlers = []
            
            # File handler
            fh = logging.FileHandler(logs_dir / "fire_detector.log")
            fh.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
            self.logger.addHandler(fh)
            
            # Console handler with immediate output
            ch = logging.StreamHandler(sys.stdout)
            ch.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
            ch.setLevel(logging.INFO)
            self.logger.addHandler(ch)
            
            # Force immediate log flushing
            for handler in self.logger.handlers:
                handler.setLevel(logging.INFO)
                if hasattr(handler, 'stream'):
                    handler.stream.reconfigure(line_buffering=True)
            
            print(f"✅ Logging configured: logs/fire_detector.log\n", flush=True)
            
        except Exception as e:
            print(f"⚠️  WARNING: Could not setup file logging: {e}", flush=True)
            print("   Will continue with console logging only\n", flush=True)
        
        print("="*80, flush=True)
        print("✅ FIRE DETECTOR READY FOR OPERATION", flush=True)
        print("="*80 + "\n", flush=True)
        
        # Log first stats immediately to confirm logging works
        self._log_startup_info()

    def _log_startup_info(self):
        """Log initial system information"""
        msg = (
            f"\n{'='*80}\n"
            f"🔥 FIRE DETECTOR STARTUP COMPLETE\n"
            f"{'='*80}\n"
            f"Device: {self.device}\n"
            f"Model: YOLO (confidence={self.conf_threshold})\n"
            f"Waiting for frames...\n"
            f"{'='*80}\n"
        )
        print(msg, flush=True)
        self.logger.info(msg)

    # ============================================================================
    # FRAME QUALITY ASSESSMENT
    # ============================================================================
    def _is_frame_valid(self, frame):
        """Fast frame quality check to skip low-quality frames"""
        try:
            quality_start = time.time()
            
            # Downscale to 1/16th size
            small = cv2.resize(frame, (frame.shape[1]//4, frame.shape[0]//4))
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            
            # Blur detection
            blur_score = cv2.Laplacian(gray, cv2.CV_64F).var()
            
            # Brightness check
            brightness = np.mean(gray)
            
            quality_time = time.time() - quality_start
            self.perf_stats["quality_check_times"].append(quality_time)
            
            # Reject low quality frames
            if blur_score < 50:
                self.perf_stats["skipped_quality"] += 1
                return False
            if brightness < 30 or brightness > 225:
                self.perf_stats["skipped_quality"] += 1
                return False
                
            return True
            
        except Exception as e:
            print(f"❌ ERROR in quality check: {e}", flush=True)
            return True  # Continue processing on error

    # ============================================================================
    # FIRE COLOR SIGNATURE ANALYSIS
    # ============================================================================
    def _color_fire_filter(self, roi):
        """Advanced color-based fire verification using dual color spaces"""
        try:
            # Resize large ROIs
            if roi.shape[0] > 100 or roi.shape[1] > 100:
                roi = cv2.resize(roi, (min(100, roi.shape[1]), min(100, roi.shape[0])))
            
            # Convert to HSV and LAB
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
            
            # Vectorized calculations
            hue_mean = np.mean(hsv[:, :, 0])
            sat_mean = np.mean(hsv[:, :, 1])
            val_mean = np.mean(hsv[:, :, 2])
            lum_mean = np.mean(lab[:, :, 0])
            a_mean = np.mean(lab[:, :, 1])
            
            # Fire color heuristic
            is_fire_color = (
                5 < hue_mean < 50 and 
                sat_mean > 50 and 
                lum_mean > 80 and
                a_mean > 128
            )
            
            if not is_fire_color:
                self.perf_stats["skipped_color"] += 1
                
            return is_fire_color
            
        except Exception as e:
            print(f"❌ ERROR in color filter: {e}", flush=True)
            return True  # Continue processing on error

    # ============================================================================
    # SPATIAL CLUSTERING
    # ============================================================================
    def _cluster_detection(self, lat, lon):
        """Check if detection is near a previously reported fire location"""
        try:
            if lat is None or lon is None:
                return False
                
            # Check against all known clusters
            for cluster in self.cluster_memory:
                if abs(cluster[0] - lat) < 0.0001 and abs(cluster[1] - lon) < 0.0001:
                    self.perf_stats["skipped_cluster"] += 1
                    return True
            
            # Add new cluster
            self.cluster_memory.append((lat, lon))
            
            # Memory management
            if len(self.cluster_memory) > 100:
                self.cluster_memory = self.cluster_memory[-100:]
            
            return False
            
        except Exception as e:
            print(f"❌ ERROR in clustering: {e}", flush=True)
            return False

    # ============================================================================
    # MAIN DETECTION PIPELINE
    # ============================================================================
    def detect(self, frame, drone_lat=None, drone_lon=None, drone_alt=None):
        """
        GPU-accelerated fire detection with comprehensive filtering
        
        Returns:
            dict or None: {
                "bbox": [[x1, y1, x2, y2, lat, lon], ...],
                "confidence": float,
                "timestamp": float
            }
        """
        try:
            start_time = time.time()
            timestamp = start_time
            self.perf_stats["frames_total"] += 1
            
            # Print periodic heartbeat every 30 frames
            if self.perf_stats["frames_total"] % 30 == 1:
                print(f"💓 Heartbeat: {self.perf_stats['frames_total']} frames received", flush=True)
            
            # ============ RATE LIMITING ============
            if timestamp - self.last_detection_time < self.detection_interval:
                self.perf_stats["skipped_rate_limit"] += 1
                return None
            
            # ============ QUALITY CHECK ============
            if not self._is_frame_valid(frame):
                return None
            
            self.last_detection_time = timestamp
            self.perf_stats["frames_processed"] += 1
            
            # ============ PREPROCESSING ============
            preprocess_start = time.time()
            h, w = frame.shape[:2]
            resized = cv2.resize(frame, (self.resize, self.resize), interpolation=cv2.INTER_LINEAR)
            preprocess_time = time.time() - preprocess_start
            self.perf_stats["preprocess_times"].append(preprocess_time)
            
            # ============ GPU INFERENCE ============
            gpu_start = time.time()
            
            if self.device == 'cuda':
                gpu_mem_before = torch.cuda.memory_allocated()
            
            results = self.model(
                resized,
                device=self.device,
                conf=self.conf_threshold,
                verbose=False,
                half=True if self.device == 'cuda' else False,
                imgsz=self.resize
            )
            
            gpu_time = time.time() - gpu_start
            self.perf_stats["gpu_infer_times"].append(gpu_time)
            
            if self.device == 'cuda':
                gpu_mem_after = torch.cuda.memory_allocated()
                gpu_mem_used = (gpu_mem_after - gpu_mem_before) / 1024 / 1024
                self.perf_stats["gpu_mem_peak"] = max(self.perf_stats["gpu_mem_peak"], gpu_mem_used)
            
            # ============ POSTPROCESSING ============
            postprocess_start = time.time()
            
            detections = []
            confidences = []
            
            x_scale = w / self.resize
            y_scale = h / self.resize
            
            for r in results:
                if r.boxes is None or len(r.boxes) == 0:
                    continue
                    
                for box in r.boxes:
                    conf = float(box.conf)
                    if conf < self.conf_threshold:
                        continue
                    
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = map(int, xyxy)
                    
                    x1 = int(x1 * x_scale)
                    x2 = int(x2 * x_scale)
                    y1 = int(y1 * y_scale)
                    y2 = int(y2 * y_scale)
                    
                    x1, x2 = max(0, x1), min(w, x2)
                    y1, y2 = max(0, y1), min(h, y2)
                    
                    area = (x2 - x1) * (y2 - y1)
                    if area < self.area_threshold:
                        continue
                    
                    roi = frame[y1:y2, x1:x2]
                    if roi.size == 0 or not self._color_fire_filter(roi):
                        continue
                    
                    if self._cluster_detection(drone_lat, drone_lon):
                        continue
                    
                    detections.append([x1, y1, x2, y2, drone_lat, drone_lon])
                    confidences.append(conf)
            
            postprocess_time = time.time() - postprocess_start
            self.perf_stats["postprocess_times"].append(postprocess_time)
            
            # ============ NO DETECTIONS ============
            if not detections:
                self._log_stats()
                return None
            
            # ============ TEMPORAL FILTERING ============
            self.temporal_history.append(len(detections))
            if sum(self.temporal_history) < self.temporal_window:
                self.perf_stats["skipped_temporal"] += 1
                self._log_stats()
                return None
            
            # ============ VALID DETECTION ============
            avg_conf = sum(confidences) / len(confidences)
            total_time = time.time() - start_time
            
            self.perf_stats["total_times"].append(total_time)
            self.perf_stats["detections"] += len(detections)
            
            # Immediate alert
            print(f"🔥🔥🔥 FIRE DETECTED! Count: {len(detections)}, Confidence: {avg_conf:.2f}", flush=True)
            
            self._log_stats()
            
            return {
                "bbox": detections,
                "confidence": round(avg_conf, 2),
                "timestamp": timestamp
            }
            
        except Exception as e:
            print(f"❌ ERROR in detect(): {e}", flush=True)
            import traceback
            traceback.print_exc()
            return None

    # ============================================================================
    # COMPREHENSIVE PERFORMANCE LOGGING
    # ============================================================================
    def _log_stats(self):
        """Log performance statistics every N seconds"""
        try:
            now = time.time()
            elapsed = now - self.perf_stats["last_log"]
            
            if elapsed < self.log_interval:
                return
            
            # Calculate stats
            avg_total = np.mean(self.perf_stats["total_times"]) if self.perf_stats["total_times"] else 0
            avg_gpu = np.mean(self.perf_stats["gpu_infer_times"]) if self.perf_stats["gpu_infer_times"] else 0
            avg_preprocess = np.mean(self.perf_stats["preprocess_times"]) if self.perf_stats["preprocess_times"] else 0
            avg_postprocess = np.mean(self.perf_stats["postprocess_times"]) if self.perf_stats["postprocess_times"] else 0
            avg_quality = np.mean(self.perf_stats["quality_check_times"]) if self.perf_stats["quality_check_times"] else 0
            
            fps_total = self.perf_stats["frames_total"] / elapsed
            fps_processed = self.perf_stats["frames_processed"] / elapsed
            
            if avg_gpu > 0:
                model_fps = 1.0 / avg_gpu
            else:
                model_fps = 0
            
            mem_mb = self.process.memory_info().rss / 1024 / 1024
            cpu_percent = self.process.cpu_percent(interval=0.1)
            
            gpu_mem_mb = 0
            gpu_util_percent = -1
            if self.device == 'cuda':
                gpu_mem_mb = torch.cuda.memory_allocated() / 1024 / 1024
                try:
                    import pynvml
                    pynvml.nvmlInit()
                    handle = pynvml.nvmlDeviceGetHandleByIndex(0)
                    gpu_util_percent = pynvml.nvmlDeviceGetUtilizationRates(handle).gpu
                    pynvml.nvmlShutdown()
                except:
                    pass
            
            total_skipped = (
                self.perf_stats["skipped_rate_limit"] +
                self.perf_stats["skipped_quality"] +
                self.perf_stats["skipped_temporal"] +
                self.perf_stats["skipped_color"] +
                self.perf_stats["skipped_cluster"]
            )
            
            log_msg = (
                f"\n{'='*80}\n"
                f"🔥 FIRE DETECTOR PERFORMANCE STATS ({elapsed:.1f}s interval)\n"
                f"{'='*80}\n"
                f"📊 FRAME PROCESSING:\n"
                f"   Total Frames:      {self.perf_stats['frames_total']:>6} ({fps_total:>5.1f} FPS)\n"
                f"   Processed:         {self.perf_stats['frames_processed']:>6} ({fps_processed:>5.1f} FPS)\n"
                f"   Detections:        {self.perf_stats['detections']:>6}\n"
                f"\n"
                f"⏭️  FRAMES SKIPPED ({total_skipped} total):\n"
                f"   Rate Limit:        {self.perf_stats['skipped_rate_limit']:>6}\n"
                f"   Quality Check:     {self.perf_stats['skipped_quality']:>6}\n"
                f"   Temporal Filter:   {self.perf_stats['skipped_temporal']:>6}\n"
                f"   Color Filter:      {self.perf_stats['skipped_color']:>6}\n"
                f"   Cluster Filter:    {self.perf_stats['skipped_cluster']:>6}\n"
                f"\n"
                f"⏱️  TIMING BREAKDOWN (avg per processed frame):\n"
                f"   Total Pipeline:    {avg_total*1000:>6.1f} ms\n"
                f"   ├─ Quality Check:  {avg_quality*1000:>6.1f} ms\n"
                f"   ├─ Preprocessing:  {avg_preprocess*1000:>6.1f} ms\n"
                f"   ├─ GPU Inference:  {avg_gpu*1000:>6.1f} ms ({model_fps:>5.1f} FPS)\n"
                f"   └─ Postprocessing: {avg_postprocess*1000:>6.1f} ms\n"
                f"\n"
                f"💻 SYSTEM RESOURCES:\n"
                f"   CPU Usage:         {cpu_percent:>5.1f}%\n"
                f"   RAM Usage:         {mem_mb:>6.1f} MB\n"
            )
            
            if self.device == 'cuda':
                log_msg += f"   GPU Memory:        {gpu_mem_mb:>6.1f} MB (peak: {self.perf_stats['gpu_mem_peak']:.1f} MB)\n"
                if gpu_util_percent >= 0:
                    log_msg += f"   GPU Utilization:   {gpu_util_percent:>5.1f}%\n"
            
            log_msg += f"{'='*80}\n"
            
            print(log_msg, flush=True)
            self.logger.info(log_msg)
            
            # Reset counters
            self.perf_stats["frames_total"] = 0
            self.perf_stats["frames_processed"] = 0
            self.perf_stats["detections"] = 0
            self.perf_stats["skipped_rate_limit"] = 0
            self.perf_stats["skipped_quality"] = 0
            self.perf_stats["skipped_temporal"] = 0
            self.perf_stats["skipped_color"] = 0
            self.perf_stats["skipped_cluster"] = 0
            self.perf_stats["last_log"] = now
            
        except Exception as e:
            print(f"❌ ERROR in logging: {e}", flush=True)

    # ============================================================================
    # VISUALIZATION
    # ============================================================================
    def draw_detections(self, frame, result):
        """Draw detection bounding boxes on frame"""
        if not result:
            return frame
        
        for bbox in result["bbox"]:
            x1, y1, x2, y2, lat, lon = bbox
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(
                frame, 
                f"Fire {result['confidence']:.2f}",
                (x1, y1 - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5,
                (0, 0, 255), 
                2
            )
        
        return frame


# ============================================================================
# EXAMPLE USAGE
# ============================================================================
if __name__ == "__main__":
    """Example usage with webcam"""
    print("\n" + "="*80)
    print("🔥 FIRE DETECTOR TEST MODE")
    print("="*80 + "\n")
    
    detector = FireDetector(
        model_path="yolov8n.pt",
        detection_interval=0.5,
        conf_threshold=0.4,
        resize=416,
        log_interval=3.0
    )
    
    cap = cv2.VideoCapture(0)
    
    print("\n🔥 Starting fire detection...")
    print("Press 'q' to quit\n")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to read frame from camera", flush=True)
            break
        
        result = detector.detect(
            frame,
            drone_lat=37.7749,
            drone_lon=-122.4194,
            drone_alt=100.0
        )
        
        if result:
            frame = detector.draw_detections(frame, result)
        
        cv2.imshow('Fire Detection', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()