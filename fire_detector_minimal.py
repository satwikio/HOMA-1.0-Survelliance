import cv2
import numpy as np
import time
import psutil
import os
import sys
import logging
import torch
from pathlib import Path
from collections import deque
from ultralytics import YOLO

# ============================================================================
# JETSON CUDA OPTIMIZATION
# ============================================================================
os.environ["CUDA_LAUNCH_BLOCKING"] = "0"
os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "max_split_size_mb:128"

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.benchmark = True


class FireDetector:
    """
    MINIMAL YOLOv8 DETECTOR
    - No image processing
    - No filters
    - No clustering
    - No temporal logic
    """

    def __init__(
        self,
        model_path="best.pt",
        conf_threshold=0.4,
        resize=416,
        log_interval=3.0,
    ):
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = YOLO(model_path).to(self.device)

        self.conf_threshold = conf_threshold
        self.resize = resize
        self.log_interval = log_interval

        self.process = psutil.Process(os.getpid())

        self.stats = {
            "frames_total": 0,
            "frames_processed": 0,
            "detections": 0,
            "gpu_times": deque(maxlen=300),
            "last_log": time.time(),
        }

        self._setup_logging()
        self._warmup()

        print("\n" + "=" * 80)
        print("🔥 MINIMAL FIRE DETECTOR READY")
        print(f"Device: {self.device}")
        print(f"Model: {model_path}")
        print("=" * 80 + "\n")

    # ----------------------------------------------------------------------
    def _setup_logging(self):
        logs_dir = Path("logs")
        logs_dir.mkdir(exist_ok=True)

        self.logger = logging.getLogger("fire_detector_minimal")
        self.logger.setLevel(logging.INFO)
        self.logger.handlers.clear()

        fh = logging.FileHandler(logs_dir / "fire_detector_minimal.log")
        ch = logging.StreamHandler(sys.stdout)

        fmt = logging.Formatter("%(asctime)s - %(message)s")
        fh.setFormatter(fmt)
        ch.setFormatter(fmt)

        self.logger.addHandler(fh)
        self.logger.addHandler(ch)

    # ----------------------------------------------------------------------
    def _warmup(self):
        dummy = np.zeros((self.resize, self.resize, 3), dtype=np.uint8)
        for _ in range(3):
            _ = self.model(
                dummy,
                device=self.device,
                half=(self.device == "cuda"),
                verbose=False,
            )

    # ----------------------------------------------------------------------
    def detect(self, frame, drone_lat=None, drone_lon=None, drone_alt=None):
        start = time.time()
        self.stats["frames_total"] += 1

        h, w = frame.shape[:2]
        resized = cv2.resize(frame, (self.resize, self.resize))

        gpu_start = time.time()
        results = self.model(
            resized,
            device=self.device,
            conf=self.conf_threshold,
            half=(self.device == "cuda"),
            verbose=False,
        )
        gpu_time = time.time() - gpu_start
        self.stats["gpu_times"].append(gpu_time)

        detections = []
        confidences = []

        x_scale = w / self.resize
        y_scale = h / self.resize

        for r in results:
            if r.boxes is None:
                continue

            boxes = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()

            for (x1, y1, x2, y2), conf in zip(boxes, confs):
                if conf < self.conf_threshold:
                    continue

                detections.append([
                    int(x1 * x_scale),
                    int(y1 * y_scale),
                    int(x2 * x_scale),
                    int(y2 * y_scale),
                    drone_lat,
                    drone_lon,
                ])
                confidences.append(float(conf))

        self.stats["frames_processed"] += 1

        if detections:
            self.stats["detections"] += len(detections)

        self._log_stats()

        if not detections:
            return None

        return {
            "bbox": detections,
            "confidence": round(sum(confidences) / len(confidences), 2),
            "timestamp": start,
        }

    # ----------------------------------------------------------------------
    def _log_stats(self):
        now = time.time()
        elapsed = now - self.stats["last_log"]

        if elapsed < self.log_interval:
            return

        avg_gpu = np.mean(self.stats["gpu_times"]) if self.stats["gpu_times"] else 0
        fps = self.stats["frames_processed"] / elapsed
        mem = self.process.memory_info().rss / 1024 / 1024
        cpu = self.process.cpu_percent(interval=None)

        log = (
            f"\n{'='*80}\n"
            f"🔥 MINIMAL FIRE DETECTOR STATS ({elapsed:.1f}s)\n"
            f"{'='*80}\n"
            f"Frames In:     {self.stats['frames_total']}\n"
            f"Frames Proc:   {self.stats['frames_processed']} ({fps:.1f} FPS)\n"
            f"Detections:    {self.stats['detections']}\n"
            f"GPU Inference: {avg_gpu*1000:.1f} ms ({1/avg_gpu:.1f} FPS)\n"
            f"CPU Usage:     {cpu:.1f}%\n"
            f"RAM Usage:     {mem:.1f} MB\n"
            f"{'='*80}\n"
        )

        print(log, flush=True)
        self.logger.info(log)

        self.stats["frames_total"] = 0
        self.stats["frames_processed"] = 0
        self.stats["detections"] = 0
        self.stats["last_log"] = now


# ============================================================================
# EXAMPLE USAGE
# ============================================================================
if __name__ == "__main__":
    detector = FireDetector(
        model_path="yolov8n.pt",
        conf_threshold=0.4,
        resize=416,
        log_interval=3.0,
    )

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        result = detector.detect(
            frame,
            drone_lat=37.7749,
            drone_lon=-122.4194,
            drone_alt=100.0,
        )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
