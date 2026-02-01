import sys
sys.path.insert(0, "/home/nanosuper/opencv_build/build/lib/python3/")

import cv2
import torch
from fire_detection_test_model import FireDetector
import time
import numpy as np

print("🧪 GPU Fire Detection Test\n")
print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"CUDA version: {torch.version.cuda}")
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"GPU Memory: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.2f} GB\n")

# Initialize detector
detector = FireDetector(
    model_path="yolov8n.pt",
    detection_interval=0.0,  # No rate limiting for test
    conf_threshold=0.4,
    resize=416
)

# Create test frame
frame = np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8)

# Warmup
print("Warming up...")
for i in range(5):
    _ = detector.detect(frame, 22.5726, 88.3639, 50.0)

# Benchmark
print("\nRunning benchmark (20 frames)...")
times = []
for i in range(20):
    start = time.time()
    result = detector.detect(frame, 22.5726, 88.3639, 50.0)
    elapsed = time.time() - start
    times.append(elapsed)
    print(f"Frame {i+1}: {elapsed*1000:.2f}ms")

print(f"\n📊 Results:")
print(f"Average: {np.mean(times)*1000:.2f}ms")
print(f"Min: {min(times)*1000:.2f}ms")
print(f"Max: {max(times)*1000:.2f}ms")
print(f"Target FPS: {1/np.mean(times):.1f}")