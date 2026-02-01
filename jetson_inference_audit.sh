#!/bin/bash

echo "================================================="
echo "      Jetson Nano Super – Inference Audit"
echo "================================================="

echo -e "\n[1] Board & SoC Info"
echo "-----------------------------------------------"
cat /proc/device-tree/model 2>/dev/null || echo "Model info not found"
uname -a

echo -e "\n[2] JetPack / L4T Version"
echo "-----------------------------------------------"
if [ -f /etc/nv_tegra_release ]; then
    cat /etc/nv_tegra_release
else
    echo "JetPack info not found"
fi

echo -e "\n[3] CUDA Toolkit"
echo "-----------------------------------------------"
if command -v nvcc >/dev/null 2>&1; then
    nvcc --version
else
    echo "CUDA not found (nvcc missing)"
fi

echo -e "\n[4] CUDA Driver"
echo "-----------------------------------------------"
if [ -f /usr/lib/aarch64-linux-gnu/tegra/libcuda.so ]; then
    ls -l /usr/lib/aarch64-linux-gnu/tegra/libcuda.so
else
    echo "CUDA driver not found"
fi

echo -e "\n[5] cuDNN"
echo "-----------------------------------------------"
dpkg -l | grep libcudnn || echo "cuDNN not installed"

echo -e "\n[6] TensorRT"
echo "-----------------------------------------------"
dpkg -l | grep tensorrt || echo "TensorRT not installed"

echo -e "\n[7] GPU Device Check"
echo "-----------------------------------------------"
ls /dev | grep -i nvgpu || echo "nvgpu device not found"

echo -e "\n[8] nvidia-smi (expected to fail on Jetson)"
echo "-----------------------------------------------"
nvidia-smi 2>/dev/null || echo "nvidia-smi not supported on Jetson (this is normal)"

echo -e "\n[9] Python & Pip"
echo "-----------------------------------------------"
python3 --version
pip3 --version

echo -e "\n[10] PyTorch GPU Test"
echo "-----------------------------------------------"
python3 - << 'EOF'
try:
    import torch
    print("PyTorch version:", torch.__version__)
    print("CUDA available:", torch.cuda.is_available())
    if torch.cuda.is_available():
        print("CUDA device:", torch.cuda.get_device_name(0))
except Exception as e:
    print("PyTorch not usable:", e)
EOF

echo -e "\n[11] OpenCV Build Info"
echo "-----------------------------------------------"
python3 - << 'EOF'
try:
    import cv2
    print("OpenCV version:", cv2.__version__)
    print("CUDA support:", cv2.cuda.getCudaEnabledDeviceCount() > 0)
except Exception as e:
    print("OpenCV not usable:", e)
EOF

echo -e "\n[12] Disk Space"
echo "-----------------------------------------------"
df -h /

echo -e "\n[13] RAM"
echo "-----------------------------------------------"
free -h

echo -e "\n================= Audit Complete ================="

