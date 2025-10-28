# GPU and Device Support Configuration

## Overview

This project provides **two configurations**:

1. **`docker-compose.yml`** - Works on macOS and Linux (GPU/device features commented out)
2. **`docker-compose.linux.yml`** - Linux-only with full GPU and device support enabled

## Platform Support

### macOS / Docker Desktop

**Configuration:** `docker-compose.yml` (default)

**Features:**
- ✅ Basic ROS2 functionality
- ✅ VNC desktop access
- ✅ All software features
- ❌ No GPU passthrough
- ❌ No device passthrough

**Usage:**
```bash
./scripts/start.sh
```

---

### Linux / Docker Engine

**Configuration:** `docker-compose.linux.yml` (recommended for Linux)

**Features:**
- ✅ Full GPU passthrough (NVIDIA)
- ✅ Complete device access (`/dev:/dev`)
- ✅ Hardware sensors (cameras, IMU, GPS)
- ✅ ReSpeaker USB Mic Array (ID 2886:0018)
- ✅ Full ROS2 hardware integration

**Requirements:**
1. Linux OS (Ubuntu 20.04/22.04 recommended)
2. Docker Engine (not Docker Desktop)
3. NVIDIA GPU drivers (for GPU features)
4. NVIDIA Container Toolkit (for GPU features)

**Usage:**
```bash
# Option 1: Use start.sh script (will auto-detect and prompt)
./scripts/start.sh

# Option 2: Manually use Linux config
docker-compose -f docker-compose.linux.yml up -d --build

# Option 3: Replace default config on Linux deployment
cp docker-compose.linux.yml docker-compose.yml
./scripts/start.sh
```

---

## Installing NVIDIA Container Toolkit (Linux Only)

### Ubuntu/Debian

```bash
# Add NVIDIA package repositories
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# Install nvidia-container-toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Restart Docker
sudo systemctl restart docker

# Test GPU access
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

---

## Docker Compose Configuration

The single `docker-compose.yml` includes GPU and device features that work differently on each platform:

### Shared Configuration (Works Everywhere)

```yaml
services:
  ros2_vnc:
    volumes:
      - .:/workspace
      - ~/Desktop/Docker-Volumns/ros2_workspace:/home/ubuntu/ros2_ws
    shm_size: 512m
    environment:
      - DISPLAY=:1
      - VNC_PASSWORD=ubuntu
```

### Linux-Only Features (Ignored on macOS)

```yaml
    devices:
      - /dev:/dev  # Full device access
      - /dev/snd:/dev/snd  # Audio devices (ReSpeaker USB Mic Array)
      - /dev/bus/usb:/dev/bus/usb  # USB devices
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
```

**Behavior:**
- **macOS**: These sections are ignored, container starts normally
- **Linux**: Full hardware access is enabled when available

---

## Automatic Detection

The `start.sh` script automatically detects your platform and provides appropriate feedback:

**Detection Logic:**
- **macOS:** Warns that GPU/device features will be ignored
- **Linux without NVIDIA GPU:** Runs with device access but no GPU
- **Linux with NVIDIA GPU:** Enables full GPU and device support

**No manual configuration needed** - the same `docker-compose.yml` works everywhere!

---

## Verifying GPU Access

### Inside the Container

```bash
# Enter the ROS2 container
docker exec -it ros2_vnc bash

# Check NVIDIA GPU (Linux only)
nvidia-smi

# Check device access
ls /dev
```

### Expected Output (Linux with GPU)

```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 525.xx.xx    Driver Version: 525.xx.xx    CUDA Version: 12.0   |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
| ...                                                                         |
+-----------------------------------------------------------------------------+
```

---

## Common Use Cases

### ROS2 with Camera (Linux)

```bash
# List video devices
ls /dev/video*

# Inside container
ros2 run usb_cam usb_cam_node
```

### ROS2 with Lidar (Linux)

```bash
# Check USB serial devices
ls /dev/ttyUSB* /dev/ttyACM*

# Inside container
ros2 run rplidar_ros rplidarNode
```

### ROS2 Simulation (Works on macOS)

```bash
# Gazebo and RViz work without GPU
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## Troubleshooting

### macOS: "could not select device driver nvidia" (FIXED)

**This error no longer occurs.** The updated `docker-compose.yml` is compatible with both macOS and Linux. GPU features are safely ignored on macOS.

### Linux: "nvidia-smi not found"

Install NVIDIA drivers and Container Toolkit:

```bash
# Check if GPU is detected
lspci | grep -i nvidia

# Install drivers
sudo ubuntu-drivers autoinstall

# Install Container Toolkit (see above)
```

### Device Permission Denied

Add your user to the `docker` group:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

---

## Migration Between Platforms

### Moving from macOS to Linux

1. Copy your workspace to Linux machine
2. Install NVIDIA drivers and Container Toolkit
3. Run `./scripts/start.sh` (auto-detects Linux)
4. Verify GPU access with `nvidia-smi`

### Testing on macOS, Deploying on Linux

1. Develop and test on macOS with `docker-compose.yml`
2. Push code to Git repository
3. Clone on Linux machine
4. Run with `docker-compose.linux.yml` for hardware access

---

## References

- [NVIDIA Container Toolkit Installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- [Docker Compose GPU Support](https://docs.docker.com/compose/gpu-support/)
- [ROS2 Hardware Integration](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
