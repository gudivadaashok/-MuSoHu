# ZED Camera Installation Guide

This guide will help you install and configure the Stereolabs ZED camera with your MuSoHu system.

## Prerequisites

- ZED camera (any model: ZED, ZED Mini, ZED 2, ZED 2i, ZED X, etc.)
- USB 3.0 port (blue USB port recommended for best performance)
- NVIDIA GPU with CUDA support (for depth processing and AI features)
- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble

## Step 1: Install ZED SDK on Host

The ZED SDK must be installed on the **host machine** (not inside Docker) before building the ROS 2 wrapper.

### For Jetson (ARM64)

```bash
# Download ZED SDK for Jetson
# Check latest version at: https://www.stereolabs.com/developers/release/
cd ~/Downloads
wget https://download.stereolabs.com/zedsdk/4.0/l4t35.4/jetsons -O ZED_SDK_Jetson.run

# Or for specific JetPack version
# wget https://download.stereolabs.com/zedsdk/4.1/l4t36.2/jetsons -O ZED_SDK_Jetson.run

# Make executable and run installer
chmod +x ZED_SDK_Jetson.run
./ZED_SDK_Jetson.run

# Follow the prompts:
# - Accept the license
# - Choose installation directory (default: /usr/local/zed)
# - Install Python API (recommended: yes)
# - Install AI modules (recommended: yes)
```

### For x86_64 Ubuntu 22.04

```bash
# Download ZED SDK for Ubuntu 22.04
cd ~/Downloads
wget https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu22 -O ZED_SDK_Ubuntu22.run

# Make executable and run installer
chmod +x ZED_SDK_Ubuntu22.run
./ZED_SDK_Ubuntu22.run
```

### Verify Installation

```bash
# Check ZED SDK installation
ls -la /usr/local/zed

# Test ZED camera detection
/usr/local/zed/tools/ZED_Diagnostic

# Launch ZED Explorer (GUI tool)
/usr/local/zed/tools/ZED_Explorer
```

## Step 2: Configure CUDA (If Not Already Done)

```bash
# Verify CUDA installation
nvidia-smi
nvcc --version

# For Jetson, install CUDA toolkit if missing
sudo apt install nvidia-jetpack nvidia-jetpack-dev

# Add CUDA to PATH (add to ~/.bashrc for persistence)
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

## Step 3: Clone ZED ROS 2 Wrapper in Docker

The ZED wrapper will be built inside the Docker container, but the SDK on the host will be mounted.

```bash
# Enter the Docker container
docker exec -it ros2_vnc bash

# Navigate to workspace
cd /home/ubuntu/ros2_ws/src

# Clone ZED ROS 2 wrapper (if not already done during build)
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git

# Exit container
exit
```

## Step 4: Update Docker Configuration

The ZED SDK needs to be mounted from the host into the container. Update your `docker-compose.override.yml`:

```yaml
services:
  ros2_vnc:
    volumes:
      # ... existing volumes ...
      - /usr/local/zed:/usr/local/zed:ro  # Mount ZED SDK (read-only)
    devices:
      # ... existing devices ...
      - /dev/video0:/dev/video0  # ZED camera video device
      - /dev/video1:/dev/video1  # ZED camera video device (if dual camera)
    environment:
      # ... existing environment ...
      - LD_LIBRARY_PATH=/usr/local/zed/lib:$LD_LIBRARY_PATH
```

## Step 5: Rebuild Docker Container

```bash
# Stop containers
docker compose down

# Rebuild with ZED support
docker compose build ros2_vnc

# Start containers
docker compose up -d
```

## Step 6: Build ZED ROS 2 Wrapper Inside Container

```bash
# Enter container
docker exec -it ros2_vnc bash

# Navigate to workspace
cd /home/ubuntu/ros2_ws

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build ZED packages
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --cmake-args=-DCMAKE_BUILD_TYPE=Release \
  --packages-select zed_wrapper zed_components \
  --parallel-workers $(nproc)

# Source the workspace
source install/local_setup.bash

# Exit container
exit
```

## Step 7: Test ZED Camera

```bash
# Check if camera is detected
docker exec ros2_vnc bash -c "lsusb | grep -i 'ZED\|Stereolabs'"

# Launch ZED node
docker exec -it ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
"

# In another terminal, check topics
docker exec -it ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic list | grep zed
"

# Check camera info
docker exec -it ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic echo /zed/zed_node/left/camera_info --once
"
```

## Step 8: Launch with Helper Script

```bash
# Use the provided launch script
docker exec -d ros2_vnc /usr/local/bin/launch-zed.sh

# Check logs
docker exec ros2_vnc tail -f /tmp/zed.log
```

## Step 9: Visualize in RViz

```bash
# Launch RViz with ZED configuration
docker exec -d ros2_vnc /usr/local/bin/launch-rviz.sh

# Access VNC desktop
# Open browser: http://localhost:6080
# Password: ubuntu
# RViz should show ZED camera, depth, and point cloud
```

## Troubleshooting

### Camera Not Detected

```bash
# Check USB connection
lsusb | grep -i "ZED\|Stereolabs"

# Check video devices
ls -la /dev/video*

# Run ZED diagnostic
/usr/local/zed/tools/ZED_Diagnostic

# Check permissions
sudo chmod 666 /dev/video*
```

### CUDA Errors

```bash
# Verify CUDA
nvidia-smi
nvcc --version

# Check ZED SDK CUDA compatibility
cat /usr/local/zed/include/sl/Camera.hpp | grep "CUDA"

# Reinstall ZED SDK if version mismatch
```

### Build Errors

```bash
# If CMake can't find ZED SDK
export ZED_SDK_ROOT_DIR=/usr/local/zed

# If missing dependencies
sudo apt install ros-humble-image-transport ros-humble-diagnostic-updater

# Clean and rebuild
cd /home/ubuntu/ros2_ws
rm -rf build install log
colcon build --symlink-install --packages-select zed_wrapper zed_components
```

### Low Frame Rate

1. Ensure USB 3.0 connection (blue port)
2. Reduce resolution: `grab_resolution:=HD720` or `VGA`
3. Disable AI features if not needed
4. Check system resources with `htop`

### Depth Quality Issues

1. Adjust confidence thresholds in config
2. Use `depth_mode:=NEURAL` for best quality
3. Ensure good lighting
4. Run camera calibration tool

## Camera Models

Replace `camera_model` parameter with your ZED model:

- `zed` - ZED (first generation)
- `zedm` - ZED Mini
- `zed2` - ZED 2
- `zed2i` - ZED 2i (improved)
- `zedx` - ZED X
- `zedxm` - ZED X Mini
- `zedxonegs` - ZED X One GS
- `zedxone4k` - ZED X One 4K
- `zedxonehdr` - ZED X One HDR

## Configuration Files

Key configuration files in the ZED wrapper:

- `common.yaml` - Common parameters for all cameras
- `zed2i.yaml` - Model-specific parameters (replace with your model)

Edit at: `/home/ubuntu/ros2_ws/src/zed-ros2-wrapper/zed_wrapper/config/`

## Advanced Features

### Enable Object Detection

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  object_detection.od_enabled:=true \
  object_detection.model:=MULTI_CLASS_BOX_MEDIUM
```

### Enable Body Tracking

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  body_tracking.bt_enabled:=true
```

### Enable Spatial Mapping

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  mapping.mapping_enabled:=true
```

## Alternative: Creating Custom ZED Docker Image

For production deployments or custom applications, you can create your own Docker image with ZED SDK built-in.

### Option A: Using Stereolabs Official Base Images

Stereolabs provides pre-built Docker images with ZED SDK already installed:

```dockerfile
# For Ubuntu 22.04 with CUDA 12.1
FROM stereolabs/zed:4.1-gl-devel-cuda12.1-ubuntu22.04

# For Ubuntu 20.04 with CUDA 11.4
FROM stereolabs/zed:4.1-gl-devel-cuda11.4-ubuntu20.04

# For Jetson with JetPack 5.1.2 (L4T 35.4)
FROM stereolabs/zed:4.1-gl-devel-jetson-jp5.1.2

# For Jetson with JetPack 6.0 (L4T 36.2)
FROM stereolabs/zed:4.1-gl-devel-jetson-jp6.0
```

### Option B: Custom Dockerfile for MuSoHu with ZED

Create a custom Dockerfile that extends the current MuSoHu setup:

```dockerfile
# File: Dockerfile.ros2.zed
FROM stereolabs/zed:4.1-gl-devel-cuda12.1-ubuntu22.04

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-diagnostic-updater \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up workspace
RUN mkdir -p /home/ubuntu/ros2_ws/src
WORKDIR /home/ubuntu/ros2_ws/src

# Clone ZED ROS 2 wrapper
RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git

# Install dependencies and build
WORKDIR /home/ubuntu/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release \
    --packages-select zed_wrapper zed_components --parallel-workers $(nproc)"

# Source ROS 2 in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR /workspace

# Default command
CMD ["/bin/bash"]
```

### Building the Custom Image

```bash
# Build the image
docker build -t musohu-zed:v1 -f Dockerfile.ros2.zed .

# For Jetson (build on x86 host for faster compilation)
docker buildx build --platform linux/arm64 -t musohu-zed:jetson-v1 -f Dockerfile.ros2.zed .
```

### Running with Docker Run

```bash
# For x86_64 with NVIDIA GPU
docker run -it --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  --privileged \
  -v /dev:/dev \
  -v /tmp:/tmp \
  -v $(pwd):/workspace \
  -v ~/.ai_models:/usr/local/zed/resources \
  musohu-zed:v1

# For Jetson
docker run -it --runtime nvidia \
  --privileged \
  -v /dev:/dev \
  -v /tmp:/tmp \
  -v $(pwd):/workspace \
  -v ~/.ai_models:/usr/local/zed/resources \
  musohu-zed:jetson-v1

# For ZED X (GMSL2 cameras)
docker run -it --runtime nvidia \
  --privileged \
  -v /dev:/dev \
  -v /tmp:/tmp \
  -v /etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service \
  -v /var/nvidia/nvcam/settings/:/var/nvidia/nvcam/settings/ \
  -v ~/.ai_models:/usr/local/zed/resources \
  musohu-zed:v1
```

### Important Volumes for ZED Docker

| Volume | Required | Purpose |
|--------|----------|---------|
| `/dev:/dev` | Yes | Access to camera devices |
| `/tmp:/tmp` | Yes (ZED X) | Temporary files for GMSL2 |
| `/usr/local/zed/resources` | Optional | Cache AI models (Object Detection, Body Tracking) |
| `/var/nvidia/nvcam/settings/` | Yes (ZED X) | GMSL2 camera settings |
| `/etc/systemd/system/zed_x_daemon.service` | Yes (ZED X) | ZED X daemon service |

### Optimizing Docker Image Size

Follow these best practices to keep your image small:

```dockerfile
# ✅ GOOD: Consolidate RUN commands, clean up in same layer
RUN apt-get update -y && \
    apt-get install --no-install-recommends -y \
    ros-humble-zed-wrapper \
    lsb-release && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/*

# ❌ BAD: Multiple layers, doesn't clean up
RUN apt-get update -y
RUN apt-get install -y ros-humble-zed-wrapper
RUN apt-get install -y lsb-release
```

**Best Practices:**
- Use `--no-install-recommends` to avoid optional packages
- Clean `/var/lib/apt/lists/*` in the same RUN step
- Remove tarballs and archives after extraction
- Use multi-stage builds for development vs production
- Consolidate RUN commands to minimize layers

### Sharing Your Image

#### Option 1: Docker Hub (Public/Private)

```bash
# Tag your image
docker tag musohu-zed:v1 yourusername/musohu-zed:v1

# Login to Docker Hub
docker login

# Push to Docker Hub
docker push yourusername/musohu-zed:v1

# Pull on target machine
docker pull yourusername/musohu-zed:v1
```

#### Option 2: Local Registry Server

```bash
# Start local registry
docker run -d -p 5000:5000 --name registry registry:2

# Tag for local registry
docker tag musohu-zed:v1 localhost:5000/musohu-zed:v1

# Push to local registry
docker push localhost:5000/musohu-zed:v1

# Pull from local registry on target machine
docker pull <registry-ip>:5000/musohu-zed:v1
```

#### Option 3: Save/Load as File

```bash
# Save image to file (useful for offline transfer)
docker save musohu-zed:v1 -o musohu-zed_v1.tar

# Transfer file to target machine (USB, SCP, etc.)
scp musohu-zed_v1.tar jetson@192.168.1.100:~

# Load image on target machine
docker load -i musohu-zed_v1.tar
```

### Multi-Stage Build Example

For production deployments, use multi-stage builds to reduce image size:

```dockerfile
# Stage 1: Build stage
FROM stereolabs/zed:4.1-gl-devel-cuda12.1-ubuntu22.04 AS builder

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/ubuntu/ros2_ws/src
RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git

WORKDIR /home/ubuntu/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release"

# Stage 2: Production stage (smaller)
FROM stereolabs/zed:4.1-runtime-cuda12.1-ubuntu22.04

# Copy only built artifacts
COPY --from=builder /home/ubuntu/ros2_ws/install /home/ubuntu/ros2_ws/install
COPY --from=builder /opt/ros/humble /opt/ros/humble

# Install only runtime dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-humble-ros-base \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
CMD ["/bin/bash"]
```

### Testing Your Image

```bash
# Test basic ZED functionality
docker run -it --gpus all --privileged -v /dev:/dev musohu-zed:v1 \
  /usr/local/zed/tools/ZED_Diagnostic

# Test ROS 2 ZED node
docker run -it --gpus all --privileged -v /dev:/dev musohu-zed:v1 bash -c \
  "source /home/ubuntu/ros2_ws/install/setup.bash && \
   ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i"
```

### Important Notes for Jetson

**⚠️ L4T Version Matching:**
When running Docker on Jetson, ensure the L4T (Linux for Tegra) version of your host matches the container:

```bash
# Check host L4T version
cat /etc/nv_tegra_release

# Use matching container
# JetPack 5.1.2 = L4T 35.4
# JetPack 6.0 = L4T 36.2
```

**💡 Cross-Compilation Tip:**
Build Jetson Docker containers on x86 host to avoid long compilation times:

```bash
# On x86 host, build for ARM64
docker buildx create --name mybuilder --use
docker buildx build --platform linux/arm64 -t musohu-zed:jetson-v1 --load .
```

## Resources

- [ZED SDK Download](https://www.stereolabs.com/developers/release/)
- [ZED ROS 2 Documentation](https://www.stereolabs.com/docs/ros2)
- [ZED ROS 2 Wrapper GitHub](https://github.com/stereolabs/zed-ros2-wrapper)
- [ZED Docker Documentation](https://www.stereolabs.com/docs/docker/)
- [Stereolabs Docker Hub](https://hub.docker.com/r/stereolabs/zed)
- [API Documentation](https://www.stereolabs.com/docs/api/)
- [Multi-stage Builds](https://docs.docker.com/build/building/multi-stage/)
