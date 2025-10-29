# Building and Deploying MuSoHu with ZED Camera

This guide explains how to build and deploy the complete MuSoHu system with ZED camera support using Docker.

## Prerequisites

- Docker Engine 20.10+
- Docker Compose v2+
- NVIDIA Container Toolkit installed
- ZED SDK installed on host machine (see [ZED_INSTALLATION.md](./ZED_INSTALLATION.md))
- NVIDIA GPU with CUDA support
- ZED camera connected via USB 3.0

## Quick Start

### Option 1: Using Pre-built Base Image (Recommended)

This method uses Stereolabs' official ZED Docker images as the base:

```bash
# Build the ZED-enabled MuSoHu image
docker compose -f docker-compose.zed.yml build

# Start all services
docker compose -f docker-compose.zed.yml up -d

# Check logs
docker compose -f docker-compose.zed.yml logs -f ros2_vnc_zed

# Access services
# - VNC Desktop: http://localhost:6080 (password: ubuntu)
# - Web App: http://localhost:5001
```

### Option 2: Manual Build with Custom Dockerfile

```bash
# Build custom image
docker build -t musohu-zed:v1 -f Dockerfile.ros2.zed .

# Run with docker run
docker run -it --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  --privileged \
  -v /dev:/dev \
  -v $(pwd):/workspace \
  -v ~/.zed_ai_models:/usr/local/zed/resources \
  -p 6080:80 \
  -p 5901:5901 \
  -p 6699:6699/udp \
  -p 7788:7788/udp \
  musohu-zed:v1
```

## Building for Production

### Step 1: Update Configuration

Edit `docker-compose.zed.yml` to match your setup:

```yaml
environment:
  - ZED_CAMERA_MODEL=zed2i  # Change to: zed, zedm, zed2, zed2i, zedx, etc.
```

### Step 2: Build Image

```bash
# Build with build args for optimization
docker compose -f docker-compose.zed.yml build --no-cache --pull

# Or build specific service
docker compose -f docker-compose.zed.yml build ros2_vnc_zed
```

### Step 3: Tag for Registry

```bash
# Tag for Docker Hub
docker tag musohu-zed:v1 yourusername/musohu-zed:v1
docker tag musohu-zed:v1 yourusername/musohu-zed:latest

# Tag for private registry
docker tag musohu-zed:v1 registry.example.com/musohu-zed:v1
```

### Step 4: Push to Registry

```bash
# Push to Docker Hub
docker login
docker push yourusername/musohu-zed:v1
docker push yourusername/musohu-zed:latest

# Push to private registry
docker push registry.example.com/musohu-zed:v1
```

## Building for NVIDIA Jetson

Building Docker images on Jetson can be slow. Build on x86 host and transfer to Jetson.

### Method 1: Cross-Platform Build (Recommended)

```bash
# On x86_64 host, set up buildx
docker buildx create --name multiarch --use
docker buildx inspect --bootstrap

# Build for ARM64 (Jetson)
docker buildx build \
  --platform linux/arm64 \
  -f Dockerfile.ros2.zed \
  -t musohu-zed:jetson-v1 \
  --load \
  .

# Save and transfer to Jetson
docker save musohu-zed:jetson-v1 -o musohu-zed-jetson.tar
scp musohu-zed-jetson.tar jetson@192.168.1.100:~

# On Jetson, load the image
docker load -i musohu-zed-jetson.tar
```

### Method 2: Build on Jetson Directly

```bash
# On Jetson, use Jetson-specific base image
# Edit Dockerfile.ros2.zed, change FROM line:
FROM stereolabs/zed:4.1-gl-devel-jetson-jp6.0

# Build (this will take time)
docker compose -f docker-compose.zed.yml build

# Monitor build progress
docker compose -f docker-compose.zed.yml build 2>&1 | tee build.log
```

### Important: L4T Version Matching

Ensure Docker image L4T matches Jetson host:

```bash
# Check host L4T version
cat /etc/nv_tegra_release
# R35 (release), REVISION: 4.1 = JetPack 5.1.2, L4T 35.4

# Use corresponding Docker base:
# JetPack 5.1.2 → stereolabs/zed:4.1-gl-devel-jetson-jp5.1.2
# JetPack 6.0   → stereolabs/zed:4.1-gl-devel-jetson-jp6.0
```

## Image Size Optimization

The ZED Docker image can be large (5-8 GB). Here are optimization strategies:

### Multi-Stage Build

Create `Dockerfile.ros2.zed.multistage`:

```dockerfile
# Stage 1: Build
FROM stereolabs/zed:4.1-gl-devel-cuda12.1-ubuntu22.04 AS builder

# ... install build dependencies ...
# ... build ROS 2 packages ...

# Stage 2: Runtime (smaller)
FROM stereolabs/zed:4.1-runtime-cuda12.1-ubuntu22.04

# Copy only built artifacts from builder
COPY --from=builder /home/ubuntu/ros2_ws/install /home/ubuntu/ros2_ws/install
COPY --from=builder /opt/ros/humble /opt/ros/humble

# Install only runtime dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-humble-ros-base \
    && rm -rf /var/lib/apt/lists/*

# ... rest of runtime configuration ...
```

Build:
```bash
docker build -t musohu-zed:slim -f Dockerfile.ros2.zed.multistage .
```

### Clean Build Artifacts

```dockerfile
# In Dockerfile, combine and clean in same RUN
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    package1 package2 && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
```

### Exclude Development Tools

For production, exclude unnecessary tools:

```dockerfile
# Don't install if not needed:
# - vim, nano (use exec with host editor)
# - build-essential (only needed for building)
# - git (clone before build, not at runtime)
```

## Deployment Scenarios

### Scenario 1: Development on Workstation

```bash
# Use docker-compose for easy development
docker compose -f docker-compose.zed.yml up -d

# Hot-reload code changes (volumes mounted)
# Edit code on host, changes reflect in container

# Access services
firefox http://localhost:6080  # VNC
firefox http://localhost:5001  # Web App
```

### Scenario 2: Production on Robot

```bash
# Pull pre-built image
docker pull yourusername/musohu-zed:latest

# Run with restart policy
docker run -d --restart=always \
  --name musohu-robot \
  --gpus all \
  --privileged \
  -v /dev:/dev \
  -v ~/.zed_ai_models:/usr/local/zed/resources \
  -p 6080:80 \
  yourusername/musohu-zed:latest
```

### Scenario 3: Edge Deployment on Jetson

```bash
# Transfer image to Jetson
docker save musohu-zed:jetson-v1 | ssh jetson@robot.local docker load

# On Jetson, run with nvidia runtime
docker run -d --restart=unless-stopped \
  --name musohu-edge \
  --runtime nvidia \
  --privileged \
  -v /dev:/dev \
  -v ~/.zed_ai_models:/usr/local/zed/resources \
  -p 6080:80 \
  musohu-zed:jetson-v1
```

## Testing Your Build

### Test 1: Basic Container Start

```bash
# Start container
docker compose -f docker-compose.zed.yml up -d

# Check if running
docker compose -f docker-compose.zed.yml ps

# Check logs for errors
docker compose -f docker-compose.zed.yml logs ros2_vnc_zed | grep -i error
```

### Test 2: ZED Camera Detection

```bash
# Check if ZED is detected
docker exec ros2_vnc_zed lsusb | grep -i "ZED\|Stereolabs"

# Run ZED diagnostic
docker exec ros2_vnc_zed /usr/local/zed/tools/ZED_Diagnostic

# Check video devices
docker exec ros2_vnc_zed ls -la /dev/video*
```

### Test 3: ROS 2 Topics

```bash
# Check all sensors
docker exec ros2_vnc_zed /usr/local/bin/check-sensors.sh

# List ZED topics
docker exec ros2_vnc_zed bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic list | grep zed
"

# Check camera feed rate
docker exec ros2_vnc_zed bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic hz /zed/zed_node/left/image_rect_color
"
```

### Test 4: VNC Access

```bash
# Check VNC server
docker exec ros2_vnc_zed ps aux | grep vnc

# Test HTTP access
curl -I http://localhost:6080

# Open in browser
firefox http://localhost:6080
```

### Test 5: All Sensors Integration

```bash
# Start all sensors
docker exec ros2_vnc_zed supervisorctl start all

# Check status
docker exec ros2_vnc_zed supervisorctl status

# Launch RViz with all sensors
docker exec -d ros2_vnc_zed /usr/local/bin/launch-rviz.sh

# View in VNC browser
firefox http://localhost:6080
```

## Troubleshooting

### Build Fails

```bash
# Clean Docker build cache
docker builder prune -a

# Rebuild without cache
docker compose -f docker-compose.zed.yml build --no-cache

# Check disk space
df -h
docker system df
```

### ZED Not Detected

```bash
# Check USB connection
lsusb | grep -i ZED

# Check permissions
sudo chmod 666 /dev/video*

# Restart container
docker compose -f docker-compose.zed.yml restart
```

### CUDA Errors

```bash
# Verify NVIDIA runtime
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi

# Check NVIDIA container toolkit
sudo systemctl status nvidia-container-toolkit

# Reinstall if needed
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### Out of Memory

```bash
# Increase shared memory
# In docker-compose.zed.yml:
shm_size: 4gb  # Increase from 2gb

# Or reduce workload
# - Lower camera resolution
# - Disable AI features
# - Reduce point cloud rate
```

## Maintenance

### Update Image

```bash
# Pull latest base image
docker pull stereolabs/zed:4.1-gl-devel-cuda12.1-ubuntu22.04

# Rebuild
docker compose -f docker-compose.zed.yml build --pull

# Restart
docker compose -f docker-compose.zed.yml up -d
```

### Clean Up

```bash
# Stop and remove containers
docker compose -f docker-compose.zed.yml down

# Remove old images
docker image prune -a

# Remove build cache
docker builder prune -a

# Remove volumes (careful!)
docker volume prune
```

### Backup

```bash
# Save current image
docker save musohu-zed:v1 | gzip > musohu-zed-v1-backup.tar.gz

# Backup volumes
tar -czf volumes-backup.tar.gz ~/Desktop/Docker-Volumns/
```

## Next Steps

- [ZED Camera Configuration](./ZED_QUICK_REFERENCE.md)
- [Advanced Features](./ZED_INSTALLATION.md#advanced-features)
- [Sensor Fusion](../README.md#integration)

## Resources

- [Docker Documentation](https://docs.docker.com/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/)
- [ZED Docker Guide](https://www.stereolabs.com/docs/docker/)
- [Stereolabs Docker Hub](https://hub.docker.com/r/stereolabs/zed)
