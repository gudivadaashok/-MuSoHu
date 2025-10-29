# ZED Camera Integration Summary

## What Was Added

This document summarizes the ZED camera integration into the MuSoHu system.

## 📁 Files Created

### Docker Configuration
1. **`Dockerfile.ros2.zed`** - Complete Dockerfile with ZED SDK and all MuSoHu sensors
2. **`docker-compose.zed.yml`** - Docker Compose configuration for ZED-enabled deployment

### Documentation
1. **`docs/ZED_INSTALLATION.md`** - Comprehensive installation guide
2. **`docs/ZED_QUICK_REFERENCE.md`** - Quick command reference
3. **`docs/ZED_DOCKER_BUILD.md`** - Docker build and deployment guide

### Scripts
1. **`scripts/launch-zed.sh`** - Helper script to launch ZED camera node

### Configuration
1. **`scripts/musohu_sensors.rviz`** - Updated RViz config with ZED displays

## 📝 Files Modified

1. **`README.md`** - Added complete ZED camera section
2. **`Dockerfile.ros2`** - Added ZED dependencies
3. **`docker-compose.override.yml`** - Added video device mappings
4. **`scripts/check-sensors.sh`** - Added ZED camera monitoring

## 🎯 Features Enabled

### Hardware Support
- ✅ ZED, ZED Mini, ZED 2, ZED 2i, ZED X camera families
- ✅ USB 3.0 connectivity
- ✅ CUDA-accelerated processing

### Data Streams
- ✅ Stereo RGB images (rectified & raw)
- ✅ Depth maps with Neural depth mode
- ✅ 3D colored point clouds
- ✅ Visual odometry and tracking
- ✅ IMU data (6/9-axis depending on model)
- ✅ Camera info and calibration

### Advanced Features
- ✅ Object Detection (AI models)
- ✅ Body Tracking (skeleton detection)
- ✅ Spatial Mapping (3D reconstruction)
- ✅ GNSS fusion (geo-localization)
- ✅ Point Cloud Transport compression

### Visualization
- ✅ RViz2 integration
- ✅ Multi-camera view displays
- ✅ Depth visualization
- ✅ Point cloud rendering
- ✅ TF frame visualization

## 🚀 Quick Start Commands

### Using Standard Setup (Host SDK)
```bash
# 1. Install ZED SDK on host (see docs/ZED_INSTALLATION.md)
# 2. Update docker-compose.override.yml with SDK mount
# 3. Build and run
docker compose build ros2_vnc
docker compose up -d
docker exec -d ros2_vnc /usr/local/bin/launch-zed.sh
```

### Using ZED Docker Image (Recommended)
```bash
# Build ZED-enabled image
docker compose -f docker-compose.zed.yml build

# Start all services
docker compose -f docker-compose.zed.yml up -d

# Check sensors
docker exec ros2_vnc_zed /usr/local/bin/check-sensors.sh

# View in RViz
firefox http://localhost:6080
```

### Manual Docker Run
```bash
# Build image
docker build -t musohu-zed:v1 -f Dockerfile.ros2.zed .

# Run container
docker run -it --gpus all --privileged \
  -v /dev:/dev \
  -v ~/.zed_ai_models:/usr/local/zed/resources \
  -p 6080:80 -p 5901:5901 \
  musohu-zed:v1
```

## 📊 Published Topics

### Images
- `/zed/zed_node/left/image_rect_color` - Left RGB image
- `/zed/zed_node/right/image_rect_color` - Right RGB image
- `/zed/zed_node/depth/depth_registered` - Depth map

### Point Clouds
- `/zed/zed_node/point_cloud/cloud_registered` - RGB point cloud

### Odometry & Tracking
- `/zed/zed_node/odom` - Visual odometry
- `/zed/zed_node/pose` - Camera pose
- `/zed/zed_node/path_map` - Trajectory

### Sensor Data
- `/zed/zed_node/imu/data` - IMU measurements
- `/zed/zed_node/left/camera_info` - Camera calibration

### AI Features (when enabled)
- `/zed/zed_node/obj_det/objects` - Detected objects
- `/zed/zed_node/body_trk/skeletons` - Skeleton tracking

## 🔧 Configuration

### Camera Model Selection
```bash
# Set in docker-compose.zed.yml
environment:
  - ZED_CAMERA_MODEL=zed2i  # Options: zed, zedm, zed2, zed2i, zedx, zedxm
```

### Performance Modes

**High Quality** (15 FPS, HD1080, Neural depth)
```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  grab_resolution:=HD1080 \
  grab_frame_rate:=15 \
  depth_mode:=NEURAL
```

**Balanced** (30 FPS, HD720, Quality depth)
```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  grab_resolution:=HD720 \
  grab_frame_rate:=30 \
  depth_mode:=QUALITY
```

**High Performance** (60 FPS, VGA, Performance depth)
```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  grab_resolution:=VGA \
  grab_frame_rate:=60 \
  depth_mode:=PERFORMANCE
```

## 🎓 Learning Resources

### Documentation
- [ZED_INSTALLATION.md](./ZED_INSTALLATION.md) - Complete setup guide
- [ZED_QUICK_REFERENCE.md](./ZED_QUICK_REFERENCE.md) - Command cheatsheet
- [ZED_DOCKER_BUILD.md](./ZED_DOCKER_BUILD.md) - Docker deployment

### External Links
- [ZED SDK Docs](https://www.stereolabs.com/docs/)
- [ZED ROS 2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- [Stereolabs Docker Hub](https://hub.docker.com/r/stereolabs/zed)

## 🐛 Troubleshooting

### Camera Not Detected
```bash
# Check USB
lsusb | grep -i ZED

# Fix permissions
sudo chmod 666 /dev/video*

# Run diagnostic
/usr/local/zed/tools/ZED_Diagnostic
```

### Build Errors
```bash
# Clean cache
docker builder prune -a

# Rebuild
docker compose -f docker-compose.zed.yml build --no-cache
```

### CUDA Issues
```bash
# Test NVIDIA runtime
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi

# Restart Docker
sudo systemctl restart docker
```

### Low Performance
- Reduce resolution: `HD720` or `VGA`
- Disable AI features
- Use `depth_mode:=PERFORMANCE`
- Check `nvidia-smi` for GPU usage

## 📈 Integration with Other Sensors

The ZED camera works alongside other MuSoHu sensors:

| Sensor | Topic | Rate | Integration |
|--------|-------|------|-------------|
| ReSpeaker | `/audio` | 16kHz | Audio-visual fusion |
| LiDAR | `/rslidar_points` | ~10Hz | Point cloud fusion |
| WitMotion IMU | `/witmotion/imu` | 50Hz | IMU fusion (if ZED lacks IMU) |
| ZED Camera | `/zed/zed_node/left/image_rect_color` | 15-60Hz | Primary vision |
| ZED Depth | `/zed/zed_node/point_cloud/cloud_registered` | 15-60Hz | 3D perception |
| ZED IMU | `/zed/zed_node/imu/data` | 400Hz | High-rate inertial |
| ZED Odom | `/zed/zed_node/odom` | 15-60Hz | Visual odometry |

## 🎯 Next Steps

1. **Install ZED SDK** - Follow [ZED_INSTALLATION.md](./ZED_INSTALLATION.md)
2. **Build Docker Image** - See [ZED_DOCKER_BUILD.md](./ZED_DOCKER_BUILD.md)
3. **Configure Camera** - Adjust settings in YAML files
4. **Launch System** - Start all sensors with docker-compose
5. **Visualize** - Access RViz at http://localhost:6080
6. **Develop** - Build your application on top of sensor data

## 📞 Support

For issues specific to:
- **ZED Camera**: [Stereolabs Support](https://support.stereolabs.com/)
- **ROS 2 Wrapper**: [GitHub Issues](https://github.com/stereolabs/zed-ros2-wrapper/issues)
- **MuSoHu Integration**: See project documentation

## 📜 License

ZED SDK requires a license agreement with Stereolabs. See:
- [ZED SDK License](https://www.stereolabs.com/developers/license/)

---

**Status**: ✅ Ready for deployment

**Last Updated**: October 29, 2025

**Tested On**:
- Ubuntu 22.04 with CUDA 12.1
- NVIDIA Jetson (JetPack 6.0, L4T 36.2)
- Docker Engine 24.0+
- ZED SDK 4.1+
