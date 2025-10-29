# ZED Camera Quick Reference

## Quick Start Commands

### Launch ZED Camera
```bash
# Using helper script (recommended)
docker exec -d ros2_vnc /usr/local/bin/launch-zed.sh

# Manual launch with specific model
docker exec -it ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
"

# Set camera model via environment variable
docker exec -d ros2_vnc bash -c "
  export ZED_CAMERA_MODEL=zed2i && \
  /usr/local/bin/launch-zed.sh
"
```

### Check Camera Status
```bash
# List ZED topics
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic list | grep zed
"

# Check sensor rates
docker exec ros2_vnc /usr/local/bin/check-sensors.sh

# View logs
docker exec ros2_vnc tail -f /tmp/zed.log
```

### Monitor Data Streams
```bash
# Check camera feed rate
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic hz /zed/zed_node/left/image_rect_color
"

# Check depth data
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic echo /zed/zed_node/depth/depth_registered --once
"

# Check point cloud
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
"
```

## Common Topics

### Image Topics
- `/zed/zed_node/left/image_rect_color` - Left rectified RGB image
- `/zed/zed_node/right/image_rect_color` - Right rectified RGB image
- `/zed/zed_node/left_raw/image_raw_color` - Left raw RGB image
- `/zed/zed_node/right_raw/image_raw_color` - Right raw RGB image

### Depth Topics
- `/zed/zed_node/depth/depth_registered` - Depth map aligned to left camera
- `/zed/zed_node/disparity/disparity_image` - Disparity map
- `/zed/zed_node/confidence/confidence_map` - Depth confidence map

### Point Cloud Topics
- `/zed/zed_node/point_cloud/cloud_registered` - Colored 3D point cloud

### IMU Topics (if available)
- `/zed/zed_node/imu/data` - IMU sensor data
- `/zed/zed_node/imu/data_raw` - Raw IMU data

### Odometry Topics
- `/zed/zed_node/odom` - Visual odometry
- `/zed/zed_node/pose` - Camera pose
- `/zed/zed_node/pose_with_covariance` - Pose with uncertainty
- `/zed/zed_node/path_map` - Trajectory path

### Camera Info
- `/zed/zed_node/left/camera_info` - Left camera calibration
- `/zed/zed_node/right/camera_info` - Right camera calibration

## Launch Parameters

### Camera Selection
```bash
camera_model:=<model>  # zed, zedm, zed2, zed2i, zedx, etc.
camera_name:=zed
node_name:=zed_node
```

### Video Settings
```bash
grab_resolution:=HD720  # HD2K, HD1080, HD720, VGA
grab_frame_rate:=30     # 15, 30, 60, 100 (model dependent)
```

### Depth Settings
```bash
depth_mode:=NEURAL              # NEURAL, ULTRA, QUALITY, PERFORMANCE
depth_confidence:=50            # 0-100, higher = more filtering
depth_texture_conf:=100         # 0-100, texture confidence threshold
min_depth:=0.3                  # Minimum depth in meters
max_depth:=20.0                 # Maximum depth in meters
```

### Tracking Settings
```bash
pos_tracking_enabled:=true
imu_fusion:=true
publish_tf:=true
publish_map_tf:=true
```

### AI Features
```bash
# Object Detection
object_detection.od_enabled:=true
object_detection.model:=MULTI_CLASS_BOX_MEDIUM

# Body Tracking
body_tracking.bt_enabled:=true
body_tracking.model:=HUMAN_BODY_MEDIUM

# Spatial Mapping
mapping.mapping_enabled:=true
mapping.resolution:=0.05
```

## Camera Models

| Model | Description | Resolution | Features |
|-------|-------------|------------|----------|
| `zed` | ZED (1st gen) | 2.2K, 1080p, 720p, VGA | Stereo, Depth |
| `zedm` | ZED Mini | 2.2K, 1080p, 720p, VGA | Stereo, Depth, IMU |
| `zed2` | ZED 2 | 2.2K, 1080p, 720p, VGA | Stereo, Depth, IMU, Neural |
| `zed2i` | ZED 2i | 2.2K, 1080p, 720p, VGA | Stereo, Depth, IMU, Neural |
| `zedx` | ZED X | 1200p | Stereo, Depth, IMU, GMSL2 |
| `zedxm` | ZED X Mini | 1200p | Compact, Depth, IMU |

## Troubleshooting Commands

### Check Hardware
```bash
# Check USB connection
lsusb | grep -i "ZED\|Stereolabs"

# Check video devices
ls -la /dev/video*

# Run ZED diagnostic (on host)
/usr/local/zed/tools/ZED_Diagnostic
```

### Check Software
```bash
# Verify ZED SDK
ls -la /usr/local/zed

# Check CUDA
nvidia-smi
nvcc --version

# Check ROS 2 packages
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 pkg list | grep zed
"
```

### Fix Permissions
```bash
# Fix video device permissions
sudo chmod 666 /dev/video*

# Add user to video group
sudo usermod -aG video $USER
```

### Restart Services
```bash
# Stop and restart container
docker compose down
docker compose up -d

# Restart ZED node
docker exec ros2_vnc pkill -f zed_wrapper
docker exec -d ros2_vnc /usr/local/bin/launch-zed.sh
```

## Performance Optimization

### High Performance Mode
```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  grab_resolution:=VGA \
  grab_frame_rate:=60 \
  depth_mode:=PERFORMANCE \
  depth_confidence:=80
```

### High Quality Mode
```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  grab_resolution:=HD1080 \
  grab_frame_rate:=15 \
  depth_mode:=NEURAL \
  depth_confidence:=50
```

### Balanced Mode (Default)
```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  grab_resolution:=HD720 \
  grab_frame_rate:=30 \
  depth_mode:=QUALITY
```

## RViz Visualization

Access RViz at: http://localhost:6080

### View Camera Feed
- Add → Image → Topic: `/zed/zed_node/left/image_rect_color`

### View Depth Map
- Add → Image → Topic: `/zed/zed_node/depth/depth_registered`

### View Point Cloud
- Add → PointCloud2 → Topic: `/zed/zed_node/point_cloud/cloud_registered`
- Color Transformer: RGB8

### View Odometry
- Add → Odometry → Topic: `/zed/zed_node/odom`
- Add → Path → Topic: `/zed/zed_node/path_map`

## Integration with Other Sensors

### TF Frames
The ZED camera publishes TF transforms:
- `base_link` → `zed_camera_center`
- `zed_camera_center` → `zed_left_camera_frame`
- `zed_camera_center` → `zed_right_camera_frame`

### Sensor Fusion
Combine ZED with other MuSoHu sensors:
- LiDAR (`/rslidar_points`) + ZED Point Cloud
- WitMotion IMU + ZED IMU (if available)
- ReSpeaker Audio + ZED Visual for AV fusion

## Resources

- 📖 [Full Installation Guide](./ZED_INSTALLATION.md)
- 🌐 [ZED SDK Documentation](https://www.stereolabs.com/docs/)
- 💻 [ZED ROS 2 Wrapper GitHub](https://github.com/stereolabs/zed-ros2-wrapper)
- 🎓 [ZED Tutorials](https://www.stereolabs.com/docs/tutorials/)
