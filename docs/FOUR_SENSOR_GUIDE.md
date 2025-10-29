# MuSoHu Four-Sensor System Guide

## Complete Sensor Suite

This guide covers the operation of all four sensors in the MuSoHu system:

1. **ReSpeaker 4-Mic Array** - Audio capture and processing
2. **RoboSense Helios-32 LiDAR** - 3D point cloud scanning
3. **WitMotion IMU** - Inertial measurement (accelerometer, gyroscope, magnetometer)
4. **Stereolabs ZED Camera** - Stereo vision, depth, and tracking

## Quick Start - All Sensors

### 1. Build the System

```bash
# Build Docker image with all sensor drivers
cd /home/jetson/MuSoHu
docker compose build ros2_vnc

# Start containers
docker compose up -d
```

### 2. Launch All Sensors

```bash
# Use the all-in-one launcher
docker exec -d ros2_vnc /usr/local/bin/launch-all-sensors.sh

# Or start individually with supervisor
docker exec ros2_vnc supervisorctl start all
```

### 3. Verify All Sensors

```bash
# Check sensor health
docker exec ros2_vnc /usr/local/bin/check-sensors.sh

# View active topics
docker exec ros2_vnc bash -c "source /home/ubuntu/ros2_ws/install/setup.bash && ros2 topic list"
```

### 4. Visualize in RViz

```bash
# Launch RViz with all sensor visualizations
docker exec -d ros2_vnc /usr/local/bin/launch-rviz.sh

# Access via browser
firefox http://localhost:6080
```

## Sensor Overview

### 1. ReSpeaker 4-Mic Array 🎤

**Topics Published:**
- `/audio` - Raw audio stream (16kHz, 4 channels)
- `/sound_direction` - Direction of arrival (DOA)
- `/is_speeching` - Voice activity detection

**Launch Command:**
```bash
docker exec -d ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 launch respeaker_ros respeaker.launch.py > /tmp/respeaker.log 2>&1
"
```

**Check Status:**
```bash
# Verify audio device
docker exec ros2_vnc arecord -l

# Check topic rate
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic hz /audio
"

# View logs
docker exec ros2_vnc tail -f /tmp/respeaker.log
```

**Specifications:**
- Channels: 4 microphones
- Sample Rate: 16kHz
- Format: S16_LE (16-bit signed)
- Features: DOA, VAD, beamforming

### 2. RoboSense Helios-32 LiDAR 📡

**Topics Published:**
- `/rslidar_points` - 3D point cloud (sensor_msgs/PointCloud2)

**Launch Command:**
```bash
docker exec -d ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 run rslidar_sdk rslidar_sdk_node > /tmp/rslidar.log 2>&1
"
```

**Check Status:**
```bash
# Check topic rate
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic hz /rslidar_points
"

# View point cloud info
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic echo /rslidar_points --once | head -30
"

# View logs
docker exec ros2_vnc tail -f /tmp/rslidar.log
```

**Specifications:**
- Model: Helios (32 channels)
- Range: 0.2m - 200m
- Frame Rate: ~10 Hz
- Resolution: 1800 x 32 points per frame
- IP: 192.168.1.200 → 192.168.1.102
- Ports: 6699 (MSOP), 7788 (DIFOP)

### 3. WitMotion IMU 🧭

**Topics Published:**
- `/witmotion/imu` - IMU data (sensor_msgs/Imu)
- `/witmotion/temperature` - Temperature readings
- `/witmotion/magnetometer` - Magnetic field
- `/witmotion/orientation` - Orientation (Quaternion)
- `/witmotion/cali` - Calibration status

**Launch Command:**
```bash
docker exec -d ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 run witmotion_ros2 witmotion_node --ros-args -p port:=/dev/ttyUSB0 > /tmp/witmotion.log 2>&1
"
```

**Check Status:**
```bash
# Check device
docker exec ros2_vnc ls -la /dev/ttyUSB*

# Check topic rate
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic hz /witmotion/imu
"

# View IMU data
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic echo /witmotion/imu --once
"

# View logs
docker exec ros2_vnc tail -f /tmp/witmotion.log
```

**Specifications:**
- Axes: 3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer
- Update Rate: 50 Hz
- Port: /dev/ttyUSB0
- Baud Rate: 115200
- Frame ID: base_link

### 4. Stereolabs ZED Camera 📷

**Topics Published:**
- `/zed/zed_node/left/image_rect_color` - Left RGB image
- `/zed/zed_node/right/image_rect_color` - Right RGB image
- `/zed/zed_node/depth/depth_registered` - Depth map
- `/zed/zed_node/point_cloud/cloud_registered` - RGB point cloud
- `/zed/zed_node/imu/data` - IMU data (if available)
- `/zed/zed_node/odom` - Visual odometry
- `/zed/zed_node/pose` - Camera pose

**Launch Command:**
```bash
docker exec -d ros2_vnc /usr/local/bin/launch-zed.sh

# Or manually:
docker exec -d ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i > /tmp/zed.log 2>&1
"
```

**Check Status:**
```bash
# Check camera detection
docker exec ros2_vnc lsusb | grep -i "ZED\|Stereolabs"

# Check topics
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic list | grep zed
"

# Check image rate
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic hz /zed/zed_node/left/image_rect_color
"

# View logs
docker exec ros2_vnc tail -f /tmp/zed.log
```

**Specifications:**
- Model: ZED 2i (configurable)
- Resolution: Up to 2.2K (configurable)
- Frame Rate: 15-60 FPS (configurable)
- Depth Range: 0.3m - 20m
- Features: Stereo, depth, tracking, odometry, AI

## All-In-One Commands

### Check All Sensors At Once

```bash
docker exec ros2_vnc /usr/local/bin/check-sensors.sh
```

Expected output:
```
🔍 MuSoHu Sensor Health Check
==============================

📊 Active ROS2 Topics:
/audio
/rslidar_points
/witmotion/imu
/zed/zed_node/left/image_rect_color
...

📈 Sensor Rates (sampling for 3 seconds each)...

🎤 ReSpeaker Audio:
   average rate: 16.000

📡 LiDAR Point Cloud:
   average rate: 9.700

🧭 IMU Data:
   average rate: 50.000

📷 ZED Camera (Left Image):
   average rate: 30.000

✅ Sensor check complete!
```

### View All Topics

```bash
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 topic list | sort
"
```

### View All Nodes

```bash
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 node list
"
```

Expected nodes:
- `/respeaker_node`
- `/rslidar_sdk_node`
- `/witmotion_node`
- `/zed/zed_node`

### View TF Frames

```bash
docker exec ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 run tf2_tools view_frames
"
```

## Sensor Integration & Data Fusion

### Synchronized Data Collection

Create a ROS 2 node that subscribes to all sensors:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu, Image
from audio_common_msgs.msg import AudioStamped

class MultiSensorNode(Node):
    def __init__(self):
        super().__init__('multi_sensor_node')
        
        # Subscribe to all sensors
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/rslidar_points', self.lidar_callback, 10)
        
        self.imu_sub = self.create_subscription(
            Imu, '/witmotion/imu', self.imu_callback, 10)
        
        self.image_sub = self.create_subscription(
            Image, '/zed/zed_node/left/image_rect_color', self.image_callback, 10)
        
        self.audio_sub = self.create_subscription(
            AudioStamped, '/audio', self.audio_callback, 10)
    
    def lidar_callback(self, msg):
        self.get_logger().info(f'LiDAR: {msg.width}x{msg.height} points')
    
    def imu_callback(self, msg):
        self.get_logger().info(f'IMU: {msg.linear_acceleration}')
    
    def image_callback(self, msg):
        self.get_logger().info(f'Image: {msg.width}x{msg.height}')
    
    def audio_callback(self, msg):
        self.get_logger().info(f'Audio: {len(msg.audio.data)} bytes')
```

### Data Rates & Bandwidth

| Sensor | Topic | Rate | Bandwidth | Data Type |
|--------|-------|------|-----------|-----------|
| ReSpeaker | `/audio` | 16kHz | ~128KB/s | Audio (4ch, 16-bit) |
| LiDAR | `/rslidar_points` | ~10Hz | ~5MB/s | PointCloud2 (1800x32) |
| WitMotion | `/witmotion/imu` | 50Hz | ~5KB/s | IMU (9-axis) |
| ZED Left | `/zed/.../left/image_rect_color` | 30Hz | ~40MB/s | Image (HD720 RGB) |
| ZED Depth | `/zed/.../depth/depth_registered` | 30Hz | ~20MB/s | Image (HD720 float) |
| ZED Cloud | `/zed/.../point_cloud/cloud_registered` | 30Hz | ~50MB/s | PointCloud2 (RGB) |

**Total Bandwidth**: ~115MB/s (with all sensors at full rate)

## Troubleshooting

### Sensor Not Detected

```bash
# ReSpeaker
docker exec ros2_vnc arecord -l

# LiDAR (check network)
docker exec ros2_vnc ss -ulnp | grep -E '6699|7788'

# IMU (check USB)
docker exec ros2_vnc ls -la /dev/ttyUSB*

# ZED (check USB)
docker exec ros2_vnc lsusb | grep -i ZED
```

### Restart Individual Sensor

```bash
# Using supervisor
docker exec ros2_vnc supervisorctl restart respeaker
docker exec ros2_vnc supervisorctl restart rslidar
docker exec ros2_vnc supervisorctl restart witmotion

# Manually
docker exec ros2_vnc pkill -f respeaker
docker exec -d ros2_vnc bash -c "
  source /home/ubuntu/ros2_ws/install/setup.bash && \
  ros2 launch respeaker_ros respeaker.launch.py > /tmp/respeaker.log 2>&1
"
```

### Check Supervisor Status

```bash
docker exec ros2_vnc supervisorctl status
```

### View All Logs

```bash
# Tail all sensor logs
docker exec ros2_vnc bash -c "
  tail -f /tmp/respeaker.log /tmp/rslidar.log /tmp/witmotion.log /tmp/zed.log
"
```

## Performance Optimization

### Reduce Bandwidth

**Lower Camera Resolution:**
```bash
# In launch command, use VGA instead of HD720
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  grab_resolution:=VGA \
  grab_frame_rate:=30
```

**Reduce LiDAR Rate:**
- Configure in `rslidar_sdk/config/config.yaml`
- Or use topic throttling

**Downsample Audio:**
- Configure in `respeaker_ros/config/respeaker.yaml`

### CPU/GPU Usage

```bash
# Monitor resources
docker stats ros2_vnc

# Inside container
docker exec ros2_vnc htop

# GPU usage (if available)
nvidia-smi
```

## Next Steps

- [Sensor Calibration](./CALIBRATION.md)
- [Data Recording](./RECORDING.md)
- [Sensor Fusion](./FUSION.md)
- [ZED Camera Details](./ZED_QUICK_REFERENCE.md)

## Summary

All four sensors working together provide:
- **360° awareness**: LiDAR for obstacles, ZED for visual context
- **Precise localization**: IMU + ZED odometry
- **Audio perception**: Direction and voice detection
- **Rich data**: 3D point clouds with RGB, audio, inertial

**Total System:**
- 4 sensors
- 15+ ROS 2 topics
- ~115MB/s data rate
- Real-time processing capability
