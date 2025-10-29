# MuSoHu

Multi-Modal Social Human Navigation Dataset - A robotics research platform developed at George Mason University.

## Overview

MuSoHu is a platform that allows to collect data for the social navigation research project. It is built using ROS2 and provides a web interface for users to interact with the system.

## Quick Start

For detailed setup and usage instructions, see our comprehensive guides:

- **🐳 [Docker Setup Guide](docs/DOCKER_GUIDE.md)** - Complete Docker containerization setup
- **📖 [Docker Setup Documentation](docs/docker-setup.md)** - Detailed Docker configuration and troubleshooting

[➡️ Read the full Docker Guide here](docs/DOCKER_GUIDE.md)

## Getting Started

### Prerequisites
- Docker Desktop
- Python 3.8+
- ROS2 Humble
- Web browser

### Quick Launch
```bash
# Clone the repository
git clone https://github.com/gudivadaashok/-MuSoHu.git
cd -MuSoHu

# Start with Docker (recommended)
./scripts/start.sh

# Or manually with Docker Compose
docker-compose up -d
```

### Access Applications
- **ROS2 VNC Desktop**: http://localhost:6080 (password: `ubuntu`, hostname: `robotixx`)
- **Web Interface**: http://localhost:5001 (hostname: `robotixx-webapp`)

### Helper Scripts

**Check System Status:**
```bash
docker exec ros2_vnc /usr/local/bin/vnc-startup.sh
```
Shows supervisor status, active topics, and access points.

**Check Sensor Health:**
```bash
docker exec ros2_vnc /usr/local/bin/check-sensors.sh
```
Displays real-time sensor publish rates for all 3 sensors.

**View Sensor Logs:**
```bash
# ReSpeaker
docker exec ros2_vnc tail -f /tmp/respeaker.log

# LiDAR
docker exec ros2_vnc tail -f /tmp/rslidar.log

# IMU
docker exec ros2_vnc tail -f /tmp/witmotion.log
```

**Visualize Sensors in RViz2:**
```bash
# Launch RViz2 with pre-configured sensor visualization
docker exec -d ros2_vnc /usr/local/bin/launch-rviz.sh

# Then access via VNC: http://localhost:6080
# RViz2 will show:
#   - LiDAR point cloud (/rslidar_points)
#   - IMU orientation axes
#   - TF transforms
```

### Network Configuration

**Container Network**: `ros2_network` (bridge mode)

**Port Forwarding**:
- `6080:80` - noVNC web interface
- `5901:5901` - VNC server (direct access)
- `6699:6699/udp` - RoboSense LiDAR MSOP (Main Stream Output Protocol)
- `7788:7788/udp` - RoboSense LiDAR DIFOP (Device Information Output Protocol)
- `5001:5001` - Web Application

**Note**: The ROS2 container uses port forwarding instead of host networking to maintain VNC compatibility while allowing LiDAR UDP packet reception.

### Testing Hardware Devices

#### ReSpeaker 4 Mic Array
To verify that the ReSpeaker microphone array is accessible in the container:
```bash
docker exec -it ros2_vnc arecord -l
```

Expected output should show:
```
card 0: ArrayUAC10 [ReSpeaker 4 Mic Array (UAC1.0)], device 0: USB Audio [USB Audio]
```

To launch the ReSpeaker ROS2 node (after the container is built with audio drivers):
```bash
docker exec -it ros2_vnc bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 launch respeaker_ros2 respeaker.launch.py
```

To test audio recording:
```bash
docker exec -it ros2_vnc arecord -D plughw:0,0 -f S16_LE -r 16000 -c 4 test.wav
```

#### RoboSense LiDAR

**Device Information:**
- **Model**: Helios 32
- **Serial Number**: 1006BEBD0417
- **MAC Address**: 08:48:57:03:5A:19
- **IP Address**: 192.168.1.200
- **Destination IP**: 192.168.1.102 (Jetson)
- **MSOP Port**: 6699
- **DIFOP Port**: 7788
- **Firmware Versions**:
  - Top Board: 01050a00
  - Bottom Board: 01031600
  - Software: 24061301
  - Motor: 24041621
- **Web Interface**: http://192.168.1.200
- **Configuration**: RSHELIOS (configured in Dockerfile.ros2)

**Status**: ✅ Fully operational, publishing at ~9.7 Hz on `/rslidar_points`

To check if LiDAR is sending packets:
```bash
docker exec -it ros2_vnc bash -c "apt-get update && apt-get install -y tcpdump && tcpdump -i enP8p1s0 port 6699 -c 10"
```

To check ROS2 topics:
```bash
docker exec -it ros2_vnc bash -c "source /home/ubuntu/ros2_ws/install/setup.bash && ros2 topic list"
```

To launch the RoboSense LiDAR node:
```bash
docker exec -it ros2_vnc bash -c "source /home/ubuntu/ros2_ws/install/setup.bash && ros2 run rslidar_sdk rslidar_sdk_node"
```

Or start in background:
```bash
docker exec -d ros2_vnc bash -c "source /home/ubuntu/ros2_ws/install/setup.bash && ros2 run rslidar_sdk rslidar_sdk_node > /tmp/rslidar.log 2>&1"
```

To check LiDAR data rate:
```bash
docker exec -it ros2_vnc bash -c "source /home/ubuntu/ros2_ws/install/setup.bash && ros2 topic hz /rslidar_points"
```

To visualize LiDAR in RViz2:
```bash
# Access VNC desktop at http://localhost:80 (password: ubuntu)
# Then in a VNC terminal:
source /home/ubuntu/ros2_ws/install/setup.bash
rviz2 -d /home/ubuntu/.rviz2/lidar.rviz
```

#### Witmotion IMU (3-Axis Sensor)

**Device Information:**
- **Driver**: [witmotion_ros2](https://github.com/ioio2995/witmotion_ros2)
- **Serial Port**: `/dev/ttyUSB0` or `/dev/ttyUSB1` (check with `ls /dev/ttyUSB*`)
- **Baud Rate**: 115200
- **Update Rate**: 50 Hz
- **Frame ID**: `base_link`

**Published Topics:**
- `/witmotion_node/imu` - IMU data (sensor_msgs/Imu)
- `/witmotion_node/imu_temperature` - Temperature
- `/witmotion_node/magnetometer` - Magnetic field
- `/witmotion_node/orientation` - Orientation (Quaternion)
- Additional topics for GPS, barometer, altitude (if supported by sensor)

To launch the Witmotion IMU node:
```bash
docker exec -it ros2_vnc bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 launch witmotion_ros2 witmotion_launch.py
```

Or run directly with custom parameters:
```bash
ros2 run witmotion_ros2 witmotion_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baud_rate:=115200 \
  -p update_rate:=50.0 \
  -p frame_id:=base_link \
  -p topic_name:=/witmotion
```

To check IMU data:
```bash
docker exec -it ros2_vnc bash -c "source /home/ubuntu/ros2_ws/install/setup.bash && ros2 topic echo /witmotion_node/imu"
```

### Troubleshooting: CH340 Driver Installation (Jetson)

If the IMU is connected but `/dev/ttyUSB0` doesn't appear, the CH340/CH341 USB-to-serial driver may be missing from the Jetson kernel.

**Symptoms:**
- `lsusb` shows device `1a86:7523 QinHeng Electronics CH340 serial converter`
- No `/dev/ttyUSB*` devices available
- `modprobe ch341` returns "module not found"

**Solution - Option 2: Compile driver from source**

```bash
# Clone the driver repository
git clone https://github.com/juliagoda/CH341SER.git /tmp/CH341SER
cd /tmp/CH341SER

# Compile the driver
make

# Load the driver into the kernel
sudo make load

# Optional: Install permanently (survives reboots)
sudo make install
```

**Note:** The `brltty` (braille display) service may claim the CH340 device. Disable it:
```bash
sudo systemctl stop brltty.service
sudo systemctl disable brltty.service
sudo killall brltty
```

After loading the driver, unbind/rebind the USB device to create `/dev/ttyUSB0`:
```bash
# Find USB port (usually 1-2.3 for device 006)
lsusb  # Note the Bus and Device number

# Unbind and rebind (replace 1-2.3 with your port)
echo "1-2.3" | sudo tee /sys/bus/usb/drivers/usb/unbind
echo "1-2.3" | sudo tee /sys/bus/usb/drivers/usb/bind

# Verify device appeared
ls -la /dev/ttyUSB0
```

Once `/dev/ttyUSB0` exists, restart the Docker container to pass the device:
```bash
docker compose down
docker compose up -d
```


## Additional Documentation

## GitHub Repositories for each Driver
- [witmotion_ros2](https://github.com/ioio2995/witmotion_ros2.git)
https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/sdk/ros-python-introduction
- [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper.git)
- [respeaker_ros2](https://github.com/hcrlab/respeaker_ros.git)
- [LiDAR Robosense H32F70](https://github.com/RoboSense-LiDAR/rslidar_sdk.git)



