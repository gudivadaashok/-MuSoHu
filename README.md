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


## Additional Documentation

## GitHub Repositories for each Driver
- [witmotion_ros2](https://github.com/ioio2995/witmotion_ros2.git)
https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/sdk/ros-python-introduction
- [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper.git)
- [respeaker_ros2](https://github.com/hcrlab/respeaker_ros.git)
- [LiDAR Robosense H32F70](https://github.com/RoboSense-LiDAR/rslidar_sdk.git)



