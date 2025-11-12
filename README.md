# MuSoHu: Multi-Modal Social Human Navigation (v2)

Multi-Modal Social Human Navigation Dataset - A robotics research platform developed at George Mason University for collecting data on human-robot interactions in social navigation contexts.

**Project Website:** [https://cs.gmu.edu/~xiao/Research/MuSoHu/](https://cs.gmu.edu/~xiao/Research/MuSoHu/)

## About

Humans are well-adept at navigating public spaces shared with others, where current autonomous mobile robots still struggle. While safely and efficiently reaching their goals, humans communicate their intentions and conform to unwritten social norms on a daily basis; conversely, robots become clumsy in those daily social scenarios, getting stuck in dense crowds, surprising nearby pedestrians, or even causing collisions.

This project utilizes rich, widely available social human navigation data in natural human-inhabited public spaces for robots to learn similar, human-like, socially compliant navigation behaviors. We provide an open-source egocentric data collection sensor suite wearable by walking humans to capture multi-modal robot perception data.

### Dataset Scale
- **~100 km** of navigation data
- **20 hours** of recording
- **300 trials** across various environments
- **13 human** participants
- Multiple public spaces with natural social navigation interactions

## Multi-Modal Sensor Suite

Our wearable data collection device provides multi-modal perceptual streams commonly available on mobile robot platforms:

- **3D LiDAR** - Robosense Helios 32 (360° environmental scanning)
- **Stereo & Depth Camera** - Stereolabs ZED 2i (visual perception and depth sensing)
- **IMU** - Inertial Measurement Unit (motion tracking)
- **Odometry/Actions** - Online processing for human navigation behavior extraction
- **Microphone Array** - Seeed Studio ReSpeaker Mic Array (spatial audio capture)

## Documentation

For complete documentation, setup instructions, and production deployment guides, see:

- **[Production Web Service Setup](docs/PRODUCTION_WEB_SERVICE.md)** - Complete guide for deploying the web application
- **[Production Setup Details](docs/PRODUCTION_SETUP.md)** - Detailed systemd configuration and monitoring

## Quick Overview

MuSoHu is a platform that allows to collect data for the social navigation research project. It is built using ROS2 and provides a web interface for users to interact with the system.

### Key Features
- ROS2-based robotics platform
- Multi-sensor integration (ZED camera, LiDAR, IMU, Audio)
- Web interface for system control and monitoring
- Production-ready deployment with automatic crash recovery
- Helmet-mounted sensor suite for data collection

### Project Structure
- `helmet_bringup/` - ROS2 launch files and sensor configurations
- `web-app/` - Flask web interface for system control
- `scripts/` - Setup, installation, and utility scripts
  - `setup/` - Installation and configuration scripts
  - `utils/` - Helper utilities and logging
  - `udev_rules/` - Device access rules for sensors
  - `hotspot/` - WiFi hotspot configuration
- `docs/` - Complete documentation and guides