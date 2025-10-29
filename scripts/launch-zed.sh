#!/bin/bash

###############################################################################
# ZED Camera Launch Script
# Starts the Stereolabs ZED camera ROS2 node with optimized settings
###############################################################################

set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

# Set display for any GUI needs
export DISPLAY=:1

# Default camera model (can be overridden with environment variable)
CAMERA_MODEL=${ZED_CAMERA_MODEL:-"zed2i"}

echo "=================================================="
echo "Starting ZED Camera Node"
echo "=================================================="
echo "Camera Model: ${CAMERA_MODEL}"
echo "Timestamp: $(date)"
echo "=================================================="

# Check if ZED SDK is installed
if [ ! -d "/usr/local/zed" ]; then
    echo "ERROR: ZED SDK not found at /usr/local/zed"
    echo "Please install ZED SDK before running this script"
    exit 1
fi

# Check if ZED camera is connected
if ! lsusb | grep -qi "ZED\|Stereolabs"; then
    echo "WARNING: ZED camera not detected via USB"
    echo "Please ensure camera is properly connected"
fi

# Launch ZED camera node
echo "Launching ZED camera node..."
ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=${CAMERA_MODEL} \
    camera_name:=zed \
    node_name:=zed_node \
    publish_tf:=true \
    publish_map_tf:=true \
    depth_mode:=NEURAL \
    depth_confidence:=50 \
    depth_texture_conf:=100 \
    grab_resolution:=HD720 \
    grab_frame_rate:=30 \
    pos_tracking_enabled:=true \
    imu_fusion:=true \
    verbose:=false \
    2>&1 | tee /tmp/zed.log

echo "ZED camera node stopped"
