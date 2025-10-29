#!/bin/bash

###############################################################################
# MuSoHu All Sensors Launcher
# Starts all four sensors: ReSpeaker, LiDAR, WitMotion IMU, and ZED Camera
###############################################################################

set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /tmp/ros2_deps/install/setup.bash 2>/dev/null || true
source /home/ubuntu/ros2_ws/install/setup.bash

# Set display
export DISPLAY=:1

# Configuration
ZED_CAMERA_MODEL=${ZED_CAMERA_MODEL:-"zed2i"}
LOG_DIR="/tmp"

echo "=========================================="
echo "  MuSoHu Multi-Sensor System Startup"
echo "=========================================="
echo "Starting all sensors..."
echo "Timestamp: $(date)"
echo "=========================================="

# Function to start a sensor in background
start_sensor() {
    local sensor_name=$1
    local command=$2
    local log_file=$3
    
    echo "Starting $sensor_name..."
    eval "$command > $log_file 2>&1 &"
    sleep 2
}

# 1. Start ReSpeaker Audio
echo ""
echo "[1/4] Starting ReSpeaker Audio Node..."
start_sensor "ReSpeaker" \
    "ros2 launch respeaker_ros respeaker.launch.py" \
    "$LOG_DIR/respeaker.log"

# 2. Start RoboSense LiDAR
echo "[2/4] Starting RoboSense LiDAR Node..."
start_sensor "LiDAR" \
    "ros2 run rslidar_sdk rslidar_sdk_node" \
    "$LOG_DIR/rslidar.log"

# 3. Start WitMotion IMU
echo "[3/4] Starting WitMotion IMU Node..."
if [ -e /dev/ttyUSB0 ]; then
    start_sensor "WitMotion IMU" \
        "ros2 run witmotion_ros2 witmotion_node --ros-args -p port:=/dev/ttyUSB0" \
        "$LOG_DIR/witmotion.log"
else
    echo "⚠️  WARNING: /dev/ttyUSB0 not found, skipping WitMotion IMU"
fi

# 4. Start ZED Camera (if available)
echo "[4/4] Starting ZED Camera Node..."
if [ -d "/usr/local/zed" ]; then
    if lsusb | grep -qi "ZED\|Stereolabs"; then
        start_sensor "ZED Camera" \
            "ros2 launch zed_wrapper zed_camera.launch.py camera_model:=$ZED_CAMERA_MODEL" \
            "$LOG_DIR/zed.log"
    else
        echo "⚠️  WARNING: ZED camera not detected via USB"
        echo "    Tip: Check USB connection and run: lsusb | grep -i ZED"
    fi
else
    echo "⚠️  WARNING: ZED SDK not found at /usr/local/zed"
    echo "    Tip: Install ZED SDK or mount it as a volume"
fi

echo ""
echo "=========================================="
echo "  All Available Sensors Started!"
echo "=========================================="
echo ""
echo "📊 View sensor status:"
echo "    /usr/local/bin/check-sensors.sh"
echo ""
echo "📝 View logs:"
echo "    tail -f $LOG_DIR/respeaker.log"
echo "    tail -f $LOG_DIR/rslidar.log"
echo "    tail -f $LOG_DIR/witmotion.log"
echo "    tail -f $LOG_DIR/zed.log"
echo ""
echo "🔍 Check topics:"
echo "    ros2 topic list"
echo ""
echo "🎯 Launch RViz:"
echo "    /usr/local/bin/launch-rviz.sh"
echo ""
echo "=========================================="

# Keep script running to monitor sensors
sleep 5

# Quick health check
echo ""
echo "🏥 Quick Health Check (3 seconds each)..."
echo ""

echo "🎤 ReSpeaker Audio:"
timeout 3 ros2 topic hz /audio 2>&1 | grep "average rate" | head -1 || echo "   ⏳ Waiting for data..."

echo ""
echo "📡 LiDAR:"
timeout 3 ros2 topic hz /rslidar_points 2>&1 | grep "average rate" | head -1 || echo "   ⏳ Waiting for data..."

echo ""
echo "🧭 IMU:"
timeout 3 ros2 topic hz /witmotion/imu 2>&1 | grep "average rate" | head -1 || echo "   ⏳ Waiting for data..."

echo ""
echo "📷 ZED Camera:"
timeout 3 ros2 topic hz /zed/zed_node/left/image_rect_color 2>&1 | grep "average rate" | head -1 || echo "   ⏳ Waiting for data..."

echo ""
echo "=========================================="
echo "✅ Startup complete!"
echo "=========================================="
