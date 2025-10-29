#!/bin/bash
# Quick sensor health check script

echo "🔍 MuSoHu Sensor Health Check"
echo "=============================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source /tmp/ros2_deps/install/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

echo "📊 Active ROS2 Topics:"
ros2 topic list | grep -E "rslidar|audio|witmotion|zed"

echo ""
echo "📈 Sensor Rates (sampling for 3 seconds each)..."
echo ""

echo "🎤 ReSpeaker Audio:"
timeout 3 ros2 topic hz /audio 2>&1 | grep "average rate" | head -1 || echo "   ❌ Not publishing"

echo ""
echo "📡 LiDAR Point Cloud:"
timeout 3 ros2 topic hz /rslidar_points 2>&1 | grep "average rate" | head -1 || echo "   ❌ Not publishing"

echo ""
echo "🧭 IMU Data:"
timeout 3 ros2 topic hz /witmotion/imu 2>&1 | grep "average rate" | head -1 || echo "   ❌ Not publishing"

echo ""
echo "📷 ZED Camera (Left Image):"
timeout 3 ros2 topic hz /zed/zed_node/left/image_rect_color 2>&1 | grep "average rate" | head -1 || echo "   ❌ Not publishing"

echo ""
echo "🗺️ ZED Point Cloud:"
timeout 3 ros2 topic hz /zed/zed_node/point_cloud/cloud_registered 2>&1 | grep "average rate" | head -1 || echo "   ❌ Not publishing"

echo ""
echo "✅ Sensor check complete!"
