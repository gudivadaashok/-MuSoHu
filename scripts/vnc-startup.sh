#!/bin/bash
# Note: Sensor nodes auto-start via supervisor (see /etc/supervisor/conf.d/ros2-sensors.conf)
# This script is kept for manual diagnostics only

echo "� MuSoHu Multi-Sensor Data Collection Platform"
echo ""
echo "📊 Checking ROS2 Sensor Status..."
echo ""

sudo chown -R $USER:$USER /usr/local/lib/python3.10/dist-packages/

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /tmp/ros2_deps/install/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

# Check supervisor status
echo "� Supervisor Status:"
supervisorctl status

echo ""
echo "📡 ROS2 Topics:"
ros2 topic list | grep -E "audio|rslidar|doa|vad|witmotion" || echo "   ⚠️  No sensor topics found yet"

echo ""
echo "📝 Log Files:"
echo "   🎤 ReSpeaker: /tmp/respeaker.log"
echo "   📡 LiDAR:     /tmp/rslidar.log"
echo "   🧭 IMU:       /tmp/witmotion.log"

echo ""
echo "🌐 Access Points:"
echo "   - VNC:  localhost:5901"
echo "   - Web:  http://localhost:6080"
echo "   - Web Manager: http://localhost:5001"
