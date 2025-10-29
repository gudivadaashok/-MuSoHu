#!/bin/bash

echo "🚀 Starting ROS2 VNC Desktop Environment"

# Start D-Bus
sudo rm -rf /var/run/dbus/pid
sudo mkdir -p /var/run/dbus
sudo dbus-daemon --system --fork

# Kill any existing VNC servers
vncserver -kill :1 > /dev/null 2>&1 || true

# Start VNC server
echo "🖥️  Starting VNC server on display :1"
vncserver :1 -geometry 1920x1080 -depth 24 -localhost no

# Start noVNC
echo "🌐 Starting noVNC web server on port 80"
/usr/share/novnc/utils/novnc_proxy --vnc localhost:5901 --listen 80 &

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /tmp/ros2_deps/install/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

# Start ReSpeaker audio node
echo "🎤 Starting ReSpeaker audio node..."
ros2 run respeaker_ros respeaker_node > /tmp/respeaker.log 2>&1 &

# Start RoboSense LiDAR driver
echo "📡 Starting RoboSense Helios 32 LiDAR driver..."
ros2 run rslidar_sdk rslidar_sdk_node > /tmp/rslidar.log 2>&1 &

# Start second LiDAR instance for dual output (if needed)
sleep 2
ros2 run rslidar_sdk rslidar_sdk_node > /tmp/lidar_output.log 2>&1 &

echo "✅ ROS2 VNC Desktop is ready!"
echo "   - VNC: localhost:5901"
echo "   - Web: http://localhost:6080"
echo "   - Password: ubuntu"
echo "   - Hostname: robotixx"
echo ""
echo "🤖 Hardware Status:"
echo "   🎤 ReSpeaker: Check /tmp/respeaker.log"
echo "   📡 LiDAR: Check /tmp/rslidar.log"
echo ""
echo "📊 ROS2 Topics:"
ros2 topic list | grep -E "audio|rslidar|doa|vad" || echo "   Waiting for topics..."

# Keep container running
tail -f /dev/null
