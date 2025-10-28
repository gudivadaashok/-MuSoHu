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

echo "✅ ROS2 VNC Desktop is ready!"
echo "   - VNC: localhost:5901"
echo "   - Web: http://localhost:6080"
echo "   - Password: ubuntu"
echo "   - Hostname: robotixx"

# Keep container running
tail -f /dev/null
