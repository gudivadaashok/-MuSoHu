#!/bin/bash
# Launch RViz2 with MuSoHu sensor visualization

echo "🎨 Launching RViz2 with sensor visualization..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /tmp/ros2_deps/install/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

# Set display and X11 auth
export DISPLAY=:1
export XAUTHORITY=/home/ubuntu/.Xauthority

# Wait a moment for X server
sleep 2

# Launch RViz2 with config as ubuntu user
su - ubuntu -c "export DISPLAY=:1 && source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && rviz2 -d /usr/local/share/musohu_sensors.rviz"
