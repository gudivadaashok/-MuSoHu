#!/bin/bash
# Launch RViz2 with MuSoHu sensor visualization

echo "🎨 Launching RViz2 with sensor visualization..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /tmp/ros2_deps/install/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

# Set display
export DISPLAY=:1

# Launch RViz2 with config
rviz2 -d /usr/local/share/musohu_sensors.rviz
