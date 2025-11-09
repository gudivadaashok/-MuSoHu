#!/bin/bash

#***********************************************************************
# Download Driver Script for MuSoHu
#***********************************************************************


# Source logging configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../utils/logging_config.sh"
log_info "Starting driver download process..."


# Create workspace directory if it doesn't exist for ROS2_MuSoHu
WORKSPACE_DIR="$HOME/ROS2_MuSoHu/src"
if [ ! -d "$WORKSPACE_DIR" ]; then
    log_info "Creating workspace directory at $WORKSPACE_DIR"
    mkdir -p "$WORKSPACE_DIR"
else
    log_info "Workspace directory already exists at $WORKSPACE_DIR"
fi

#***********************************************************************
# audio_common for ROS2
#***********************************************************************

log_info "Cloning audio_common package for ROS2..."
cd "$WORKSPACE_DIR"

git clone -b ros2 https://github.com/ros-drivers/audio_common.git
if [ $? -eq 0 ]; then
    log_success "audio_common package cloned successfully"
else
    log_error "Failed to clone audio_common package"
    exit 1
fi


#***********************************************************************
# Clone respeaker_ros
#***********************************************************************


log_info "Cloning respeaker_ros package..."
cd "$WORKSPACE_DIR"

git clone https://github.com/hcrlab/respeaker_ros.git
if [ $? -eq 0 ]; then
    log_success "respeaker_ros package cloned successfully"
else
    log_error "Failed to clone respeaker_ros package"
    exit 1
fi

#***********************************************************************
# RoboSense LiDAR SDK and message definitions
#***********************************************************************

log_info "Cloning RoboSense LiDAR SDK and message definitions..."
cd "$WORKSPACE_DIR"
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git && \
    cd rslidar_sdk && \
    git submodule init && \
    git submodule update
if [ $? -eq 0 ]; then
    log_success "RoboSense LiDAR SDK cloned and submodules updated successfully"
else
    log_error "Failed to clone RoboSense LiDAR SDK or update submodules"
    exit 1
fi


log_info "Cloning rslidar_msgs package..."
cd "$WORKSPACE_DIR"
git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
if [ $? -eq 0 ]; then
    log_success "rslidar_msgs package cloned successfully"
else
    log_error "Failed to clone rslidar_msgs package"
    exit 1
fi

#***********************************************************************
# Witmotion IMU ROS2 driver
#***********************************************************************

log_info "Cloning Witmotion IMU ROS2 driver..."
cd "$WORKSPACE_DIR"
git clone https://github.com/ioio2995/witmotion_ros2.git
if [ $? -eq 0 ]; then
    log_success "Witmotion IMU ROS2 driver cloned successfully"
else
    log_error "Failed to clone Witmotion IMU ROS2 driver"
    exit 1
fi

#***********************************************************************
#  ZED ROS2
#***********************************************************************

log_info "Cloning ZED ROS2 package..."
cd "$WORKSPACE_DIR"
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
if [ $? -eq 0 ]; then
    log_success "ZED ROS2 package cloned successfully"
else
    log_error "Failed to clone ZED ROS2 package"
    exit 1
fi

#***********************************************************************
# Configure LiDAR type to RSHELIOS (Helios 32)
#***********************************************************************

log_info "Configuring LiDAR type to RSHELIOS..."
CONFIG_FILE="$WORKSPACE_DIR/rslidar_sdk/config/config.yaml"
if [ -f "$CONFIG_FILE" ]; then
    log_info "Found config file: $CONFIG_FILE"
    sed -i.bak 's/lidar_type: RSM1/lidar_type: RSHELIOS/' "$CONFIG_FILE"
    if [ $? -eq 0 ]; then
        log_success "LiDAR type configured to RSHELIOS"
    else
        log_error "Failed to configure LiDAR type"
    fi
else
    log_error "Config file not found at: $CONFIG_FILE"
fi

#***********************************************************************
# Configure Witmotion IMU serial port to /dev/ttyUSB0
#***********************************************************************

log_info "Configuring Witmotion IMU serial port..."
IMU_CONFIG="$WORKSPACE_DIR/witmotion_ros2/config/witmotion.yaml"
if [ -f "$IMU_CONFIG" ]; then
    log_info "Found IMU config file: $IMU_CONFIG"
    sed -i.bak 's|/dev/ttyUSB1|/dev/ttyUSB0|' "$IMU_CONFIG"
    if [ $? -eq 0 ]; then
        log_success "Witmotion IMU serial port configured to /dev/ttyUSB0"
    else
        log_error "Failed to configure Witmotion IMU serial port"
    fi
else
    log_error "IMU config file not found at: $IMU_CONFIG"
fi

#***********************************************************************
# Copy udev rules for ReSpeaker
#***********************************************************************

log_info "Copying udev rules for ReSpeaker..."
RULES_DIR="$WORKSPACE_DIR/respeaker_ros/config"
if [ -d "$RULES_DIR" ]; then
    log_info "Found ReSpeaker config directory: $RULES_DIR"
    if [ -f "$RULES_DIR"/*.rules ]; then
        log_info "Copying rules files to /etc/udev/rules.d/"
        sudo cp "$RULES_DIR"/*.rules /etc/udev/rules.d/ 2>/dev/null || log_warning "Could not copy udev rules (may require sudo)"
        log_success "udev rules copied successfully"
    else
        log_warning "No .rules files found in $RULES_DIR"
    fi
else
    log_error "ReSpeaker config directory not found at: $RULES_DIR"
fi

#***********************************************************************
# Install Python dependencies for ReSpeaker
#***********************************************************************

log_info "Installing Python dependencies for ReSpeaker..."
log_info "Installing: pyusb, click, pyaudio, pixel-ring"
pip3 install --no-cache-dir pyusb click pyaudio pixel-ring
if [ $? -eq 0 ]; then
    log_success "ReSpeaker Python dependencies installed successfully"
else
    log_error "Failed to install ReSpeaker Python dependencies"
fi

#***********************************************************************
# Install additional ROS2 dependencies and fix transforms3d compatibility
#***********************************************************************

log_info "Installing additional ROS2 dependencies..."
log_info "Installing: ros-humble-tf-transformations"
sudo apt-get update && sudo apt-get install -y ros-humble-tf-transformations
if [ $? -eq 0 ]; then
    log_success "ROS2 tf-transformations installed successfully"
else
    log_warning "Failed to install ROS2 tf-transformations (may not be available)"
fi

log_info "Installing transforms3d Python package..."
pip3 install --no-cache-dir --ignore-installed transforms3d
if [ $? -eq 0 ]; then
    log_success "transforms3d installed successfully"
else
    log_error "Failed to install transforms3d"
fi

log_info "Driver download and configuration process completed successfully."




