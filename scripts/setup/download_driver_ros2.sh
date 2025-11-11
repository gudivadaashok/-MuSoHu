#!/bin/bash

#***********************************************************************
# Download Driver Script for MuSoHu
#***********************************************************************


# Source logging configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../utils/logging_config.sh"
log_info "Starting driver download process..."


# Create workspace directory if it doesn't exist for ros2_musohu_ws
WORKSPACE_DIR="$HOME/ros2_musohu_ws/src"
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

if [ -d "audio_common" ]; then
    log_warning "audio_common directory already exists, skipping clone"
    log_info "To update: cd $WORKSPACE_DIR/audio_common && git pull"
else
    git clone -b ros2 https://github.com/ros-drivers/audio_common.git
    if [ $? -eq 0 ]; then
        log_success "audio_common package cloned successfully"
    else
        log_error "Failed to clone audio_common package"
        exit 1
    fi
fi


#***********************************************************************
# Clone respeaker_ros
#***********************************************************************

log_info "Cloning respeaker_ros package..."
cd "$WORKSPACE_DIR"

if [ -d "respeaker_ros" ]; then
    log_warning "respeaker_ros directory already exists, skipping clone"
    log_info "To update: cd $WORKSPACE_DIR/respeaker_ros && git pull"
else
    git clone https://github.com/hcrlab/respeaker_ros.git
    if [ $? -eq 0 ]; then
        log_success "respeaker_ros package cloned successfully"
    else
        log_error "Failed to clone respeaker_ros package"
        exit 1
    fi
fi

#***********************************************************************
# RoboSense LiDAR SDK and message definitions
#***********************************************************************

log_info "Cloning RoboSense LiDAR SDK and message definitions..."
cd "$WORKSPACE_DIR"

if [ -d "rslidar_sdk" ]; then
    log_warning "rslidar_sdk directory already exists, skipping clone"
    log_info "To update: cd $WORKSPACE_DIR/rslidar_sdk && git pull"
else
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
fi


log_info "Cloning rslidar_msgs package..."
cd "$WORKSPACE_DIR"

if [ -d "rslidar_msg" ]; then
    log_warning "rslidar_msg directory already exists, skipping clone"
    log_info "To update: cd $WORKSPACE_DIR/rslidar_msg && git pull"
else
    git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
    if [ $? -eq 0 ]; then
        log_success "rslidar_msgs package cloned successfully"
    else
        log_error "Failed to clone rslidar_msgs package"
        exit 1
    fi
fi

#***********************************************************************
# Witmotion IMU ROS2 driver
#***********************************************************************

log_info "Cloning Witmotion IMU ROS2 driver..."
cd "$WORKSPACE_DIR"

if [ -d "witmotion_ros2" ]; then
    log_warning "witmotion_ros2 directory already exists, skipping clone"
    log_info "To update: cd $WORKSPACE_DIR/witmotion_ros2 && git pull"
else
    git clone https://github.com/ioio2995/witmotion_ros2.git
    if [ $? -eq 0 ]; then
        log_success "Witmotion IMU ROS2 driver cloned successfully"
    else
        log_error "Failed to clone Witmotion IMU ROS2 driver"
        exit 1
    fi
fi

#***********************************************************************
#  ZED ROS2
#***********************************************************************

log_info "Cloning ZED ROS2 package..."
cd "$WORKSPACE_DIR"

if [ -d "zed-ros2-wrapper" ]; then
    log_warning "zed-ros2-wrapper directory already exists, skipping clone"
    log_info "To update: cd $WORKSPACE_DIR/zed-ros2-wrapper && git pull --recurse-submodules"
else
    git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
    if [ $? -eq 0 ]; then
        log_success "ZED ROS2 package cloned successfully"
    else
        log_error "Failed to clone ZED ROS2 package"
        exit 1
    fi
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
# Install Python dependencies for ReSpeaker
#***********************************************************************

log_info "Installing system dependencies for ReSpeaker..."
log_info "Installing PortAudio development files..."
sudo apt-get update
sudo apt-get install -y portaudio19-dev libasound2-dev libsndfile1-dev
if [ $? -eq 0 ]; then
    log_success "System dependencies installed successfully"
else
    log_error "Failed to install system dependencies"
fi

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




