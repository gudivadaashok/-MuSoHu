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
sudo apt-get install -y portaudio19-dev libasound2-dev libsndfile1-dev python3-venv
if [ $? -eq 0 ]; then
    log_success "System dependencies installed successfully"
else
    log_error "Failed to install system dependencies"
fi

log_info "Installing Python dependencies for ReSpeaker..."
log_info "Installing: pyusb, click, pyaudio, pixel-ring"

# Comprehensive fix for permission issues with all packages in /usr/local/lib/python3.10/dist-packages/
log_info "Fixing potential permission issues with system Python packages..."
if [ -d "/usr/local/lib/python3.10/dist-packages" ]; then
    log_info "Found /usr/local/lib/python3.10/dist-packages/, fixing permissions..."
    sudo chmod -R 755 /usr/local/lib/python3.10/dist-packages/ 2>/dev/null || true
    sudo find /usr/local/lib/python3.10/dist-packages/ -type f -name "*.dist-info" -exec chmod 755 {} \; 2>/dev/null || true
    sudo find /usr/local/lib/python3.10/dist-packages/ -type d -name "*dist-info*" -exec chmod 755 {} \; 2>/dev/null || true
fi

# Try system packages first
log_info "Installing available system packages..."
sudo apt-get install -y python3-click python3-usb python3-numpy python3-matplotlib || true

# Create a clean virtual environment as the primary installation method
log_info "Creating virtual environment for Python packages..."
VENV_PATH="$HOME/musohu_python_env"

# Remove existing virtual environment if it exists
if [ -d "$VENV_PATH" ]; then
    log_info "Removing existing virtual environment..."
    rm -rf "$VENV_PATH"
fi

# Create new virtual environment
python3 -m venv "$VENV_PATH"
if [ $? -eq 0 ]; then
    log_success "Virtual environment created successfully at $VENV_PATH"
    
    # Activate and install packages
    source "$VENV_PATH/bin/activate"
    
    # Upgrade pip in virtual environment
    pip install --upgrade pip setuptools wheel
    
    # Install required packages
    log_info "Installing Python packages in virtual environment..."
    pip install pyusb click pyaudio pixel-ring transforms3d
    
    if [ $? -eq 0 ]; then
        log_success "All Python packages installed successfully in virtual environment"
        log_info "Virtual environment location: $VENV_PATH"
        log_info "To use this environment, run: source $VENV_PATH/bin/activate"
        
        # Create activation script for easy access
        cat > "$HOME/activate_musohu_env.sh" << 'EOF'
#!/bin/bash
echo "Activating MuSoHu Python environment..."
source ~/musohu_python_env/bin/activate
echo "Environment activated. Python packages are now available."
echo "To deactivate, run: deactivate"
EOF
        chmod +x "$HOME/activate_musohu_env.sh"
        log_info "Created activation script at: $HOME/activate_musohu_env.sh"
        
    else
        log_error "Failed to install packages in virtual environment"
    fi
    
    deactivate
else
    log_error "Failed to create virtual environment"
    log_info "Attempting system-wide installation as last resort..."
    
    # Last resort: try to fix pip completely and install globally
    log_info "Attempting to rebuild pip environment..."
    sudo apt-get install -y --reinstall python3-pip python3-setuptools python3-wheel
    
    # Try to install with pip system-wide but with --break-system-packages flag
    log_info "Installing with --break-system-packages flag..."
    python3 -m pip install --break-system-packages pyusb click pyaudio pixel-ring transforms3d || log_error "All installation methods failed"
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

# transforms3d should already be installed in the virtual environment above
# But provide alternative installation methods if needed
log_info "Checking transforms3d installation..."
if [ -f "$HOME/musohu_python_env/bin/activate" ]; then
    source "$HOME/musohu_python_env/bin/activate"
    python3 -c "import transforms3d; print('transforms3d is available in virtual environment')" 2>/dev/null
    if [ $? -eq 0 ]; then
        log_success "transforms3d is available in virtual environment"
    else
        log_warning "transforms3d may not be installed in virtual environment"
    fi
    deactivate
else
    log_info "Virtual environment not found, checking system installation..."
    python3 -c "import transforms3d; print('transforms3d is available system-wide')" 2>/dev/null || \
    log_info "transforms3d not available system-wide (this is expected if using virtual environment)"
fi

log_info "Driver download and configuration process completed successfully."




