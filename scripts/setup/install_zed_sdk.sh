#!/bin/bash

#***********************************************************************
# ZED SDK Installation Script
#***********************************************************************
# This script installs the ZED SDK and its dependencies
# It automatically detects the system configuration or uses environment variables
#***********************************************************************

# Get script directory and source utilities
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../utils/logging_config.sh"

log_info "Starting ZED SDK installation..."
log_separator

#***********************************************************************
# Check if running with sudo
#***********************************************************************

if [ "$EUID" -ne 0 ]; then
    log_error "This script must be run with sudo"
    log_info "Usage: sudo bash install_zed_sdk.sh"
    exit 1
fi

#***********************************************************************
# Update package lists and install dependencies
#***********************************************************************

log_info "Updating package lists..."
apt-get update -y || true

log_info "Installing dependencies..."
apt-get install --no-install-recommends \
    lsb-release \
    wget \
    less \
    zstd \
    udev \
    sudo \
    python3 \
    python3-pip \
    libpng-dev \
    libgomp1 \
    -y

if [ $? -eq 0 ]; then
    log_success "Dependencies installed successfully"
else
    log_error "Failed to install dependencies"
    exit 1
fi

#***********************************************************************
# Install Python packages
#***********************************************************************

log_info "Installing Python packages..."
# Uncomment the next line if you want to upgrade pip
#python3 -m pip install --upgrade pip

python3 -m pip install numpy opencv-python

if [ $? -eq 0 ]; then
    log_success "Python packages installed successfully"
else
    log_warning "Some Python packages may have failed to install"
fi

#***********************************************************************
# Download and install ZED SDK
#***********************************************************************

log_info "Downloading ZED SDK..."

#***********************************************************************
# ZED SDK Download Only work for the Jetson L4T 36.4
# ZED SDK for JetPack 6.1 and 6.2 (L4T 36.4) 5.1 (Jetson Orin, CUDA 12.6)
# for more details visit: https://www.stereolabs.com/developers/release/
#***********************************************************************

ZED_SDK_URL="https://download.stereolabs.com/zedsdk/5.1/l4t36.4/jetsons"
ZED_SDK_FILENAME="ZED_SDK_Linux.run"

log_info "Download URL: $ZED_SDK_URL"
log_info "Filename: $ZED_SDK_FILENAME"

# Download the ZED SDK
log_info "Downloading ZED SDK installer..."
if wget --progress=dot -O "$ZED_SDK_FILENAME" "$ZED_SDK_URL"; then
    log_success "ZED SDK download completed successfully"
    log_info "Downloaded file: $ZED_SDK_FILENAME"
    log_info "File size: $(du -h "$ZED_SDK_FILENAME" | cut -f1)"
else
    log_error "Failed to download ZED SDK"
    log_info "Please check:"
    log_info "  1. Internet connection"
    log_info "  2. ZED SDK download page: https://www.stereolabs.com/developers/release/"
    exit 1
fi

#***********************************************************************
# Install ZED SDK
#***********************************************************************

log_info "Installing ZED SDK..."

# Make the installer executable
chmod +x "$ZED_SDK_FILENAME"

# Install ZED SDK in silent mode with runtime only and skip CUDA
./"$ZED_SDK_FILENAME" -- silent runtime_only skip_cuda

if [ $? -eq 0 ]; then
    log_success "ZED SDK installed successfully"
else
    log_error "Failed to install ZED SDK"
    log_info "Check the installer output above for details"
    exit 1
fi

#***********************************************************************
# Cleanup
#***********************************************************************

log_info "Cleaning up..."

# Remove the installer file
rm -f "$ZED_SDK_FILENAME"

# Clean up apt cache
rm -rf /var/lib/apt/lists/*

log_success "Cleanup completed"

#***********************************************************************
# Installation summary
#***********************************************************************

log_separator
log_success "ZED SDK installation completed!"
log_info "Installation mode: Runtime only, CUDA skipped"
log_separator
log_info "You may need to:"
log_info "  1. Reboot the system or reconnect USB devices"
log_info "  2. Add your user to the necessary groups"
log_info "  3. Check ZED camera permissions with: ls -l /dev/video*"