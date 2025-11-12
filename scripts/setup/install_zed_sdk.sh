#!/bin/bash

#***********************************************************************
# ZED SDK Installation Script
#***********************************************************************
# This script installs the ZED SDK and its dependencies
# It automatically detects the system configuration or uses environment variables
# Designed for JetPack 6.1/6.2 (L4T 36.4) with CUDA 12.6
#***********************************************************************

#***********************************************************************
# Help function
#***********************************************************************

show_help() {
    cat << EOF
Usage: sudo bash install_zed_sdk.sh [OPTIONS]

Install ZED SDK for Jetson devices with JetPack 6.1/6.2.

This script:
  - Installs ZED SDK dependencies
  - Runs Python packages installation script
  - Downloads ZED SDK 5.1 for JetPack 6.1/6.2 (L4T 36.4)
  - Installs SDK in runtime-only mode with CUDA skipped
  - Cleans up temporary files

ZED SDK Version:
  Version: 5.1
  Target:  JetPack 6.1 and 6.2 (L4T 36.4)
  Platform: Jetson Orin
  CUDA:    12.6
  URL:     https://www.stereolabs.com/developers/release/

Installation Mode:
  - Runtime only (no samples/tools)
  - CUDA skipped (uses existing CUDA installation)

Dependencies Installed:
  System packages:
    - lsb-release, wget, less, zstd
    - udev, sudo
    - libpng-dev, libgomp1
  Python packages:
    - Installed via install_python_packages.sh

Options:
  -h, --help     Display this help message and exit

Examples:
  sudo bash install_zed_sdk.sh
  sudo bash install_zed_sdk.sh --help

Post-Installation:
  1. Reboot system or reconnect USB devices
  2. Add user to necessary groups
  3. Check camera permissions: ls -l /dev/video*
  4. Test with ZED SDK tools

Troubleshooting:
  - Check ZED SDK documentation: https://www.stereolabs.com/docs/
  - Verify JetPack version: sudo apt-cache show nvidia-jetpack
  - Check CUDA version: nvcc --version

Note: This script requires root privileges.
      For other JetPack versions, visit:
      https://www.stereolabs.com/developers/release/

EOF
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
    shift
done

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

log_info "Installing ZED SDK dependencies..."
apt-get install --no-install-recommends \
    lsb-release \
    wget \
    less \
    zstd \
    udev \
    sudo \
    libpng-dev \
    libgomp1 \
    -y

if [ $? -eq 0 ]; then
    log_success "ZED SDK dependencies installed successfully"
else
    log_error "Failed to install ZED SDK dependencies"
    exit 1
fi

#***********************************************************************
# Install Python packages
#***********************************************************************

log_info "Installing Python packages..."
log_info "Running separate Python packages installation script..."

# Run the Python packages installation script
if bash "$SCRIPT_DIR/install_python_packages.sh"; then
    log_success "Python packages installation completed"
else
    log_warning "Python packages installation had issues, but continuing with ZED SDK installation"
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
log_info "Python packages installed via: install_python_packages.sh"
log_separator
log_info "You may need to:"
log_info "  1. Reboot the system or reconnect USB devices"
log_info "  2. Add your user to the necessary groups"
log_info "  3. Check ZED camera permissions with: ls -l /dev/video*"
log_info ""
log_info "For additional Python packages, run:"
log_info "  sudo bash scripts/setup/install_python_packages.sh"