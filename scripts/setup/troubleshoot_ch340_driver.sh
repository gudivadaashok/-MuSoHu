#!/bin/bash

#***********************************************************************
# Troubleshooting: CH340 Driver Installation (Jetson)
#***********************************************************************
# If the IMU is connected but /dev/ttyUSB0 doesn't appear, the 
# CH340/CH341 USB-to-serial driver may be missing from the Jetson kernel.
#***********************************************************************

# Source logging configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../utils/logging_config.sh"

log_info "CH340 Driver Troubleshooting Script"
log_separator

#***********************************************************************
# Check for CH340 Device
#***********************************************************************

log_info "Checking for CH340 USB device..."
log_info "Running lsusb to detect connected devices..."

if lsusb | grep -q "1a86:7523"; then
    log_success "CH340 device detected (1a86:7523 QinHeng Electronics)"
else
    log_warning "CH340 device not detected via lsusb"
    log_info "Connected devices:"
    lsusb
fi

#***********************************************************************
# Check for /dev/ttyUSB* devices
#***********************************************************************

log_info "Checking for /dev/ttyUSB* devices..."
if ls /dev/ttyUSB* 2>/dev/null | wc -l > /dev/null; then
    log_success "Found /dev/ttyUSB* devices:"
    ls -la /dev/ttyUSB*
else
    log_error "No /dev/ttyUSB* devices found"
    log_info "This indicates the CH340 driver may not be loaded"
fi

#***********************************************************************
# Check if CH341 module is already loaded
#***********************************************************************

log_info "Checking if CH341 kernel module is loaded..."
if lsmod | grep -q "ch341"; then
    log_success "CH341 module is already loaded"
    lsmod | grep ch341
else
    log_warning "CH341 module is not loaded"
fi

#***********************************************************************
# Attempt to load module via modprobe
#***********************************************************************

log_info "Attempting to load CH341 module via modprobe..."
if sudo modprobe ch341 2>/dev/null; then
    log_success "CH341 module loaded successfully via modprobe"
    log_info "Verifying /dev/ttyUSB* devices..."
    sleep 2
    if ls /dev/ttyUSB* 2>/dev/null; then
        log_success "CH340 device is now available at:"
        ls -la /dev/ttyUSB*
        log_separator
        log_success "CH340 driver is working! No further action needed."
        log_separator
        exit 0
    fi
else
    log_warning "modprobe ch341 failed: module not found or requires compilation"
fi

#***********************************************************************
# Compile driver from source
#***********************************************************************

log_info "CH341 module not found. Proceeding with source compilation..."
log_separator

TEMP_DIR="/tmp/CH341SER"
if [ -d "$TEMP_DIR" ]; then
    log_info "Removing existing directory: $TEMP_DIR"
    rm -rf "$TEMP_DIR"
fi

log_info "Cloning CH341SER driver repository..."
log_info "Repository: https://github.com/juliagoda/CH341SER.git"
git clone https://github.com/juliagoda/CH341SER.git "$TEMP_DIR"
if [ $? -ne 0 ]; then
    log_error "Failed to clone CH341SER repository"
    exit 1
fi
log_success "Repository cloned successfully to: $TEMP_DIR"

log_info "Entering driver directory..."
cd "$TEMP_DIR"
log_info "Current directory: $(pwd)"

#***********************************************************************
# Compile the driver
#***********************************************************************

log_info "Compiling CH341 driver..."
log_info "Running: make"
make
if [ $? -ne 0 ]; then
    log_error "Failed to compile CH341 driver"
    log_info "You may need to install build tools: sudo apt-get install build-essential"
    exit 1
fi
log_success "CH341 driver compiled successfully"

#***********************************************************************
# Load driver into kernel
#***********************************************************************

log_info "Loading compiled driver into kernel..."
log_info "Running: sudo make load"
sudo make load
if [ $? -ne 0 ]; then
    log_error "Failed to load driver into kernel"
    exit 1
fi
log_success "Driver loaded into kernel successfully"

log_info "Waiting for device to appear..."
sleep 2

#***********************************************************************
# Verify driver loaded
#***********************************************************************

log_info "Verifying CH340 device availability..."
if ls /dev/ttyUSB* 2>/dev/null; then
    log_success "CH340 device is now available:"
    ls -la /dev/ttyUSB*
else
    log_error "Device still not available after loading driver"
    exit 1
fi

#***********************************************************************
# Install permanently (survives reboots)
#***********************************************************************

log_separator
log_info "Driver loaded successfully (temporary)"
log_info "Installing driver permanently (survives reboots)..."
log_info "Running: sudo make install"

sudo make install
if [ $? -eq 0 ]; then
    log_success "Driver installed permanently"
    log_info "The CH341 driver will now load automatically on boot"
else
    log_warning "Failed to install driver permanently"
    log_info "Driver is currently loaded but may not survive reboot"
fi

#***********************************************************************
# Summary
#***********************************************************************

log_separator
log_success "CH340 Driver Troubleshooting Complete!"
log_separator
log_info "Next steps:"
log_info "1. Verify IMU connection: ls -la /dev/ttyUSB*"
log_info "2. Test with your ROS2 node or serial monitor"
log_info "3. If issues persist, check dmesg: dmesg | grep -i ch341"
log_separator
log_info "Logs saved to: $LOG_FILE"
log_separator
