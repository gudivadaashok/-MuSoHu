#!/bin/bash

#***********************************************************************
# Troubleshooting: CH340 Driver Installation (Jetson)
#***********************************************************************
# If the IMU is connected but /dev/ttyUSB0 doesn't appear, the 
# CH340/CH341 USB-to-serial driver may be missing from the Jetson kernel.
# This script detects, compiles, and installs the CH340 driver from source
#***********************************************************************

#***********************************************************************
# Help function
#***********************************************************************

show_help() {
    cat << EOF
Usage: bash troubleshoot_ch340_driver.sh [OPTIONS]

Troubleshoot and install CH340/CH341 USB-to-serial driver.

This script:
  - Detects CH340 device (1a86:7523)
  - Checks for /dev/ttyUSB* devices
  - Attempts to load existing ch341 module
  - If not found, downloads and compiles driver from source
  - Installs driver permanently (survives reboots)
  - Verifies device availability

CH340/CH341 Device:
  Vendor ID:  1a86
  Product ID: 7523
  Chip:       QinHeng Electronics CH340/CH341
  Used by:    Witmotion IMU and other USB-serial devices

Expected Devices:
  /dev/ttyUSB0, /dev/ttyUSB1, etc.
  or
  /dev/ttyACM0, /dev/ttyACM1, etc.

Driver Source:
  Repository: https://github.com/juliagoda/CH341SER.git
  Compilation: make
  Installation: make load && make install

Options:
  -h, --help     Display this help message and exit

Examples:
  bash troubleshoot_ch340_driver.sh
  bash troubleshoot_ch340_driver.sh --help

Troubleshooting Steps:
  1. Check if CH340 detected: lsusb | grep 1a86:7523
  2. Check if module loaded: lsmod | grep ch34x
  3. Check devices: ls /dev/ttyUSB* /dev/ttyACM*
  4. View kernel messages: dmesg | grep -i ch34
  5. Trigger udev: sudo udevadm trigger
  6. Reload udev rules: sudo udevadm control --reload-rules

Manual Commands:
  Load module:     sudo modprobe ch341
  Remove module:   sudo rmmod ch34x
  Manual load:     sudo insmod /tmp/CH341SER/ch34x.ko
  Check status:    lsmod | grep ch34x

Prerequisites:
  - build-essential (for compiling)
  - git (for cloning repository)
  - sudo privileges (for module installation)

Note: If device still doesn't work:
      - Try a different USB port
      - Check cable connection
      - Verify device on another computer
      - Check if device is in use by another process

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

# Check if module is already loaded and remove it first
log_info "Checking for existing ch34x module..."
if lsmod | grep -q "ch34x"; then
    log_warning "ch34x module already loaded, removing first..."
    sudo rmmod ch34x
    if [ $? -eq 0 ]; then
        log_success "Existing module removed"
    else
        log_warning "Could not remove existing module (continuing anyway)"
    fi
fi

log_info "Running: sudo make load"
sudo make load
if [ $? -ne 0 ]; then
    log_error "Failed to load driver into kernel"
    log_info "Attempting manual module insertion..."
    log_info "Running: sudo insmod ch34x.ko"
    if sudo insmod ch34x.ko 2>/dev/null; then
        log_success "Module loaded manually"
    else
        log_error "Manual module insertion also failed"
        exit 1
    fi
else
    log_success "Driver loaded into kernel successfully"
fi

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
    log_info ""
    log_info "Additional troubleshooting steps:"
    log_separator
    
    # Check if module is actually loaded
    log_info "Checking if ch34x module is loaded..."
    if lsmod | grep -q ch34x; then
        log_success "ch34x module is loaded:"
        lsmod | grep ch34x
    else
        log_warning "ch34x module not found in lsmod"
    fi
    
    # Check dmesg for errors
    log_info "Checking dmesg for CH34x/USB errors..."
    dmesg | grep -i "ch34\|usb\|tty" | tail -10
    
    # Check if device is detected but not assigned
    log_info "Checking USB devices again..."
    lsusb | grep -i "1a86\|qinheng"
    
    # Check for any USB or ACM serial devices
    log_info "Checking for any USB/ACM serial devices..."
    USB_ACM_DEVICES=$(ls /dev/tty* 2>/dev/null | grep -E "(USB|ACM)")
    if [ -n "$USB_ACM_DEVICES" ]; then
        log_success "Found USB/ACM devices:"
        echo "$USB_ACM_DEVICES"
    else
        log_warning "No USB/ACM devices found"
    fi
    
    # Check all tty devices
    log_info "All available tty devices:"
    ls /dev/tty* | head -20
    
    # Check udev rules
    log_info "Checking udev rules for serial devices..."
    ls -la /etc/udev/rules.d/*serial* /etc/udev/rules.d/*usb* 2>/dev/null || log_info "No serial/USB udev rules found"
    
    # Manual device creation attempt
    log_info "Attempting manual device creation..."
    log_info "Try unplugging and reconnecting the device"
    log_info "Or run: sudo udevadm trigger"
    
    log_separator
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
log_info "Manual troubleshooting commands:"
log_info "1. Check module status: lsmod | grep ch34x"
log_info "2. Check USB devices: lsusb | grep -i qinheng"
log_info "3. Check kernel messages: dmesg | grep -i ch34"
log_info "4. Check all serial devices: ls /dev/tty* | grep -E '(USB|ACM)'"
log_info "5. Trigger udev: sudo udevadm trigger"
log_info "6. Reload udev rules: sudo udevadm control --reload-rules"
log_info "7. Check permissions: ls -la /dev/ttyUSB* /dev/ttyACM*"
log_info "8. Force module reload: sudo rmmod ch34x && sudo insmod /tmp/CH341SER/ch34x.ko"
log_info ""
log_info "If device still not working:"
log_info "- Try a different USB port"
log_info "- Check if device works on another computer"
log_info "- Verify device is not already in use by another process"
log_info "- Check cable connection"
log_separator
log_info "Logs saved to: $LOG_FILE"
log_separator
