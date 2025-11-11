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
source "$SCRIPT_DIR/../utils/detect_system_info.sh"

# Run system detection if variables are not already set
if [ -z "$UBUNTU_RELEASE_YEAR" ] || [ -z "$CUDA_MAJOR" ] || [ -z "$CUDA_MINOR" ]; then
    log_info "Detecting system information..."
    detect_all
fi

log_info "Starting ZED SDK installation..."
log_separator

#***********************************************************************
# Set ZED SDK version (can be overridden with environment variables)
#***********************************************************************

# Default ZED SDK version if not set
export ZED_SDK_MAJOR=${ZED_SDK_MAJOR:-4}
export ZED_SDK_MINOR=${ZED_SDK_MINOR:-1}

log_info "System Information:"
log_info "  Ubuntu Release Year: $UBUNTU_RELEASE_YEAR"
log_info "  CUDA Major Version: $CUDA_MAJOR"
log_info "  CUDA Minor Version: $CUDA_MINOR"
log_info "  ZED SDK Version: $ZED_SDK_MAJOR.$ZED_SDK_MINOR"

# Validate that all required variables are set
if [ -z "$UBUNTU_RELEASE_YEAR" ] || [ "$UBUNTU_RELEASE_YEAR" = "unknown" ]; then
    log_error "Ubuntu release year could not be detected or is unknown"
    log_info "Please set manually: export UBUNTU_RELEASE_YEAR=22"
    exit 1
fi

if [ -z "$CUDA_MAJOR" ] || [ "$CUDA_MAJOR" = "unknown" ]; then
    log_error "CUDA major version could not be detected or is unknown"
    log_info "Please set manually: export CUDA_MAJOR=12"
    exit 1
fi

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

# Construct the download URL and filename
ZED_SDK_FILENAME="ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run"

# Try different URL formats as Stereolabs may have changed their download structure
ZED_SDK_URL=https://download.stereolabs.com/zedsdk/5.1/l4t36.4/jetsons

log_debug "URL components:"
log_debug "  ZED_SDK_MAJOR: '$ZED_SDK_MAJOR'"
log_debug "  ZED_SDK_MINOR: '$ZED_SDK_MINOR'"
log_debug "  CUDA_MAJOR: '$CUDA_MAJOR'"
log_debug "  UBUNTU_RELEASE_YEAR: '$UBUNTU_RELEASE_YEAR'"

log_info "Filename: $ZED_SDK_FILENAME"
log_info "Will try multiple URL formats to find the correct download"

# Download the ZED SDK
log_info "Attempting to download ZED SDK..."

# Try different URL formats
download_success=false
for ZED_SDK_URL in "${ZED_SDK_URLS[@]}"; do
    log_info "Trying URL: $ZED_SDK_URL"
    
    # Check if URL is accessible
    if curl -s --head "$ZED_SDK_URL" 2>/dev/null | head -n 1 | grep -q "200 OK"; then
        log_info "URL is accessible, attempting download..."
        
        # Download with progress indication
        if wget --progress=dot -O "$ZED_SDK_FILENAME" "$ZED_SDK_URL" 2>&1 | \
           while read line; do log_debug "$line"; done; then
            
            # Verify downloaded file
            if [ -f "$ZED_SDK_FILENAME" ] && [ -s "$ZED_SDK_FILENAME" ]; then
                # Check if it's actually an installer
                if file "$ZED_SDK_FILENAME" 2>/dev/null | grep -q "HTML\|text"; then
                    log_warning "Downloaded file is HTML/text, not binary - URL may be incorrect"
                    rm -f "$ZED_SDK_FILENAME"
                    continue
                else
                    log_success "Downloaded valid binary installer"
                    download_success=true
                    break
                fi
            else
                log_warning "Download failed or file is empty"
                rm -f "$ZED_SDK_FILENAME"
                continue
            fi
        else
            log_warning "wget failed for this URL"
            continue
        fi
    else
        log_debug "URL not accessible or returns error"
    fi
done

if [ "$download_success" = true ]; then
    log_success "ZED SDK download completed successfully"
    log_info "Downloaded file: $ZED_SDK_FILENAME"
    log_info "File size: $(du -h "$ZED_SDK_FILENAME" | cut -f1)"
else
    log_error "Failed to download ZED SDK from any URL"
    log_error "All attempted URLs failed or returned invalid files"
    log_info "Attempted URLs:"
    for url in "${ZED_SDK_URLS[@]}"; do
        log_info "  $url"
    done
    log_info ""
    log_info "Please try one of the following:"
    log_info "  1. Check the ZED SDK download page: https://www.stereolabs.com/developers/release/"
    log_info "  2. Manually download the installer and place it in the current directory"
    log_info "  3. Verify the ZED SDK version and CUDA version compatibility"
    log_info "  4. Check your internet connection"
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
log_info "Installation summary:"
log_info "  Ubuntu: $UBUNTU_RELEASE_YEAR.04"
log_info "  CUDA: $CUDA_MAJOR.x compatible"
log_info "  ZED SDK: $ZED_SDK_MAJOR.$ZED_SDK_MINOR"
log_info "  Installation mode: Runtime only, CUDA skipped"
log_separator
log_info "You may need to:"
log_info "  1. Reboot the system or reconnect USB devices"
log_info "  2. Add your user to the necessary groups"
log_info "  3. Check ZED camera permissions with: ls -l /dev/video*"
log_info "Logs saved to: $LOG_FILE"