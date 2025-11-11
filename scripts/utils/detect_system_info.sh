#!/bin/bash

#***********************************************************************
# System Information Detection Utility
#***********************************************************************
# This script detects Ubuntu release year and CUDA version from the host
# Can be sourced by other scripts or run standalone
#***********************************************************************

detect_ubuntu_version() {
    if [ -f /etc/os-release ]; then
        # Extract Ubuntu version (e.g., "22.04" -> "22")
        UBUNTU_VERSION=$(grep "VERSION_ID=" /etc/os-release | cut -d'"' -f2)
        export UBUNTU_RELEASE_YEAR=$(echo "$UBUNTU_VERSION" | cut -d'.' -f1)
        echo "Detected Ubuntu: $UBUNTU_VERSION (Release Year: $UBUNTU_RELEASE_YEAR)"
    else
        export UBUNTU_RELEASE_YEAR="unknown"
        echo "Could not detect Ubuntu version"
    fi
}

detect_cuda_version() {
    local cuda_found=false
    
    # Try nvcc first
    if command -v nvcc >/dev/null 2>&1; then
        CUDA_VERSION=$(nvcc --version | grep "release" | sed 's/.*release \([0-9]*\.[0-9]*\).*/\1/')
        if [ ! -z "$CUDA_VERSION" ]; then
            export CUDA_MAJOR=$(echo "$CUDA_VERSION" | cut -d'.' -f1)
            export CUDA_MINOR=$(echo "$CUDA_VERSION" | cut -d'.' -f2)
            echo "Detected CUDA (nvcc): $CUDA_VERSION"
            cuda_found=true
        fi
    fi
    
    # Try nvidia-smi as fallback
    if [ "$cuda_found" = false ] && command -v nvidia-smi >/dev/null 2>&1; then
        CUDA_VERSION=$(nvidia-smi | grep "CUDA Version:" | sed 's/.*CUDA Version: \([0-9]*\.[0-9]*\).*/\1/')
        if [ ! -z "$CUDA_VERSION" ]; then
            export CUDA_MAJOR=$(echo "$CUDA_VERSION" | cut -d'.' -f1)
            export CUDA_MINOR=$(echo "$CUDA_VERSION" | cut -d'.' -f2)
            echo "Detected CUDA (nvidia-smi): $CUDA_VERSION"
            cuda_found=true
        fi
    fi
    
    # Set defaults if not found
    if [ "$cuda_found" = false ]; then
        export CUDA_MAJOR="unknown"
        export CUDA_MINOR="unknown"
        echo "CUDA not detected or not available"
    fi
}

display_system_info() {
    echo "=== System Information ==="
    echo "Ubuntu Release Year: $UBUNTU_RELEASE_YEAR"
    echo "CUDA Major Version: $CUDA_MAJOR"
    echo "CUDA Minor Version: $CUDA_MINOR"
    echo "Full CUDA Version: $CUDA_MAJOR.$CUDA_MINOR"
    echo "=========================="
}

# Main function to detect all system info
detect_all() {
    detect_ubuntu_version
    detect_cuda_version
    display_system_info
}

# Export the functions
export -f detect_ubuntu_version
export -f detect_cuda_version
export -f display_system_info
export -f detect_all

# If script is run directly (not sourced), run detection
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    detect_all
fi