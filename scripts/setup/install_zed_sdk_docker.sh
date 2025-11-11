#!/bin/bash

#***********************************************************************
# ZED SDK Docker Installation Script
#***********************************************************************
# Simplified version for Docker containers or minimal environments
# Uses environment variables for configuration
#***********************************************************************

# Default values if not set
export UBUNTU_RELEASE_YEAR=${UBUNTU_RELEASE_YEAR:-22}
export CUDA_MAJOR=${CUDA_MAJOR:-11}
export ZED_SDK_MAJOR=${ZED_SDK_MAJOR:-4}
export ZED_SDK_MINOR=${ZED_SDK_MINOR:-1}

echo "Installing ZED SDK with configuration:"
echo "  Ubuntu: ${UBUNTU_RELEASE_YEAR}.04"
echo "  CUDA: ${CUDA_MAJOR}.x"
echo "  ZED SDK: ${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}"
echo

# Update and install dependencies
echo "Installing dependencies..."
apt-get update -y || true
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

# Install Python packages
echo "Installing Python packages..."
python3 -m pip install numpy opencv-python

# Download and install ZED SDK
echo "Downloading and installing ZED SDK..."
ZED_SDK_FILENAME="ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run"
ZED_SDK_URL="https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/cu${CUDA_MAJOR}/ubuntu${UBUNTU_RELEASE_YEAR}"

wget -q -O "$ZED_SDK_FILENAME" "$ZED_SDK_URL" && \
    chmod +x "$ZED_SDK_FILENAME" && \
    ./"$ZED_SDK_FILENAME" -- silent runtime_only skip_cuda && \
    rm -f "$ZED_SDK_FILENAME" && \
    rm -rf /var/lib/apt/lists/*

if [ $? -eq 0 ]; then
    echo "ZED SDK installation completed successfully!"
else
    echo "ZED SDK installation failed!"
    exit 1
fi