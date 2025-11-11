#!/bin/bash

#***********************************************************************
# Test System Information Detection
#***********************************************************************

echo "Testing system information detection..."
echo

# Method 1: Source the detection script
echo "=== Method 1: Using detection utility ==="
source "$(dirname "${BASH_SOURCE[0]}")/detect_system_info.sh"

echo
echo "=== Method 2: Manual environment variables ==="
echo "You can also set them manually:"
echo "export UBUNTU_RELEASE_YEAR=22"
echo "export CUDA_MAJOR=11"
echo "export CUDA_MINOR=8"

echo
echo "=== Method 3: Current environment ==="
if [ ! -z "$UBUNTU_RELEASE_YEAR" ]; then
    echo "UBUNTU_RELEASE_YEAR is already set to: $UBUNTU_RELEASE_YEAR"
else
    echo "UBUNTU_RELEASE_YEAR is not set"
fi

if [ ! -z "$CUDA_MAJOR" ] && [ ! -z "$CUDA_MINOR" ]; then
    echo "CUDA version is already set to: $CUDA_MAJOR.$CUDA_MINOR"
else
    echo "CUDA version is not set"
fi

echo
echo "=== Export commands for manual setup ==="
echo "export UBUNTU_RELEASE_YEAR=$UBUNTU_RELEASE_YEAR"
echo "export CUDA_MAJOR=$CUDA_MAJOR"
echo "export CUDA_MINOR=$CUDA_MINOR"