#!/bin/bash

# MuSoHu Project Startup Script
echo "🤖 Starting MuSoHu - Multi-Modal Social Human Navigation Dataset 🤖"
echo "=================================================="

# Detect OS
OS_TYPE=$(uname -s)

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Error: Docker is not running."
    exit 1
fi

# Check if docker-compose is available
if ! command -v docker-compose > /dev/null 2>&1; then
    echo "❌ Error: docker-compose is not installed."
    exit 1
fi

# Check OS and create override file for Linux if needed
if [ "$OS_TYPE" = "Darwin" ]; then
    echo ""
    echo "ℹ️  Running on macOS"
    echo "   GPU and device passthrough not available"
    echo ""
    rm -f docker-compose.override.yml
elif [ "$OS_TYPE" = "Linux" ]; then
    echo "✅ Running on Linux - enabling hardware access"
    
    HAS_GPU=false
    if command -v nvidia-smi > /dev/null 2>&1; then
        echo "🎮 NVIDIA GPU detected"
        HAS_GPU=true
    fi
    
    cat > docker-compose.override.yml <<'EOF'
services:
  ros2_vnc:
    devices:
      - /dev:/dev
      - /dev/snd:/dev/snd
      - /dev/bus/usb:/dev/bus/usb
EOF
    
    if [ "$HAS_GPU" = true ]; then
        cat >> docker-compose.override.yml <<'EOF'
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
EOF
    fi
    
    echo "   Hardware access configured"
fi

# Create bind mount directories
if [ ! -d "$HOME/Desktop/Docker-Volumns" ]; then
    ./scripts/setup-volumes.sh > /dev/null 2>&1
fi

echo "🔧 Building and starting services..."
docker-compose up -d --build

echo "⏳ Waiting for services to initialize..."
sleep 5

echo "🔐 Setting VNC password..."
docker exec ros2_vnc bash -c "mkdir -p /home/ubuntu/.vnc && echo 'robotixx' | vncpasswd -f > /home/ubuntu/.vnc/passwd && chmod 600 /home/ubuntu/.vnc/passwd && chown ubuntu:ubuntu /home/ubuntu/.vnc/passwd"
docker-compose restart ros2_vnc > /dev/null 2>&1
sleep 3

echo ""
echo "�� Service Status:"
docker-compose ps

echo ""
echo "🌐 Access URLs:"
echo "  ROS2 VNC Desktop: http://localhost:6080"
echo "  Web App Interface: http://localhost:5001"
echo ""
echo "✅ MuSoHu is ready!"
