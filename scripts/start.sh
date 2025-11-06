#!/bin/bash

# MuSoHu Project Startup Script
echo "🤖 Starting MuSoHu - Multi-Modal Social Human Navigation Dataset 🤖"
echo "=================================================="

# Detect OS
OS_TYPE=$(uname -s)

# Function to install Docker
install_docker() {
    echo "🔧 Installing Docker..."
    if [ "$OS_TYPE" = "Linux" ]; then
        # Update package index
        sudo apt-get update
        # Install prerequisites
        sudo apt-get install -y ca-certificates curl gnupg lsb-release
        # Add Docker's official GPG key
        sudo mkdir -p /etc/apt/keyrings
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
        # Set up repository
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
        # Install Docker Engine
        sudo apt-get update
        sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
        # Add current user to docker group
        sudo usermod -aG docker $USER
        echo "✅ Docker installed successfully"
        echo "⚠️  Please log out and log back in for group changes to take effect"
        echo "    Or run: newgrp docker"
    else
        echo "❌ Automatic Docker installation is only supported on Linux"
        echo "   Please install Docker Desktop from: https://www.docker.com/products/docker-desktop"
        exit 1
    fi
}

# Check if Docker is installed
if ! command -v docker > /dev/null 2>&1; then
    echo "⚠️  Docker is not installed"
    read -p "Do you want to install Docker? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        install_docker
    else
        echo "❌ Docker is required to run MuSoHu"
        exit 1
    fi
fi

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "⚠️  Docker is not running"
    if [ "$OS_TYPE" = "Linux" ]; then
        echo "🔧 Starting Docker service..."
        sudo systemctl start docker
        sudo systemctl enable docker
        sleep 2
        if ! docker info > /dev/null 2>&1; then
            echo "❌ Failed to start Docker. Please start it manually: sudo systemctl start docker"
            exit 1
        fi
        echo "✅ Docker is now running"
    else
        echo "❌ Please start Docker Desktop manually"
        exit 1
    fi
fi

# Check if docker compose is available
if ! docker compose version > /dev/null 2>&1; then
    echo "⚠️  Docker Compose plugin is not installed"
    if [ "$OS_TYPE" = "Linux" ]; then
        echo "🔧 Installing Docker Compose plugin..."
        sudo apt-get update
        sudo apt-get install -y docker-compose-plugin
        if ! docker compose version > /dev/null 2>&1; then
            echo "❌ Failed to install Docker Compose plugin"
            exit 1
        fi
        echo "✅ Docker Compose plugin installed successfully"
    else
        echo "❌ Docker Compose should be included with Docker Desktop"
        exit 1
    fi
fi

# Check OS and display info
if [ "$OS_TYPE" = "Darwin" ]; then
    echo ""
    echo "ℹ️  Running on macOS"
    echo "   GPU and device passthrough not available"
    echo ""
elif [ "$OS_TYPE" = "Linux" ]; then
    echo "✅ Running on Linux - hardware access enabled"
    
    if command -v nvidia-smi > /dev/null 2>&1; then
        echo "🎮 NVIDIA GPU detected"
    fi
    
    echo "   All hardware devices configured in docker-compose.yml"
fi

# Remove any existing override file (all config is in main docker-compose.yml)
rm -f docker-compose.override.yml

# Create bind mount directories
if [ ! -d "$HOME/Desktop/Docker-Volumns" ]; then
    ./scripts/setup-volumes.sh > /dev/null 2>&1
fi

echo "🔧 Building and starting services..."
docker compose up -d --build

echo "⏳ Waiting for services to initialize..."
sleep 5

echo "🔐 Setting VNC password..."
docker exec ros2_vnc bash -c "mkdir -p /home/ubuntu/.vnc && echo 'robotixx' | vncpasswd -f > /home/ubuntu/.vnc/passwd && chmod 600 /home/ubuntu/.vnc/passwd && chown ubuntu:ubuntu /home/ubuntu/.vnc/passwd"
docker compose restart ros2_vnc > /dev/null 2>&1
sleep 3

echo ""
echo "📊 Service Status:"
docker compose ps

echo ""
echo "🌐 Access URLs:"
echo "  ROS2 VNC Desktop: http://localhost:6080"
echo "  Web App Interface: http://localhost:5001"
echo ""
echo "✅ MuSoHu is ready!"
