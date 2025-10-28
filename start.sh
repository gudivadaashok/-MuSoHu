#!/bin/bash

# MuSoHu Project Startup Script
echo "🤖 Starting MuSoHu - Multi-Modal Social Human Navigation Dataset 🤖"
echo "=================================================="

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Error: Docker is not running."
    echo "📋 Please start Docker Desktop first:"
    echo "   1. Open Docker Desktop application"
    echo "   2. Wait for Docker to start (green light in menu bar)"
    echo "   3. Run this script again"
    echo ""
    echo "💡 Alternative: Start Docker Desktop automatically"
    echo "   open -a Docker"
    exit 1
fi

# Check if docker-compose is available
if ! command -v docker-compose > /dev/null 2>&1; then
    echo "❌ Error: docker-compose is not installed."
    echo "Please install docker-compose first."
    exit 1
fi

echo "�️  Setting up Docker volumes..."
./setup-volumes.sh > /dev/null 2>&1

echo "�🔧 Building and starting services..."

# Build and start services
docker-compose up -d --build

# Wait a moment for services to start
echo "⏳ Waiting for services to initialize..."
sleep 5

# Set VNC password
echo "🔐 Setting VNC password..."
docker exec ros2_vnc bash -c "mkdir -p /home/ubuntu/.vnc && echo 'robotixx' | vncpasswd -f > /home/ubuntu/.vnc/passwd && chmod 600 /home/ubuntu/.vnc/passwd && chown ubuntu:ubuntu /home/ubuntu/.vnc/passwd"
docker-compose restart ros2_vnc > /dev/null 2>&1
sleep 3

# Check service status
echo ""
echo "📊 Service Status:"
docker-compose ps

echo ""
echo "🌐 Access URLs:"
echo "  ROS2 VNC Desktop: http://localhost:6080"
echo "  Web App Interface: http://localhost:5001"
echo ""
echo "🔧 Management Commands:"
echo "  View logs:        docker-compose logs -f"
echo "  Stop services:    docker-compose down"
echo "  Restart:          docker-compose restart"
echo ""
echo "✅ MuSoHu is ready! Open the URLs above in your browser."

# Optional: Open URLs automatically (uncomment if desired)
# sleep 2
# open http://localhost:6080
# open http://localhost:5001