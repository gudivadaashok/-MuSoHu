# MuSoHu Docker Quick Start Guide

## 🚀 Quick Setup

### 1. Start Docker Desktop
Make sure Docker Desktop is running on your system.

### 2. Run the Application
```bash
# Option A: Use the startup script (recommended)
./start.sh

# Option B: Manual Docker Compose
docker-compose up -d --build
```

### 3. Access Applications
- **ROS2 VNC Desktop**: http://localhost:6080
- **Web App Interface**: http://localhost:5001

## 📋 What's Running

| Service | Port | Description |
|---------|------|-------------|
| ROS2 VNC | 6080 | Full ROS2 desktop environment with GUI |
| Web App | 5001 | Flask web interface for ROS2 script management |

## 🔧 Management Commands

```bash
# Start services
docker-compose up -d

# Stop services
docker-compose down

# View logs
docker-compose logs -f web_app
docker-compose logs -f ros2_vnc

# Restart specific service
docker-compose restart web_app

# Rebuild and restart
docker-compose up -d --build
```

## 🎯 Using the Applications

### ROS2 VNC Desktop (Port 6080)
1. Open http://localhost:6080 in browser
2. Enter VNC password: `robotixx`
3. Use full ROS2 desktop environment
4. Run ROS2 commands in terminal:
   ```bash
   ros2 run turtlesim turtlesim_node
   rviz2
   ```

### Web App Interface (Port 5001)
1. Open http://localhost:5001 in browser
2. Use web interface to:
   - Start/stop ROS2 scripts
   - Monitor running processes
   - Manage ROS2 applications

## 🔍 Troubleshooting

### Docker Not Running
```bash
# Start Docker Desktop manually
open -a Docker

# Wait for Docker to start, then run:
./start.sh
```

### Port Conflicts
If ports are already in use, modify `docker-compose.yml`:
```yaml
ports:
  - "6081:80"    # Change 6080 to 6081
  - "5002:5001"  # Change 5001 to 5002
```

### Container Issues
```bash
# Remove all containers and start fresh
docker-compose down --remove-orphans
docker-compose up -d --build
```

## 🌐 Network Architecture

```
Host Machine
├── Port 6080 → ROS2 VNC Container (VNC Desktop)
├── Port 5001 → Web App Container (Flask Interface)
└── Docker Network: ros2_network
    ├── ros2_vnc container
    └── web_app container
```

## 📁 File Structure

- `/workspace` - Mounted project directory in containers
- `ros2_script_manager/` - Web app source code
- `docker-compose.yml` - Service configuration
- `Dockerfile.webapp` - Web app container definition

## 🔐 Security Notes

- Default VNC password: `robotixx` (change in production)
- Web app runs in development mode
- Docker socket mounted for container management
- Local development only - not production ready

## 📚 Additional Resources

- [Docker Desktop](https://www.docker.com/products/docker-desktop)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Flask Documentation](https://flask.palletsprojects.com/)