# MuSoHu

Multi-Modal Social Human Navigation Dataset - A robotics research platform developed at George Mason University.

## Overview

MuSoHu is a platform that allows to collect data for the social navigation research project. It is built using ROS2 and provides a web interface for users to interact with the system.

## Quick Start

For detailed setup and usage instructions, see our comprehensive guides:

- **🐳 [Docker Setup Guide](docs/DOCKER_GUIDE.md)** - Complete Docker containerization setup
- **📖 [Docker Setup Documentation](docs/docker-setup.md)** - Detailed Docker configuration and troubleshooting

[➡️ Read the full Docker Guide here](docs/DOCKER_GUIDE.md)

## Getting Started

### Prerequisites
- Docker Desktop
- Python 3.8+
- ROS2 Humble
- Web browser

### Quick Launch
```bash
# Clone the repository
git clone https://github.com/gudivadaashok/-MuSoHu.git
cd -MuSoHu

# Start with Docker (recommended)
./scripts/start.sh

# Or manually with Docker Compose
docker-compose up -d
```

### Access Applications
- **ROS2 VNC Desktop**: http://localhost:6080 (password: `ubuntu`, hostname: `robotixx`)
- **Web Interface**: http://localhost:5001 (hostname: `robotixx-webapp`)



