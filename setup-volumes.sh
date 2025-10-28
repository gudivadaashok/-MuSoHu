#!/bin/bash

# MuSoHu Docker Volumes Setup Script
echo "🗂️  Setting up Docker bind mount directories..."

# Define base directory
VOLUMES_DIR="$HOME/Desktop/Docker-Volumns"

# Create directory structure if it doesn't exist
echo "📁 Creating directory structure at $VOLUMES_DIR"
mkdir -p "$VOLUMES_DIR"/{app_logs,ros2_logs,user_data,ros2_workspace,app_data}

# Set appropriate permissions
echo "🔐 Setting directory permissions..."
chmod 755 "$VOLUMES_DIR"/{app_logs,ros2_logs,user_data,ros2_workspace,app_data}

# Create sample files to indicate what each directory is for
echo "📝 Creating directory documentation..."

cat > "$VOLUMES_DIR/README.md" << 'EOF'
# MuSoHu Docker Volumes

This directory contains bind-mounted volumes for the MuSoHu Docker containers.

## Directory Structure:

- **app_logs/** - Flask web application logs
- **ros2_logs/** - ROS2 system logs and debug output
- **user_data/** - User profiles and application data
- **ros2_workspace/** - ROS2 workspace with packages and builds
- **app_data/** - General application data and configuration

## Benefits:

✅ **Easy Access** - All container data accessible from Desktop
✅ **Persistent Storage** - Data survives container recreation
✅ **Development Friendly** - Direct file access for debugging
✅ **Backup Ready** - Easy to backup important data

## Usage:

The containers automatically mount these directories:
- Web App logs: `~/Desktop/Docker-Volumns/app_logs`
- ROS2 workspace: `~/Desktop/Docker-Volumns/ros2_workspace`
- User data: `~/Desktop/Docker-Volumns/user_data`
EOF

# Create .gitkeep files to preserve directory structure in git
for dir in app_logs ros2_logs user_data ros2_workspace app_data; do
    touch "$VOLUMES_DIR/$dir/.gitkeep"
    echo "# This directory is used for Docker bind mounts" > "$VOLUMES_DIR/$dir/README.md"
done

# Create specific documentation for each directory
cat > "$VOLUMES_DIR/app_logs/README.md" << 'EOF'
# App Logs Directory

Flask application logs are stored here.

Files you might see:
- `flask.log` - Main application log
- `error.log` - Error messages
- `access.log` - HTTP request logs
EOF

cat > "$VOLUMES_DIR/ros2_logs/README.md" << 'EOF'
# ROS2 Logs Directory

ROS2 system logs and debug output.

Files you might see:
- Node logs for running ROS2 processes
- Build logs from colcon
- Runtime diagnostics
EOF

cat > "$VOLUMES_DIR/user_data/README.md" << 'EOF'
# User Data Directory

User profiles and application-specific data.

Structure:
- User profiles
- Preferences
- Session data
EOF

cat > "$VOLUMES_DIR/ros2_workspace/README.md" << 'EOF'
# ROS2 Workspace Directory

ROS2 development workspace.

Structure:
- `src/` - Source packages
- `build/` - Build artifacts
- `install/` - Installed packages
- `log/` - Build logs
EOF

cat > "$VOLUMES_DIR/app_data/README.md" << 'EOF'
# App Data Directory

General application data and configuration.

Structure:
- Configuration files
- Database files
- Temporary data
EOF

echo "✅ Docker volumes setup complete!"
echo ""
echo "📁 Directory structure created at: $VOLUMES_DIR"
echo ""
echo "🗂️  Available directories:"
ls -la "$VOLUMES_DIR"
echo ""
echo "🚀 You can now start your Docker containers with:"
echo "   docker-compose up -d"
echo ""
echo "📊 Access your data directly from:"
echo "   Finder → Desktop → Docker-Volumns"