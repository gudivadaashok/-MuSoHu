#!/bin/bash

# MuSoHu Docker Volumes Setup Script
echo "🗂️  Setting up Docker bind mount directories..."

# Define base directory
VOLUMES_DIR="$HOME/Desktop/Docker-Volumns"

# Check if directory already exists
if [ -d "$VOLUMES_DIR" ]; then
    echo "📁 Directory already exists at $VOLUMES_DIR"
    echo "ℹ️  Skipping creation to preserve existing data"
    echo "✅ Setup complete - using existing volumes"
    exit 0
fi

# Create directory structure if it doesn't exist
echo "📁 Creating directory structure at $VOLUMES_DIR"
mkdir -p "$VOLUMES_DIR"/{app_logs,ros2_logs,user_data,ros2_workspace,app_data}

# Set appropriate permissions
echo "🔐 Setting directory permissions..."
chmod 755 "$VOLUMES_DIR"/{app_logs,ros2_logs,user_data,ros2_workspace,app_data}

# Create sample files to indicate what each directory is for
echo "📝 Creating directory documentation..."

cat > "$VOLUMES_DIR/README.md" << 'EOF'
# MuSoHu Docker Volumes Directory

This directory contains persistent storage for the MuSoHu Docker containers, using **bind mounts** to provide easy access to container data from your Desktop.

## 📁 Directory Structure

```
~/Desktop/Docker-Volumns/
├── README.md              # This file
├── app_logs/              # Flask web application logs
│   ├── musohu.log        # Production logs
│   └── musohu_dev.log    # Development logs
├── app_data/              # Application data & configuration
├── user_data/             # User profiles & preferences
├── ros2_workspace/        # ROS2 development workspace
│   ├── src/              # Source packages
│   ├── build/            # Build artifacts
│   ├── install/          # Installed packages
│   └── log/              # Build logs
└── ros2_logs/             # ROS2 runtime logs
```

## 🔄 How to Recreate This Directory

If you need to recreate this directory structure (e.g., after deletion or on a new machine), follow these steps:

### Method 1: Automatic Setup (Recommended)

From the MuSoHu project root directory:

```bash
# Navigate to project directory
cd ~/Git/GMU/-MuSoHu

# Run the setup script
./scripts/setup-volumes.sh
```

This will automatically create all required directories with proper permissions and README files.

### Method 2: Using Docker Compose

The directory structure will be automatically created when you start the containers:

```bash
# Navigate to project directory
cd ~/Git/GMU/-MuSoHu

# Start containers (creates volumes if missing)
./scripts/start.sh
```

### Method 3: Manual Creation

If you prefer to create the structure manually:

```bash
# Create base directory
mkdir -p ~/Desktop/Docker-Volumns

# Create all subdirectories
mkdir -p ~/Desktop/Docker-Volumns/{app_logs,ros2_logs,user_data,ros2_workspace,app_data}

# Create ROS2 workspace subdirectories
mkdir -p ~/Desktop/Docker-Volumns/ros2_workspace/{src,build,install,log}

# Set proper permissions
chmod -R 755 ~/Desktop/Docker-Volumns
```

## 📊 Volume Mappings

| Container Path | Host Path | Purpose |
|---------------|-----------|---------|
| `/app/logs` | `~/Desktop/Docker-Volumns/app_logs` | Flask application logs |
| `/app/data` | `~/Desktop/Docker-Volumns/app_data` | General app data |
| `/app/users` | `~/Desktop/Docker-Volumns/user_data` | User profiles & data |
| `/home/ubuntu/ros2_ws` | `~/Desktop/Docker-Volumns/ros2_workspace` | ROS2 workspace |
| `/home/ubuntu/.ros/log` | `~/Desktop/Docker-Volumns/ros2_logs` | ROS2 system logs |

## 🎯 Common Operations

### View Logs in Real-time
```bash
# Watch Flask application logs
tail -f ~/Desktop/Docker-Volumns/app_logs/musohu_dev.log

# Monitor ROS2 logs
ls ~/Desktop/Docker-Volumns/ros2_logs/
```

### Access ROS2 Workspace
```bash
# View ROS2 packages
ls ~/Desktop/Docker-Volumns/ros2_workspace/src/

# Check build outputs
ls ~/Desktop/Docker-Volumns/ros2_workspace/build/
```

### Backup All Data
```bash
# Create timestamped backup
cp -r ~/Desktop/Docker-Volumns ~/Desktop/MuSoHu-Backup-$(date +%Y%m%d-%H%M%S)
```

### Clean Logs
```bash
# Remove all log files (keeps directory structure)
find ~/Desktop/Docker-Volumns -name "*.log" -delete
```

### Reset Workspace
```bash
# ⚠️ WARNING: This deletes all workspace data
rm -rf ~/Desktop/Docker-Volumns/ros2_workspace/*
```

## 🔍 Troubleshooting

### Permission Issues
If you encounter permission errors:

```bash
# Fix permissions for all volumes
chmod -R 755 ~/Desktop/Docker-Volumns

# Or fix specific directory
chmod -R 755 ~/Desktop/Docker-Volumns/ros2_workspace
```

### Missing Directories
If containers can't access volumes:

```bash
# Recreate all directories
./scripts/setup-volumes.sh

# Restart containers
docker-compose restart
```

### Disk Space
Check volume disk usage:

```bash
# Check size of all volumes
du -sh ~/Desktop/Docker-Volumns/*

# Check specific directory
du -sh ~/Desktop/Docker-Volumns/ros2_workspace
```

## 📝 Important Notes

- **Data Persistence**: All data in this directory persists even when containers are stopped or removed
- **Easy Access**: Files can be browsed directly from Finder or Terminal
- **Version Control**: Do NOT commit this directory to Git (already in .gitignore)
- **Backup**: Regular backups recommended for important data
- **Permissions**: Directory permissions are set to 755 by default

## 🚀 Quick Access

### From Finder
1. Open **Finder**
2. Navigate to **Desktop**
3. Open **Docker-Volumns** folder

### From Terminal
```bash
# Quick navigation
cd ~/Desktop/Docker-Volumns

# Open in Finder
open ~/Desktop/Docker-Volumns
```

## 📚 Related Documentation

- [Docker Setup Guide](../../Git/GMU/-MuSoHu/docs/DOCKER_GUIDE.md)
- [Volumes Configuration](../../Git/GMU/-MuSoHu/docs/VOLUMES_GUIDE.md)
- [Main README](../../Git/GMU/-MuSoHu/README.md)

## 💡 Tips

1. **Monitor Logs**: Use `tail -f` for real-time log viewing
2. **Disk Space**: Periodically clean old logs to free up space
3. **Backups**: Create backups before major updates
4. **IDE Integration**: Point your IDE to `ros2_workspace` for ROS2 development

---

**Project**: MuSoHu - Multi-Modal Social Human Navigation Dataset  
**Institution**: George Mason University  
**Lab**: [RobotiXX Lab](https://robotixx.cs.gmu.edu/index.html)
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