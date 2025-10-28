# 📁 Docker Volumes Configuration - MuSoHu

## 🗂️ Bind Mount Setup

Your MuSoHu project now uses **bind mounts** to the `~/Desktop/Docker-Volumns` directory for easy access to all container data.

### 📊 Volume Mapping

| Container Path | Host Path | Purpose |
|---------------|-----------|---------|
| `/app/logs` | `~/Desktop/Docker-Volumns/app_logs` | Flask application logs |
| `/app/data` | `~/Desktop/Docker-Volumns/app_data` | General app data |
| `/app/users` | `~/Desktop/Docker-Volumns/user_data` | User profiles & data |
| `/home/ubuntu/ros2_ws` | `~/Desktop/Docker-Volumns/ros2_workspace` | ROS2 workspace |
| `/home/ubuntu/.ros/log` | `~/Desktop/Docker-Volumns/ros2_logs` | ROS2 system logs |

---

## 🚀 Benefits

### ✅ **Easy Access**
- All container data accessible from **Desktop/Docker-Volumns**
- **Finder integration** - browse files directly
- **No Docker commands** needed to access data

### ✅ **Development Friendly**
- **Real-time log monitoring**
- **Direct file editing** when needed
- **Easy debugging** access

### ✅ **Data Persistence**
- **Survives container recreation**
- **Easy backup** - just copy the folder
- **Version control friendly** with .gitkeep files

### ✅ **Performance**
- **Native filesystem performance**
- **No Docker overhead** for file operations
- **Direct disk access**

---

## 📱 Usage Examples

### **Monitor Logs in Real-time:**
```bash
# Watch Flask app logs
tail -f ~/Desktop/Docker-Volumns/app_logs/musohu_dev.log

# Monitor ROS2 logs
ls ~/Desktop/Docker-Volumns/ros2_logs/
```

### **Access ROS2 Workspace:**
```bash
# View ROS2 packages
ls ~/Desktop/Docker-Volumns/ros2_workspace/src/

# Check build outputs
ls ~/Desktop/Docker-Volumns/ros2_workspace/build/
```

### **Manage User Data:**
```bash
# Browse user uploads
open ~/Desktop/Docker-Volumns/user_data/
```

---

## 🔧 Docker Compose Configuration

```yaml
volumes:
  # Development files (bind mount)
  - .:/workspace

  # Web App data (bind mounts to Desktop)
  - ~/Desktop/Docker-Volumns/app_logs:/app/logs
  - ~/Desktop/Docker-Volumns/app_data:/app/data
  - ~/Desktop/Docker-Volumns/user_data:/app/users

  # ROS2 data (bind mounts to Desktop)
  - ~/Desktop/Docker-Volumns/ros2_workspace:/home/ubuntu/ros2_ws
  - ~/Desktop/Docker-Volumns/ros2_logs:/home/ubuntu/.ros/log
```

---

## 📋 Directory Structure

```
~/Desktop/Docker-Volumns/
├── README.md              # This documentation
├── app_logs/              # Flask application logs
│   ├── musohu.log        # Production logs
│   └── musohu_dev.log    # Development logs
├── app_data/              # Application data & config
├── user_data/             # User profiles & preferences
├── ros2_workspace/        # ROS2 development workspace
│   ├── src/              # Source packages
│   ├── build/            # Build artifacts
│   ├── install/          # Installed packages
│   └── log/              # Build logs
└── ros2_logs/             # ROS2 runtime logs
```

---

## 🛠️ Management Commands

### **Setup & Initialization:**
```bash
# Initialize volume directories
./scripts/setup-volumes.sh

# Start with volume setup
./scripts/start.sh
```

### **Container Operations:**
```bash
# Restart containers (data persists)
docker-compose restart

# Rebuild containers (data persists)
docker-compose up -d --build

# Clean restart (data persists)
docker-compose down && docker-compose up -d
```

### **Data Operations:**
```bash
# Backup all data
cp -r ~/Desktop/Docker-Volumns ~/Desktop/MuSoHu-Backup-$(date +%Y%m%d)

# Clean logs (keep structure)
find ~/Desktop/Docker-Volumns -name "*.log" -delete

# Reset workspace (careful!)
rm -rf ~/Desktop/Docker-Volumns/ros2_workspace/*
```

---

## 🎯 Quick Access

### **From Finder:**
1. Open **Finder**
2. Go to **Desktop**
3. Open **Docker-Volumns** folder
4. Browse your container data!

### **From Terminal:**
```bash
# Quick navigation
cd ~/Desktop/Docker-Volumns

# Open in Finder
open ~/Desktop/Docker-Volumns
```

Your MuSoHu project data is now easily accessible and persistent! 🎵📁🤖