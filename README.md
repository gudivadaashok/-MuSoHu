# MuSoHu: Multi-Modal Social Human Navigation:
 
Multi-Modal Social Human Navigation Dataset - A robotics research platform developed at George Mason University for collecting data on human-robot interactions in social navigation contexts.

Overview

MuSoHu is a platform that allows to collect data for the social navigation research project. It is built using ROS2 and provides a web interface for users to interact with the system.


## Create a workspace and clone sources

## In terminal 1 :
```bash
roslaunch musohu_package musohu_suite.launch
```

## In terminal 2 :
```bash
python3 record.py
```

## Configure Firewall for VNC and SSH Access

Before accessing noVNC, you need to configure the firewall to allow VNC and SSH connections:

```bash
# Allow SSH (port 22)
sudo ufw allow ssh

# Allow VNC ports
sudo ufw allow 5900:5901/tcp    # VNC server
sudo ufw allow 6080/tcp         # noVNC web interface

# Enable the firewall if not already enabled
sudo ufw enable

# Verify the rules
sudo ufw status
```

## Installation of NoVnc on Jetson Orin Nano

You can install noVNC on your Jetson Orin Nano using our installation script:

1. Make the script executable:
   ```bash
   chmod +x install_novnc.sh
   ```

2. Run the installation script:
   ```bash
   ./install_novnc.sh
   ```

3. Start the noVNC service:
   ```bash
   sudo systemctl start novnc.service
   ```

4. Access noVNC through your web browser:
   ```
   https://YOUR_JETSON_IP:6080/vnc.html
   ```

Note: Replace `YOUR_JETSON_IP` with your Jetson Orin Nano's IP address.

### Fixing Browser Issue on Jetson Orin Nano

If you encounter a browser issue on the Jetson Orin Nano, the installation script automatically handles this by installing the correct version of `snapd`. However, if you need to do it manually:

1. Download the required revision of `snapd`:
   ```bash
   snap download snapd --revision=24724
   ```

2. Acknowledge the downloaded snap package:
   ```bash
   sudo snap ack snapd_24724.assert
   ```

3. Install the snap package:
   ```bash
   sudo snap install snapd_24724.snap
   ```