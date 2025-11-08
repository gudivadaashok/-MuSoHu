# MuSoHu: Multi-Modal Social Human Navigation Dataset.
 
## Create a workspace and clone sources

## In terminal 1 :
```bash
roslaunch musohu_package musohu_suite.launch
```

## In terminal 2 :
```bash
python3 record.py
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