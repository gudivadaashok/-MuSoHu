# WiFi Hotspot Setup Guide

This guide explains how to set up and manage the Robotixx_MuSoHu WiFi hotspot on your Jetson device.

## Network Credentials

![WiFi Hotspot](images/Wi-Fi_Hotspot.png)

- **SSID:** `Robotixx_MuSoHu`
- **Password:** `Robotixx`
- **Interface:** `wlP1p1s0`

## Manual Hotspot Start

Start the hotspot manually using the provided script:

```bash
# Start hotspot manually
./scripts/hotspot/start-hotspot.sh
```

Or use the direct nmcli command:

```bash
nmcli dev wifi hotspot ifname wlP1p1s0 ssid Robotixx_MuSoHu password Robotixx
```

To stop the hotspot:

```bash
nmcli connection down Hotspot
```

## Auto-Start Hotspot on Boot

To configure the hotspot to start automatically when the Jetson boots:

### 1. Install and Enable the Service

```bash
# Install and enable hotspot service to start automatically on boot
sudo bash /home/jetson/MuSoHu/scripts/hotspot/setup-hotspot-service.sh
```

This script will:
- Copy the systemd service file to `/etc/systemd/system/`
- Reload the systemd daemon
- Enable the service to start on boot

### 2. Start the Service Immediately

```bash
# Start the service immediately (without rebooting)
sudo systemctl start hotspot.service
```

### 3. Check Service Status

```bash
# Check service status
sudo systemctl status hotspot.service
```

## Service Management

Once the service is installed, you can manage it with these commands:

```bash
# Stop hotspot
sudo systemctl stop hotspot.service

# Start hotspot
sudo systemctl start hotspot.service

# Restart hotspot
sudo systemctl restart hotspot.service

# Disable auto-start on boot
sudo systemctl disable hotspot.service

# Enable auto-start on boot (if previously disabled)
sudo systemctl enable hotspot.service

# View logs
journalctl -u hotspot.service

# View real-time logs
journalctl -u hotspot.service -f
```

## Troubleshooting

### Check if Hotspot is Running

```bash
# Check active connections
nmcli connection show --active

# Check WiFi interface status
nmcli device status
```

### Check WiFi Interface Name

If `wlP1p1s0` doesn't work, find your WiFi interface name:

```bash
# List all network interfaces
ip link show

# Or use nmcli
nmcli device status
```

### View System Logs

```bash
# View service logs
journalctl -u hotspot.service

# View NetworkManager logs
journalctl -u NetworkManager
```

### Restart NetworkManager

If the hotspot fails to start:

```bash
sudo systemctl restart NetworkManager
```

## Files Location

- **Start Script:** `/home/jetson/MuSoHu/scripts/hotspot/start-hotspot.sh`
- **Setup Script:** `/home/jetson/MuSoHu/scripts/hotspot/setup-hotspot-service.sh`
- **Service File:** `/home/jetson/MuSoHu/scripts/hotspot/hotspot.service` (copied to `/etc/systemd/system/`)

## Advanced Configuration

To modify the hotspot settings, edit the start script:

```bash
nano /home/jetson/MuSoHu/scripts/hotspot/start-hotspot.sh
```

You can change:
- SSID name
- Password
- WiFi interface
- Additional network settings