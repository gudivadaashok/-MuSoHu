#!/bin/bash

# Start Robotixx_MuSoHu WiFi Hotspot
# This script creates a WiFi hotspot on the Jetson device

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Starting Robotixx_MuSoHu WiFi Hotspot..."

# Check if hotspot is already running
if nmcli connection show --active | grep -q "Hotspot"; then
    echo "Hotspot is already active. Stopping it first..."
    nmcli connection down Hotspot
fi

# Start the hotspot
nmcli dev wifi hotspot ifname wlP1p1s0 ssid Robotixx_MuSoHu password Robotixx

if [ $? -eq 0 ]; then
    echo "[OK] Hotspot started successfully!"
    echo ""
    echo "Network Details:"
    echo "  SSID: Robotixx_MuSoHu"
    echo "  Password: Robotixx"
    echo "  Interface: wlP1p1s0"
    echo ""
    
    # Wait for interface to be ready
    sleep 2
    
    # Configure DNS for hostname resolution
    if [ -f "$SCRIPT_DIR/configure-dns.sh" ]; then
        echo "Configuring DNS..."
        bash "$SCRIPT_DIR/configure-dns.sh"
    fi
    
    echo ""
    echo "Access URLs (when connected to hotspot):"
    echo "  Using hostname:"
    echo "    - http://robotixx:6080 (ROS2 VNC Desktop)"
    echo "    - http://robotixx:5001 (Web Interface)"
    echo "  Using IP address:"
    echo "    - http://10.42.0.1:6080 (ROS2 VNC Desktop)"
    echo "    - http://10.42.0.1:5001 (Web Interface)"
    echo ""
    echo "To stop the hotspot, run:"
    echo "  nmcli connection down Hotspot"
else
    echo "[ERROR] Failed to start hotspot"
    exit 1
fi
