#!/bin/bash

# Start Robotixx_MuSoHu WiFi Hotspot
# This script creates a WiFi hotspot on the Jetson device

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
    echo "To stop the hotspot, run:"
    echo "  nmcli connection down Hotspot"
else
    echo "[ERROR] Failed to start hotspot"
    exit 1
fi
