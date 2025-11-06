#!/bin/bash

# Setup hotspot to start automatically on boot

echo "Setting up hotspot service to start on boot..."

# Copy service file to systemd directory
sudo cp /home/jetson/MuSoHu/scripts/hotspot.service /etc/systemd/system/

# Reload systemd daemon
sudo systemctl daemon-reload

# Enable the service to start on boot
sudo systemctl enable hotspot.service

echo "[OK] Hotspot service installed and enabled"
echo ""
echo "Commands:"
echo "  Start now:        sudo systemctl start hotspot.service"
echo "  Stop:             sudo systemctl stop hotspot.service"
echo "  Check status:     sudo systemctl status hotspot.service"
echo "  Disable on boot:  sudo systemctl disable hotspot.service"
echo "  View logs:        journalctl -u hotspot.service"
