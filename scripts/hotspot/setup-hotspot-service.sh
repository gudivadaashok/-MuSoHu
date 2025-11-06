#!/bin/bash

# Setup hotspot to start automatically on boot

echo "Setting up hotspot service to start on boot..."

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Copy service file to systemd directory
sudo cp "$SCRIPT_DIR/hotspot.service" /etc/systemd/system/

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
