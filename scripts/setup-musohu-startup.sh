#!/bin/bash

# Setup MuSoHu startup service to run on boot

echo "Setting up MuSoHu startup service..."

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Copy service file to systemd directory
sudo cp "$SCRIPT_DIR/musohu-startup.service" /etc/systemd/system/

# Reload systemd daemon
sudo systemctl daemon-reload

# Enable the service to start on boot
sudo systemctl enable musohu-startup.service

echo "[OK] MuSoHu startup service installed and enabled"
echo ""
echo "Commands:"
echo "  Start now:        sudo systemctl start musohu-startup.service"
echo "  Stop:             sudo systemctl stop musohu-startup.service"
echo "  Check status:     sudo systemctl status musohu-startup.service"
echo "  Disable on boot:  sudo systemctl disable musohu-startup.service"
echo "  View logs:        journalctl -u musohu-startup.service"
