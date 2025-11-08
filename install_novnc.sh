#!/bin/bash
set -e

echo "Installing noVNC on Jetson Orin Nano..."

# Install required packages
echo "Installing required packages..."
sudo apt-get update
sudo apt-get install -y git python3-pip tigervnc-standalone-server tigervnc-common supervisor

# Clone noVNC repository
echo "Cloning noVNC repository..."
git clone https://github.com/novnc/noVNC.git ~/noVNC

# Install websockify
echo "Installing websockify..."
pip3 install websockify

# Create symbolic link for convenience
ln -s ~/noVNC/vnc.html ~/noVNC/index.html

# Create self-signed certificate for secure connections
echo "Creating self-signed certificate..."
cd ~/noVNC
openssl req -newkey rsa:2048 -x509 -days 365 -nodes -out self.pem -keyout self.pem -subj "/CN=$(hostname)"
chmod 600 self.pem

# Fix browser issues by installing specific snapd version
echo "Installing specific snapd version to fix browser issues..."
snap download snapd --revision=24724
sudo snap ack snapd_24724.assert
sudo snap install snapd_24724.snap

# Create a systemd service for noVNC
echo "Creating systemd service for noVNC..."
sudo tee /etc/systemd/system/novnc.service << EOF
[Unit]
Description=noVNC service
After=network.target

[Service]
Type=simple
User=$USER
ExecStart=/home/$USER/noVNC/utils/novnc_proxy --vnc localhost:5901 --cert /home/$USER/noVNC/self.pem
Restart=always

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd and enable the service
sudo systemctl daemon-reload
sudo systemctl enable novnc.service

echo "Installation complete!"
echo "To start the service, run: sudo systemctl start novnc.service"
echo "Access noVNC at: https://$(hostname):6080/vnc.html"
echo "Make sure your VNC server is running on port 5901 before accessing noVNC"
