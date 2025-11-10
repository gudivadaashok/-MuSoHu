#!/bin/bash
set -e

echo "Setting up MuSoHu Web Service Manager..."

# Install required packages
echo "Installing required packages..."
sudo apt-get update
sudo apt-get install -y python3-pip python3-venv

# Create virtual environment
echo "Creating Python virtual environment..."
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies
echo "Installing Python dependencies..."
pip install flask flask-cors

# Create logs directory
echo "Creating logs directory..."
mkdir -p logs

# Create systemd service
echo "Creating systemd service..."
cat << EOF | sudo tee /etc/systemd/system/musohu-web.service
[Unit]
Description=MuSoHu Web Service Manager
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$(pwd)
Environment=PATH=$(pwd)/venv/bin:/usr/local/bin:/usr/bin:/bin
ExecStart=$(pwd)/venv/bin/python3 app.py
Restart=always
StandardOutput=append:$(pwd)/logs/web_service.log
StandardError=append:$(pwd)/logs/web_service.log

[Install]
WantedBy=multi-user.target
EOF

# Set permissions
echo "Setting permissions..."
sudo chmod 644 /etc/systemd/system/musohu-web.service

# Reload systemd and enable service
echo "Configuring service..."
sudo systemctl daemon-reload
sudo systemctl enable musohu-web.service

echo "Starting service..."
sudo systemctl start musohu-web.service

echo "Setup complete!"
echo "The web service is now running on port 5001"
echo "You can manage the service using:"
echo "  sudo systemctl start musohu-web"
echo "  sudo systemctl stop musohu-web"
echo "  sudo systemctl restart musohu-web"
echo "  sudo systemctl status musohu-web"
echo "View logs with:"
echo "  tail -f logs/web_service.log"
