#!/bin/bash

# Configure DNS for hotspot hostname resolution

echo "Configuring DNS for hotspot..."

# Install dnsmasq if not already installed
if ! command -v dnsmasq &> /dev/null; then
    echo "Installing dnsmasq..."
    sudo apt-get update
    sudo apt-get install -y dnsmasq
fi

# Stop dnsmasq temporarily
sudo systemctl stop dnsmasq

# Backup original dnsmasq config
sudo cp /etc/dnsmasq.conf /etc/dnsmasq.conf.backup 2>/dev/null || true

# Create dnsmasq.d directory if it doesn't exist
sudo mkdir -p /etc/dnsmasq.d

# Create dnsmasq configuration for hotspot
sudo tee /etc/dnsmasq.d/hotspot.conf > /dev/null <<EOF
# Hotspot DNS Configuration
interface=wlP1p1s0
dhcp-range=10.42.0.10,10.42.0.100,255.255.255.0,12h
dhcp-option=3,10.42.0.1
dhcp-option=6,10.42.0.1
server=8.8.8.8
server=8.8.4.4

# Local DNS records
address=/robotixx/10.42.0.1
address=/robotixx.local/10.42.0.1
EOF

# Enable and restart dnsmasq
sudo systemctl enable dnsmasq
sudo systemctl restart dnsmasq

echo "[OK] DNS configured for hotspot"
echo "Devices connected to hotspot can now access:"
echo "  - http://robotixx:6080 (ROS2 VNC)"
echo "  - http://robotixx:5001 (Web App)"
echo "  - http://10.42.0.1:6080 (Alternative IP access)"
echo "  - http://10.42.0.1:5001 (Alternative IP access)"
