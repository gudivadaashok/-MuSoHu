#!/bin/bash

#***********************************************************************
# Script to install and configure UFW firewall with automatic restart
#***********************************************************************

# Source logging configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../utils/logging_config.sh"

log_info "Installing and configuring UFW firewall..."
log_info "-----------------------------------------------"

log_info "Updating package lists..."
sudo apt update

log_info "Installing UFW..."
sudo apt install ufw -y

if [ $? -eq 0 ]; then
    log_success "UFW installed successfully"
else
    log_error "Failed to install UFW"
    exit 1
fi

log_info "Configuring UFW rules..."
log_info "Setting default policies (deny incoming, allow outgoing)..."
sudo ufw default deny incoming
sudo ufw default allow outgoing

log_info "Allowing SSH (port 22)..."
sudo ufw allow ssh

log_info "Allowing VNC (port 5901)..."
sudo ufw allow 5901/tcp

log_info "Allowing HTTP (port 80) and HTTPS (port 443)..."
sudo ufw allow http
sudo ufw allow https

log_success "UFW rules configured"

log_info "Creating UFW service override for auto-restart..."
sudo mkdir -p /etc/systemd/system/ufw.service.d

cat << 'EOF' | sudo tee /etc/systemd/system/ufw.service.d/override.conf
[Unit]
Description=Uncomplicated firewall with auto-restart
After=network.target

[Service]
# Restart on failure
Restart=on-failure
RestartSec=5s

# Also restart on abnormal exits
StartLimitInterval=60s
StartLimitBurst=3

[Install]
WantedBy=multi-user.target
EOF

if [ $? -eq 0 ]; then
    log_success "UFW service override created"
else
    log_error "Failed to create service override"
    exit 1
fi

log_info "Reloading systemd daemon..."
sudo systemctl daemon-reload

log_info "Enabling UFW service to start on boot..."
sudo systemctl enable ufw

log_info "Starting UFW service..."
sudo systemctl start ufw

log_info "Enabling UFW firewall..."
sudo ufw --force enable

if [ $? -eq 0 ]; then
    log_success "UFW firewall enabled and running"
else
    log_error "Failed to enable UFW firewall"
    exit 1
fi

log_separator
log_info "UFW Status:"
sudo ufw status verbose
log_separator

log_separator
log_success "UFW setup complete!"
log_info "Firewall is enabled and will start automatically on boot"
log_info "Service will automatically restart on failure"
log_info ""
log_info "Useful commands:"
log_info "  Check status: sudo ufw status verbose"
log_info "  Check service: sudo systemctl status ufw"
log_info "  Add rule: sudo ufw allow <port>/tcp"
log_info "  Delete rule: sudo ufw delete allow <port>/tcp"
log_info "  Reload firewall: sudo ufw reload"
log_separator
