#!/bin/bash

#***********************************************************************
# Script to install and configure Vino VNC server on Jetson devices
#***********************************************************************

# Source logging configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../utils/logging_config.sh"

log_info "Installing and configuring Vino VNC server on Jetson..."
log_info "-----------------------------------------------"

log_info "Installing Vino VNC server..."
sudo apt update
sudo apt install vino -y

if [ $? -eq 0 ]; then
    log_success "Vino VNC server installed successfully"
else
    log_error "Failed to install Vino VNC server"
    exit 1
fi

log_info "Creating symbolic link for systemd service..."
sudo ln -s ../vino-server.service /usr/lib/systemd/user/graphical-session.target.wants

log_info "Configuring VNC server..."
gsettings set org.gnome.Vino prompt-enabled false
gsettings set org.gnome.Vino require-encryption false
gsettings set org.gnome.Vino enabled true

log_success "Configuration complete. Rebooting system..."
sudo reboot