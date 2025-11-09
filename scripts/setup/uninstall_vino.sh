#!/bin/bash

#***********************************************************************
# Script to remove Vino VNC server from Jetson devices
#***********************************************************************

# Source logging configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../utils/logging_config.sh"

log_info "Removing Vino VNC server from Jetson..."
log_info "-----------------------------------------------"

log_info "Disabling Vino VNC server..."
gsettings set org.gnome.Vino enabled false

log_info "Removing symbolic link for systemd service..."
sudo rm -f /usr/lib/systemd/user/graphical-session.target.wants/vino-server.service

if [ $? -eq 0 ]; then
    log_success "Symbolic link removed successfully"
else
    log_warning "Could not remove symbolic link (may not exist)"
fi

log_info "Stopping Vino service if running..."
systemctl --user stop vino-server.service 2>/dev/null || true

log_info "Uninstalling Vino VNC server..."
sudo apt remove vino -y
sudo apt autoremove -y

if [ $? -eq 0 ]; then
    log_success "Vino VNC server uninstalled successfully"
else
    log_error "Failed to uninstall Vino VNC server"
    exit 1
fi

log_info "Cleaning up Vino configuration files..."
rm -rf ~/.config/vino 2>/dev/null || true

log_separator
log_success "Vino VNC server removal complete!"
log_info "System is ready for alternative VNC server installation"
log_separator
