#!/bin/bash

#***********************************************************************
# Script to install and configure TigerVNC server on Jetson devices
#***********************************************************************

# Source logging configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../utils/logging_config.sh"

log_info "Installing and configuring TigerVNC server on Jetson..."
log_info "-----------------------------------------------"

log_info "Updating package lists..."
sudo apt update

log_info "Installing TigerVNC server..."
sudo apt install tigervnc-standalone-server tigervnc-common -y

if [ $? -eq 0 ]; then
    log_success "TigerVNC server installed successfully"
else
    log_error "Failed to install TigerVNC server"
    exit 1
fi

log_info "Setting up VNC password..."
log_warning "You will be prompted to enter a VNC password"
vncpasswd

log_info "Creating VNC xstartup configuration..."
mkdir -p ~/.vnc

cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
exec startxfce4
EOF

chmod +x ~/.vnc/xstartup

log_info "Installing XFCE desktop environment (if not already installed)..."
sudo apt install xfce4 xfce4-goodies -y

log_info "Creating VNC service configuration..."
VNC_SERVICE="$HOME/.config/systemd/user/vncserver@.service"
mkdir -p "$HOME/.config/systemd/user"

cat > "$VNC_SERVICE" << EOF
[Unit]
Description=Remote desktop service (VNC)
After=syslog.target network.target

[Service]
Type=forking
ExecStartPre=/bin/sh -c '/usr/bin/vncserver -kill :%i > /dev/null 2>&1 || :'
ExecStart=/usr/bin/vncserver :%i -geometry 1920x1080 -depth 24
ExecStop=/usr/bin/vncserver -kill :%i

[Install]
WantedBy=default.target
EOF

log_success "VNC service configuration created"

log_info "Detecting host screen resolution..."
if command -v xdpyinfo &> /dev/null && [ -n "$DISPLAY" ]; then
    RESOLUTION=$(xdpyinfo | grep dimensions | awk '{print $2}')
    log_info "Detected resolution: $RESOLUTION"
else
    RESOLUTION="1920x1080"
    log_warning "Could not detect resolution, using default: $RESOLUTION"
fi

log_info "Updating VNC service with resolution: $RESOLUTION..."
sed -i "s/-geometry [0-9]*x[0-9]*/-geometry $RESOLUTION/" "$VNC_SERVICE"

log_info "Enabling VNC service for display :1..."
systemctl --user enable vncserver@1.service
systemctl --user start vncserver@1.service

if [ $? -eq 0 ]; then
    log_success "VNC server started successfully on display :1"
    log_info "VNC server is accessible on port 5901 with resolution $RESOLUTION"
    log_info "Connect using: <jetson-ip>:5901"
else
    log_error "Failed to start VNC server"
    exit 1
fi

log_separator
log_success "TigerVNC setup complete!"
log_info "Resolution configured: $RESOLUTION"
log_info "To manually start VNC: vncserver :1 -geometry $RESOLUTION -depth 24"
log_info "To stop VNC: vncserver -kill :1"
log_info "To check status: systemctl --user status vncserver@1.service"
log_separator
