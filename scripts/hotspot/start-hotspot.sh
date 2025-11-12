#!/bin/bash

#***********************************************************************
# Start Robotixx MuSoHu WiFi Hotspot
#***********************************************************************
# This script creates and starts a WiFi hotspot on the Jetson device
# Network Details:
#   - SSID: Robotixx_MuSoHu
#   - Password: Robotixx
#   - Interface: wlP1p1s0
#   - Gateway IP: 10.42.0.1
# The script also configures DNS for hostname-based access
#***********************************************************************

#***********************************************************************
# Help function
#***********************************************************************

show_help() {
    cat << EOF
Usage: bash start-hotspot.sh [OPTIONS]

Start the Robotixx MuSoHu WiFi hotspot.

This script:
  - Creates a WiFi hotspot on the Jetson device
  - Configures DNS for hostname resolution
  - Enables access to ROS2 VNC and Web Interface

Network Details:
  SSID:          Robotixx_MuSoHu
  Password:      Robotixx
  Interface:     wlP1p1s0
  Gateway IP:    10.42.0.1
  DHCP Range:    10.42.0.10 - 10.42.0.100

Access URLs (when connected to hotspot):
  Hostname-based:
    - http://robotixx:6080 (ROS2 VNC Desktop)
    - http://robotixx:5001 (Web Interface)
  IP-based:
    - http://10.42.0.1:6080 (ROS2 VNC Desktop)
    - http://10.42.0.1:5001 (Web Interface)

Management Commands:
  Stop hotspot:  nmcli connection down Hotspot
  Check status:  nmcli connection show --active

Options:
  -h, --help     Display this help message and exit

Examples:
  bash start-hotspot.sh
  bash start-hotspot.sh --help

Prerequisites:
  - Wireless interface wlP1p1s0 must be available
  - NetworkManager (nmcli) must be installed
  - DNS should be configured (optional but recommended)

Note: If hotspot is already running, it will be restarted.

EOF
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
    shift
done

# Get script directory and source utilities
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$(dirname "$SCRIPT_DIR")/utils/logging_config.sh"

log_info "Starting Robotixx_MuSoHu WiFi Hotspot..."
log_separator

#***********************************************************************
# Check for existing hotspot connection
#***********************************************************************

# Check if hotspot is already running
if nmcli connection show --active | grep -q "Hotspot"; then
    log_warning "Hotspot is already active. Stopping it first..."
    nmcli connection down Hotspot
    
    if [ $? -eq 0 ]; then
        log_success "Existing hotspot stopped"
    fi
fi

#***********************************************************************
# Start the WiFi hotspot
#***********************************************************************

log_info "Creating WiFi hotspot on interface wlP1p1s0..."
nmcli dev wifi hotspot ifname wlP1p1s0 ssid Robotixx_MuSoHu password Robotixx

if [ $? -eq 0 ]; then
    log_success "Hotspot started successfully!"
    
    echo ""
    log_separator
    log_info "Network Configuration"
    log_separator
    log_info "  SSID:      Robotixx_MuSoHu"
    log_info "  Password:  Robotixx"
    log_info "  Interface: wlP1p1s0"
    log_info "  Gateway:   10.42.0.1"
    echo ""
    
    #***********************************************************************
    # Configure DNS for hostname resolution
    #***********************************************************************
    
    # Wait for interface to be ready
    log_info "Waiting for interface to initialize..."
    sleep 2
    
    # Configure DNS if script is available
    if [ -f "$SCRIPT_DIR/configure-dns.sh" ]; then
        log_separator
        log_info "Configuring DNS for hostname resolution..."
        bash "$SCRIPT_DIR/configure-dns.sh"
    else
        log_warning "DNS configuration script not found"
        log_info "Hotspot will work but hostname resolution may not be available"
    fi
    
    #***********************************************************************
    # Display access information
    #***********************************************************************
    
    echo ""
    log_separator
    log_success "Hotspot is ready for connections!"
    log_separator
    echo ""
    
    log_info "Access URLs (when connected to hotspot):"
    echo ""
    log_info "  Hostname-based URLs:"
    log_info "    - http://robotixx:6080 (ROS2 VNC Desktop)"
    log_info "    - http://robotixx:5001 (Web Interface)"
    echo ""
    log_info "  IP-based URLs (alternative):"
    log_info "    - http://10.42.0.1:6080 (ROS2 VNC Desktop)"
    log_info "    - http://10.42.0.1:5001 (Web Interface)"
    echo ""
    
    log_separator
    log_info "Management Commands"
    log_separator
    log_info "  Stop hotspot:  nmcli connection down Hotspot"
    log_info "  Check status:  nmcli connection show --active"
    echo ""
    
else
    log_error "Failed to start hotspot"
    log_error "Please check if the wireless interface wlP1p1s0 is available"
    exit 1
fi
