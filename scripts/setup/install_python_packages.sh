#!/bin/bash

#***********************************************************************
# Python Packages Installation Script
#***********************************************************************
# This script installs Python and its essential packages for ZED SDK
# It handles both system packages and user-level pip installations safely
#***********************************************************************

#***********************************************************************
# Help function
#***********************************************************************

show_help() {
    cat << EOF
Usage: sudo bash install_python_packages.sh [OPTIONS]

Install Python and essential packages for ZED SDK and development.

This script:
  - Installs Python 3 and pip
  - Installs development tools (python3-dev, setuptools, wheel)
  - Installs scientific packages (numpy, opencv, matplotlib, scipy)
  - Verifies package installation
  - Provides guidance for user-level pip installations

System Packages Installed:
  Core:
    - python3, python3-pip, python3-dev
    - python3-setuptools, python3-wheel
  Scientific:
    - python3-numpy
    - python3-opencv
    - python3-matplotlib
    - python3-scipy

Options:
  -h, --help     Display this help message and exit

Examples:
  sudo bash install_python_packages.sh
  sudo bash install_python_packages.sh --help

User-Level Package Installation:
  For additional packages without sudo:
    pip install --user <package_name>
  
  Examples:
    pip install --user pandas
    pip install --user jupyter
    pip install --user tensorflow

Note: This script requires root privileges for system package installation.

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
source "$SCRIPT_DIR/../utils/logging_config.sh"

log_info "Starting Python packages installation..."
log_separator

#***********************************************************************
# Check if running with sudo for system packages
#***********************************************************************

if [ "$EUID" -ne 0 ]; then
    log_error "This script must be run with sudo for system package installation"
    log_info "Usage: sudo bash install_python_packages.sh"
    exit 1
fi

#***********************************************************************
# Install Python system packages
#***********************************************************************

log_info "Installing Python system packages..."

apt-get install --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    python3-numpy \
    python3-opencv \
    python3-matplotlib \
    python3-scipy \
    -y

if [ $? -eq 0 ]; then
    log_success "Python system packages installed successfully"
else
    log_error "Failed to install Python system packages"
    exit 1
fi

#***********************************************************************
# Verify Python packages installation
#***********************************************************************

log_info "Verifying Python packages installation..."

# Test core packages
if python3 -c "import numpy, cv2" 2>/dev/null; then
    log_success "Core Python packages (numpy, opencv) are available"
else
    log_warning "Core Python packages may not be properly installed"
fi

# Test additional packages
if python3 -c "import matplotlib, scipy" 2>/dev/null; then
    log_success "Additional Python packages (matplotlib, scipy) are available"
else
    log_warning "Additional Python packages may not be properly installed"
fi

#***********************************************************************
# Provide user-level pip guidance
#***********************************************************************

log_separator
log_info "Python packages installation completed!"

# Get the original user who called sudo (if available)
if [ -n "$SUDO_USER" ]; then
    log_info "User-level pip installation guidance for user '$SUDO_USER':"
    log_info "  For additional packages, use:"
    log_info "  sudo -u $SUDO_USER pip install --user <package_name>"
    log_info ""
    log_info "  Examples:"
    log_info "  sudo -u $SUDO_USER pip install --user pandas"
    log_info "  sudo -u $SUDO_USER pip install --user jupyter"
    log_info "  sudo -u $SUDO_USER pip install --user tensorflow"
else
    log_info "For additional Python packages, use:"
    log_info "  pip install --user <package_name>"
fi

log_separator
log_info "Python environment is ready for ZED SDK and development!"