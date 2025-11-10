#!/bin/bash

#***********************************************************************
# Setup Web Service for MuSoHu Script Management
#***********************************************************************
# This script sets up a Flask web service to manage and execute
# MuSoHu setup scripts (download drivers, configure devices, etc.)
#***********************************************************************

# Source logging configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../utils/logging_config.sh"

log_info "MuSoHu Web Service Setup"
log_separator

#***********************************************************************
# Check Python installation
#***********************************************************************

log_info "Checking Python installation..."
if ! command -v python3 &> /dev/null; then
    log_error "Python 3 is not installed"
    log_info "Install with: sudo apt-get install python3 python3-pip"
    exit 1
fi

PYTHON_VERSION=$(python3 --version | awk '{print $2}')
log_success "Python 3 found: $PYTHON_VERSION"

#***********************************************************************
# Check pip installation
#***********************************************************************

log_info "Checking pip installation..."
if ! command -v pip3 &> /dev/null; then
    log_error "pip3 is not installed"
    log_info "Install with: sudo apt-get install python3-pip"
    exit 1
fi

log_success "pip3 found"

#***********************************************************************
# Navigate to web-app directory
#***********************************************************************

log_info "Setting up web application..."
WEB_APP_DIR="$SCRIPT_DIR/../../web-app"
cd "$WEB_APP_DIR"
log_info "Working directory: $(pwd)"

if [ ! -f "requirements.txt" ]; then
    log_error "requirements.txt not found in $WEB_APP_DIR"
    exit 1
fi
log_success "Found requirements.txt"

#***********************************************************************
# Create virtual environment
#***********************************************************************

log_info "Creating Python virtual environment..."
VENV_DIR="$WEB_APP_DIR/venv"

if [ -d "$VENV_DIR" ]; then
    log_warning "Virtual environment already exists at: $VENV_DIR"
    log_info "Skipping creation"
else
    python3 -m venv "$VENV_DIR"
    if [ $? -eq 0 ]; then
        log_success "Virtual environment created at: $VENV_DIR"
    else
        log_error "Failed to create virtual environment"
        exit 1
    fi
fi

#***********************************************************************
# Activate virtual environment and install dependencies
#***********************************************************************

log_info "Activating virtual environment..."
source "$VENV_DIR/bin/activate"
log_success "Virtual environment activated"

log_info "Upgrading pip..."
pip install --upgrade pip
if [ $? -eq 0 ]; then
    log_success "pip upgraded"
else
    log_warning "Failed to upgrade pip (continuing anyway)"
fi

log_info "Installing Python dependencies..."
log_info "From: $WEB_APP_DIR/requirements.txt"
pip install -r requirements.txt
if [ $? -eq 0 ]; then
    log_success "Dependencies installed successfully"
else
    log_error "Failed to install dependencies"
    exit 1
fi

#***********************************************************************
# Create necessary directories
#***********************************************************************

log_info "Creating necessary directories..."

DIRS_TO_CREATE=(
    "instance"
    "logs"
    "uploads"
)

for dir in "${DIRS_TO_CREATE[@]}"; do
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        log_info "Created directory: $dir"
    else
        log_info "Directory already exists: $dir"
    fi
done

#***********************************************************************
# Create systemd service file
#***********************************************************************

log_info "Creating systemd service file..."
SERVICE_FILE="/etc/systemd/system/musohu-web.service"

cat << 'EOF' > /tmp/musohu-web.service
[Unit]
Description=MuSoHu Web Service for Script Management
After=network.target

[Service]
Type=notify
User=$USER
WorkingDirectory=$WEB_APP_DIR
Environment="PATH=$WEB_APP_DIR/venv/bin"
ExecStart=$WEB_APP_DIR/venv/bin/python app.py --host=0.0.0.0 --port=80
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# Replace variables in service file
sed -i "s|\$USER|$(whoami)|g" /tmp/musohu-web.service
sed -i "s|\$WEB_APP_DIR|$WEB_APP_DIR|g" /tmp/musohu-web.service

log_info "Service file created at: /tmp/musohu-web.service"
log_info "To install as system service (requires sudo):"
log_info "  sudo cp /tmp/musohu-web.service $SERVICE_FILE"
log_info "  sudo systemctl daemon-reload"
log_info "  sudo systemctl enable musohu-web"
log_info "  sudo systemctl start musohu-web"

#***********************************************************************
# Create startup script
#***********************************************************************

log_info "Creating startup script..."
STARTUP_SCRIPT="$WEB_APP_DIR/start_web_service.sh"

cat << 'EOF' > "$STARTUP_SCRIPT"
#!/bin/bash
# Start MuSoHu Web Service on Port 80

WEB_APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$WEB_APP_DIR/venv"

echo "Starting MuSoHu Web Service..."
echo "Web app directory: $WEB_APP_DIR"

if [ ! -d "$VENV_DIR" ]; then
    echo "Error: Virtual environment not found at $VENV_DIR"
    echo "Run setup_web_service.sh first"
    exit 1
fi

cd "$WEB_APP_DIR"
source "$VENV_DIR/bin/activate"

echo "Virtual environment activated"
echo "Starting Flask application on port 80..."
echo ""
echo "Web service will be available at:"
echo "  http://localhost"
echo ""
echo "Note: Port 80 requires root/sudo privileges"
echo "Press Ctrl+C to stop"
echo ""

python app.py --host=0.0.0.0 --port=80
EOF

chmod +x "$STARTUP_SCRIPT"
log_success "Startup script created: $STARTUP_SCRIPT"

#***********************************************************************
# Test Flask installation
#***********************************************************************

log_info "Testing Flask installation..."
python -c "import flask; print('Flask version:', flask.__version__)"
if [ $? -eq 0 ]; then
    log_success "Flask is properly installed"
else
    log_error "Flask installation test failed"
    exit 1
fi

#***********************************************************************
# Summary
#***********************************************************************

log_separator
log_success "Web Service Setup Complete!"
log_separator

log_info "Summary:"
log_info "  Python: $PYTHON_VERSION"
log_info "  Virtual Environment: $VENV_DIR"
log_info "  Web App Directory: $WEB_APP_DIR"
log_info "  Startup Script: $STARTUP_SCRIPT"
log_info ""

log_info "To start the web service:"
log_info "  1. Run: sudo bash $STARTUP_SCRIPT"
log_info "  2. Open browser: http://localhost"
log_info ""
log_info "Note: Port 80 requires sudo/root privileges"

log_info "To install as system service (requires sudo):"
log_info "  1. sudo cp /tmp/musohu-web.service /etc/systemd/system/"
log_info "  2. sudo systemctl daemon-reload"
log_info "  3. sudo systemctl enable musohu-web"
log_info "  4. sudo systemctl start musohu-web"
log_info ""

log_info "To check service status:"
log_info "  sudo systemctl status musohu-web"
log_info ""

log_info "To view service logs:"
log_info "  sudo journalctl -u musohu-web -f"
log_info ""

log_separator
log_info "Logs saved to: $LOG_FILE"
log_separator
