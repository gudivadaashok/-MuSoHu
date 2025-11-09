# Scripts Directory Structure

This directory contains scripts organized by category for the MuSoHu project.

## Directory Structure

```
scripts/
├── setup/              # Installation and setup scripts
│   └── install_novnc.sh
├── utils/              # Utility scripts and helpers
│   ├── logging_config.sh
│   └── example_with_logging.sh
└── logs/               # Log files (auto-generated, gitignored)
```

## Usage

### Setup Scripts
Scripts for installing and configuring system components:
- `setup/install_novnc.sh` - Install and configure Vino VNC server on Jetson

### Utility Scripts
Reusable utilities and helpers:
- `utils/logging_config.sh` - Logging configuration for bash scripts
- `utils/example_with_logging.sh` - Example showing logging usage

## Adding New Scripts

When adding new scripts:
1. Place them in the appropriate subdirectory
2. Make them executable: `chmod +x script_name.sh`
3. Add shebang: `#!/bin/bash`
4. Source logging if needed: `source "$(dirname "$0")/../utils/logging_config.sh"`
5. Document in this README

## Logging

All scripts can use the centralized logging configuration:

```bash
#!/bin/bash
source "$(dirname "$0")/../utils/logging_config.sh"

log_info "Starting script..."
log_success "Task completed!"
log_warning "Warning message"
log_error "Error occurred"
```

Logs are automatically saved to `logs/` with timestamps.
