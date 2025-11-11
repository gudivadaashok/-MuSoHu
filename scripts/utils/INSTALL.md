This document provides installation instructions for the MuSoHu project.

# Installation Instructions

## Installing VNC on Jetson Orin Nano
For headless VNC setup on Jetson Orin Nano, follow the detailed guide at:
https://mauroarcidiacono.github.io/jetson-headless-vnc/

## Installing ROS2 Humble on Ubuntu 22.04
https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html

## Install ZED SDK
Follow the instructions

## Install Jetson Orin CH341 USB to Serial Module Installation
https://www.youtube.com/watch?v=RHqSR3Wj_K0

## Logging Configuration

The project includes a comprehensive logging system located in `scripts/utils/logging_config.sh`. This provides the following log levels:

- **DEBUG**: Detailed diagnostic information (purple text)
- **INFO**: General informational messages (blue text)
- **SUCCESS**: Success confirmation messages (green text)
- **WARNING**: Warning messages (yellow text)
- **ERROR**: Error messages (red text)

### Debug Logging

Debug logging is available and can be controlled using the `DEBUG_ENABLED` environment variable:

```bash
# Enable debug logging in terminal
export DEBUG_ENABLED=true
./your_script.sh

# Or run with debug enabled for a single execution
DEBUG_ENABLED=true ./your_script.sh
```

Debug messages are always written to log files but only displayed in the terminal when `DEBUG_ENABLED=true`.

### Usage Examples

```bash
# Test the logging functionality
./scripts/utils/test_logging.sh

# Run with debug output visible
DEBUG_ENABLED=true ./scripts/utils/test_logging.sh

# View example usage
./scripts/utils/example_with_logging.sh
```

## Source Setup for ROS2 Workspace

## WEB UI Setup Instructions

## Create Hotspot on Jetson Orin Nano

## Configure the services to start on boot and crash recovery

gsettings set org.gnome.Vino authentication-methods "['vnc']"
gsettings set org.gnome.Vino vnc-password $(echo -n 'Robotixx'|base64)

