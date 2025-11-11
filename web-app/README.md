# Web Application Configuration

This directory contains the MuSoHu web application for managing ROS2 scripts, viewing logs, and monitoring system resources.

## Configuration File: config.yml

The `config.yml` file allows you to customize various aspects of the web application without modifying the code.

### Configuration Sections

#### 1. Logging Configuration

Control how the application logs are managed:

```yaml
logging:
  app_log:
    enabled: true                    # Enable/disable application logging
    path: logs/musohu.log           # Path to the log file
    level: INFO                      # Log level: DEBUG, INFO, WARNING, ERROR, CRITICAL
    max_size: 10485760              # Maximum log file size in bytes (10 MB)
    backup_count: 5                 # Number of backup log files to keep
    format: '[%(asctime)s] %(levelname)s: %(message)s'
    date_format: '%Y-%m-%d %H:%M:%S'
```

#### 2. Server Configuration

Configure the Flask server settings:

```yaml
server:
  host: 0.0.0.0      # Listen on all network interfaces
  port: 5001         # Port number for the web application
  debug: false       # Enable/disable debug mode
  threaded: true     # Enable/disable multi-threading
```

#### 3. Auto-Refresh Intervals

Set how frequently (in seconds) different views refresh:

```yaml
refresh_intervals:
  scripts: 5         # Scripts status refresh interval
  disk_space: 10     # Disk space information refresh interval
  logs: 5           # Log viewer refresh interval
```

#### 4. Log Viewer Settings

Configure the log viewer functionality:

```yaml
log_viewer:
  max_lines: 100    # Maximum number of log lines to display
  sources:
    - id: app
      name: Application Logs
      path: logs/musohu.log
      enabled: true
    - id: ros2
      name: ROS2 Logs
      path: src/install/helmet_bringup/share/helmet_bringup/config/ros2.log
      enabled: true
```

### Adding New Log Sources

To add a new log source to the viewer:

1. Edit `config.yml`
2. Add a new entry under `log_viewer.sources`:

```yaml
- id: my_custom_log
  name: My Custom Log
  path: path/to/my/log.log
  enabled: true
```

3. Restart the application
4. The new log source will appear in the dropdown on the logs page

### Changing Log Levels

To get more detailed logs, change the level from `INFO` to `DEBUG`:

```yaml
logging:
  app_log:
    level: DEBUG
```

Available log levels (from least to most verbose):
- `CRITICAL` - Only critical errors
- `ERROR` - Errors and critical issues
- `WARNING` - Warnings, errors, and critical issues
- `INFO` - General information (default)
- `DEBUG` - Detailed debugging information

### Installation

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Make sure the config.yml file exists in the web-app directory

3. Run the application:
   ```bash
   python app.py
   ```

### Notes

- If `config.yml` is not found, the application will use default values
- Log files and their parent directories will be created automatically if they don't exist
- Changes to most configuration values require restarting the application
- Invalid YAML syntax will cause the application to fall back to default settings

