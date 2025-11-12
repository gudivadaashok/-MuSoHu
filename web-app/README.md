# Web Application Configuration

This directory contains the MuSoHu web application for managing ROS2 scripts, viewing logs, and monitoring system resources.

## Technology Stack

- **Framework**: FastAPI (modern, fast async web framework)
- **Server**: Uvicorn (ASGI server for production)
- **Templating**: Jinja2
- **Configuration**: YAML-based config file

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

Configure the FastAPI/Uvicorn server settings:

```yaml
server:
  host: 0.0.0.0      # Listen on all network interfaces
  port: 80           # Port number for the web application (requires sudo)
  debug: false       # Enable/disable debug mode
  threaded: true     # Enable/disable multi-threading
```

**Note**: Port 80 requires root/sudo privileges to run.

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
  allowed_extensions:
    - .log
    - .txt
  scan_directories:
    - path: ../logs/
      enabled: true
      name_prefix: ""
    - path: logs/
      enabled: true
      name_prefix: "WebApp - "
  sources:
    - id: app
      name: Application Logs
      path: logs/musohu.log
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

### Installation & Running

#### Development Mode (Recommended for Testing)

1. **Navigate to the web-app directory**:
   ```bash
   cd /path/to/MuSoHu/web-app
   ```

2. **Set up Python virtual environment** (if not already done):
   ```bash
   # The project uses a virtual environment in the parent directory
   # It should already exist at ../venv/
   # If not, create it:
   cd ..
   python3 -m venv .venv
   cd web-app
   ```

3. **Install dependencies**:
   ```bash
   # Activate the virtual environment
   source ../.venv/bin/activate
   
   # Install required packages
   pip install -r requirements.txt
   ```

4. **Run the development server**:
   ```bash
   # Option 1: Using the virtual environment (recommended)
   ../.venv/bin/uvicorn app:app --host 0.0.0.0 --port 8000 --reload
   
   # Option 2: If venv is activated
   uvicorn app:app --host 0.0.0.0 --port 8000 --reload
   ```

5. **Access the application**:
   - Open your browser to: `http://localhost:8000`
   - API documentation: `http://localhost:8000/docs`
   - Alternative docs: `http://localhost:8000/redoc`

**Development Mode Features**:
- `--reload`: Auto-reloads on code changes
- Port 8000: No sudo required
- Detailed error messages and stack traces
- Interactive API documentation at `/docs`

**Stopping the server**: Press `Ctrl+C` in the terminal

#### Production Mode

For production deployment with automatic restart and systemd integration:

```bash
# For production deployment, see production setup guide
sudo bash ../scripts/setup/setup_production_web_service.sh
```

**Production Mode Features**:
- Runs on port 80 (standard HTTP)
- Automatic restart on failure
- Systemd integration with logging
- Resource limits and security hardening
- Auto-start on system boot

See [PRODUCTION_WEB_SERVICE.md](../docs/guides/PRODUCTION_WEB_SERVICE.md) for complete production setup documentation.

### Installation

### API Endpoints

The application provides both HTML pages and REST API endpoints:

#### Pages
- `GET /` - Main dashboard (scripts management)
- `GET /disk-space` - Disk space monitoring
- `GET /logs` - Log viewer

#### API Endpoints
- `GET /api/health` - Health check endpoint
- `GET /api/scripts` - List all available scripts
- `POST /api/scripts/{script_id}/start` - Start a script
- `POST /api/scripts/{script_id}/stop` - Stop a script
- `GET /api/disk-space` - Get disk space information
- `GET /api/logs` - Get log entries (accepts `?source=log_id` parameter)
- `GET /api/logs/sources` - Get available log sources
- `GET /api/logs/download` - Download a log file (accepts `?source=log_id` parameter)

### Notes

- If `config.yml` is not found, the application will use default values
- Log files and their parent directories will be created automatically if they don't exist
- Changes to most configuration values require restarting the application
- Invalid YAML syntax will cause the application to fall back to default settings
- Port 80 provides standard HTTP access without needing to specify a port number
- FastAPI provides automatic API documentation at `/docs` and `/redoc`

