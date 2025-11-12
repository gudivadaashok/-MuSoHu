"""
MuSoHu Web Application
Manages ROS2 scripts, displays logs, and monitors system resources
"""
from fastapi import FastAPI, Request, Query
from fastapi.responses import JSONResponse, FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
import subprocess
import os
import shutil
import logging
from datetime import datetime
import yaml

# Get the base directory of this file
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Setup logger first
logger = logging.getLogger("uvicorn")

# Prepare paths
static_dir = os.path.join(BASE_DIR, "static")
templates_dir = os.path.join(BASE_DIR, "templates")

if not os.path.exists(static_dir):
    logger.error(f"Static directory not found: {static_dir}")
if not os.path.exists(templates_dir):
    logger.error(f"Templates directory not found: {templates_dir}")

# Initialize FastAPI app
app = FastAPI()

# Add middleware FIRST
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Setup templates
templates = Jinja2Templates(directory=templates_dir)
logger.info(f"Templates directory: {templates_dir}")

# Mount static files LAST - this is critical for proper route registration
try:
    app.mount("/static", StaticFiles(directory=static_dir), name="static")
    logger.info(f"Successfully mounted static files from: {static_dir}")
except Exception as e:
    logger.error(f"Failed to mount static files: {e}")

# Load configuration
def load_config():
    """Load configuration from config.yml"""
    config_path = os.path.join(BASE_DIR, 'config.yml')
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            logger.error(f"Error loading config.yml: {e}")
            return get_default_config()
    else:
        logger.warning("config.yml not found, using default configuration")
        return get_default_config()

def get_default_config():
    """Return default configuration"""
    return {
        'log_viewer': {
            'max_lines': 100,
            'scan_directories': [],
            'sources': [],
            'allowed_extensions': ['.log', '.txt']
        },
        'server': {
            'host': '0.0.0.0',
            'port': 8000
        },
        'refresh_intervals': {
            'scripts': 5,
            'disk_space': 10,
            'logs': 5
        }
    }

# Load configuration
config = load_config()

# Define available ROS2 scripts
ROS2_SCRIPTS = {
    'helmet_nodes': {
        'command': 'ros2 launch helmet_bringup helmet.launch.py',
        'description': 'MuSoHu Helmet Nodes',
        'status': 'stopped',
        'pid': None
    },
    'rviz2': {
        'command': 'ros2 run rviz2 rviz2',
        'description': 'RViz2 Visualization',
        'status': 'stopped',
        'pid': None
    }
}

running_processes = {}

# ---------------------------- Page Routes -----------------------------

@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    """Main page - Scripts management"""
    refresh_interval = config.get('refresh_intervals', {}).get('scripts', 5)
    return templates.TemplateResponse("index.html", {
        "request": request,
        "refresh_interval": refresh_interval
    })


@app.get("/disk-space", response_class=HTMLResponse)
async def disk_space(request: Request):
    """Disk space monitoring page"""
    refresh_interval = config.get('refresh_intervals', {}).get('disk_space', 10)
    return templates.TemplateResponse("disk_space.html", {
        "request": request,
        "refresh_interval": refresh_interval
    })


@app.get("/logs", response_class=HTMLResponse)
async def logs(request: Request):
    """Log viewer page"""
    refresh_interval = config.get('refresh_intervals', {}).get('logs', 5)
    return templates.TemplateResponse("logs.html", {
        "request": request,
        "refresh_interval": refresh_interval
    })


# ============================================================================
# API Routes - Health Check
# ----------------------- API Routes - Health Check --------------------------
@app.get("/api/health")
async def health_check():
    """Health check endpoint for monitoring"""
    return JSONResponse(content={
        'status': 'healthy',
        'running_scripts': len(running_processes),
        'timestamp': datetime.now().isoformat()
    })


# ============================================================================
# API Routes - Scripts
# ============================================================================

# ------------------------- API Routes - Scripts -----------------------------

@app.get("/api/scripts")
async def list_scripts():
    """List all available ROS2 scripts and their status"""
    # Update status based on actual process state
    for script_id in list(running_processes):
        process = running_processes[script_id]
        if process.poll() is not None:
            # Process has terminated
            del running_processes[script_id]
            ROS2_SCRIPTS[script_id]['status'] = 'stopped'
            ROS2_SCRIPTS[script_id]['pid'] = None

    return JSONResponse(content=ROS2_SCRIPTS)


# -------------------- API Routes - System Monitoring -----------------------

@app.get("/api/disk-space")
async def get_disk_space():
    """Get disk space information"""
    try:
        total, used, free = shutil.disk_usage('/')

        # Convert bytes to GB
        total_gb = total / (1024 ** 3)
        used_gb = used / (1024 ** 3)
        free_gb = free / (1024 ** 3)

        disk_info = {
            'total': f'{total_gb:.2f} GB',
            'used': f'{used_gb:.2f} GB',
            'free': f'{free_gb:.2f} GB',
            'percent_used': f'{(used / total) * 100:.1f}%'
        }

        return JSONResponse(content=disk_info)
    except Exception as e:
        return JSONResponse(content={'error': str(e)}, status_code=500)

@app.get("/api/logs")
async def get_logs(source: str = Query(default="musohu", alias="source")):
    try:
        log_source = source
        log_viewer_config = config.get('log_viewer', {})
        max_lines = log_viewer_config.get('max_lines', 100)

        # Build log files mapping by scanning directories for ALL files
        log_files = {}
        scan_directories = log_viewer_config.get('scan_directories', [])

        for dir_config in scan_directories:
            if not dir_config.get('enabled', True):
                continue

            dir_path = dir_config.get('path', '')

            if os.path.exists(dir_path) and os.path.isdir(dir_path):
                try:
                    for filename in os.listdir(dir_path):
                        file_path = os.path.join(dir_path, filename)
                        # Only include files, not directories
                        if os.path.isfile(file_path):
                            log_id = filename.replace('.', '_')
                            # Handle ID collisions
                            if log_id in log_files:
                                log_id = f"{os.path.basename(dir_path.rstrip('/'))}_{log_id}"
                            log_files[log_id] = file_path
                except Exception as e:
                    logger.error(f"Error scanning directory {dir_path}: {e}")

        # Add individual sources from config
        sources = log_viewer_config.get('sources', [])
        for source in sources:
            if source.get('enabled', True):
                log_files[source['id']] = source['path']

        # Fallback to scanning logs/ folder if nothing configured
        if not log_files:
            logs_dir = 'logs'
            if os.path.exists(logs_dir) and os.path.isdir(logs_dir):
                for filename in os.listdir(logs_dir):
                    file_path = os.path.join(logs_dir, filename)
                    if os.path.isfile(file_path):
                        log_id = filename.replace('.', '_')
                        log_files[log_id] = file_path

        log_file = log_files.get(log_source, log_files.get('musohu', 'logs/musohu.log'))
        logs = []

        if os.path.exists(log_file):
            # Check if file extension is allowed
            allowed_extensions = log_viewer_config.get('allowed_extensions', ['.log', '.txt'])
            file_extension = os.path.splitext(log_file)[1]

            if file_extension not in allowed_extensions:
                return JSONResponse(content={
                    'logs': [],
                    'error': f'Unable to display this file type. Only {", ".join(allowed_extensions)} files can be displayed.'
                }, status_code=400)

            try:
                with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
                    lines = f.readlines()
                    # Get last N lines based on config
                    lines = lines[-max_lines:]

                    for line in lines:
                        line = line.strip()
                        if line:
                            # Parse log format: [timestamp] LEVEL: message
                            parts = line.split('] ', 1)
                            if len(parts) == 2:
                                timestamp = parts[0].replace('[', '')
                                rest = parts[1]
                                level_parts = rest.split(': ', 1)
                                if len(level_parts) == 2:
                                    level = level_parts[0]
                                    message = level_parts[1]
                                    logs.append({
                                        'timestamp': timestamp,
                                        'level': level,
                                        'message': message
                                    })
                                else:
                                    logs.append({
                                        'timestamp': timestamp,
                                        'level': 'INFO',
                                        'message': rest
                                    })
                            else:
                                logs.append({
                                    'timestamp': '',
                                    'level': 'INFO',
                                    'message': line
                                })
            except UnicodeDecodeError:
                return JSONResponse(content={
                    'logs': [],
                    'error': 'Unable to display this file. The file contains non-text or binary data.'
                }, status_code=400)
            except Exception as read_error:
                logger.error(f'Error reading file {log_file}: {str(read_error)}')
                return JSONResponse(content={
                    'logs': [],
                    'error': f'Error reading file: {str(read_error)}'
                }, status_code=500)
        else:
            return JSONResponse(content={'logs': [], 'error': f'Log file not found: {log_file}'}, status_code=404)

        return JSONResponse(content={'logs': logs, 'source': log_source})
    except Exception as e:
        logger.error(f'Failed to read logs: {str(e)}')
        return JSONResponse(content={'error': str(e)}, status_code=500)


# --------------------- API Routes - Log Management -------------------------

@app.get("/api/logs/sources")
async def get_log_sources():
    """Get all available log sources"""
    sources = {}
    # Get scan directories from config
    log_viewer_config = config.get('log_viewer', {})
    scan_directories = log_viewer_config.get('scan_directories', [])

    # Scan each configured directory for ALL files
    for dir_config in scan_directories:
        if not dir_config.get('enabled', True):
            continue

        dir_path = dir_config.get('path', '')
        name_prefix = dir_config.get('name_prefix', '')

        if os.path.exists(dir_path) and os.path.isdir(dir_path):
            try:
                for filename in sorted(os.listdir(dir_path)):
                    file_path = os.path.join(dir_path, filename)
                    # Skip directories, only include files
                    if os.path.isfile(file_path):
                        # Create unique ID from filename
                        log_id = filename.replace('.', '_')
                        # If there's a collision, add directory name to make it unique
                        if log_id in sources:
                            log_id = f"{os.path.basename(dir_path.rstrip('/'))}_{log_id}"

                        # Create display name
                        display_name = f"{name_prefix}{filename}"

                        sources[log_id] = {
                            'name': display_name,
                            'path': file_path,
                            'exists': True
                        }
            except Exception as e:
                logger.error(f"Error scanning directory {dir_path}: {e}")

    # Add individual sources from config (for files outside scan directories)
    sources_config = log_viewer_config.get('sources', [])
    for source in sources_config:
        if source.get('enabled', True):
            sources[source['id']] = {
                'name': source['name'],
                'path': source['path'],
                'exists': os.path.exists(source['path'])
            }

    # Fallback: scan logs/ folder if no config
    if not sources:
        logs_dir = 'logs'
        if os.path.exists(logs_dir) and os.path.isdir(logs_dir):
            for filename in sorted(os.listdir(logs_dir)):
                file_path = os.path.join(logs_dir, filename)
                if os.path.isfile(file_path):
                    log_id = filename.replace('.', '_')
                    sources[log_id] = {
                        'name': filename,
                        'path': file_path,
                        'exists': True
                    }

    return JSONResponse(content=sources)

@app.get("/api/logs/download")
async def download_log(source: str = Query(default="", alias="source")):
    """Download a log file"""
    try:
        log_source = source
        log_viewer_config = config.get('log_viewer', {})

        # Build log files mapping by scanning directories
        log_files = {}
        scan_directories = log_viewer_config.get('scan_directories', [])

        for dir_config in scan_directories:
            if not dir_config.get('enabled', True):
                continue

            dir_path = dir_config.get('path', '')

            if os.path.exists(dir_path) and os.path.isdir(dir_path):
                try:
                    for filename in os.listdir(dir_path):
                        file_path = os.path.join(dir_path, filename)
                        # Only include files, not directories
                        if os.path.isfile(file_path):
                            log_id = filename.replace('.', '_')
                            # Handle ID collisions
                            if log_id in log_files:
                                log_id = f"{os.path.basename(dir_path.rstrip('/'))}_{log_id}"
                            log_files[log_id] = (file_path, filename)
                except Exception as e:
                    logger.error(f"Error scanning directory {dir_path}: {e}")

        # Add individual sources from config
        sources = log_viewer_config.get('sources', [])
        for source in sources:
            if source.get('enabled', True):
                log_files[source['id']] = (source['path'], os.path.basename(source['path']))

        if log_source not in log_files:
            return JSONResponse(content={'error': 'Log file not found'}, status_code=404)

        log_file_path, original_filename = log_files[log_source]

        if not os.path.exists(log_file_path):
            return JSONResponse(content={'error': 'Log file not found on disk'}, status_code=404)

        logger.info(f'Downloading log file: {original_filename}')
        return FileResponse(
            log_file_path,
            media_type='application/octet-stream',
            filename=original_filename
        )
    except Exception as e:
        logger.error(f'Failed to download log: {str(e)}')
        return JSONResponse(content={'error': str(e)}, status_code=500)

@app.post("/api/scripts/{script_id}/start")
async def start_script(script_id: str):
    """Start a ROS2 script"""
    logger.info(f'Attempting to start script: {script_id}')
    
    if script_id not in ROS2_SCRIPTS:
        logger.error(f'Script not found: {script_id}')
        return JSONResponse(content={'error': 'Script not found'}, status_code=404)

    if script_id in running_processes:
        logger.warning(f'Script already running: {script_id}')
        return JSONResponse(content={'error': 'Script already running'}, status_code=400)

    try:
        command = ROS2_SCRIPTS[script_id]['command']
        script_name = ROS2_SCRIPTS[script_id]['description']

        # Start the process
        process = subprocess.Popen(
            command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # Create new process group
        )

        # Update tracking
        running_processes[script_id] = process
        ROS2_SCRIPTS[script_id]['status'] = 'running'
        ROS2_SCRIPTS[script_id]['pid'] = str(process.pid)  # Fix type mismatch

        logger.info(f'Successfully started {script_id} with PID: {process.pid}')
        return JSONResponse(content={
            'message': f'Started {script_name} (PID: {process.pid})',
            'pid': process.pid
        })
    except Exception as e:
        logger.error(f'Failed to start {script_id}: {str(e)}')
        return JSONResponse(content={'error': str(e)}, status_code=500)


@app.post("/api/scripts/{script_id}/stop")
async def stop_script(script_id: str):
    """Stop a running ROS2 script"""
    if script_id not in ROS2_SCRIPTS:
        return JSONResponse(content={'error': 'Invalid script ID'}, status_code=400)

    logger.info(f'Attempting to stop script: {script_id}')
    
    script_name = ROS2_SCRIPTS[script_id]['description']
    
    # Always ensure status is set to stopped
    ROS2_SCRIPTS[script_id]['status'] = 'stopped'
    ROS2_SCRIPTS[script_id]['pid'] = None
    
    if script_id not in running_processes:
        logger.warning(f'{script_id} is already stopped')
        return JSONResponse(content={'message': f'{script_name} is already stopped'})

    try:
        process = running_processes[script_id]
        pid = process.pid
        
        # Check if process is still alive
        if process.poll() is not None:
            # Process already terminated
            del running_processes[script_id]
            return JSONResponse(content={'message': f'Stopped {script_name} (PID: {pid})'})

        # Terminate the process gracefully
        try:
            process.terminate()
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            # Force kill if termination times out
            process.kill()
            process.wait()
        except ProcessLookupError:
            # Process doesn't exist anymore
            pass

        del running_processes[script_id]
        
        logger.info(f"Successfully stopped {script_id} (PID: {pid})")
        return JSONResponse(content={'message': f'Stopped {script_name} (PID: {pid})'})
    except Exception as e:
        logger.error(f'Error stopping {script_id}: {str(e)}')
        # Clean up even on error
        if script_id in running_processes:
            del running_processes[script_id]
        return JSONResponse(content={'message': f'Stopped {script_name}'})

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app:app", host="0.0.0.0", port=80, reload=True)
