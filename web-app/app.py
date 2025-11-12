"""
MuSoHu Web Application
Manages ROS2 scripts, displays logs, and monitors system resources
"""
from flask import Flask, render_template, jsonify, request, send_file
from flask_cors import CORS
import subprocess
import os
import shutil
import yaml
from logging_config import setup_logging

# Initialize Flask app
app = Flask(__name__)
CORS(app)


def load_config():
    """Load configuration from YAML file"""
    config_path = 'config.yml'
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    return {}


# Load configuration
config = load_config()

# Setup logging
logger = setup_logging(app)

# Store running processes
running_processes = {}

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


# ============================================================================
# Page Routes
# ============================================================================

@app.route('/')
def index():
    """Main page - Scripts management"""
    return render_template('index.html')


@app.route('/disk-space')
def disk_space():
    """Disk space monitoring page"""
    return render_template('disk_space.html')


@app.route('/logs')
def logs():
    """Log viewer page"""
    return render_template('logs.html')


# ============================================================================
# API Routes - Scripts
# ============================================================================

@app.route('/api/scripts', methods=['GET'])
def get_scripts():
    """Get status of all ROS2 scripts"""
    # Update status based on actual process state
    for script_id in running_processes.copy():
        process = running_processes[script_id]
        if process.poll() is not None:
            # Process has terminated
            del running_processes[script_id]
            ROS2_SCRIPTS[script_id]['status'] = 'stopped'
            ROS2_SCRIPTS[script_id]['pid'] = None
    
    return jsonify(ROS2_SCRIPTS)


# ============================================================================
# API Routes - System Monitoring
# ============================================================================

@app.route('/api/disk-space', methods=['GET'])
def get_disk_space():
    """Get disk space information"""
    try:
        # Get disk space statistics for the current directory
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

        logger.info(f'Disk space checked: {disk_info}')
        return jsonify(disk_info)
    except Exception as e:
        logger.error(f'Failed to get disk space: {str(e)}')
        return jsonify({'error': str(e)}), 500

@app.route('/api/logs', methods=['GET'])
def get_logs():
    try:
        # Get log source from query parameter (default: 'musohu')
        log_source = request.args.get('source', 'musohu')

        # Load log sources from config
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
                return jsonify({
                    'logs': [],
                    'error': f'Unable to display this file type. Only {", ".join(allowed_extensions)} files can be displayed.'
                }), 400

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
                return jsonify({
                    'logs': [],
                    'error': 'Unable to display this file. The file contains non-text or binary data.'
                }), 400
            except Exception as read_error:
                logger.error(f'Error reading file {log_file}: {str(read_error)}')
                return jsonify({
                    'logs': [],
                    'error': f'Error reading file: {str(read_error)}'
                }), 500
        else:
            return jsonify({'logs': [], 'error': f'Log file not found: {log_file}'}), 404

        return jsonify({'logs': logs, 'source': log_source})
    except Exception as e:
        logger.error(f'Failed to read logs: {str(e)}')
        return jsonify({'error': str(e)}), 500


# ============================================================================
# API Routes - Log Management
# ============================================================================

@app.route('/api/logs/sources', methods=['GET'])
def get_log_sources():
    """Return available log sources - auto-discover all files from configured directories"""
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

    return jsonify(sources)

@app.route('/api/logs/download', methods=['GET'])
def download_log():
    """Download a log file"""
    try:
        # Get log source from query parameter
        log_source = request.args.get('source', '')

        if not log_source:
            return jsonify({'error': 'No source specified'}), 400

        # Load log sources from config
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
            return jsonify({'error': 'Log file not found'}), 404

        log_file_path, original_filename = log_files[log_source]

        if not os.path.exists(log_file_path):
            return jsonify({'error': 'Log file not found on disk'}), 404

        logger.info(f'Downloading log file: {original_filename}')
        return send_file(
            log_file_path,
            as_attachment=True,
            download_name=original_filename,
            mimetype='text/plain'
        )
    except Exception as e:
        logger.error(f'Failed to download log: {str(e)}')
        return jsonify({'error': str(e)}), 500

@app.route('/api/scripts/<script_id>/start', methods=['POST'])
def start_script(script_id):
    """Start a ROS2 script"""
    logger.info(f'Attempting to start script: {script_id}')
    
    if script_id not in ROS2_SCRIPTS:
        logger.error(f'Script not found: {script_id}')
        return jsonify({'error': 'Script not found'}), 404
    
    if script_id in running_processes:
        logger.warning(f'Script already running: {script_id}')
        return jsonify({'error': 'Script already running'}), 400
    
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
        ROS2_SCRIPTS[script_id]['pid'] = process.pid
        
        logger.info(f'Successfully started {script_id} with PID: {process.pid}')
        return jsonify({
            'message': f'✓ Started {script_name} (PID: {process.pid})',
            'pid': process.pid
        })
    except Exception as e:
        logger.error(f'Failed to start {script_id}: {str(e)}')
        return jsonify({'error': str(e)}), 500


@app.route('/api/scripts/<script_id>/stop', methods=['POST'])
def stop_script(script_id):
    """Stop a running ROS2 script"""
    if script_id not in ROS2_SCRIPTS:
        return jsonify({'error': 'Invalid script ID'}), 400

    logger.info(f'Attempting to stop script: {script_id}')
    
    script_name = ROS2_SCRIPTS[script_id]['description']
    
    # Always ensure status is set to stopped
    ROS2_SCRIPTS[script_id]['status'] = 'stopped'
    ROS2_SCRIPTS[script_id]['pid'] = None
    
    if script_id not in running_processes:
        logger.warning(f'{script_id} is already stopped')
        return jsonify({'message': f'✓ {script_name} is already stopped'})
    
    try:
        process = running_processes[script_id]
        pid = process.pid
        
        # Check if process is still alive
        if process.poll() is not None:
            # Process already terminated
            del running_processes[script_id]
            return jsonify({'message': f'✓ Stopped {script_name} (PID: {pid})'})
        
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
        
        logger.info(f'Successfully stopped {script_id} (PID: {pid})')
        return jsonify({'message': f'✓ Stopped {script_name} (PID: {pid})'})
    except Exception as e:
        logger.error(f'Error stopping {script_id}: {str(e)}')
        # Clean up even on error
        if script_id in running_processes:
            del running_processes[script_id]
        return jsonify({'message': f'✓ Stopped {script_name}'})


# ============================================================================
# Application Entry Point
# ============================================================================

if __name__ == '__main__':
    server_config = config.get('server', {})
    host = server_config.get('host', '0.0.0.0')
    port = server_config.get('port', 5001)
    debug = server_config.get('debug', False)

    logger.info('=' * 70)
    logger.info('Starting MuSoHu Web Application')
    logger.info(f'Available scripts: {", ".join(ROS2_SCRIPTS.keys())}')
    logger.info(f'Server: http://{host}:{port}')
    logger.info('=' * 70)

    app.run(host=host, port=port, debug=debug)
