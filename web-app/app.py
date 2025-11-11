from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
import subprocess
import os
import signal
import shutil
from logging_config import setup_logging

app = Flask(__name__)
CORS(app)

# Setup logging
logger = setup_logging(app)

# Store running processes
running_processes = {}

# Docker container name
DOCKER_CONTAINER = 'ros2_vnc'

# Define available ROS2 scripts
ROS2_SCRIPTS = {
    'turtlesim': {
        'command': f'"TODO"',
        'description': 'MuSoHu_Helmet_Nodes',
        'status': 'stopped',
        'pid': None
    },
    'rviz2': {
        'command': f'"TODO"',
        'description': 'RViz2 Visualization',
        'status': 'stopped',
        'pid': None
    }
}

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/disk-space')
def disk_space():
    return render_template('disk_space.html')

@app.route('/logs')
def logs():
    return render_template('logs.html')

@app.route('/api/scripts', methods=['GET'])
def get_scripts():
    # Update status based on actual process state
    for script_id in running_processes.copy():
        process = running_processes[script_id]
        if process.poll() is not None:
            # Process has terminated
            del running_processes[script_id]
            ROS2_SCRIPTS[script_id]['status'] = 'stopped'
            ROS2_SCRIPTS[script_id]['pid'] = None
    
    return jsonify(ROS2_SCRIPTS)

@app.route('/api/disk-space', methods=['GET'])
def get_disk_space():
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
        # Get log source from query parameter (default: 'app')
        log_source = request.args.get('source', 'app')

        # Define available log sources
        log_files = {
            'app': 'logs/musohu.log',
            'ros2': 'src/install/helmet_bringup/share/helmet_bringup/config/ros2.log',
            'helmet': 'src/install/helmet_bringup/share/helmet_bringup/config/helmet.log'
        }

        log_file = log_files.get(log_source, log_files['app'])
        logs = []

        if os.path.exists(log_file):
            with open(log_file, 'r') as f:
                lines = f.readlines()
                # Get last 100 lines
                lines = lines[-100:]

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
        else:
            return jsonify({'logs': [], 'error': f'Log file not found: {log_file}'})

        return jsonify({'logs': logs, 'source': log_source})
    except Exception as e:
        logger.error(f'Failed to read logs: {str(e)}')
        return jsonify({'error': str(e)}), 500

@app.route('/api/logs/sources', methods=['GET'])
def get_log_sources():
    """Return available log sources and their paths"""
    sources = {
        'app': {
            'name': 'Application Logs',
            'path': 'logs/musohu.log',
            'exists': os.path.exists('logs/musohu.log')
        },
        'ros2': {
            'name': 'ROS2 Logs',
            'path': 'src/install/helmet_bringup/share/helmet_bringup/config/ros2.log',
            'exists': os.path.exists('src/install/helmet_bringup/share/helmet_bringup/config/ros2.log')
        },
        'helmet': {
            'name': 'Helmet Bringup Logs',
            'path': 'src/install/helmet_bringup/share/helmet_bringup/config/helmet.log',
            'exists': os.path.exists('src/install/helmet_bringup/share/helmet_bringup/config/helmet.log')
        }
    }
    return jsonify(sources)

@app.route('/api/scripts/<script_id>/start', methods=['POST'])
def start_script(script_id):
    logger.info(f'Attempting to start script: {script_id}')
    
    if script_id not in ROS2_SCRIPTS:
        logger.error(f'Script not found: {script_id}')
        return jsonify({'error': 'Script not found'}), 404
    
    if script_id in running_processes:
        logger.warning(f'Script already running: {script_id}')
        return jsonify({'error': 'Script already running'}), 400
    
    try:
        command = ROS2_SCRIPTS[script_id]['command']
        process = subprocess.Popen(
            command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        running_processes[script_id] = process
        ROS2_SCRIPTS[script_id]['status'] = 'running'
        ROS2_SCRIPTS[script_id]['pid'] = process.pid
        
        script_name = ROS2_SCRIPTS[script_id]['description']
        logger.info(f'Successfully started {script_id} with PID: {process.pid}')
        return jsonify({'message': f'✓ Started {script_name} (PID: {process.pid})', 'pid': process.pid})
    except Exception as e:
        logger.error(f'Failed to start {script_id}: {str(e)}')
        return jsonify({'error': str(e)}), 500

@app.route('/api/scripts/<script_id>/stop', methods=['POST'])
def stop_script(script_id):
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
        
        # Kill the process inside the Docker container
        # Find and kill the actual ROS2 process by name
        process_name = 'turtlesim_node' if script_id == 'turtlesim' else 'rviz2'
        kill_command = f'docker exec {DOCKER_CONTAINER} bash -c "pkill -f {process_name}"'
        subprocess.run(kill_command, shell=True, timeout=5)
        
        # Try to kill the local docker exec process group
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
        except ProcessLookupError:
            # Process doesn't exist anymore
            pass
        except Exception as e:
            # If termination fails, try SIGKILL
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except:
                pass
        
        del running_processes[script_id]
        
        return jsonify({'message': f'✓ Stopped {script_name} (PID: {pid})'})
    except Exception as e:
        # Clean up even on error
        if script_id in running_processes:
            del running_processes[script_id]
        return jsonify({'message': f'✓ Stopped {script_name}'})


if __name__ == '__main__':
    logger.info('Starting MuSoHu ROS2 Script Manager')
    logger.info(f'Available scripts: {", ".join(ROS2_SCRIPTS.keys())}')
    app.run(host='0.0.0.0', port=5001, debug=True)
