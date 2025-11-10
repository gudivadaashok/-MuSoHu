# ROS2 Script Manager Web App

A simple Flask web application to control and monitor ROS2 scripts through a browser interface.

## Features

- 🎮 Start/Stop ROS2 scripts with a single click
- 📊 Real-time status monitoring
- 🎨 Modern, responsive UI with gradient design
- 🔄 Auto-refresh every 5 seconds

## Prerequisites

- Python 3.7+
- ROS2 Humble (installed and sourced)
- pip

## Installation

1. Navigate to the project directory:
```bash
cd ros2_script_manager
```

2. Install Python dependencies:
```bash
pip install -r requirements.txt
```

## Usage

1. Start the Flask server:
```bash
python app.py
```

2. Open your browser and navigate to:
```
http://localhost:5000
```

3. Click "Start" or "Stop" buttons to control ROS2 scripts

## Adding New Scripts

Edit the `ROS2_SCRIPTS` dictionary in `app.py`:

```python
ROS2_SCRIPTS = {
    'script_id': {
        'command': 'ros2 run package_name node_name',
        'description': 'Your Script Description',
        'status': 'stopped'
    }
}
```

## Directory Structure

```
ros2_script_manager/
├── app.py                  # Flask backend
├── requirements.txt        # Python dependencies
├── templates/
│   └── index.html         # HTML template
├── static/
│   └── style.css          # CSS styling
└── README.md              # This file
```

## API Endpoints

- `GET /api/scripts` - List all available scripts
- `POST /api/scripts/<script_id>/start` - Start a script
- `POST /api/scripts/<script_id>/stop` - Stop a script

## Troubleshooting

**Scripts won't start:**
- Ensure ROS2 is properly sourced in your environment
- Check that the ROS2 commands work in your terminal first

**Port 5000 already in use:**
- Change the port in `app.py`: `app.run(host='0.0.0.0', port=<new_port>)`

## License

MIT License
