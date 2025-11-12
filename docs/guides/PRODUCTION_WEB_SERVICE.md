# MuSoHu Production Web Service Setup

Complete production-ready setup for the MuSoHu web application with automatic restart, resource limits, and professional deployment features.

---

## Development vs Production

### Development Mode (Port 8000, No sudo required)

**For testing and development:**

```bash
# 1. Navigate to web-app directory
cd /path/to/MuSoHu/web-app

# 2. Activate virtual environment and install dependencies
source ../.venv/bin/activate
pip install -r requirements.txt

# 3. Run development server with auto-reload
uvicorn app:app --host 0.0.0.0 --port 8000 --reload

# 4. Access at http://localhost:8000
```

**Development Features:**
- Auto-reload on code changes
- No sudo required (runs on port 8000)
- Detailed error messages
- Interactive API docs at `/docs`

**Use when:** Testing, developing features, debugging

---

### Production Mode - Option 1: User Port (No sudo) **RECOMMENDED**

**For production without sudo (runs on port 8000):**

```bash
# 1. Navigate to web-app directory
cd /path/to/MuSoHu/web-app

# 2. Activate virtual environment
source ../.venv/bin/activate

# 3. Run production server (without reload)
uvicorn app:app --host 0.0.0.0 --port 8000

# 4. Access at http://localhost:8000
```

**To run in background:**
```bash
# Option A: Using nohup
nohup ../.venv/bin/uvicorn app:app --host 0.0.0.0 --port 8000 > logs/uvicorn.log 2>&1 &

# Option B: Using screen
screen -dmS musohu-web ../.venv/bin/uvicorn app:app --host 0.0.0.0 --port 8000

# Option C: Using tmux
tmux new-session -d -s musohu-web '../.venv/bin/uvicorn app:app --host 0.0.0.0 --port 8000'
```

**Production Features:**
- No sudo required
- Runs on port 8000 (non-privileged)
- No reload (stable for production)
- Can use systemd user service
- Better security (no root privileges)

**Use when:** Production deployment, security is priority, don't need port 80

---

### Production Mode - Option 2: System Service (Requires sudo)

**For production with systemd on port 80:**

```bash
# One-command setup
sudo bash scripts/setup/setup_production_web_service.sh
```

**Production Features:**
- Runs on port 80 (standard HTTP)
- Automatic restart on failure
- Systemd integration
- Resource limits
- Auto-start on boot
- Requires sudo for setup

**Use when:** Need port 80, want systemd auto-restart, deploying to production server

**Note:** Port 80 requires root privileges. Better alternatives:
- Use port 8000 (no sudo)
- Use reverse proxy (nginx/Apache) forwarding from port 80
- Use `setcap` to allow port binding without sudo (Linux only)

---

---

## Avoiding Sudo in Production (Best Practices)

### Why Avoid Sudo?

Running applications with sudo/root privileges is a security risk. Here are better alternatives:

### Recommended Approaches

#### 1. Use Non-Privileged Port (Port 8000)

**Best and simplest solution:**

```bash
# Run on port 8000 (no sudo needed)
cd /path/to/MuSoHu/web-app
../.venv/bin/uvicorn app:app --host 0.0.0.0 --port 8000
```

Access at: `http://your-server:8000`

**Pros:**
- No sudo required
- More secure
- Simple to set up
- Works everywhere

**Cons:**
- Need to specify port in URL
- Non-standard port

---

#### 2. Use Reverse Proxy (nginx/Apache)

**Professional production setup:**

```bash
# 1. Run app on port 8000 (no sudo)
../.venv/bin/uvicorn app:app --host 127.0.0.1 --port 8000

# 2. Install nginx
sudo apt install nginx

# 3. Configure nginx to forward port 80 → 8000
sudo nano /etc/nginx/sites-available/musohu
```

Add this configuration:
```nginx
server {
    listen 80;
    server_name your-domain.com;  # or your IP address
    
    location / {
        proxy_pass http://127.0.0.1:8000;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}
```

```bash
# Enable the site
sudo ln -s /etc/nginx/sites-available/musohu /etc/nginx/sites-enabled/
sudo nginx -t
sudo systemctl restart nginx
```

**Pros:**
- App runs without sudo
- Standard port 80 accessible
- Can add HTTPS easily
- Better performance with static files
- Production-grade setup

**Cons:**
- Requires nginx installation
- More complex setup

---

#### 3. Use systemd User Service (No sudo for runtime)

**Systemd without root privileges:**

```bash
# 1. Create user service directory
mkdir -p ~/.config/systemd/user/

# 2. Create service file
nano ~/.config/systemd/user/musohu-web.service
```

Service file content:
```ini
[Unit]
Description=MuSoHu Web Application
After=network.target

[Service]
Type=simple
WorkingDirectory=/path/to/MuSoHu/web-app
ExecStart=/path/to/MuSoHu/.venv/bin/uvicorn app:app --host 0.0.0.0 --port 8000
Restart=always
RestartSec=10

[Install]
WantedBy=default.target
```

```bash
# 3. Enable and start (no sudo!)
systemctl --user daemon-reload
systemctl --user enable musohu-web
systemctl --user start musohu-web

# 4. Enable linger (start on boot)
sudo loginctl enable-linger $USER

# Manage service (no sudo!)
systemctl --user status musohu-web
systemctl --user restart musohu-web
systemctl --user stop musohu-web
```

**Pros:**
- No sudo for daily operations
- Systemd auto-restart
- User-level service
- Auto-start on boot

**Cons:**
- Still runs on port 8000
- Requires initial linger setup with sudo

---

#### 4. Use Port Forwarding/iptables (Linux only)

**Forward port 80 to 8000:**

```bash
# One-time setup (requires sudo)
sudo iptables -t nat -A PREROUTING -p tcp --dport 80 -j REDIRECT --to-port 8000

# Make persistent (Ubuntu/Debian)
sudo apt install iptables-persistent
sudo netfilter-persistent save

# Then run app normally (no sudo)
../.venv/bin/uvicorn app:app --host 0.0.0.0 --port 8000
```

**Pros:**
- Port 80 accessible
- App runs without sudo

**Cons:**
- Linux-only
- Requires one-time sudo setup
- iptables knowledge needed

---

#### 5. Use setcap (Linux only)

**Allow Python to bind to privileged ports:**

```bash
# Give Python capability to bind to port 80 (one-time)
sudo setcap CAP_NET_BIND_SERVICE=+eip /path/to/MuSoHu/.venv/bin/python3

# Then run on port 80 without sudo
../.venv/bin/uvicorn app:app --host 0.0.0.0 --port 80
```

**Pros:**
- Can use port 80
- No sudo for running app

**Cons:**
- Security risk (any script can use port 80)
- Linux-only
- Lost on Python upgrade

---

### Comparison

| Method | Sudo Needed | Port 80 | Security | Complexity | Recommended |
|--------|-------------|---------|----------|------------|-------------|
| **Port 8000** | No | No | 5-star | Easy | **Development** |
| **Reverse Proxy** | Only for nginx | Yes | 5-star | Medium | **Production** |
| **User Service** | Only once | No | 4-star | Easy | **Production** |
| **Port Forward** | Only once | Yes | 3-star | Medium | Linux only |
| **setcap** | Only once | Yes | 2-star | Easy | Not recommended |
| **Direct sudo** | Always | Yes | 1-star | Easy | **Avoid** |

---

### Recommendations

**For Development:**
```bash
# Just use port 8000
uvicorn app:app --host 0.0.0.0 --port 8000 --reload
```

**For Production (Best):**
```bash
# Option 1: nginx reverse proxy
# App on port 8000 + nginx forwards from port 80
# Most professional, secure, and flexible

# Option 2: User systemd service  
# Runs on port 8000, systemd manages it, no sudo needed
# Good balance of convenience and security
```

**For Quick Production:**
```bash
# Just use port 8000
# Simple, secure, works everywhere
../.venv/bin/uvicorn app:app --host 0.0.0.0 --port 8000
```

---

## Quick Start

```bash
# 1. Navigate to project directory
cd /path/to/MuSoHu

# 2. Run production setup (one command!)
sudo bash scripts/setup/setup_production_web_service.sh

# 3. Verify installation
bash scripts/setup/test_production_setup.sh

# 4. Access web interface
curl http://localhost/api/health
# Open browser: http://localhost
```

**That's it!** Your production web service is now running with automatic restart capabilities.

---

## Features

| Feature | Description | Benefit |
|---------|-------------|---------|
| **Automatic Restart** | `Restart=always` | Service recovers from crashes automatically |
| **Unlimited Restarts** | `StartLimitBurst=0` | No artificial limits on restart attempts |
| **Restart Delay** | `RestartSec=10` | 10-second delay prevents rapid restart loops |
| **Production Server** | Uvicorn ASGI | Modern async server for FastAPI |
| **FastAPI Framework** | High-performance async | 2-3x faster than traditional frameworks |
| **Resource Limits** | 256MB RAM, 25% CPU | Prevents runaway processes while being lightweight |
| **Systemd Logging** | Journal integration | Centralized log management |
| **Auto-start** | Boot integration | Starts automatically on system boot |
| **Health Checks** | `/api/health` endpoint | Monitor service health easily |
| **Security** | Process isolation | NoNewPrivileges, PrivateTmp |
| **Standard Port** | Port 80 | No port number needed in URLs |

---

## What Gets Installed

### Files Created:
- `/etc/systemd/system/musohu-web.service` - Systemd service file
- `web-app/venv/` - Python virtual environment  
- All dependencies including Uvicorn ASGI server

### Files Modified:
- `web-app/requirements.txt` - Updated with FastAPI dependencies
- `web-app/app.py` - FastAPI application with async support

### Scripts Available:
- `scripts/setup/setup_production_web_service.sh` - Installation script
- `scripts/utils/manage_web_service.sh` - Easy management tool
- `scripts/setup/test_production_setup.sh` - Verification tests

---

## Documentation

- **[PRODUCTION_SETUP.md](PRODUCTION_SETUP.md)** - Complete setup guide with all details
- **[CHANGES_SUMMARY.md](CHANGES_SUMMARY.md)** - What was changed and why
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - Visual architecture diagrams
- **[QUICK_REFERENCE.md](../reference/QUICK_REFERENCE.md)** - Quick command reference card
- **[INSTALLATION_SUMMARY.md](INSTALLATION_SUMMARY.md)** - Installation overview and next steps

---

## Service Management

### Easy Way (Recommended)

```bash
# Start the service
sudo bash scripts/utils/manage_web_service.sh start

# Stop the service
sudo bash scripts/utils/manage_web_service.sh stop

# Restart the service
sudo bash scripts/utils/manage_web_service.sh restart

# Check status
bash scripts/utils/manage_web_service.sh status

# View logs
bash scripts/utils/manage_web_service.sh logs       # Last 50 lines
bash scripts/utils/manage_web_service.sh follow    # Stream logs

# Check health
bash scripts/utils/manage_web_service.sh health

# Run tests
bash scripts/utils/manage_web_service.sh test

# See all commands
bash scripts/utils/manage_web_service.sh --help
```

### Direct systemctl Commands

```bash
sudo systemctl start musohu-web
sudo systemctl stop musohu-web
sudo systemctl restart musohu-web
sudo systemctl status musohu-web
sudo systemctl enable musohu-web    # Enable auto-start on boot
sudo systemctl disable musohu-web   # Disable auto-start
```

---

## Monitoring

### View Logs

```bash
# Using management script
bash scripts/utils/manage_web_service.sh logs 100   # Last 100 lines
bash scripts/utils/manage_web_service.sh follow    # Real-time

# Using journalctl directly
sudo journalctl -u musohu-web -n 100               # Last 100 lines
sudo journalctl -u musohu-web -f                   # Follow logs
sudo journalctl -u musohu-web --since "1 hour ago" # Time-based
sudo journalctl -u musohu-web -p err               # Errors only
```

### Check Service Status

```bash
# Quick check
sudo systemctl is-active musohu-web

# Detailed status
bash scripts/utils/manage_web_service.sh stats

# Check restart count
sudo systemctl show musohu-web -p NRestarts
```

### Check Health

```bash
# Using management script
bash scripts/utils/manage_web_service.sh health

# Using curl
curl http://localhost/api/health

# Expected response:
{
  "status": "healthy",
  "running_scripts": 0,
  "timestamp": "2025-11-12T10:30:00.123456"
}
```

---

## Testing

### Run Verification Tests

```bash
# Run full test suite
bash scripts/setup/test_production_setup.sh
```

This tests:
- File system structure
- Python environment
- Systemd service configuration
- Runtime status
- Network connectivity
- Health endpoint

### Manual Testing

```bash
# 1. Check if service is running
sudo systemctl status musohu-web

# 2. Test health endpoint
curl http://localhost/api/health

# 3. Test web interface
curl http://localhost/

# 4. Test from another machine (replace <your-ip>)
curl http://<your-ip>/api/health

# 5. View recent logs
sudo journalctl -u musohu-web -n 50
```

---

## Configuration

### Application Settings

Edit `web-app/config.yml`:

```yaml
server:
  host: '0.0.0.0'      # Listen on all interfaces
  port: 80             # Standard HTTP port (requires sudo)
  debug: false         # false = Production mode
```

**Note**: Port 80 requires root/sudo privileges. The service runs as your user but binds to port 80 through the systemd service configuration.

### Service Settings

Edit `/etc/systemd/system/musohu-web.service`:

```ini
[Service]
# Restart configuration
Restart=always              # Always restart on failure
RestartSec=10              # Wait 10 seconds before restart
StartLimitBurst=0          # No limit on restart attempts

# Resource limits (lightweight for single user)
MemoryMax=256M             # Maximum memory (sufficient for single user)
MemoryHigh=200M            # Soft limit
CPUQuota=25%               # CPU usage limit (1/4 of one core)

# Adjust as needed for your system
```

After editing, reload:
```bash
sudo systemctl daemon-reload
sudo systemctl restart musohu-web
```

---

## Troubleshooting

### Service Won't Start

```bash
# Check detailed logs
sudo journalctl -u musohu-web -n 100

# Check service status
sudo systemctl status musohu-web

# Check if port is already in use
sudo ss -tuln | grep :80

# Kill process on port 80 if needed
sudo lsof -ti:80 | xargs sudo kill -9
```

### Service Keeps Restarting

```bash
# Check restart count
sudo systemctl show musohu-web -p NRestarts

# View logs around restart time
sudo journalctl -u musohu-web --since "10 minutes ago"

# Look for Python errors
sudo journalctl -u musohu-web | grep -i "error\|exception\|traceback"
```

### Port Not Accessible

```bash
# Check if service is listening
sudo ss -tuln | grep :80

# Check firewall
sudo ufw status
sudo ufw allow 80/tcp

# Test locally first
curl http://localhost/api/health

# Then test network access
curl http://$(hostname -I | awk '{print $1}')/api/health
```

### High Memory Usage

```bash
# Check current memory usage
sudo systemctl show musohu-web -p MemoryCurrent

# Adjust memory limit
sudo nano /etc/systemd/system/musohu-web.service
# Change: MemoryMax=2G

# Reload and restart
sudo systemctl daemon-reload
sudo systemctl restart musohu-web
```

---

## Updating the Service

### Update Application Code

```bash
# 1. Stop the service
sudo systemctl stop musohu-web

# 2. Update code (git pull, edit files, etc.)
cd /path/to/MuSoHu
git pull

# 3. Update dependencies if needed
source web-app/venv/bin/activate
pip install -r web-app/requirements.txt
deactivate

# 4. Restart the service
sudo systemctl start musohu-web

# 5. Verify
bash scripts/utils/manage_web_service.sh test
```

### Reinstall Service

```bash
# Re-run setup script
sudo bash scripts/setup/setup_production_web_service.sh
```

---

## Uninstall

```bash
# 1. Stop and disable service
sudo systemctl stop musohu-web
sudo systemctl disable musohu-web

# 2. Remove service file
sudo rm /etc/systemd/system/musohu-web.service

# 3. Reload systemd
sudo systemctl daemon-reload
sudo systemctl reset-failed

# 4. (Optional) Remove virtual environment
rm -rf web-app/venv
```

---

## Performance Tips

### When to Adjust Settings

**For single user or local use (default is fine):**
- The default limits (256MB RAM, 25% CPU) are sufficient for single user
- Typical usage: 50-100MB RAM, <10% CPU
- No changes needed for typical usage

**For systems with limited RAM (4-8GB total):**
- Keep defaults (256MB RAM limit)
- This reserves less than 4% of your system RAM

**For multiple concurrent users (5-10+ users) or high RAM systems (16GB+):**
- Consider increasing to 512MB-1GB
- Monitor actual usage first before adjusting

### Uvicorn Workers

**Single user setup (default - recommended):**
```ini
# No workers needed, runs single process
ExecStart=/path/to/venv/bin/uvicorn app:app --host 0.0.0.0 --port 80
```

**Multiple concurrent users (10+ users):**
```ini
# Run with multiple worker processes for better concurrency
ExecStart=/path/to/venv/bin/uvicorn app:app --host 0.0.0.0 --port 80 --workers 2
```

**Note:** Each worker uses additional memory. Start with 1 worker (default) and only add more if needed.

### Async Performance

FastAPI supports async/await for better concurrent performance. The application already uses async where appropriate for handling multiple requests efficiently, even with a single worker process.

### Increase Resource Limits (Only if needed)

**Check current usage first:**
```bash
# Monitor memory usage
sudo systemctl show musohu-web -p MemoryCurrent

# Monitor CPU usage
top -p $(pgrep -f "uvicorn app:app")
```

**Typical usage for single user:**
- Memory: 50-100MB (well under 256MB limit)
- CPU: 5-15% (well under 25% limit)

**Only increase if you see:**
- Memory consistently over 200MB
- CPU consistently at 25%+
- Application becoming slow under load

**Then edit** `/etc/systemd/system/musohu-web.service`:
```ini
MemoryMax=512M    # Only if memory usage is consistently high
CPUQuota=50%      # Only if CPU usage is consistently high
```

**For single user on 8GB system:** Default limits (256MB/25% CPU) are perfect. Don't change unless you have performance issues.

---

## Security Considerations

The service includes security hardening:
- Runs as non-root user
- `NoNewPrivileges=true` - Prevents privilege escalation
- `PrivateTmp=true` - Isolated temporary directory
- Resource limits prevent DoS attacks
- Firewall-ready (UFW configuration included)

---

## Integration Examples

### With Nginx Reverse Proxy

If you want to add HTTPS or additional security:

```nginx
server {
    listen 443 ssl;
    server_name your-domain.com;
    
    ssl_certificate /path/to/cert.pem;
    ssl_certificate_key /path/to/key.pem;
    
    location / {
        proxy_pass http://localhost:80;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
    
    location /api/health {
        proxy_pass http://localhost:80/api/health;
        access_log off;
    }
}
```

### With Prometheus Monitoring

```yaml
scrape_configs:
  - job_name: 'musohu-web'
    static_configs:
      - targets: ['localhost:80']
    metrics_path: '/api/health'
```

---

## Additional Resources

- **systemd Documentation**: https://www.freedesktop.org/software/systemd/man/
- **FastAPI Documentation**: https://fastapi.tiangolo.com/
- **Uvicorn Documentation**: https://www.uvicorn.org/
- **FastAPI Deployment**: https://fastapi.tiangolo.com/deployment/

---

## Getting Help

1. **Run tests**: `bash scripts/setup/test_production_setup.sh`
2. **Check logs**: `sudo journalctl -u musohu-web -n 100`
3. **View status**: `bash scripts/utils/manage_web_service.sh stats`
4. **Read docs**: See [PRODUCTION_SETUP.md](PRODUCTION_SETUP.md)
5. **Quick reference**: See [docs/reference/QUICK_REFERENCE.md](../reference/QUICK_REFERENCE.md)

---

## Quick Checklist

After installation, verify:

- [ ] Service is running: `sudo systemctl is-active musohu-web`
- [ ] Auto-start enabled: `sudo systemctl is-enabled musohu-web`
- [ ] Health responds: `curl http://localhost/api/health`
- [ ] Web UI accessible: Open http://localhost in browser
- [ ] Logs visible: `sudo journalctl -u musohu-web -n 10`
- [ ] Tests pass: `bash scripts/setup/test_production_setup.sh`
- [ ] Port 80 listening: `sudo ss -tuln | grep :80`

---

**Created**: November 12, 2025  
**Version**: 1.0  
**Status**: Production Ready 

**Enjoy your production-ready MuSoHu web service!**
