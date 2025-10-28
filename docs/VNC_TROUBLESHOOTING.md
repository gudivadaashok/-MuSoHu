# VNC Password Troubleshooting Guide

## Common Issues and Solutions

### Issue: VNC Password Not Working

The `tiryoh/ros2-desktop-vnc:humble` image may have different password requirements.

### Try These Passwords (in order):

1. **vncpassword** (our configured password)
2. **ubuntu** (default user password)
3. **password** (common default)
4. **vnc** (simple default)
5. **empty** (no password - just click connect)

### Manual Password Reset Inside Container:

```bash
# Connect to container
docker exec -it ros2_vnc bash

# Set VNC password manually
echo 'vncpassword' | vncpasswd -f > /home/ubuntu/.vnc/passwd
chmod 600 /home/ubuntu/.vnc/passwd
chown ubuntu:ubuntu /home/ubuntu/.vnc/passwd

# Restart VNC (container will auto-restart services)
exit
docker-compose restart ros2_vnc
```

### Alternative: No Authentication Mode

If password continues to fail, you can try modifying the VNC server to run without authentication:

```bash
# Inside container, modify VNC startup
docker exec -it ros2_vnc bash
# Edit the VNC startup to disable auth
# This is not recommended for production but works for local development
```

### Check VNC Status:

```bash
# Check if VNC server is running
docker exec ros2_vnc ps aux | grep vnc

# Check VNC logs
docker exec ros2_vnc tail -f /home/ubuntu/.vnc/*.log
```

### Access Methods:

1. **Web Browser**: http://localhost:6080
2. **VNC Client**: localhost:5901 (direct VNC connection)

### Current Status:

- VNC_PASSWORD environment variable: ✅ Set
- VNC server: ✅ Running 
- Password file: ✅ Created
- Web interface: ✅ Accessible

Try the passwords in order above, or try connecting without entering any password.