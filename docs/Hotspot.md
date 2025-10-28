To start the Robotixx_MuSoHu hotspot with the network name "Robotixx_MuSoHu" automatically on every restart, you should create a systemd service that runs the hotspot command at boot.

Here’s what you need to do on your Jetson:


### 1. Create a script to start the hotspot
Save this as `/usr/local/bin/start_hotspot.sh`:
```bash
#!/bin/bash
lsof -i :8080
nmcli dev wifi hotspot ifname wlP1p1s0 ssid Robotixx_MuSoHu password Robotixx
python3 /home/jetson/git/simple_web_server/simple_web_server.py &
```

Make the script executable:
```bash
sudo chmod +x /usr/local/bin/start_hotspot.sh
```

### 2. Create a systemd service file
Save this as `/etc/systemd/system/hotspot.service`:
```ini
[Unit]
Description=Start Robotixx_MuSoHu Hotspot at boot
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/start_hotspot.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

### 3. Enable the service to start at boot
```bash
sudo systemctl daemon-reload
sudo systemctl enable hotspot.service
```

### 4. Start the service immediately (optional)
```bash
sudo systemctl start hotspot.service
```

### 5. Verify the service status
You can check if the service is running correctly with:
```bash
sudo systemctl status hotspot.service 
sudo systemctl status hotspot.service
journalctl -u hotspot.service
```