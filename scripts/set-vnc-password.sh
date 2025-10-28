#!/bin/bash
# Set VNC password on container startup

echo "Setting VNC password to: robotixx"
mkdir -p /home/ubuntu/.vnc
echo 'robotixx' | vncpasswd -f > /home/ubuntu/.vnc/passwd
chmod 600 /home/ubuntu/.vnc/passwd
chown ubuntu:ubuntu /home/ubuntu/.vnc/passwd
echo "VNC password set successfully"
