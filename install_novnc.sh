#!/bin/bash

#***********************************************************************
# Script to install and configure Vino VNC server on Jetson devices
#***********************************************************************


echo "Installing and configuring Vino VNC server on Jetson..."
echo "-----------------------------------------------"

echo "Installing Vino VNC server..."
sudo apt update
sudo apt install vino -y

echo "Creating symbolic link for systemd service..."
sudo ln -s ../vino-server.service /usr/lib/systemd/user/graphical-session.target.wants

echo "Configuring VNC server..."
gsettings set org.gnome.Vino prompt-enabled false
gsettings set org.gnome.Vino require-encryption false
gsettings set org.gnome.Vino enabled true

echo "Configuration complete. Rebooting system..."
sudo reboot