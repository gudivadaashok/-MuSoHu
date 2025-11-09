This document provides installation instructions for the MuSoHu project.

# Installation Instructions

## Installing VNC on Jetson Orin Nano
For headless VNC setup on Jetson Orin Nano, follow the detailed guide at:
https://mauroarcidiacono.github.io/jetson-headless-vnc/

## Installing ROS2 Humble on Ubuntu 22.04
https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html

## Install ZED SDK
Follow the instructions

## Install Jetson Orin CH341 USB to Serial Module Installation
https://www.youtube.com/watch?v=RHqSR3Wj_K0


## Source Setup for ROS2 Workspace

## WEB UI Setup Instructions

## Create Hotspot on Jetson Orin Nano

## Configure the services to start on boot and crash recovery

gsettings set org.gnome.Vino authentication-methods "['vnc']"
gsettings set org.gnome.Vino vnc-password $(echo -n 'Robotixx'|base64)

