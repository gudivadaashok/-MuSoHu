# ZED Camera Test Results

## Test Date: October 29, 2025
## Update: DNS Configuration Added ✅

### 1. Hardware Detection ✅
- **ZED Camera Model**: ZED 2i
- **USB Detection**: PASS
  ```
  Bus 002 Device 003: ID 2b03:f880 STEREOLABS ZED 2i
  Bus 001 Device 006: ID 2b03:f881 STEREOLABS ZED-2i HID INTERFACE
  ```
- **Serial Number**: 34359395
- **Firmware Version**: 1523

### 2. Software Build ✅
- **ZED SDK Version**: 5.1.0 - Build 106231_f4e0c540
- **ROS2 Packages Built Successfully**:
  - `zed_components` ✅
  - `zed_wrapper` ✅
  - `zed_msgs` ✅

### 3. Dependencies Installed ✅
- geographic_msgs
- backward_ros
- robot_localization
- libturbojpeg0-dev
- CUDA toolkit (configured at /usr/local/cuda)

### 4. Camera Initialization ⚠️
- **Status**: Partial Success
- **Issue**: Calibration file download failed
- **Reason**: Container lacks internet connectivity to download factory calibration
- **Camera Opened**: YES
- **Video Mode**: HD1080@15fps

### 5. ROS2 Node Status ⚠️
- **Node Loaded**: YES
- **Services Advertised**: 12 services available
  - `/zed/zed_node/reset_odometry`
  - `/zed/zed_node/reset_pos_tracking`
  - `/zed/zed_node/set_pose`
  - `/zed/zed_node/save_area_memory`
  - `/zed/zed_node/enable_obj_det`
  - `/zed/zed_node/enable_body_trk`
  - `/zed/zed_node/enable_mapping`
  - `/zed/zed_node/enable_streaming`
  - `/zed/zed_node/start_svo_rec`
  - `/zed/zed_node/stop_svo_rec`
  - `/zed/zed_node/set_roi`
  - `/zed/zed_node/reset_roi`

### 6. Configuration Summary
- **Camera Model**: zed2i
- **Resolution**: HD1080 (downscaled by 2 for publishing)
- **Frame Rate**: 15 Hz
- **Depth Mode**: NEURAL LIGHT
- **Positional Tracking**: Enabled (GEN 3)
- **IMU Fusion**: Enabled
- **Point Cloud**: Enabled (10 Hz)

### Next Steps to Resolve Calibration Issue

#### ✅ **Completed: DNS Configuration Added**
Added Google DNS servers (8.8.8.8, 8.8.4.4) to docker-compose.yml
Container now has internet connectivity verified.

#### ⚠️ **Current Issue: Calibration Download Failing**
The Stereolabs calibration download endpoint for SN 34359395 returns a 302 redirect to the main website instead of providing the calibration file. This suggests:
1. The calibration file for this specific serial number may not be in their online database
2. The camera may be a newer model that requires manual calibration
3. The download API endpoint may have changed

#### **Recommended Solutions:**

**Option 1: Contact Stereolabs Support** (RECOMMENDED)
Contact Stereolabs support at support@stereolabs.com with:
- Camera Model: ZED 2i
- Serial Number: 34359395
- Request: Factory calibration file
They can provide the SN34359395.conf file directly.

**Option 2: Use ZED Calibration Tool on Host**
If you have the ZED SDK installed on your Jetson host:
```bash
# Run ZED Calibration tool on host
/usr/local/zed/tools/ZED_Calibration

# After calibration, copy the file to container
sudo cp ~/.zed/settings/SN34359395.conf /tmp/
docker cp /tmp/SN34359395.conf ros2_vnc:/home/ubuntu/.zed/settings/
```

**Option 3: Manual Stereo Calibration**
Use ROS2 camera_calibration package to manually calibrate the stereo pair (advanced, not recommended unless necessary).

### Test Commands Used

1. **Check Camera Detection**:
   ```bash
   docker exec ros2_vnc bash -c "lsusb | grep -i zed"
   ```

2. **Build ZED Packages**:
   ```bash
   docker exec ros2_vnc bash -c "cd /home/ubuntu/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --packages-select zed_wrapper zed_components --parallel-workers 4"
   ```

3. **Launch ZED Camera**:
   ```bash
   docker exec ros2_vnc bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i"
   ```

4. **Check ROS2 Topics**:
   ```bash
   docker exec ros2_vnc bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && ros2 topic list"
   ```

## Conclusion

✅ **Build**: Successful  
✅ **Hardware Detection**: Successful  
⚠️  **Calibration**: Needs internet access or manual file transfer  
✅ **ROS2 Integration**: Functional (node loads, services advertised)  

The ZED camera ROS2 wrapper has been successfully built and integrated. The only remaining issue is the calibration file download, which can be resolved using one of the three options listed above.
