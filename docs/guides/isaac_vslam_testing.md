# Testing NVIDIA Isaac Visual SLAM with ZED X Mini Camera

This tutorial guides you through testing NVIDIA Isaac ROS Visual SLAM independently on Jetson AGX Orin with the ZED X Mini stereo camera.

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Hardware Setup](#hardware-setup)
4. [Launching the System](#launching-the-system)
5. [Visualization](#visualization)
6. [Monitoring and Validation](#monitoring-and-validation)
7. [Troubleshooting](#troubleshooting)
8. [References](#references)

---

## Prerequisites

### Hardware Requirements
- **Jetson AGX Orin** with JetPack 6.x (or x86_64 with NVIDIA GPU)
- **ZED X Mini** stereo camera
- **GMSL cable connection** (for ZED X Mini)
- Sufficient lighting and textured environment for SLAM

### Software Requirements
- Ubuntu 22.04 (Jammy)
- ROS 2 Humble
- CUDA 12.x (included in JetPack, or installed separately on x86_64)
- **ZED SDK 5.x** installed
- AutoSDV workspace built

### Verify Prerequisites

```bash
# Check ROS 2 installation
ros2 --version
# Expected: ros2 doctor version 0.10.5

# Check CUDA
nvcc --version
# Expected: release 12.x

# Check ZED SDK 5.x
grep "^set(PACKAGE_VERSION" /usr/local/zed/zed-config-version.cmake
# Should show: set(PACKAGE_VERSION "5.x.x")

# Source ROS workspace
source /opt/ros/humble/setup.bash
cd ~/AutoSDV
source install/setup.bash
```

---

## Installation

### Installation Methods by Platform

**For Jetson AGX Orin (arm64):** Use the setup script (recommended) or APT packages
**For x86_64 systems:** Build from source (APT packages not available for amd64)

---

### For Jetson AGX Orin (arm64) - Setup Script (Recommended)

The easiest way to install Isaac ROS on Jetson is via the AutoSDV setup script:

```bash
cd ~/AutoSDV
./setup.sh isaac-ros
```

This automatically:
- Configures the NVIDIA Isaac ROS APT repository
- Installs `ros-humble-isaac-ros-visual-slam`
- Installs `ros-humble-isaac-ros-image-proc`
- Verifies the installation

**Note:** Isaac ROS is also offered as an option during `./setup.sh` interactive setup.

### For Jetson AGX Orin (arm64) - Manual APT Installation

If you prefer manual installation:

**1. Configure Isaac ROS APT Repository:**

```bash
sudo apt update && sudo apt install -y curl gnupg
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg

echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-3 jammy main" | \
  sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list

sudo apt update
```

**2. Install Isaac ROS Visual SLAM Packages:**

```bash
sudo apt install -y ros-humble-isaac-ros-visual-slam ros-humble-isaac-ros-image-proc

# Verify installation
ros2 pkg list | grep isaac_ros
```

---

### For x86_64 (amd64) - Build from Source

**1. Clone Isaac ROS packages:**

```bash
cd ~/AutoSDV/src/localization
git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
```

**2. Build the workspace:**

```bash
cd ~/AutoSDV
make build
source install/setup.bash
```

**3. Verify installation:**

```bash
ros2 pkg list | grep isaac_ros
# Should show: isaac_ros_visual_slam, isaac_ros_image_proc
```

---

## Hardware Setup

### Connect ZED X Mini Camera

1. **Physical Connection**:
   - **Connect ZED X Mini to AGX Orin via GMSL cable**
   - The ZED X Mini uses GMSL (Gigabit Multimedia Serial Link) interface, NOT USB
   - Ensure secure connection of GMSL cable on both ends
   - Camera LED should illuminate when powered

2. **Verify Camera Detection**:
   ```bash
   # Test ZED SDK 5.x detection
   /usr/local/zed/tools/ZED_Explorer
   # Should detect ZED X Mini via GMSL and show live camera feed

   # Check ZED X Mini is detected
   /usr/local/zed/tools/ZED_Diagnostic
   # Should show camera model, serial number, and GMSL connection status
   ```

   **Expected output:**
   ```
   Camera Model: ZED X Mini
   Serial Number: XXXXXXXX
   Firmware: 2xxx
   Connection: GMSL
   SDK Version: 5.x.x
   ```

3. **Camera Placement**:
   - Mount camera firmly to vehicle (minimize vibration)
   - Ensure IMU axes align with vehicle frame coordinate system
   - Forward-facing with clear, unobstructed field of view
   - GMSL cable should be secured to prevent disconnection during motion

---

## Launching the System

### Method 1: Standalone Test Launch (Recommended)

This method launches everything in one command with pre-configured visualization:

```bash
# Source workspace
cd ~/repos/AutoSDV
source install/setup.bash

# Launch complete standalone test
ros2 launch autosdv_isaac_slam_launch standalone_test.launch.xml
```

This single command will start:
- ZED X Mini camera driver
- Isaac ROS Visual SLAM node
- Image format converters (RGB8 → Mono8)
- Topic relays (camera_info, IMU)
- RViz with pre-configured displays

### Method 2: Manual Step-by-Step Launch

For debugging or learning, launch components separately:

#### Terminal 1: ZED Camera Driver

```bash
source install/setup.bash

ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zedxm \
  camera_name:=zedxm \
  node_name:=zed_node
```

**Expected output:**
```
[INFO] [zed_wrapper]: Camera model: ZED X Mini
[INFO] [zed_wrapper]: Camera Serial Number: XXXXXXXX
[INFO] [zed_wrapper]: Camera Firmware: 1790
[INFO] [zed_wrapper]: Camera resolution: 1920x1200 @ 30 Hz
```

#### Terminal 2: Isaac Visual SLAM

```bash
source install/setup.bash

ros2 launch autosdv_isaac_slam_launch isaac_slam_with_zed.launch.xml \
  enable_imu_fusion:=true \
  enable_visualization:=true
```

**Expected output:**
```
[INFO] [visual_slam_node]: cuVSLAM version: 14.x
[INFO] [visual_slam_node]: Stereo camera mode enabled
[INFO] [visual_slam_node]: IMU fusion enabled
[INFO] [visual_slam_node]: Waiting for images...
```

#### Terminal 3: Verification

```bash
source install/setup.bash

# Check all topics are publishing
ros2 topic list | grep -E "visual_slam|zed_node"

# Monitor SLAM status
ros2 topic echo /isaac_slam/visual_slam_node/status
```

---

## Visualization

### Using RViz (Included in Standalone Launch)

If using Method 2 (manual launch), start RViz separately:

```bash
source install/setup.bash

# Launch with pre-configured displays
rviz2 -d $(ros2 pkg prefix autosdv_isaac_slam_launch)/share/autosdv_isaac_slam_launch/rviz/isaac_slam_test.rviz
```

---

## Monitoring and Validation

### Check SLAM Status

```bash
# Monitor SLAM status continuously
ros2 topic echo /isaac_slam/visual_slam_node/status

# Key fields:
#   vo_state: 1 = TRACKING (good), 2 = LOST (bad)
#   integrator_state_id: Increases when tracking is stable
#   num_observations: Number of features tracked
```

**Status Interpretation:**
- `vo_state: 1` → **SUCCESS**: SLAM is tracking correctly
- `vo_state: 2` → **LOST**: Tracking failure (see troubleshooting)
- `num_observations > 50` → Healthy feature tracking
- `num_observations < 20` → Poor tracking quality

### Check Topic Rates

```bash
# Image topics (should be ~30 Hz)
ros2 topic hz /sensing/camera/zedxm/zed_node/left/image_rect_color
ros2 topic hz /visual_slam/image_0

# Odometry output (should be ~30 Hz)
ros2 topic hz /isaac_slam/visual_slam_node/tracking/odometry

# IMU (should be ~200 Hz)
ros2 topic hz /sensing/camera/zedxm/zed_node/imu/data
```

### Verify Image Format Conversion

```bash
# Check ZED publishes RGB8
ros2 topic info /sensing/camera/zedxm/zed_node/left/image_rect_color --verbose
# Should show: encoding: rgb8

# Check Isaac receives Mono8
ros2 topic info /visual_slam/image_0 --verbose
# Should show: encoding: mono8
```

### Validate TF Tree

```bash
# Generate TF tree diagram
ros2 run tf2_tools view_frames

# View generated PDF
evince frames.pdf

# Expected tree:
#   map → odom → base_link → zedxm_camera_center → camera frames
```

### Test Movement

To validate SLAM is working:

1. **Initialization** (first 2-3 seconds):
   - Keep camera still
   - Wait for `vo_state: 1` in status
   - Check RViz shows landmarks appearing

2. **Translation Test**:
   - Move camera slowly forward/backward
   - Path should appear in RViz
   - Odometry pose should update smoothly

3. **Rotation Test**:
   - Rotate camera slowly left/right
   - Path should show rotation
   - Landmarks should stay stable in 3D view

4. **Loop Closure Test**:
   - Move camera in a loop back to start
   - SLAM should recognize the starting location
   - Path should close the loop (may have small drift)

---

## Troubleshooting

### No Odometry Output

**Check topic publishing:**
```bash
ros2 topic hz /visual_slam/image_0
ros2 topic hz /isaac_slam/visual_slam_node/tracking/odometry
```

**Verify image format:**
```bash
ros2 topic info /visual_slam/image_0 --verbose
# Should show: encoding: mono8
```

### Tracking Lost (vo_state: 2)

**Check SLAM status:**
```bash
ros2 topic echo /isaac_slam/visual_slam_node/status
# vo_state: 1 = tracking, 2 = lost
```

**Common causes:**
- Insufficient lighting or texture in environment
- Too rapid camera motion
- IMU data not publishing

**Verify IMU:**
```bash
ros2 topic hz /sensing/camera/zedxm/zed_node/imu/data
# Should be ~200 Hz
```

### Camera Not Detected

**Check camera connection:**
```bash
# Verify GMSL connection (not USB)
/usr/local/zed/tools/ZED_Diagnostic

# Test camera
/usr/local/zed/tools/ZED_Explorer
```

**Check ZED SDK:**
```bash
grep "^set(PACKAGE_VERSION" /usr/local/zed/zed-config-version.cmake
# Should show: set(PACKAGE_VERSION "5.x.x")
```

---

## Visual Global Localization (cuVGL)

For camera-only global localization (automatic initial pose without GNSS or manual input), see:
- **Roadmap**: `docs/roadmaps/visual_global_localization.md`
- **Usage**: `pose_source:=visual` (includes cuVGL + cuVSLAM)

cuVGL provides automatic startup localization using pre-built visual maps, eliminating the need for LiDAR-based NDT or manual pose initialization.

---

## References

- **Isaac ROS 3.2 Documentation**: https://nvidia-isaac-ros.github.io/v/release-3.2/
- **Isaac ROS Visual SLAM**: https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/
- **Isaac ROS Visual Global Localization**: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mapping_and_localization/isaac_ros_visual_global_localization/
- **ZED ROS2 Wrapper**: https://github.com/stereolabs/zed-ros2-wrapper
- **AutoSDV Isaac SLAM Integration**: `docs/design/isaac_vslam_integration.md`
- **AutoSDV Visual Global Localization Roadmap**: `docs/roadmaps/visual_global_localization.md`
