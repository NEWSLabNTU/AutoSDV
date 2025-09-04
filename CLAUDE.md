# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
AutoSDV is a software-defined autonomous vehicle platform built on ROS 2 and Autoware for research and education. It supports multiple LiDAR configurations (Robin-W, Velodyne 32C, Blickfeld Cube1) and is designed for small-scale autonomous vehicles.

## Essential Commands

### Build System (ROS 2 with colcon)
- `make prepare` - Install ROS dependencies using rosdep
- `make build` - Build all ROS packages with colcon (Release mode, symlink-install)
- `make launch` - Launch AutoSDV using systemd service (installs service if needed, then starts)
- `make stop` - Stop the running AutoSDV system
- `make restart` - Restart the AutoSDV system
- `make status` - Show AutoSDV system status and logs
- `make controller` - Run keyboard manual control
- `make clean` - Remove build, install, and log directories (with confirmation)
- `make checkout` - Initialize and update all git submodules
- `make setup` - Set up development environment using Ansible scripts

### AutoSDV Service Management (autosdv command)
After building (`make build`), the `autosdv` command is available:
- `autosdv install` - Install systemd user service (done automatically by `make launch`)
- `autosdv start` - Start the AutoSDV system
- `autosdv stop` - Stop the AutoSDV system
- `autosdv restart` - Restart the system
- `autosdv status` - Show system status and recent logs
- `autosdv enable` - Enable automatic startup at login
- `autosdv disable` - Disable automatic startup
- `autosdv monitor` - Open web monitor in browser (http://localhost:8080/)
- `autosdv uninstall` - Remove the systemd service

### Manual Commands
- `source install/setup.bash` - Source the ROS workspace (required before running nodes)
- `colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` - Manual build
- `rosdep install -y --from-paths src --ignore-src -r` - Install dependencies

## Architecture Overview

### Core Structure
- **src/launcher/autosdv_launch/** - Main launch configurations and system monitor
  - Provides web-based system monitor at http://localhost:8080/
  - Main launch file: `autosdv.launch.yaml`
- **src/param/autoware_individual_params/** - Parameter configurations for different sensor kits
- **src/sensor_kit/autosdv_sensor_kit_launch/** - Sensor integration and launch files
- **src/vehicle/autosdv_vehicle_launch/** - Vehicle interface and description
- **src/sensor_component/external/** - External sensor drivers (submodules)

### Key Submodules (8 total)
- autoware_manual_control - Keyboard control interface
- autosdv_sensor_kit_launch - Sensor kit configurations
- gnss_locator - GNSS positioning
- zed-ros2-wrapper - ZED camera integration
- ros2_mpu9250_driver - IMU driver
- ros-nmea-reader - NMEA GPS data parser

### Data Structure
- **data/COSS-map-planning/** - Default map data
- **data/models/** - ML models (YOLOX, CenterPoint, TensorRT)
- **data/zed-sdk/** - ZED camera SDK and calibration

### Build Artifacts
- **build/** - Compiled binaries (gitignored)
- **install/** - Installed packages and setup files
- **log/** - Build and runtime logs

## Development Workflow

### LiDAR Sensor Kits
The platform supports three main configurations:
1. **Robin-W Solid-State LiDAR Kit** - Compact solid-state solution
2. **Velodyne 32C LiDAR Kit** - Traditional spinning LiDAR
3. **Blickfeld Cube1 + MOXA 5G Kit** - Cube1 LiDAR with 5G connectivity

Sensor configurations are in `src/param/autoware_individual_params/individual_params/config/default/autosdv_sensor_kit/`

### Launch System
- Main launch uses Autoware's standard launch system
- Vehicle model: `autosdv_vehicle`
- Sensor model: `autosdv_sensor_kit`
- Default map: `./data/COSS-map-planning`

### Sensor Configuration
AutoSDV supports flexible sensor configurations through launch parameters:

#### LiDAR Models
```bash
# Robin-W Solid-State LiDAR (default)
make launch ARGS="lidar_model:=robin-w"

# Velodyne VLP-32C LiDAR
make launch ARGS="lidar_model:=vlp32c"

# Blickfeld Cube1 LiDAR
make launch ARGS="lidar_model:=cube1"
```

#### Camera Models
```bash
# ZED stereo camera (default)
make launch ARGS="camera_model:=zedxm"

# USB cameras
make launch ARGS="camera_model:=usb"

# No camera
make launch ARGS="camera_model:=none"
```

#### GNSS Receivers
```bash
# Garmin GNSS (default)
make launch ARGS="gnss_receiver:=garmin"

# u-blox GNSS
make launch ARGS="gnss_receiver:=ublox"

# Septentrio GNSS
make launch ARGS="gnss_receiver:=septentrio"
```

#### Indoor Operation (No GPS)
For indoor testing without GNSS, use manual pose initialization via RViz:
```bash
# Disable GNSS for indoor operation
make launch ARGS="use_gnss:=false"
```

When running indoors:
1. The system uses NDT localization instead of GNSS
2. Use RViz's "2D Pose Estimate" tool to set initial vehicle position
3. Click and drag on the map to set pose and orientation
4. The `/initialpose` topic receives the manual pose input

#### Combined Configuration Example
```bash
# Indoor setup with specific sensors
make launch ARGS="lidar_model:=robin-w camera_model:=usb use_gnss:=false"
```

### Python Packages
Python packages follow ROS 2 conventions with:
- Standard setup.py/setup.cfg structure
- Test files for copyright, flake8, pep257
- Resource directories for ROS package discovery

## Important Notes
- Always source ROS environment: `source /opt/ros/humble/setup.bash`
- Requires ROS 2 Humble distribution
- Built for Ubuntu with NVIDIA GPU support
- Uses colcon build system (not catkin)
- Symlink installs enabled for faster development iteration
- System monitor available at http://localhost:8080/ when launched

## System Management

### Systemd Service Integration
- AutoSDV now runs as a systemd user service for better process management
- Service is automatically installed on first `make launch`
- Provides clean shutdown with no orphan processes
- Logs accessible via `autosdv status` or `systemctl --user status autosdv`
- Service is NOT enabled for automatic startup by default (use `autosdv enable` if needed)

### Process Management
- The system handles multiple Ctrl-C presses gracefully
- First Ctrl-C: Graceful shutdown attempt
- Second Ctrl-C: Force shutdown all processes
- No orphan processes left after shutdown

### Known Issues and Solutions

#### Journal Logging
If `journalctl --user` doesn't show logs:
1. Run `sudo ./enable_journal.sh` to enable persistent journal storage
2. Log out and back in for group changes to take effect
3. Alternatively, use `systemctl --user status autosdv` to view logs

#### Network Monitor Error
- Network monitor may show socket connection errors
- This is a known non-critical issue related to AWS Greengrass
- Can be safely ignored - doesn't affect system functionality

## Seyond Robin-W LiDAR Integration

### PointXYZIRC Format Support
The Seyond Robin-W driver has been modified to output Autoware-compatible PointXYZIRC format:
- Located in: `src/sensor_component/external/seyond_ros_driver/`
- CMakeLists.txt: `set(POINT_TYPE PointXYZIRC)`
- Custom point type defined in: `src/driver/point_xyzirc.h`
- Field mapping:
  - `x, y, z`: Position (FLOAT32)
  - `intensity`: Intensity value (FLOAT32)
  - `return_type`: Return type (UINT8) - 1=strongest/first, 2=last/second
  - `ring`: Channel/scanning line ID (UINT16)

### Robin-W Coordinate Transformation
The Robin-W uses a non-standard coordinate system that needs transformation:
- Robin-W native: X:up, Y:right, Z:forward
- ROS standard (REP-103): X:forward, Y:left, Z:up
- Transformation configured in: `sensor_kit_calibration.yaml`
  - roll: 3.14159 (180°)
  - pitch: -1.5708 (-90°)
  - yaw: 0.0

### Network Configuration
- Robin-W default IP: 172.168.1.10
- Configure in: `autosdv_sensor_kit_launch/launch/lidar.launch.xml`

## TensorRT Model Compilation

### First Run Behavior
On first launch, TensorRT will compile ONNX models to optimized CUDA engines:
- This process can take 10-30 minutes depending on hardware
- Compiled engines are cached in `./data/` directory
- Key models:
  - `lidar_centerpoint/pts_voxel_encoder_centerpoint_tiny.engine`
  - `lidar_centerpoint/pts_backbone_neck_head_centerpoint_tiny.engine`
  - Traffic light classifiers (if enabled)

### Optimized Perception Configuration
For faster startup and LiDAR-only operation, configure in `autosdv.launch.yaml`:
```yaml
- name: perception_mode
  value: "lidar"
- name: use_traffic_light_recognition
  value: "false"
- name: use_detection_by_tracker
  value: "false"
- name: use_image_segmentation_based_filter
  value: "false"
```

## Recent Updates
- Fixed ROS2 node discovery in systemd service with Autoware environment variables
- Added flexible sensor configuration parameters (lidar_model, camera_model, gnss_receiver, use_gnss)
- Integrated Seyond Robin-W LiDAR with PointXYZIRC format compatibility
- Fixed Robin-W coordinate transformation for proper pointcloud orientation
- Optimized perception pipeline for LiDAR-only mode to reduce TensorRT compilation
- Added data_path parameter to correctly locate ML models in ./data directory
- Fixed systemd template to use .in file instead of embedded Python string
- Configured ZED camera to use shared pointcloud_container for zero-copy I/O
- Updated web monitor to track correct camera topics and removed unused traffic light topics
- With --symlink-install flag in colcon build, edits on yaml, xml, py source files immediately apply if the file was installed earlier. There is no need to rebuild. In case can you create a new file, you need to run colcon build again to create the symlink in the install/ dir.