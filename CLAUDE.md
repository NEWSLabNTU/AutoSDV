# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
AutoSDV is a software-defined autonomous vehicle platform built on ROS 2 and Autoware for research and education. It supports multiple LiDAR configurations (Robin-W, Velodyne 32C, Blickfeld Cube1) and is designed for small-scale autonomous vehicles.

## Essential Commands

### Build System (ROS 2 with colcon)
- `make prepare` - Install ROS dependencies using rosdep
- `make build` - Build all ROS packages with colcon (Release mode, symlink-install)
- `make launch` - Launch the complete AutoSDV system
- `make controller` - Run keyboard manual control
- `make clean` - Remove build, install, and log directories (with confirmation)
- `make checkout` - Initialize and update all git submodules
- `make setup` - Set up development environment using Ansible scripts

### Manual Commands
- `source install/setup.bash` - Source the ROS workspace (required before running nodes)
- `colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` - Manual build
- `ros2 launch autosdv_launch autosdv.launch.yaml` - Launch main system
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