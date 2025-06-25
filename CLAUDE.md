# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AutoSDV is a ROS2-based autonomous vehicle platform built on top of Autoware. It runs on Jetson Linux (AGX Orin) and provides educational/research autonomous driving capabilities.

## Essential Commands

### Build and Development
```bash
make checkout    # Initialize/update git submodules
make setup       # Install development environment via Ansible
make prepare     # Install ROS2 dependencies using rosdep
make build       # Build project with colcon (Release mode)
make clean       # Clean build artifacts
```

### Running the System
```bash
make launch      # Launch AutoSDV (runs ./launch.sh)
make controller  # Run manual keyboard control node
```

### Testing and Linting
```bash
colcon test                    # Run colcon tests
pre-commit run --all-files     # Run all linting checks
python3 -m pytest              # Run Python tests
```

### Docker Operations
```bash
cd docker && make build    # Build Docker container
cd docker && make run      # Run Docker container
```

### Documentation
```bash
cd book && make serve      # Start documentation server on port 3000
```

## Architecture

### Package Structure
- **src/launcher/** - Main system launch files
  - `autosdv_launch` - Primary launch package
  - `autosdv_sensor_kit_launch` - Sensor configuration launches
  - `autosdv_vehicle_launch` - Vehicle-specific launches

- **src/sensor_component/** - Sensor drivers
  - External IMU (MPU9250)
  - LiDAR drivers (Velodyne, Blickfeld, Robin-W)
  - ZED camera integration
  - GPS/GNSS drivers

- **src/vehicle/** - Vehicle interface and control
  - `autosdv_vehicle_interface` - CAN interface for vehicle control
  - `autosdv_vehicle_msgs` - Custom message definitions

- **src/param/** - Autoware parameter configurations
  - `individual_params` - Vehicle-specific Autoware parameters

### Key Technologies
- **ROS2 Humble** - Core framework
- **Autoware** - Autonomous driving stack (via submodules)
- **Colcon** - Build system
- **CAN bus** - Vehicle communication
- **Zenoh** - Distributed communication (for multi-box setups)

### Launch System
The main entry point is `launch.sh` which:
1. Sources ROS2 and local workspace
2. Launches AutoSDV with system monitoring on port 8080
3. Uses `autosdv.launch.xml` as the primary launch file

### Multi-Box Architecture
Recent development includes multi-box Autoware setup capabilities:
- Zenoh daemon for distributed communication
- Split launch files for distributed deployment
- Setup scripts in `scripts/setup-multibox-autoware.sh`

## Sensor Architecture

### Sensor Components (Drivers)
Located in `src/sensor_component/`, each sensor has its own ROS2 driver package:

- **Blickfeld Cube1 LiDAR** (`ros2_blickfeld_driver_src-v1.5.5`)
  - 3D LiDAR with 70° FOV
  - IP: 192.168.26.26 (Jetson: 192.168.26.1)
  - Supports multiple returns, IMU integration
  
- **ZED X Mini Camera** (`zed-ros2-wrapper`)
  - Stereo camera with depth sensing
  - Requires ZED SDK 4.2
  - Provides RGB, depth, and point cloud data
  
- **MPU9250 IMU** (`ros2_mpu9250_driver`)
  - 9-axis IMU (accel, gyro, magnetometer)
  - I2C interface on bus 1
  - Auto-calibration on startup
  
- **GNSS/GPS** (`gnss_locator`, `ros-nmea-reader`)
  - Supports Garmin GPS 18x 5Hz (default)
  - NMEA format parsing
  - Baud rate: 9600

### Sensor Kit Integration
Located in `src/sensor_kit/`, defines how sensors are mounted and launched together:

#### Coordinate Frames
```
base_link (vehicle)
└── sensor_kit_base_link
    ├── cube1 (Blickfeld LiDAR)
    ├── vlp32c (Velodyne LiDAR)
    ├── robin_w (Seyond LiDAR)
    ├── imu_link (MPU9250)
    ├── gnss_base_link
    └── zedxm_camera_link
```

#### Calibration Files
- `sensor_kit_calibration.yaml` - Sensor positions relative to sensor_kit_base_link
- `sensors_calibration.yaml` - Sensor kit position relative to vehicle base_link

#### Launch Structure
- Main: `sensing.launch.xml` - Launches all sensors
- Individual: `lidar.launch.xml`, `camera.launch.xml`, `imu.launch.xml`, `gnss.launch.xml`
- LiDAR fusion: `pointcloud_preprocessor.launch.py`

### Working with Sensors

#### Adding a New Sensor
1. Add driver package to `src/sensor_component/`
2. Define mount point in `sensor_kit.xacro`
3. Add calibration values to YAML files
4. Create launch file in sensor kit
5. Include in `sensing.launch.xml`

#### Sensor Configuration
- LiDAR IPs: Check `config/` directory
- Camera: ZED SDK handles calibration
- IMU: Bias correction via gyro_bias_estimator
- GNSS: MGRS projection for map coordinates

#### Common Sensor Commands
```bash
# Launch individual sensors
ros2 launch autosdv_sensor_kit_launch camera.launch.xml
ros2 launch autosdv_sensor_kit_launch lidar.launch.xml lidar_model:=cube1

# Check sensor data
ros2 topic echo /sensing/lidar/concatenated/pointcloud
ros2 topic echo /sensing/imu/imu_data
ros2 topic echo /sensing/gnss/pose
```

## Development Guidelines

### Pre-commit Hooks
The project uses extensive pre-commit hooks for code quality:
- Python: flake8, black, isort, pep257
- YAML/JSON: yamllint, validation checks
- Markdown: markdownlint
- Shell: shellcheck, shfmt
- ROS-specific: include guards, package.xml sorting

### Building Individual Packages
```bash
colcon build --packages-select <package_name>
```

### Common Issues
- Always run `make checkout` after cloning to get submodules
- Use `make prepare` when adding new dependencies
- The system requires proper CAN interface setup for vehicle control
- Port 8080 is used for system monitoring web interface
- Sensor networking: Ensure proper IP configuration for networked sensors