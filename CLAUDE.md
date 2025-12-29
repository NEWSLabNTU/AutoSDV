# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
AutoSDV is a software-defined autonomous vehicle platform built on ROS 2 and Autoware for research and education. It supports multiple LiDAR configurations (Robin-W, Velodyne 32C, Blickfeld Cube1) and is designed for small-scale autonomous vehicles.

## Essential Commands

### Build System (ROS 2 with colcon)
- `make setup` or `./setup.sh` - Run interactive setup (installs ROS 2, dependencies, etc.)
  - Interactive wizard asks about optional components (Autoware Debian packages)
  - Installs: ROS 2 Humble, dev tools, GeographicLib, Python deps, rosdep dependencies
  - Use `./setup.sh status` to check installation status
  - Use `./setup.sh <recipe>` to run specific setup steps (e.g., `./setup.sh ros2`)
- `make build` - Build all ROS packages with colcon (Release mode, symlink-install)
- `make test` - Run tests for all packages in src/ directory and show results
- `make launch` - Launch AutoSDV system with web UI at http://localhost:8081
  - Logs are saved to `play_log/latest/` directory
  - Use Ctrl+C to stop the system
- `make run-controller` - Run keyboard manual control
- `make play-basic-control` - Launch vehicle control test (basic_control.launch.xml)
- `make run-straight-10m` - Run trajectory player with straight_10m.yaml
- `make run-circle` - Run trajectory player with circle.yaml
- `make run-rviz` - Launch RViz with AutoSDV configuration
- `make clean` - Remove build, install, and log directories (with confirmation)
- `make checkout` - Initialize and update all git submodules

### Simulation Commands
- `make start-simulation` - Start Autoware logging simulator using systemd
- `make stop-simulation` - Stop the running simulation
- `make status-simulation` - Show simulation status
- `make logs-simulation` - Follow simulation logs

### Manual Commands
- `source install/setup.bash` - Source the ROS workspace (required before running nodes)
- `colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` - Manual build
- `colcon test --base-paths src --return-code-on-test-failure` - Manual test (src/ packages only)
- `colcon test-result --verbose` - Show detailed test results
- `rosdep install -y --from-paths src --ignore-src -r` - Install dependencies (handled automatically by setup script)

## Architecture Overview

### Core Structure
- **src/launcher/autosdv_launch/** - Main launch configurations
  - Main launch file: `autosdv.launch.yaml`
  - Web-based node management UI at http://localhost:8081/ (via play_launch)
- **src/localization/** - Localization-related packages
  - `odometry_pose_bridge/` - Generic odometry-to-pose converter
  - `autosdv_isaac_slam_launch/` - Isaac ROS Visual SLAM integration
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

### Documentation Structure
- **docs/design/** - Design documents and architecture decisions
  - `isaac_vslam_integration.md` - Isaac ROS Visual SLAM integration design
- **docs/guides/** - Operational guides and testing procedures
  - `control_testing.md` - Control system testing procedures
  - `isaac_vslam_testing.md` - Isaac SLAM standalone testing tutorial
  - `mrm_configuration.md` - MRM (Minimum Risk Maneuver) configuration guide
  - `simulation_testing.md` - Simulation and rosbag replay guide
- **docs/research/** - Empirical research studies and experimental findings
  - `localization/ndt_parameter_tuning_coss_map.md` - NDT parameter tuning for COSS map
- **docs/roadmaps/** - Feature roadmaps and development plans
  - `ar_tag_integration/` - AR tag-based localization roadmap

### Build Artifacts
- **build/** - Compiled binaries (gitignored)
- **install/** - Installed packages and setup files
- **log/** - Build and runtime logs

## Development Workflow

### Development Practices

**Temporary Files**
- Write temporary test files and logs to `$PROJECT_ROOT/tmp/` directory
- The `tmp/` directory is gitignored and safe for disposable files
- Examples: test logs, debug outputs, temporary scripts, tutorials, notes
- Do NOT use system `/tmp/` - use project-local `./tmp/` instead
- **Tool usage**: Use the Write tool to create files in `./tmp/`, not bash `cat` with heredocs or echo redirects
  - Example: `Write(file_path="/home/aeon/repos/AutoSDV/2025.02/tmp/tutorial.md", content="...")`
  - Avoid: `cat << 'EOF' > /tmp/file` or `echo "text" > /tmp/file`

### Setup System

The project uses a modular setup system based on `just` (command runner) with an interactive wrapper script:

**Structure:**
- `setup/setup.sh` - Interactive wrapper script (symlinked to `./setup.sh` at project root)
- `setup/justfile` - Recipe definitions for individual setup steps
- `setup/scripts/` - Individual setup scripts for each component
- `setup/files/` - Configuration files (udev rules, etc.)
- `setup/.markers/` - Marker files to track completed steps (prevents re-running)

**Usage:**
```bash
# Interactive full setup (recommended for first-time setup)
./setup.sh

# Check what's already installed
./setup.sh status

# Run specific setup step
./setup.sh ros2
./setup.sh dev-tools
./setup.sh autoware-debian

# Reset all markers (force re-run all steps)
./setup.sh clean-markers
```

**Setup Steps (in order):**
1. `ros2` - Install ROS 2 Humble
2. `ros2-dev-tools` - Install colcon, rosdep, etc.
3. `gdown` - Google Drive downloader
4. `geographiclib` - Geographic coordinate tools
5. `pacmod` - AutonomouStuff repository
6. `dev-tools` - git-lfs, pre-commit, PlotJuggler
7. `blickfeld` - Blickfeld scanner library
8. `autoware-debian` - Autoware packages (optional, ~2-3 GB)
9. `python-deps` - AutoSDV Python dependencies
10. `ublox-udev` - u-blox GPS udev rules
11. `ros-deps` - ROS workspace dependencies (rosdep)

**Features:**
- Marker-based checkpoint system (resume from failure)
- Interactive prompts for optional components
- Ctrl-C to cancel during interactive questions
- Works from any directory (resolves symlinks automatically)
- Sudo keep-alive for long installations

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
AutoSDV supports flexible sensor configurations through both predefined sensor suites and individual sensor parameters.

#### Sensor Suites (Recommended)
Predefined sensor suites provide convenient configurations for common hardware combinations:

```bash
# Robin-W LiDAR + ZED X Mini + ZED IMU (integrated)
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=robin_zed

# Robin-W LiDAR + ZED X Mini + MPU9250 IMU
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=robin_zed_mpu

# Velodyne VLP-32C + ZED X Mini + ZED IMU (integrated)
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed

# Velodyne VLP-32C + ZED X Mini + MPU9250 IMU
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed_mpu

# Velodyne VLP-32C + ZED IMU only (no camera streams, lightweight)
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed_imu

# Blickfeld Cube1 + USB Cameras + MPU9250 IMU
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=cube1_usb

# Custom configuration (use individual parameters)
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=custom lidar_model:=robin-w camera_model:=zedxm imu_source:=mpu9250
```

**Suite Configuration Matrix:**

| Suite | LiDAR | Camera | IMU | GNSS | ZED OD |
|-------|-------|--------|-----|------|--------|
| robin_zed | Robin-W | ZED X Mini | ZED built-in | u-blox | Yes |
| robin_zed_mpu | Robin-W | ZED X Mini | MPU9250 | u-blox | Yes |
| vlp32c_zed | Velodyne 32C | ZED X Mini | ZED built-in | u-blox | Yes |
| vlp32c_zed_mpu | Velodyne 32C | ZED X Mini | MPU9250 | u-blox | Yes |
| vlp32c_zed_imu | Velodyne 32C | None | ZED built-in | u-blox | No |
| cube1_usb | Cube1 | USB cameras | MPU9250 | u-blox | No |
| custom | (manual) | (manual) | (manual) | (manual) | (manual) |

**Overriding Suite Defaults:**
```bash
# Use robin_zed suite but disable GNSS for indoor testing
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=robin_zed use_gnss:=false

# Use vlp32c_zed suite but switch to Septentrio GNSS
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed gnss_receiver:=septentrio
```

#### Individual Sensor Parameters (Custom Configuration)
When using `sensor_suite:=custom` (default), configure sensors individually:

##### IMU Sources
```bash
# MPU9250 external I2C IMU (default for custom suite)
ros2 launch autosdv_launch autosdv.launch.yaml imu_source:=mpu9250

# ZED camera built-in IMU (factory-calibrated, more stable)
ros2 launch autosdv_launch autosdv.launch.yaml imu_source:=zed camera_model:=zedxm
```

**Note:** When using ZED IMU (`imu_source:=zed`), the ZED camera must be used (`camera_model:=zedxm`). The system automatically relays IMU data from the camera node to avoid launching duplicate ZED drivers.

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

#### NTRIP/RTK Configuration (u-blox only)

AutoSDV supports RTK (Real-Time Kinematic) positioning for centimeter-level accuracy using NTRIP (Networked Transport of RTCM via Internet Protocol).

**Hardware Requirements:**
- u-blox ZED-F9R GNSS receiver (e.g., SimpleRTK2B Fusion board)
- Connected via USB (udev rules create `/dev/ublox-gps` symlink)
- Clear sky view for optimal satellite reception

**NTRIP Service:**
- **Default**: e-GNSS Taiwan VRS (Virtual Reference Station)
  - Server: 210.241.63.193:81
  - Mountpoint: Taiwan
  - Uses VRS technology for reduced differential errors
  - Credentials configured in `/src/sensor_kit/autosdv_sensor_kit_launch/launch/ntrip.launch.xml`

**Enable NTRIP:**
```bash
# Basic RTK setup with u-blox + NTRIP
make launch ARGS="gnss_receiver:=ublox use_ntrip:=true"

# Full outdoor autonomous setup with RTK
make launch ARGS="gnss_receiver:=ublox use_ntrip:=true lidar_model:=robin-w camera_model:=zedxm"
```

**How It Works:**
1. NTRIP client connects to e-GNSS Taiwan VRS server
2. Client subscribes to `/sensing/gnss/ublox/nmea_sentence` (rover position)
3. Server sends RTCM corrections via `/sensing/gnss/ntrip/rtcm` topic
4. u-blox driver applies corrections to achieve RTK fix
5. Output: cm-level accuracy on `/sensing/gnss/ublox/nav_sat_fix`

**Monitor RTK Status:**
```bash
# Check RTCM corrections being received
ros2 topic hz /sensing/gnss/ntrip/rtcm

# Check NMEA sentences from GPS
ros2 topic echo /sensing/gnss/ublox/nmea_sentence

# Check RTK fix quality (look for RTK_FIXED or RTK_FLOAT status)
ros2 topic echo /sensing/gnss/ublox/nav_sat_fix

# Check RTCM reception status
ros2 topic echo /sensing/gnss/ublox/rxmrtcm
```

**Configuration Files:**
- NTRIP client: `src/sensor_kit/autosdv_sensor_kit_launch/launch/ntrip.launch.xml`
- ZED-F9R config: `src/sensor_kit/autosdv_sensor_kit_launch/config/zed_f9r_rover.yaml`

**Troubleshooting:**
- **No RTCM data**: Check internet connection and e-GNSS credentials
- **Poor accuracy**: Ensure clear sky view (avoid buildings/trees)
- **Recommended location**: Open areas like 社科院 (College of Social Sciences) at NTU

**Alternative NTRIP Services:**
- RTK2go (free, community-based, less stable)
  - Previously tested mountpoint: `TWNTPEDATONG1` (Taipei)
  - Requires registration at rtk2go.com

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

### Localization Sources (pose_source)

AutoSDV supports multiple localization methods through the `pose_source` parameter:

#### NDT Scan Matching (Default)
```bash
# LiDAR-based localization using NDT algorithm (default)
make launch ARGS="pose_source:=ndt"
# or simply
make launch
```

**NDT Configuration:**
- Tuned for VLP-32C LiDAR on COSS map (NTU Campus)
- Resolution: 2.0m, Score threshold: 2.2, Voxel size: 0.5m, Points: 3000
- Performance: 95% pose acceptance, 7.4ms execution time
- See `docs/research/localization/ndt_parameter_tuning_coss_map.md` for empirical tuning results

#### Isaac ROS Visual SLAM (GPU-Accelerated)
```bash
# Stereo camera-based localization using NVIDIA Isaac ROS Visual SLAM
make launch ARGS="pose_source:=isaac use_gnss:=false camera_model:=zedxm"
```

**Isaac SLAM Features:**
- GPU-accelerated stereo visual-inertial odometry (cuVSLAM)
- Suitable for indoor/GNSS-denied environments
- Requires ZED X Mini stereo camera (GMSL connection) + IMU
- ZED SDK 5.x required
- Automatically launches when `pose_source:=isaac`

**Packages:**
- `odometry_pose_bridge` - Generic odometry-to-pose converter
- `autosdv_isaac_slam_launch` - Isaac SLAM + ZED integration

**Integration:**
- Image converters: RGB8 → Mono8 (Isaac SLAM requirement)
- Topic relays: Camera info and IMU data
- Bridge node: Converts Isaac SLAM odometry to Autoware pose format
- EKF fusion: Integrates with Autoware's localization pipeline

**Hardware Requirements:**
- ZED X Mini camera connected via GMSL cable (NOT USB)
- ZED SDK 5.x installed
- NVIDIA GPU with CUDA support

**Isaac ROS Installation:**
- ⚠️ APT packages available for **arm64 only** (Jetson AGX Orin)
- x86_64/amd64 users must build from source
- See `docs/guides/isaac_vslam_testing.md` for installation guide

**Status:**
- ✅ **Implementation Complete**: All code integrated and built
- ✅ **Standalone Testing Tutorial**: Available in `docs/guides/isaac_vslam_testing.md`
- ⏸️ **Testing Deferred**: Requires stereo camera data (rosbag or hardware with GMSL)
- 📖 **Documentation**: See `docs/design/isaac_vslam_integration.md` and `docs/guides/simulation_testing.md`

**Testing Notes:**
- Standalone test: `ros2 launch autosdv_isaac_slam_launch standalone_test.launch.py`
- Standard Autoware rosbags do not include camera images (privacy)
- For testing: Use Bus-ODD dataset, record custom rosbag, or use CARLA simulator
- See `docs/guides/simulation_testing.md` for rosbag replay setup
- See `docs/guides/isaac_vslam_testing.md` for complete testing tutorial

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
- Web-based node management UI available at http://localhost:8081/ when launched

## System Management

### Launch System (play_launch)
- AutoSDV uses `play_launch` for launching and managing nodes
- Web UI available at http://localhost:8081/ for node management
- Logs are saved to `play_log/latest/` directory
- Use Ctrl+C to stop the system gracefully

### Process Management
- The system handles Ctrl-C gracefully for clean shutdown
- No orphan processes left after shutdown

### Known Issues and Solutions

#### MRM False Emergency Stops (RESOLVED)
- **Issue**: Vehicle would unexpectedly trigger emergency stop during normal outdoor operation
- **Cause**: Default localization accuracy thresholds (1.5m position, 0.3m lateral) too strict for VLP-32C LiDAR
- **Solution**: Disabled `/autoware/localization/accuracy` diagnostic check in `config/system/diagnostics/localization.yaml`
- **Status**: Localization quality still monitored via NDT scan_matching_status (score threshold 2.2)
- **Reference**: See `docs/guides/mrm_configuration.md` for complete MRM system documentation
- **Monitoring**: Use `ros2 topic echo /system/fail_safe/mrm_state` to monitor MRM status

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

## Vehicle Interface Calibration

### Motor PWM Control
- **Stop position**: PWM = 370 (not 340 as previously configured)
- **Forward motion**: PWM > 370 (e.g., 380, 390, 400+)
- **Reverse motion**: PWM < 370 (e.g., 360, 350, 340-)
- **Brake position**: PWM = 340 (used when transitioning from forward to stop/reverse)

### Steering PWM Control
- **Center position**: PWM = 400
- **Left limit**: PWM = 350 (50 units from center)
- **Right limit**: PWM = 450 (50 units from center)
- **Range**: Symmetrical ±50 PWM units from center

### Brake Sequence for Reverse
When transitioning from forward to reverse:
1. If moving forward (PWM > 370), set to 340 to engage brake
2. Return to 370 (stop position)
3. Then decrease below 370 for reverse motion

### Testing Tools
- `/home/jetson/AutoSDV/motor_pwm_interactive.py` - Interactive PWM control for testing
- `/home/jetson/AutoSDV/stop_motor.py` - Emergency stop script (sets motor to 370)
- `/home/jetson/AutoSDV/test_steering_pwm.py` - Steering calibration tool
- `./scripts/control/gpio_speed.py` - Non-ROS GPIO-based speed measurement (minimalist)
- `./scripts/control/gpio_read.py` - Simple GPIO pin state and event monitor for debugging

### Control System Testing
For comprehensive testing procedures, see `docs/guides/control_testing.md` and `PLOTJUGGLER_QUICKSTART.md`. 

**Key Commands:**
- `make test-control` - Launch PID speed control test in tmux (controller + speedometer + monitor)
- `make plot-test` - Launch PlotJuggler for real-time PID visualization
- `make controller` - Launch keyboard manual control interface (requires RViz "Local" mode)

**Debug Topics:**
- Actuator node: `/autosdv/actuator_node/debug/{control_values,pwm_values,pid_values}`
- PID speed control: `/pid_speed_control_node/debug/{control_values,pwm_values,pid_values}`

**PlotJuggler Workflow:**
1. Terminal 1: `make test-control` (launches PID controller)
2. Terminal 2: `make plot-test` (launches PlotJuggler)
3. In PlotJuggler: Click "Start" to stream topics
4. Use `w/s` keys to change target speed, observe response in real-time
5. Tune PID: `ros2 launch control_test pid_tuning.launch.xml kp:=10.0 ki:=0.2 kd:=0.5`

**PID Tuning Guidelines:**
- Slow response → Increase Kp
- Oscillation → Decrease Kp, increase Kd
- Steady-state error → Increase Ki
- Noisy control → Decrease all gains, increase speed filter
- Derivative noise → Enable differential_on_measurement, increase speed_filter_size

**Known Issues**:
- Steering direction is reversed (left/right inverted)
- For manual control: Set "Local" mode in RViz, use keyboard controller
- For autonomous: Set "Remote" mode in RViz, set pose and goal in RViz

## ZED Camera Integration

### SDK and Driver Versions
- **ZED SDK**: Version 5.1.2
- **ZED Link Duo Driver**: Version 1.3.2 for L4T 36.3.0
- **ZED ROS2 Wrapper**: Version 5.1.0 (humble-v5.1.0, from `src/sensor_component/external/zed-ros2-wrapper/`)
- **Supported models**: ZED, ZED M, ZED 2, ZED 2i, ZED X, ZED X Mini
- **Status**: ✅ Working - Namespace fix completed

### Python Launch File Namespace Handling
**Important**: The ZED Python launch file (`zed_camera.launch.py`) does NOT respect XML `<push-ros-namespace>` directives.

**Solution**: Load the ZED composable node directly via XML's `<load_composable_node>` instead of including the Python launch file. This gives full control over namespaces:

```xml
<!-- Container with ABSOLUTE namespace (bypasses push-ros-namespace) -->
<node_container pkg="rclcpp_components" exec="component_container_isolated"
                name="zed_container" namespace="/sensing/camera/$(var camera_name)">
  <param name="use_multi_threaded_executor" value="true"/>
</node_container>

<!-- Load composable node: namespace="camera" + name=camera_name = /sensing/camera/<name> -->
<load_composable_node target="/sensing/camera/$(var camera_name)/zed_container">
  <composable_node pkg="zed_components" plugin="stereolabs::ZedCamera"
                   name="$(var camera_name)" namespace="camera">
    <param from="$(find-pkg-share zed_wrapper)/config/common_stereo.yaml"/>
    <param from="$(find-pkg-share zed_wrapper)/config/$(var camera_model).yaml"/>
    <!-- ... other config files ... -->
  </composable_node>
</load_composable_node>
```

This creates:
- Container at `/sensing/camera/zedxm/zed_container` (absolute)
- ZED node at `/sensing/camera/zedxm` (clean path)
- Topics at `/sensing/camera/zedxm/<topic>` (no double nesting)

### Object Detection Integration
ZED camera object detection has been integrated with Autoware's perception pipeline. The system can operate in two modes:
1. **Normal mode** (default): ZED publishes colored point cloud for visualization
2. **Object detection mode**: ZED performs object detection and converts to Autoware format

### Configuration Files
- **Launch file**: `autosdv_sensor_kit_launch/launch/zed_with_object_detection.launch.xml`
  - Modular launch structure for ZED camera with optional object detection
  - Handles both normal and object detection modes
- **Config file**: `autosdv_sensor_kit_launch/config/zed_object_detection.yaml`
  - Object detection parameters (model, confidence threshold, tracking)
  - Point cloud settings to ensure colored point cloud is always published

### Namespace Structure
Topics follow Autoware convention with clean paths:
- ZED node: `/sensing/camera/zedxm`
- ZED objects: `/sensing/camera/zedxm/obj_det/objects`
- Autoware format: `/perception/object_recognition/detection/camera_objects`
- Colored point cloud: `/sensing/camera/zedxm/point_cloud/cloud_registered`
- IMU data: `/sensing/camera/zedxm/imu/data`

### Container Integration
- ZED camera runs as a composable node in a dedicated container at `/sensing/camera/<camera_name>/zed_container`
- The container uses an absolute namespace to bypass parent `push-ros-namespace` directives
- Composable node is loaded directly via XML's `<load_composable_node>` (not the Python launch file)

### Usage
```bash
# Normal operation with colored point cloud (default)
make launch

# Enable object detection
make launch ARGS="enable_zed_object_detection:=true"
```

### ZED X Camera Troubleshooting

#### Hardware Acceleration Required (VNC Sessions)
The ZED node requires OpenGL hardware acceleration to start. Running `make launch` in a standard VNC session (e.g., TigerVNC) will cause the ZED node to fail.

**Solution**: Use TurboVNC with VirtualGL for hardware-accelerated remote desktop:

```bash
# Start TurboVNC with VirtualGL (VGL_DISPLAY must point to GPU X server)
VGL_DISPLAY=:1 /opt/TurboVNC/bin/vncserver :2 -vgl
```

A systemd user service is configured to start this automatically on boot:
- Service file: `~/.config/systemd/user/turbovnc.service`
- Commands: `systemctl --user {start|stop|status|restart} turbovnc`

**Why this happens**: The ZED SDK uses OpenGL for GPU-accelerated image processing. Standard VNC provides only software rendering, which causes the ZED driver to fail during initialization.

#### Camera Freeze / Stream Failed to Start
If the ZED camera fails to start or freezes during operation, you may see errors like:
```
(Argus) Error Timeout: (propagating from src/rpc/socket/client/ClientSocketManager.cpp...)
[WARN] Error opening camera: CAMERA MOTION SENSORS NOT DETECTED
[WARN] Error opening camera: CAMERA STREAM FAILED TO START
```

Or in `zed_x_daemon` logs (`journalctl -u zed_x_daemon`):
```
[ZED-X Daemon] Received invalid message: "ZEDX#0#0#FROZEN"
```

**Recovery Procedure:**
```bash
# Restart zed_x_daemon (this is the key fix)
sudo service zed_x_daemon restart

# Wait at least 25 seconds for driver to reconfigure
sleep 25
```

**Note:** Restarting only `nvargus-daemon` is NOT sufficient. The `zed_x_daemon` must be restarted to properly reinitialize the GMSL camera connection.

#### Check Service Logs
```bash
# ZED X daemon logs (look for FROZEN messages)
journalctl -u zed_x_daemon --since "30 minutes ago"

# NVIDIA Argus daemon logs (camera framework)
journalctl -u nvargus-daemon --since "30 minutes ago"

# Kernel logs for hardware-level camera errors
journalctl -k | grep -i "zed\|gmsl\|ar0234"
```

#### Hardware Version Info
- **ZED SDK**: 5.1.2 (check with `ZED_Explorer --version`)
- **ZED Link Duo Driver**: 1.3.2 for L4T 36.3.0
- **Camera Model**: ZED X Mini (GMSL connection)

### Known Issues
- Detection box positions may not perfectly align with point cloud coordinates (coordinate transformation issue to be resolved in future update)

## Recent Updates
- **ZED Wrapper 5.1.0 Namespace Fix (COMPLETED)** (2025-12-23)
  - Updated zed-ros2-wrapper submodule to humble-v5.1.0
  - **Fixed**: Double `/sensing` namespace issue in ZED nodes
  - Solution: Load ZED composable node directly via XML `<load_composable_node>` instead of including Python launch
  - Root cause: Python launch files don't respect XML `push-ros-namespace`
  - Key: Container uses absolute namespace, composable node uses relative namespace
  - ZED node now correctly at `/sensing/camera/zedxm/zedxm`
  - Topics correctly at `/sensing/camera/zedxm/zedxm/...`
- **Refactored control_test Package with PID Tuning and PlotJuggler Integration** (2025-11-19)
  - **Package Restructuring**: Cleaned up control_test package structure
    - Removed backup directory (control_test_backup_direct_pwm)
    - Removed keyboard controller from launch files (run via `make controller` instead)
    - Created `control_command_service` node for service-based control testing without TTY
    - Added comprehensive README.md with usage examples for all nodes
  - **PID Speed Controller Improvements**: Enhanced pid_speed_control.py node
    - Added debug topic publishing for real-time visualization
    - Implemented speed filtering (moving average) to reduce sensor noise
    - Added PWM slew rate limiting for smoother control (configurable)
    - Fixed PID architecture: base PWM on forward/reverse start thresholds, not stop position
    - Enabled derivative-on-measurement mode to reduce noise sensitivity
    - Safe handling of uninitialized PID internal variables
  - **PlotJuggler Visualization**: Complete integration for PID tuning
    - Created `plotjuggler_pid_test.xml` layout with 5 plots (speed tracking, error, PWM, PID components, total output)
    - Added `make plot-test` command for easy PlotJuggler launch
    - Published debug topics at 100 Hz:
      - `/pid_speed_control_node/debug/control_values` - [target_speed, current_speed, error, target_steering, current_steering]
      - `/pid_speed_control_node/debug/pwm_values` - [motor_pwm, steering_pwm]
      - `/pid_speed_control_node/debug/pid_values` - [P_term, I_term, D_term, total_output]
    - Created `PLOTJUGGLER_QUICKSTART.md` comprehensive guide with tuning methodology
    - Fixed PlotJuggler detection script (check ROS package instead of command)
  - **PID Tuning Methodology**: Documented systematic approach
    - Identified and fixed derivative term noise amplification from sensor quantization
    - Increased speed filter from 2→10 samples to reduce measurement noise
    - Adjusted default gains: Kp=10.0, Ki=0.2, Kd=0.5 (optimized for noisy sensor)
    - Increased slew rate from 2→5 PWM/cycle for better response
    - Enabled differential_on_measurement to reduce noise kick
  - **Debug Topics Data Structure**: Well-documented message formats
    - control_values: Speed tracking and error monitoring
    - pwm_values: Hardware command verification
    - pid_values: P/I/D term analysis for tuning
  - **Testing Improvements**: Enhanced `make test-control` workflow
    - Launches tmux session with PID controller, speedometer, and monitor
    - Integrates with PlotJuggler for real-time visualization
    - Provides interactive tuning with keyboard control (w/s/a/d/x/c/q/h)
- **Implemented Sensor Suite System** (2025-11-17)
  - Added `sensor_suite` parameter with 6 predefined configurations (robin_zed, robin_zed_mpu, vlp32c_zed, vlp32c_zed_mpu, cube1_usb, custom)
  - Suite parameters can be overridden individually for flexibility
  - Centralized sensor configuration in sensing.launch.xml
  - Prevents ZED camera conflicts when both camera and IMU use ZED hardware
- **Added IMU Source Selection** (2025-11-17)
  - New `imu_source` parameter: mpu9250 (external I2C) or zed (ZED built-in)
  - Smart IMU relay: When camera uses ZED, IMU launch only relays data (no duplicate driver)
  - ZED IMU is factory-calibrated and more stable than MPU9250
  - Created minimal ZED config for IMU-only operation
- **Simplified Control Testing Launch** (2025-11-17)
  - Created control_testing.launch.yaml for minimal vehicle control testing
  - Includes only: vehicle interface, IMU, velocity converter (no perception/planning)
  - Fast startup, ideal for actuator calibration and control debugging
  - Supports both MPU9250 and ZED IMU sources
- Calibrated vehicle interface PWM values for motor and steering control
- Fixed motor stop position from 340 to 370 based on hardware testing
- Updated steering limits to symmetrical ±50 units from center (400)
- Created interactive PWM control tools for calibration and testing
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
- Added ZED object detection integration with Autoware converter
- Created modular launch structure for ZED camera with object detection support
- Fixed namespace structure in camera.launch.xml to prevent double namespacing
- Configured object detection to preserve colored point cloud functionality
- **Integrated Isaac ROS Visual SLAM for indoor localization** (2025-10-10)
  - Created `odometry_pose_bridge` package - generic odometry-to-pose converter (6/6 tests passed)
  - Created `autosdv_isaac_slam_launch` package - Isaac SLAM + ZED camera integration
  - Added `pose_source` parameter to autosdv.launch.yaml (ndt/isaac)
  - Conditional Isaac SLAM launch when `pose_source:=isaac`
  - GPU-accelerated stereo visual-inertial odometry using NVIDIA cuVSLAM
  - Implementation complete, testing deferred pending stereo camera data availability
  - Documentation: `docs/design/isaac_vslam_integration.md`, `docs/guides/simulation_testing.md`
  - Status: ✅ Code complete, ready for testing when camera data is available
- **Upgraded ZED SDK and ROS2 wrapper** (2025-10-29)
  - Upgraded ZED SDK from 4.x to 5.0.5
  - Updated zed-ros2-wrapper submodule to version 5.0
  - Fixed container_name parameter to exclude leading slash (ZED wrapper 5.0 requirement)
  - Container path construction: `/<namespace>/<container_name>` now works correctly
  - Fixed launch error: "Invalid service name: topic name must not contain repeated '/'"
  - Updated camera.launch.xml and zed_with_object_detection.launch.xml
- **Added GPIO testing utilities** (2025-11-12)
  - Created `scripts/control/gpio_speed.py` - Minimalist non-ROS speed measurement using GPIO
  - Created `scripts/control/gpio_read.py` - GPIO pin state and event monitor for debugging wheel sensors
  - Both tools use Jetson.GPIO library for direct hardware access without ROS dependencies
- **Created control system testing guide** (2025-11-12)
  - Comprehensive testing documentation in `docs/guides/control_testing.md`
  - Covers manual and autonomous control testing procedures
  - Documents debug topics: `/autosdv/actuator_node/debug/{control_values,pwm_values,pid_values}`
  - Troubleshooting section for common issues (steering reversal, PID not working)
  - Quick start with `make test-control` - launches system + controller + monitor in tmux
- **Integrated NTRIP/RTK for centimeter-level positioning** (2025-11-18)
  - Added NTRIP client support using LORD-MicroStrain `ros-humble-ntrip-client` package
  - Configured for e-GNSS Taiwan VRS (Virtual Reference Station) by default
  - Created ZED-F9R rover configuration: `config/zed_f9r_rover.yaml`
  - New launch parameter: `use_ntrip:=true` to enable RTK corrections
  - RTCM corrections flow: NTRIP server → `/sensing/gnss/ntrip/rtcm` → u-blox driver
  - NMEA feedback: u-blox → `/sensing/gnss/ublox/nmea_sentence` → NTRIP client
  - Added monitoring topics: `/sensing/gnss/ntrip/rtcm`, `/sensing/gnss/ublox/nmea_sentence`, `/sensing/gnss/ublox/rxmrtcm`
  - Hardware ready for SimpleRTK2B Fusion (ZED-F9R) when connected
  - Usage: `make launch ARGS="gnss_receiver:=ublox use_ntrip:=true"`
- **Migrated to play_launch and fixed ZED topic naming** (2025-12-22)
  - Replaced systemd service with `play_launch` for launching AutoSDV
  - Web UI for node management at http://localhost:8081/
  - Logs saved to `play_log/latest/` directory
  - Removed `make stop/restart/status` targets (use Ctrl+C to stop)
  - Fixed ZED camera topic naming mismatch:
    - ZED wrapper overrides `node_name` with `camera_name` when namespace is specified
    - Topics are now correctly at `/sensing/camera/zedxm/zedxm/...`
    - Updated all subscriber configs: isaac_slam, monitor_topics, imu.launch.xml, rviz
  - Added `use_mapless_mode` parameter for indoor operation without localization
- **Refactored setup system with interactive wizard** (2025-12-25)
  - Moved setup directory from `scripts/setup/` to `setup/` at project root
  - Removed `make prepare` recipe, integrated into setup script as `ros-deps` step
  - Added interactive wizard for optional components (Autoware Debian packages)
  - Created marker-based checkpoint system for resumable installation
  - Fixed symlink support: resolves actual script location with `readlink -f`
  - Fixed Ctrl-C handling in interactive questions (exit code 130)
  - Fixed ros-deps path: changed from `../..` to `..` relative to setup directory
  - Setup script works from any directory and through symlinks
  - Minimalist output style with clear status indicators (✓/→/○)
  - 11 modular setup steps: ros2, ros2-dev-tools, gdown, geographiclib, pacmod, dev-tools, blickfeld, autoware-debian, python-deps, ublox-udev, ros-deps
  - Usage: `./setup.sh` (interactive), `./setup.sh status` (check), `./setup.sh <recipe>` (specific step)
- **Documented NDT parameter tuning for COSS map** (2025-12-28)
  - Created comprehensive research documentation in `docs/research/localization/ndt_parameter_tuning_coss_map.md`
  - Empirical testing of 3 configurations with VLP-32C LiDAR on NTU Campus COSS map
  - **Final configuration for AGX Orin**: resolution 2.0m, threshold 2.2, voxel 0.5m, 3000 points
  - **Results**: 95% pose acceptance, 16% faster execution (7.4ms vs 8.8ms), 50% memory reduction
  - **Key findings**:
    - Higher NDT resolution (2.0m) compensates for VLP-32C sparse vertical sampling
    - Crop box ±20m eliminates distant building noise, improves yaw stability
    - Coarser voxel downsampling (0.5m) provides better efficiency without sacrificing robustness
  - Analysis includes: score distributions, execution times, rejection rates, point density effects
  - Documented trade-offs: statistical confidence vs geometric precision, CPU budget on embedded hardware
- **Disabled MRM localization accuracy check to prevent false emergency stops** (2025-12-28)
  - Modified `src/launcher/autosdv_launch/config/system/diagnostics/localization.yaml`
  - Commented out `/autoware/localization/accuracy` check that was triggering false MRM emergency stops
  - **Problem**: Default error ellipse thresholds (1.5m position, 0.3m lateral) too strict for VLP-32C
  - **Solution**: Disabled accuracy check; localization quality still monitored via NDT scan_matching_status
  - Created comprehensive MRM configuration guide: `docs/guides/mrm_configuration.md`
  - Documentation covers: MRM architecture, trigger conditions, monitoring topics, testing recommendations
  - Allows outdoor testing without false emergency stops from brief localization uncertainty
- **Organized documentation structure** (2025-12-28)
  - Created `docs/research/` directory for empirical research studies and experimental findings
  - Structure: `docs/research/localization/` for localization-specific research
  - Merged `config/README.md` into `autosdv_launch/README.md` for centralized package documentation
  - Updated autosdv_launch README with package structure, configuration categories, and related docs links