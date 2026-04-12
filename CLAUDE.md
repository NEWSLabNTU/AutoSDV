# CLAUDE.md

Guidance for Claude Code when working with this repository.

## Project Overview

AutoSDV is a software-defined autonomous vehicle platform built on ROS 2 and Autoware. Supports multiple LiDAR configurations (Robin-W, Velodyne 32C, Blickfeld Cube1) for small-scale autonomous vehicles.

## Versioning

AutoSDV uses [Semantic Versioning](https://semver.org/): `MAJOR.MINOR.PATCH[-PRERELEASE]`

### Single Source of Truth: `versions.yaml`

All version information is centralized in `versions.yaml` at the repo root:
- **AutoSDV version**: Project version and release channel (stable/development)
- **Autoware version**: Pinned Autoware base version
- **ROS**: Distribution, installation type, RMW implementation
- **NVIDIA stack**: CUDA, cuDNN, TensorRT versions (AMD64 and ARM64/Jetson)
- **JetPack/L4T**: Jetson platform versions
- **Tool versions**: clang-format, etc.
- **Package checksums**: SHA256 for verification

### Version Helper Scripts

```bash
# Get a specific version value
./scripts/version/get-version.sh autosdv.version      # Returns "0.1.0-dev"
./scripts/version/get-version.sh autoware.version     # Returns "1.5.0"
./scripts/version/get-version.sh nvidia_amd64.cuda    # Returns "12.3"

# Export all versions as environment variables
source ./scripts/version/export-versions.sh
echo $AUTOSDV_VERSION    # 0.1.0-dev
echo $AUTOWARE_VERSION   # 1.5.0
echo $CUDA_VERSION_AMD64 # 12.3
```

### Version Bumping Guidelines

| Change Type | Version Bump |
|-------------|--------------|
| Breaking changes (vehicle interface, sensor configs, launch API) | MAJOR |
| Autoware base upgrade | MAJOR |
| New sensor/feature support | MINOR |
| New launch parameters | MINOR |
| Bug fixes, parameter tuning | PATCH |
| Documentation only | PATCH |

### Branch Strategy

| Branch | Version | Channel |
|--------|---------|---------|
| `main` | `X.Y.Z` (stable) | `stable` |
| `develop` | `X.Y.Z-dev` | `development` |
| `release/X.Y` | `X.Y.Z-rc.N` | `stable` |

## Essential Commands

### Build & Run
```bash
./setup.sh              # Interactive setup (ROS 2, dependencies)
./setup.sh status       # Check installation status
just build              # Build all packages
just test               # Run tests
just launch             # Launch system (web UI: http://localhost:8081)
just launch ARGS="..."  # Launch with parameters
just clean              # Remove build artifacts
just checkout           # Update git submodules
just --list             # Show all available commands
```

### Tools
```bash
just tool-rviz          # Launch RViz
just tool-plotjuggler   # PlotJuggler visualization
just tool-controller    # Keyboard manual control
just tool-tui           # Drive monitor TUI (pose, speed, states)
```

### Control Testing
```bash
just control-basic      # Launch vehicle control test
just control-straight   # Run 10m straight trajectory
just control-circle     # Run circular trajectory
```

### Rosbag
```bash
just bag-record         # Record outdoor sensor topics
just bag-play           # Play most recent recording
```

### Simulation
```bash
just launch-sim-planning  # Autoware planning simulator (no sensors needed)
just launch-sim-logging   # Logging simulation (rosbag replay)
just sim-coss-park        # Full COSS Park simulation scenario
just download-data        # Download test rosbag (~2.8 GB)
```
See `docs/guides/simulation_testing.md` for the full simulation guide
(planning sim, rosbag replay, CARLA integration).

### Manual Build
```bash
source install/setup.bash
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Or use justfile
just build

# Build specific package (must include all standard flags)
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <package_name>
```

**Important**: Always use `--base-paths src` and other standard flags from `just build` when running colcon commands manually.

## Architecture

### Core Structure
```
src/
├── launcher/autosdv_launch/
│   ├── launch/                     # Main launch files
│   │   ├── autosdv.launch.yaml     # Main launch (real hardware)
│   │   ├── logging_simulation.launch.yaml  # Simulation/replay mode
│   │   └── autosdv_autoware.launch.xml     # Autoware wrapper
│   └── config/                     # Configuration files
│       ├── perception/preset/      # Perception presets
│       │   ├── lidar_only_preset.yaml
│       │   ├── camera_lidar_fusion_preset.yaml
│       │   └── minimal_preset.yaml
│       ├── localization/preset/    # Localization presets
│       │   ├── default_preset.yaml
│       │   └── eagleye_preset.yaml
│       ├── control/                # Control parameters
│       ├── localization/           # Localization parameters (NDT, EKF, etc.)
│       └── perception/             # Perception parameters
├── localization/                   # Localization packages
├── param/autoware_individual_params/  # Sensor configs
├── sensor_kit/autosdv_sensor_kit_launch/
├── vehicle/autosdv_vehicle_launch/
└── sensor_component/external/      # Sensor driver submodules
data/
├── COSS-map-planning/              # Default map
├── models/                         # ML models (YOLOX, CenterPoint)
└── zed-sdk/                        # ZED calibration
scripts/
└── leodrive-bus-launch/            # Leo Drive Bus-ODD dataset tools (submodule)
```

### Key Files
- Main launch: `src/launcher/autosdv_launch/launch/autosdv.launch.yaml`
- Sensor calibration: `src/param/.../autosdv_sensor_kit/sensor_kit_calibration.yaml`
- Preset files: `src/launcher/autosdv_launch/config/{perception,localization}/preset/`
- Web UI: http://localhost:8081 (via play_launch)

### Isaac Visual Localization

Camera-only localization using NVIDIA Isaac ROS (cuVGL + cuVSLAM). Eliminates need for LiDAR NDT.
Supported on both x86_64 (Ampere+ GPU) and ARM64 (Jetson). Install via `./setup.sh isaac-ros`.

**Architecture:**
```
pose_source:=visual
       │
       ├── cuVGL (Global Localization)
       │   └── Initial pose from visual map keyframes
       │   └── Publishes: /visual_localization/.../pose
       │
       ├── cuVSLAM (Visual Odometry)
       │   └── Continuous tracking via stereo camera + IMU
       │   └── Publishes: /localization/pose_estimator/pose_with_covariance
       │
       └── Pose Initializer Bridge
           └── Calls /localization/initialize on first cuVGL pose
```

**Packages:**
```
src/localization/autoware_isaac_localization/  # Standalone repo (github.com/NEWSLabNTU/autoware_isaac_localization)
├── autoware_isaac_localization_launch/        # cuVSLAM + cuVGL launch files
│   ├── launch/visual_localization.launch.xml  # Entry point for pose_source:=visual
│   ├── launch/isaac_slam.launch.py            # cuVSLAM wrapper
│   └── launch/visual_global_localization.launch.py
└── autoware_isaac_pose_bridge/                # cuVGL → Autoware pose initializer bridge
```

**Usage:**
```bash
# Full visual localization (requires visual map)
just launch pose_source:=visual visual_map_dir:=/path/to/visual_map

# Visual odometry only (no global init, manual pose required)
just launch pose_source:=isaac
```

**Creating Visual Maps:**
```bash
# 1. Record rosbag with ZED stereo + IMU
./scripts/visual-map/record.sh ./data/visual_maps/my_location

# 2. Create map (generates cuvgl_map/, cuvslam_map/, occupancy_map/)
./scripts/visual-map/create-map.sh ./data/visual_maps/my_location_recording
```

**Roadmap:** See `docs/roadmaps/visual_global_localization.md`

### CUDA NDT Localization

CUDA-accelerated NDT scan matching for faster localization on NVIDIA GPUs. This package is **maintained by AutoSDV** (not upstream Autoware).

**Package location:**
```
src/localization/cuda_ndt_matcher/  # AutoSDV-maintained, can be modified directly
├── cuda_ndt_matcher/               # Core CUDA NDT implementation
└── cuda_ndt_matcher_launch/        # Launch files and config
```

**Usage:**
```bash
# Use CUDA NDT instead of standard NDT
just launch pose_source:=cuda_ndt

# In logging simulation
just launch-sim-logging pose_source:=cuda_ndt
```

**Performance:** 1.3-1.6x faster than standard NDT, 57% less CPU usage on Jetson platforms.

**Development notes:**
- This package can be freely modified for AutoSDV-specific optimizations
- Uses same input/output interfaces as standard Autoware NDT
- Config files: `src/launcher/autosdv_launch/config/localization/ndt_scan_matcher/`

### Leo Drive Bus-ODD Dataset

The `scripts/leodrive-bus-launch` submodule provides tools for the [Leo Drive Bus-ODD dataset](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) - an Autoware dataset with camera streams for testing visual localization.

**Sensors in dataset:**
| Sensor | Model | Quantity |
|--------|-------|----------|
| LiDAR | Velodyne VLP16 | 1 (front) |
| LiDAR | Velodyne VLP32C | 2 (left, right) |
| Camera | Lucid Vision Triton 5.4MP | 3 |
| GNSS/INS | Applanix POS LV 120 | 1 |

**Usage:**
```bash
cd scripts/leodrive-bus-launch

# Full setup (download ~10.9GB + migrate to Autoware 1.5.0)
just setup

# Or step by step:
just setup-python    # Install rosbags Python package
just build           # Build ROS packages (applanix_msgs, sensor_kit, vehicle)
just download        # Download dataset
just migrate-all     # Migrate rosbags from autoware_auto_* to autoware_* msgs

# Play a migrated rosbag
just play data/all-sensors-bag1_migrated
```

**Packages included:**
- `leodrive_bus_sensor_kit_launch` - Sensor kit configuration
- `leodrive_bus_vehicle_launch` - Vehicle description (Isuzu bus)
- `applanix_msgs` - Applanix GNSS/INS message definitions

## Development

### Temporary Files
Write temp files to `./tmp/` (gitignored). Do NOT use system `/tmp/`.

### Build Notes
- With `--symlink-install`, edits to yaml/xml/py apply immediately (no rebuild needed)
- New files require rebuild to create symlinks
- First launch compiles TensorRT models (10-30 min)

### ROS 2 Launch Testing

**IMPORTANT**: When testing launch files, use `play_launch` instead of `ros2 launch`:

```bash
# PREFERRED: play_launch supports multi-stage kill (SIGINT → SIGTERM → SIGKILL)
play_launch launch autosdv_launch logging_simulation.launch.yaml

# If you must use ros2 launch directly, kill by process group (PGID) to avoid orphans:
ros2 launch autosdv_launch logging_simulation.launch.yaml &
LAUNCH_PID=$!
# ... do testing ...
kill -- -$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')  # Kill entire process group
```

**Killing play_launch by PGID** (for scripts/background processes):
```bash
# Start play_launch and capture PGID
setsid bash -c "play_launch launch autosdv_launch logging_simulation.launch.yaml" &
sleep 2
PLAY_PID=$(pgrep -f "play_launch.*logging_simulation" | head -1)
PGID=$(ps -o pgid= -p $PLAY_PID | tr -d " ")
echo $PGID > /tmp/sim_pgid.txt

# Later, kill by PGID (kills entire process group including child nodes)
kill -- -$(cat /tmp/sim_pgid.txt)

# Or directly if you have the PGID
kill -- -$PGID
```

**Why this matters**: Killing `ros2 launch` or `play_launch` with SIGKILL (`kill -9`) by PID only kills the parent process, leaving child processes (component_containers, nodes) as orphans. These orphan nodes continue running and consume resources. Always kill by PGID to terminate the entire process group.

**Cleaning up orphans** (if they occur):
```bash
# List orphan nodes
ros2 node list

# Kill all ROS-related processes
ps aux | grep -E "ros|component_container|autoware" | grep -v grep | awk '{print $2}' | xargs -r kill -9
```

### Python Packages
Standard ROS 2 conventions: setup.py/setup.cfg, test files for copyright/flake8/pep257.

### Setup Script Architecture

The setup system (`setup/`) uses a two-layer design:

1. **`setup.sh`** - Interactive wrapper that asks all questions upfront before any installation begins
2. **`justfile`** - Recipe definitions that perform actual installations

**Adding new optional components:**

1. Add installation script to `setup/scripts/install-<name>.sh`
2. Add recipe to `setup/justfile`:
   ```just
   # Direct recipe (for manual invocation)
   my-component: _init
       @just _run my-component "{{scripts_dir}}/install-my-component.sh"

   # Conditional recipe (for interactive setup)
   _setup-my-component:
       #!/usr/bin/env bash
       if [[ "${INSTALL_MY_COMPONENT}" == "y" ]]; then
           just my-component
       else
           printf "{{yellow}}⊘{{nc}} my-component skipped (user choice)\n"
       fi
   ```
3. Add `_setup-my-component` to the `setup:` recipe chain
4. Add question in `setup.sh` `interactive_setup()` function:
   ```bash
   INSTALL_MY_COMPONENT="n"
   printf "${YELLOW}Optional:${NC} My Component description\n"
   if ask_yes_no "Install My Component?" "n"; then
       INSTALL_MY_COMPONENT="y"
   fi
   export INSTALL_MY_COMPONENT="$INSTALL_MY_COMPONENT"
   ```
5. Update summary output and status display

**Key pattern:** Questions are asked at the start, choices exported as env vars, justfile conditionals execute based on those vars.

### Preset System

AutoSDV uses a **preset system** (following Autoware's pattern) to manage component-level configurations. Presets group related parameters for common use cases.

#### How Presets Work

**Preset files** are YAML launch files that define launch arguments:

```yaml
# config/perception/preset/lidar_only_preset.yaml
launch:
  - arg:
      name: perception_mode
      default: "lidar"
  - arg:
      name: use_traffic_light_recognition
      default: "false"
  # ... more args
```

**Main launch file** includes presets:

```yaml
# autosdv.launch.yaml
- arg:
    name: perception_preset
    default: "lidar_only"

- include:
    file: "$(find-pkg-share autosdv_launch)/config/perception/preset/$(var perception_preset)_preset.yaml"
```

**Benefits**:
- ✅ Select presets for convenience: `perception_preset:=camera_lidar_fusion`
- ✅ Override individual parameters for experimentation: `use_traffic_light_recognition:=true`
- ✅ Easy to extend: Add new preset file without modifying launch files

#### Creating Custom Presets

1. Copy existing preset: `cp lidar_only_preset.yaml custom_preset.yaml`
2. Modify parameter defaults in the new file
3. Use with: `just launch perception_preset:=custom`

**Note**: Preset files must use `<name>_preset.yaml` naming convention.

#### Available Presets

**Perception** (`config/perception/preset/`):
- `lidar_only` - LiDAR only, no camera features (default)
- `camera_lidar_fusion` - Camera + LiDAR with traffic light recognition
- `minimal` - Minimal features for development/debugging

**Localization** (`config/localization/preset/`):
- `default` - Gyro odometry twist estimation (default)
- `eagleye` - GNSS-based odometry (requires GNSS)

See `config/{perception,localization}/preset/README.md` for detailed documentation.

## Quick Reference

### Common Launch Parameters

#### Preset-Based Configuration (Recommended)
```bash
# Perception presets (controls perception mode and features)
perception_preset:=lidar_only           # Default: LiDAR only, no camera features
perception_preset:=camera_lidar_fusion  # Camera + LiDAR with traffic light recognition
perception_preset:=minimal              # Minimal features for development

# Localization presets (controls twist estimation)
localization_preset:=default            # Default: gyro_odom
localization_preset:=eagleye            # GNSS-based odometry (requires GNSS)

# Example: Use camera-lidar fusion
just launch perception_preset:=camera_lidar_fusion sensor_suite:=robin_zed
```

#### Sensor Configuration
```bash
# Sensor suites (predefined combinations)
sensor_suite:=robin_zed          # Robin-W + ZED + ZED IMU
sensor_suite:=vlp32c_zed         # Velodyne + ZED + ZED IMU
sensor_suite:=vlp32c_zed_imu     # Velodyne + ZED + ZED IMU + MPU9250

# Individual sensor overrides
lidar_model:=robin-w|vlp32c|cube1
camera_model:=zedxm|usb|none
imu_source:=mpu9250|zed
gnss_receiver:=garmin|ublox|septentrio
```

#### Localization (pose_source)
```bash
# pose_source options:
pose_source:=cuda_ndt  # Default: CUDA-accelerated NDT (1.3-1.6x faster, 57% less CPU on Jetson)
pose_source:=ndt       # Autoware NDT (OpenMP CPU, fallback)
pose_source:=isaac     # cuVSLAM visual odometry only (relative tracking, manual init)
pose_source:=visual    # cuVGL + cuVSLAM (camera-only, auto init from visual map)

# For visual localization, specify map directory:
visual_map_dir:=/path/to/visual_map  # Contains cuvgl_map/, cuvslam_map/
```

#### System Features
```bash
# Localization
use_gnss:=false                  # Indoor operation (no GNSS)
use_ntrip:=true                  # RTK positioning (ublox only)
use_mapless_mode:=true           # Indoor operation without localization

# Perception
enable_zed_object_detection:=true  # ZED camera object detection
launch_perception:=false           # Disable entire perception module

# Advanced: Override preset-defined parameters
perception_mode:=lidar                      # Override preset perception mode
use_traffic_light_recognition:=true        # Override preset setting
use_detection_by_tracker:=false            # Override preset setting
use_image_segmentation_based_filter:=false # Override preset setting
use_pointcloud_map:=true                   # Override preset setting
twist_source:=gyro_odom|eagleye            # Override preset twist source
```

### Vehicle Interface (Quick Ref)

**Motor PWM** (PCA9685 I2C, channel 0):
- Range: 280-460, Init: 370 (neutral), Brake: 340
- Forward: 371-460, Reverse: 280-369
- Multi-mode controller: Emergency Brake, Full Stop, Deadband Hold, Active Control (PID)

**Steering PWM** (PCA9685 I2C, channel 1):
- Range: 350-450, Init: 400 (center)
- Max angle: 0.349 rad ≈ 20°
- Dual-mode controller: Fallback (v<0.3m/s), Normal (yaw rate feedback)

**Velocity Sensing**:
- Hall effect sensor (KY-003) on GPIO
- Parameters: `params/velocity_report.yaml`

**Actuator Parameters**: `src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/actuator.yaml`

## Documentation

### Main Documentation Book (MkDocs)
- **Framework**: MkDocs with Material theme (following Autoware conventions)
- **Setup**: `cd book && just setup` (installs dependencies)
- **Build**: `cd book && just build` (builds to `site/`)
- **Serve**: `cd book && just serve` (http://localhost:3000)
- **Source**: `book/src/` (Markdown files)
- **Config**: `book/mkdocs.yml`

**Features**:
- ✅ Material Design theme
- ✅ Mermaid diagram support
- ✅ Multi-language (English + 繁體中文)
- ✅ Math rendering (MathJax)
- ✅ Search, dark mode, mobile responsive

**Key Guides** (in book):
- **Sensor Integration**: `book/src/guides/sensor-integration/`
  - Simple usage guide, Robin-W walkthrough, sensor-specific details
- **Vehicle Control**: `book/src/guides/vehicle-control/`
  - Overview, hardware, control details, tuning & testing
  - Multi-mode controllers, PCA9685 I2C, hall effect sensor, PID tuning

### Legacy Guides (docs/)
| Guide | Description |
|-------|-------------|
| [docs/guides/sensor_configuration.md](docs/guides/sensor_configuration.md) | Sensor suites, NTRIP/RTK, localization |
| [docs/guides/vehicle_calibration.md](docs/guides/vehicle_calibration.md) | PWM control, PID tuning, testing tools |
| [docs/guides/zed_camera.md](docs/guides/zed_camera.md) | ZED setup, troubleshooting |
| [docs/guides/lidar_integration.md](docs/guides/lidar_integration.md) | Robin-W, Velodyne, TensorRT |
| [docs/guides/control_testing.md](docs/guides/control_testing.md) | Control system testing procedures |
| [docs/guides/mrm_configuration.md](docs/guides/mrm_configuration.md) | MRM (emergency stop) configuration |
| [docs/guides/isaac_vslam_testing.md](docs/guides/isaac_vslam_testing.md) | Isaac SLAM testing |
| [docs/design/isaac_vslam_integration.md](docs/design/isaac_vslam_integration.md) | Isaac SLAM architecture |
| [docs/research/localization/ndt_parameter_tuning_coss_map.md](docs/research/localization/ndt_parameter_tuning_coss_map.md) | NDT tuning research |

## Known Issues

- **Steering reversed**: Left/right inverted in manual control
- **Network monitor errors**: AWS Greengrass socket errors (non-critical, ignore)
- **ZED in VNC**: Requires TurboVNC with VirtualGL for hardware acceleration
- **Isaac ROS GXF libraries**: If `pose_source:=visual` or `pose_source:=isaac` fails with "libgxf_*.so not found", the GXF library paths are not in `LD_LIBRARY_PATH`. Re-source the setup files:
  ```bash
  source /opt/ros/humble/setup.bash
  source /opt/autoware/1.5.0/setup.bash
  source install/setup.bash
  ```
  GXF libraries are located at `/opt/ros/humble/share/*/gxf/lib/` and should be added by Isaac ROS environment hooks.

## Important Notes

- **Autoware 1.5.0**: Installed at `/opt/autoware/1.5.0/` via the setup script (autoware-localrepo)
- Source Autoware environment: `source /opt/autoware/1.5.0/setup.bash` (includes ROS 2)
- Source ROS only: `source /opt/ros/humble/setup.bash`
- Requires ROS 2 Humble, Ubuntu, NVIDIA GPU
- Uses colcon (not catkin)
- Logs: `play_log/latest/`
- Stop system: Ctrl+C
