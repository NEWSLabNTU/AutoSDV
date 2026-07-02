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
./scripts/version/get-version.sh autoware.version     # Returns "2025.02"
./scripts/version/get-version.sh nvidia_amd64.cuda    # Returns "12.3"

# Export all versions as environment variables
source ./scripts/version/export-versions.sh
echo $AUTOSDV_VERSION    # 0.1.0-dev
echo $AUTOWARE_VERSION   # 2025.02
echo $CUDA_VERSION_AMD64 # 12.3
```

### Version Bumping Guidelines

| Change Type                                                      | Version Bump |
|------------------------------------------------------------------|--------------|
| Breaking changes (vehicle interface, sensor configs, launch API) | MAJOR        |
| Autoware base upgrade                                            | MAJOR        |
| New sensor/feature support                                       | MINOR        |
| New launch parameters                                            | MINOR        |
| Bug fixes, parameter tuning                                      | PATCH        |
| Documentation only                                               | PATCH        |

### Branch Strategy

| Branch        | Version          | Channel       |
|---------------|------------------|---------------|
| `main`        | `X.Y.Z` (stable) | `stable`      |
| `develop`     | `X.Y.Z-dev`      | `development` |
| `release/X.Y` | `X.Y.Z-rc.N`     | `stable`      |

## Essential Commands

### Build & Run
```bash
./setup.sh              # Interactive setup (ROS 2, dependencies)
./setup.sh status       # Check installation status
make build              # Build all packages
make test               # Run tests
make launch             # Launch system (web UI: http://localhost:8081)
make launch ARGS="..."  # Launch with parameters
make run-rviz           # Launch RViz
make clean              # Remove build artifacts
make checkout           # Update git submodules
```

### Control Testing
```bash
make test-control       # PID controller + speedometer (tmux)
make plot-test          # PlotJuggler visualization
make controller         # Keyboard manual control
```

### Manual
```bash
source install/setup.bash
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

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
```

### Key Files
- Main launch: `src/launcher/autosdv_launch/launch/autosdv.launch.yaml`
- Sensor calibration: `src/param/.../autosdv_sensor_kit/sensor_kit_calibration.yaml`
- Preset files: `src/launcher/autosdv_launch/config/{perception,localization}/preset/`
- Web UI: http://localhost:8081 (via play_launch)

## Development

### Temporary Files
Write temp files to `./tmp/` (gitignored). Do NOT use system `/tmp/`.

### Build Notes
- With `--symlink-install`, edits to yaml/xml/py apply immediately (no rebuild needed)
- New files require rebuild to create symlinks
- First launch compiles TensorRT models (10-30 min)

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
3. Use with: `make launch ARGS="perception_preset:=custom"`

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
make launch ARGS="perception_preset:=camera_lidar_fusion sensor_suite:=robin_zed"
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

#### System Features
```bash
# Localization
use_gnss:=false                  # Indoor operation (no GNSS)
use_ntrip:=true                  # RTK positioning (ublox only)
pose_source:=ndt|isaac           # Localization method (NDT or Visual SLAM)
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
- **Setup**: `cd book && make setup` (installs dependencies)
- **Build**: `cd book && make build` (builds to `site/`)
- **Serve**: `cd book && make serve` (http://localhost:8000)
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
| Guide                                                                                                                      | Description                            |
|----------------------------------------------------------------------------------------------------------------------------|----------------------------------------|
| [docs/guides/sensor_configuration.md](docs/guides/sensor_configuration.md)                                                 | Sensor suites, NTRIP/RTK, localization |
| [docs/guides/vehicle_calibration.md](docs/guides/vehicle_calibration.md)                                                   | PWM control, PID tuning, testing tools |
| [docs/guides/zed_camera.md](docs/guides/zed_camera.md)                                                                     | ZED setup, troubleshooting             |
| [docs/guides/lidar_integration.md](docs/guides/lidar_integration.md)                                                       | Robin-W, Velodyne, TensorRT            |
| [docs/guides/control_testing.md](docs/guides/control_testing.md)                                                           | Control system testing procedures      |
| [docs/guides/mrm_configuration.md](docs/guides/mrm_configuration.md)                                                       | MRM (emergency stop) configuration     |
| [docs/guides/isaac_vslam_testing.md](docs/guides/isaac_vslam_testing.md)                                                   | Isaac SLAM testing                     |
| [docs/design/isaac_vslam_integration.md](docs/design/isaac_vslam_integration.md)                                           | Isaac SLAM architecture                |
| [docs/research/localization/ndt_parameter_tuning_coss_map.md](docs/research/localization/ndt_parameter_tuning_coss_map.md) | NDT tuning research                    |

## Known Issues

- **Steering reversed**: Left/right inverted in manual control
- **Network monitor errors**: AWS Greengrass socket errors (non-critical, ignore)
- **ZED in VNC**: Requires TurboVNC with VirtualGL for hardware acceleration

## Important Notes

- Source ROS: `source /opt/ros/humble/setup.bash`
- Requires ROS 2 Humble, Ubuntu, NVIDIA GPU
- Uses colcon (not catkin)
- Logs: `play_log/latest/`
- Stop system: Ctrl+C
