# CLAUDE.md

Guidance for Claude Code when working with this repository.

## Project Overview

AutoSDV is a software-defined autonomous vehicle platform built on ROS 2 and Autoware. Supports multiple LiDAR configurations (Robin-W, Velodyne 32C, Blickfeld Cube1) for small-scale autonomous vehicles.

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
├── launcher/autosdv_launch/     # Main launch (autosdv.launch.yaml)
├── localization/                # Localization packages
├── param/autoware_individual_params/  # Sensor configs
├── sensor_kit/autosdv_sensor_kit_launch/
├── vehicle/autosdv_vehicle_launch/
└── sensor_component/external/   # Sensor driver submodules
data/
├── COSS-map-planning/           # Default map
├── models/                      # ML models (YOLOX, CenterPoint)
└── zed-sdk/                     # ZED calibration
```

### Key Files
- Main launch: `src/launcher/autosdv_launch/launch/autosdv.launch.yaml`
- Sensor calibration: `src/param/.../autosdv_sensor_kit/sensor_kit_calibration.yaml`
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

## Quick Reference

### Common Launch Parameters
```bash
# Sensor suites
sensor_suite:=robin_zed          # Robin-W + ZED + ZED IMU
sensor_suite:=vlp32c_zed         # Velodyne + ZED + ZED IMU

# Individual sensors
lidar_model:=robin-w|vlp32c|cube1
camera_model:=zedxm|usb|none
imu_source:=mpu9250|zed
gnss_receiver:=garmin|ublox|septentrio

# Features
use_gnss:=false                  # Indoor operation
use_ntrip:=true                  # RTK positioning
pose_source:=ndt|isaac           # Localization method
enable_zed_object_detection:=true
```

### Motor/Steering PWM (Quick Ref)
- Motor: 370=stop, >370=forward, <370=reverse, 340=brake
- Steering: 400=center, 350=left, 450=right

## Documentation

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

## Important Notes

- Source ROS: `source /opt/ros/humble/setup.bash`
- Requires ROS 2 Humble, Ubuntu, NVIDIA GPU
- Uses colcon (not catkin)
- Logs: `play_log/latest/`
- Stop system: Ctrl+C
