# AutoSDV Visual Localization Launch

Launch files for camera-only localization using NVIDIA Isaac ROS (cuVGL + cuVSLAM).

## Overview

This package provides a complete visual localization stack that eliminates the need for LiDAR or GNSS:

- **cuVGL** (Visual Global Localization) - Determines initial pose from pre-built visual map
- **cuVSLAM** (Visual SLAM) - Provides continuous visual odometry
- **Pose Initializer Bridge** - Connects cuVGL to Autoware's pose initialization

## Architecture

```
                    ┌─────────────────────────────────────────┐
                    │     visual_localization.launch.xml      │
                    │              (Entry Point)              │
                    └─────────────────┬───────────────────────┘
                                      │
        ┌─────────────────────────────┼─────────────────────────────┐
        │                             │                             │
        ▼                             ▼                             ▼
┌───────────────────┐   ┌─────────────────────────┐   ┌─────────────────────┐
│ visual_global_    │   │ isaac_slam_with_zed     │   │ visual_pose_        │
│ localization      │   │ (from autosdv_isaac_    │   │ initializer         │
│ .launch.xml       │   │  slam_launch)           │   │ .launch.xml         │
│                   │   │                         │   │                     │
│ cuVGL Node        │   │ cuVSLAM + Bridges       │   │ Init Bridge         │
└─────────┬─────────┘   └───────────┬─────────────┘   └──────────┬──────────┘
          │                         │                            │
          │                         │                            │
          ▼                         ▼                            ▼
/visual_localization/pose    /localization/pose_          /localization/
(initial global pose)         estimator/pose_with_         initialize
                              covariance (tracking)        (service call)
```

## Usage

### Standalone Launch

```bash
ros2 launch autosdv_visual_localization_launch visual_localization.launch.xml \
    map_dir:=/path/to/visual_map
```

### From AutoSDV Main Launch

```bash
just launch pose_source:=visual visual_map_dir:=/path/to/visual_map
```

### Component Launch (cuVGL only)

```bash
ros2 launch autosdv_visual_localization_launch visual_global_localization.launch.xml \
    map_dir:=/path/to/visual_map/cuvgl_map
```

## Arguments

### Entry Point (visual_localization.launch.xml)

| Argument | Default | Description |
|----------|---------|-------------|
| `map_dir` | (required) | Path to visual map directory |
| `camera_namespace` | /sensing/camera/zedxm | ZED camera namespace |
| `camera_model` | zedxm | Camera model for frame names |
| `enable_global_localization` | true | Enable cuVGL |
| `enable_visual_slam` | true | Enable cuVSLAM |
| `enable_pose_initializer` | true | Enable pose init bridge |
| `enable_imu_fusion` | true | Enable IMU fusion in cuVSLAM |
| `auto_initialize` | true | Auto-init on first cuVGL pose |

### cuVGL Component (visual_global_localization.launch.xml)

| Argument | Default | Description |
|----------|---------|-------------|
| `map_dir` | (required) | Path to cuvgl_map directory |
| `camera_namespace` | /sensing/camera/zedxm | ZED camera namespace |
| `camera_model` | zedxm | Camera model |

## Map Directory Structure

The `map_dir` should contain:

```
visual_map/
├── cuvgl_map/      # cuVGL keyframes (for global localization)
├── cuvslam_map/    # cuVSLAM landmarks (for tracking) [optional]
└── occupancy_map/  # 2D occupancy grid [optional]
```

Create maps using:
```bash
./scripts/visual-map/record.sh ./data/visual_maps/my_location
./scripts/visual-map/create-map.sh ./data/visual_maps/my_location_recording
```

## Topics

### Published

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/visual_localization/.../pose` | PoseWithCovarianceStamped | cuVGL | Global pose |
| `/localization/pose_estimator/pose_with_covariance` | PoseWithCovarianceStamped | cuVSLAM | Tracking pose |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `<camera_ns>/left/color/rect/image` | Image | Left camera image |
| `<camera_ns>/right/color/rect/image` | Image | Right camera image |
| `<camera_ns>/imu/data` | Imu | IMU data |

## Dependencies

- `isaac_ros_visual_slam` - cuVSLAM
- `isaac_ros_visual_global_localization` - cuVGL
- `autosdv_isaac_slam_launch` - cuVSLAM launch and bridges
- `visual_pose_initializer_bridge` - Autoware integration
