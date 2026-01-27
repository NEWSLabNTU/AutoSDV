# Visual Map Creation Scripts

Scripts for creating visual maps for camera-only localization using NVIDIA Isaac ROS.

## Overview

These scripts help you create visual maps that enable:
- **cuVGL**: Global localization (automatic initial pose)
- **cuVSLAM**: Visual odometry (continuous tracking)

## Prerequisites

1. Install Isaac ROS packages:
   ```bash
   ./setup.sh isaac-ros
   ```

2. ZED camera configured and working

## Quick Start

### 1. Record a Mapping Session

Start the ZED camera:
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm
```

In another terminal, record:
```bash
./scripts/visual-map/record.sh ~/AutoSDV/data/visual_maps/my_location
```

**Recording tips:**
- Keep camera **stationary for 10+ seconds** at start
- Move **slowly** (walking speed)
- Create **closed loops** (return to starting point)
- Cover the entire area you want to localize in
- Ensure **good lighting** and **textured surfaces**

### 2. Create the Map

```bash
./scripts/visual-map/create-map.sh ~/AutoSDV/data/visual_maps/my_location_recording
```

This creates three map types:
- `cuvslam_map/` - Visual landmarks for tracking
- `cuvgl_map/` - Keyframe database for global localization
- `occupancy_map/` - 2D grid for navigation

### 3. Use the Map

```bash
just launch pose_source:=visual visual_map_dir:=~/AutoSDV/data/visual_maps/my_location
```

## Scripts

| Script | Purpose |
|--------|---------|
| `record.sh` | Record rosbag with ZED camera for mapping |
| `create-map.sh` | Create visual maps from recorded rosbag |

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `CAMERA_NS` | `/sensing/camera/zedxm/zed_node` | ZED camera namespace |

## Troubleshooting

### Recording Issues

**Missing topics:**
```bash
# Check ZED is running
ros2 topic list | grep zedxm

# Verify topic rates
ros2 topic hz /sensing/camera/zedxm/zed_node/left/image_rect_color
```

### Map Creation Issues

**Out of memory:**
- Close other GPU applications
- Reduce rosbag duration (try 1-2 minutes first)

**Poor map quality:**
- Ensure adequate lighting
- Avoid featureless areas (blank walls)
- Move more slowly during recording
- Create tighter closed loops

## References

- [Isaac ROS Mapping Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mapping_and_localization/)
- [Visual Global Localization Roadmap](../../docs/roadmaps/visual_global_localization.md)
