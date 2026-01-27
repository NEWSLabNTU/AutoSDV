# Visual Maps

This directory contains visual maps for camera-only localization.

## Directory Structure

Each map directory contains:
```
<map_name>/
├── cuvslam_map/      # cuVSLAM landmarks for tracking
├── cuvgl_map/        # cuVGL keyframes for global localization
└── occupancy_map/    # 2D occupancy grid for navigation
```

## Creating Maps

Use the visual-map scripts:
```bash
# 1. Record a mapping session
./scripts/visual-map/record.sh ./data/visual_maps/my_location

# 2. Create the map
./scripts/visual-map/create-map.sh ./data/visual_maps/my_location_recording
```

## Using Maps

Launch with visual localization:
```bash
just launch pose_source:=visual visual_map_dir:=./data/visual_maps/my_location
```

## Notes

- Maps are created using NVIDIA Isaac ROS mapping tools
- Requires `./setup.sh isaac-ros` to be run first
- Maps are specific to the environment and camera calibration
