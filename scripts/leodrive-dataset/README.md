# Leo Drive Bus-ODD Dataset Tools

Tools for downloading and processing the Leo Drive Bus-ODD dataset for use with AutoSDV/Autoware 1.5.0.

## Dataset Info

- **Size**: ~10.9GB
- **Sensors**: 3x Lucid Vision cameras, VLP16 + 2x VLP32C LiDARs, Applanix POS LV 120 GNSS/INS
- **Source**: [Autoware Datasets](https://autowarefoundation.github.io/autoware-documentation/main/datasets/)

## Quick Start

```bash
# Full setup (build dependencies + download)
just setup

# Or step by step:
just build-applanix  # Build applanix_msgs for rosbag playback
just download        # Download dataset (~10.9GB)
```

## Autoware 1.5.0 Migration

The dataset was recorded with older Autoware.Auto message types. To convert to Autoware 1.5.0 format:

```bash
# Install rosbags library
pip install rosbags

# Dry run (analyze without writing)
just migrate data/leodrive-busood/all-sensors-bag1_compressed output_bag --dry-run

# Migrate
just migrate data/leodrive-busood/all-sensors-bag1_compressed output_bag
```

## Playing the Rosbag

```bash
# Source applanix_msgs (if needed for INS topics)
source scripts/leodrive-dataset/install/setup.bash

# Play
ros2 bag play data/leodrive-busood/all-sensors-bag1_compressed
```

## Message Type Changes

| Old (Autoware.Auto) | New (Autoware 1.5.0) |
|---------------------|----------------------|
| `autoware_auto_vehicle_msgs/*` | `autoware_vehicle_msgs/*` |
| `autoware_auto_perception_msgs/*` | `autoware_perception_msgs/*` |
| `autoware_auto_planning_msgs/*` | `autoware_planning_msgs/*` |
