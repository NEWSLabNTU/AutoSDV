# Perception Presets

Perception presets define the perception configuration for AutoSDV. They control which perception features are enabled and which perception mode is used.

## Available Presets

### `lidar_only` (Default)
Default preset for AutoSDV. Uses only LiDAR for perception.

**Configuration**:
- `perception_mode`: `lidar`
- `use_traffic_light_recognition`: `false`
- `use_detection_by_tracker`: `false`
- `use_image_segmentation_based_filter`: `false`
- `use_pointcloud_map`: `true`

**Use cases**:
- Standard outdoor operation with LiDAR
- When camera is not available or not needed
- Faster perception processing

**Launch**:
```bash
make launch ARGS="perception_preset:=lidar_only"
# Or omit (it's the default)
make launch
```

### `camera_lidar_fusion`
Uses both camera and LiDAR for enhanced perception. Enables camera-based features.

**Configuration**:
- `perception_mode`: `camera_lidar_fusion`
- `use_traffic_light_recognition`: `true`
- `use_detection_by_tracker`: `true`
- `use_image_segmentation_based_filter`: `true`
- `use_pointcloud_map`: `true`

**Use cases**:
- Urban environments with traffic lights
- When enhanced object detection is needed
- Research on sensor fusion

**Requirements**:
- Camera sensor must be available (e.g., ZED camera)
- Higher computational resources required

**Launch**:
```bash
make launch ARGS="perception_preset:=camera_lidar_fusion sensor_suite:=robin_zed"
```

### `minimal`
Minimal perception configuration for development and debugging.

**Configuration**:
- `perception_mode`: `lidar`
- `use_traffic_light_recognition`: `false`
- `use_detection_by_tracker`: `false`
- `use_image_segmentation_based_filter`: `false`
- `use_pointcloud_map`: `false`

**Use cases**:
- Development and debugging
- Faster startup times
- Testing without perception features

**Launch**:
```bash
make launch ARGS="perception_preset:=minimal"
```

## Usage

Specify the preset when launching AutoSDV:

```bash
# Real hardware mode
make launch ARGS="perception_preset:=<preset_name>"

# Logging simulation mode
ros2 launch autosdv_launch logging_simulation.launch.yaml perception_preset:=<preset_name>
```

## Creating Custom Presets

To create a custom preset:

1. Copy an existing preset file (e.g., `lidar_only.yaml`)
2. Rename it (e.g., `custom.yaml`)
3. Modify the parameters as needed
4. Add conditional logic in `autosdv.launch.yaml` to support the new preset
5. Update this README with the new preset documentation

## Parameter Reference

| Parameter | Type | Description |
|-----------|------|-------------|
| `perception_mode` | string | Perception mode: `lidar`, `camera_lidar_fusion`, `camera_lidar_radar_fusion`, `radar` |
| `use_traffic_light_recognition` | bool | Enable traffic light recognition using camera |
| `use_detection_by_tracker` | bool | Enable object detection by tracking |
| `use_image_segmentation_based_filter` | bool | Enable image-based segmentation filtering |
| `use_pointcloud_map` | bool | Use pointcloud map for localization and perception |

## Notes

- Perception presets are independent of sensor suite configuration
- Camera-based features require appropriate camera sensor (configured via `sensor_suite` or `camera_model`)
- Some features may require additional computational resources (GPU for ZED object detection, etc.)
