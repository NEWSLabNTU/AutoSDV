# LiDAR Integration Guide

This guide covers LiDAR sensor integration for AutoSDV.

## Supported LiDAR Models

| Model | Type | Configuration |
|-------|------|---------------|
| Robin-W | Solid-state | `lidar_model:=robin-w` |
| Velodyne VLP-32C | Spinning | `lidar_model:=vlp32c` |
| Blickfeld Cube1 | Solid-state | `lidar_model:=cube1` |

Configuration location: `src/param/autoware_individual_params/individual_params/config/default/autosdv_sensor_kit/`

## Seyond Robin-W Integration

### PointXYZIRC Format Support

The Robin-W driver outputs Autoware-compatible PointXYZIRC format:
- Location: `src/sensor_component/external/seyond_ros_driver/`
- CMakeLists.txt: `set(POINT_TYPE PointXYZIRC)`
- Point type: `src/driver/point_xyzirc.h`

**Field Mapping**:
| Field | Type | Description |
|-------|------|-------------|
| x, y, z | FLOAT32 | Position |
| intensity | FLOAT32 | Intensity value |
| return_type | UINT8 | 1=strongest/first, 2=last/second |
| ring | UINT16 | Channel/scanning line ID |

### Coordinate Transformation

Robin-W uses non-standard coordinates requiring transformation:

| Axis | Robin-W Native | ROS Standard (REP-103) |
|------|----------------|------------------------|
| X | Up | Forward |
| Y | Right | Left |
| Z | Forward | Up |

Configured in `sensor_kit_calibration.yaml`:
- roll: 3.14159 (180 deg)
- pitch: -1.5708 (-90 deg)
- yaw: 0.0

### Network Configuration

- Default IP: 172.168.1.10
- Config: `autosdv_sensor_kit_launch/launch/lidar.launch.xml`

## TensorRT Model Compilation

### First Run Behavior

On first launch, TensorRT compiles ONNX models to CUDA engines:
- Duration: 10-30 minutes
- Cache location: `./data/` directory

Key models:
- `lidar_centerpoint/pts_voxel_encoder_centerpoint_tiny.engine`
- `lidar_centerpoint/pts_backbone_neck_head_centerpoint_tiny.engine`

### LiDAR-Only Perception

For faster startup, configure in `autosdv.launch.yaml`:
```yaml
perception_mode: "lidar"
use_traffic_light_recognition: "false"
use_detection_by_tracker: "false"
use_image_segmentation_based_filter: "false"
```
