# ZED Camera Integration

This guide covers ZED camera setup, configuration, and troubleshooting.

## Version Information

| Component | Version |
|-----------|---------|
| ZED SDK | 5.1.2 |
| ZED Link Duo Driver | 1.3.2 for L4T 36.3.0 |
| ZED ROS2 Wrapper | 5.1.0 (humble-v5.1.0) |
| Supported models | ZED, ZED M, ZED 2, ZED 2i, ZED X, ZED X Mini |

Source: `src/sensor_component/external/zed-ros2-wrapper/`

## Launch File Namespace Handling

**Important**: The ZED Python launch file (`zed_camera.launch.py`) does NOT respect XML `<push-ros-namespace>` directives.

**Solution**: Load the ZED composable node directly via XML's `<load_composable_node>`:

```xml
<!-- Container with ABSOLUTE namespace -->
<node_container pkg="rclcpp_components" exec="component_container_isolated"
                name="zed_container" namespace="/sensing/camera/$(var camera_name)">
  <param name="use_multi_threaded_executor" value="true"/>
</node_container>

<!-- Load composable node -->
<load_composable_node target="/sensing/camera/$(var camera_name)/zed_container">
  <composable_node pkg="zed_components" plugin="stereolabs::ZedCamera"
                   name="$(var camera_name)" namespace="camera">
    <param from="$(find-pkg-share zed_wrapper)/config/common_stereo.yaml"/>
    <param from="$(find-pkg-share zed_wrapper)/config/$(var camera_model).yaml"/>
  </composable_node>
</load_composable_node>
```

This creates:
- Container at `/sensing/camera/zedxm/zed_container`
- ZED node at `/sensing/camera/zedxm`
- Topics at `/sensing/camera/zedxm/<topic>`

## Topic Structure

| Topic | Purpose |
|-------|---------|
| `/sensing/camera/zedxm` | ZED node |
| `/sensing/camera/zedxm/obj_det/objects` | ZED detected objects |
| `/perception/object_recognition/detection/camera_objects` | Autoware format objects |
| `/sensing/camera/zedxm/point_cloud/cloud_registered` | Colored point cloud |
| `/sensing/camera/zedxm/imu/data` | IMU data |

## Object Detection

Two operation modes:
1. **Normal mode** (default): ZED publishes colored point cloud
2. **Object detection mode**: ZED performs detection, converts to Autoware format

### Enable Object Detection

```bash
make launch ARGS="enable_zed_object_detection:=true"
```

### Configuration Files

- Launch: `autosdv_sensor_kit_launch/launch/zed_with_object_detection.launch.xml`
- Config: `autosdv_sensor_kit_launch/config/zed_object_detection.yaml`

## Troubleshooting

### Hardware Acceleration Required (VNC Sessions)

The ZED node requires OpenGL hardware acceleration. Standard VNC will cause failures.

**Solution**: Use TurboVNC with VirtualGL:

```bash
VGL_DISPLAY=:1 /opt/TurboVNC/bin/vncserver :2 -vgl
```

Systemd service available:
- File: `~/.config/systemd/user/turbovnc.service`
- Commands: `systemctl --user {start|stop|status|restart} turbovnc`

### Camera Freeze / Stream Failed to Start

Errors like:
```
(Argus) Error Timeout: ...
[WARN] Error opening camera: CAMERA STREAM FAILED TO START
```

Or in logs (`journalctl -u zed_x_daemon`):
```
[ZED-X Daemon] Received invalid message: "ZEDX#0#0#FROZEN"
```

**Recovery**:
```bash
sudo service zed_x_daemon restart
sleep 25  # Wait for driver reconfiguration
```

**Note**: Restarting `nvargus-daemon` alone is NOT sufficient.

### Check Service Logs

```bash
journalctl -u zed_x_daemon --since "30 minutes ago"     # ZED X daemon
journalctl -u nvargus-daemon --since "30 minutes ago"   # NVIDIA Argus
journalctl -k | grep -i "zed\|gmsl\|ar0234"            # Kernel camera errors
```

## Known Issues

- Detection box positions may not perfectly align with point cloud coordinates (coordinate transformation issue)
