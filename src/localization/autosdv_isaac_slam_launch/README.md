# autosdv_isaac_slam_launch

Launch package for Isaac ROS Visual SLAM integration with ZED camera in AutoSDV.

## Overview

This package provides launch configuration for NVIDIA Isaac ROS Visual SLAM (cuVSLAM) to enable GPU-accelerated stereo visual-inertial odometry for indoor and GNSS-denied environments. It integrates with AutoSDV's localization pipeline by converting Isaac SLAM's odometry output to Autoware's expected pose format.

## Features

- **GPU-accelerated SLAM**: Uses NVIDIA cuVSLAM for real-time visual odometry
- **Stereo + IMU fusion**: Combines ZED stereo camera with IMU data for robust tracking
- **Seamless Autoware integration**: Converts odometry to `PoseWithCovarianceStamped` for EKF fusion
- **Configurable parameters**: Tune IMU noise, visualization, and SLAM behavior

## Dependencies

- `isaac_ros_visual_slam`: NVIDIA Isaac ROS Visual SLAM package (apt for arm64, build from source for amd64)
- `isaac_ros_image_proc`: Image format conversion (rgb8 → mono8)
- `zed_wrapper`: ZED camera driver (ZED SDK 5.x required)
- `odometry_pose_bridge`: Odometry-to-Pose converter
- `topic_tools`: Topic relay nodes

## Package Contents

```
autosdv_isaac_slam_launch/
├── config/
│   └── isaac_slam_params.yaml           # Visual SLAM parameters
├── launch/
│   ├── isaac_slam_with_zed.launch.xml   # Main launch file
│   └── standalone_test.launch.py        # Standalone testing launch
├── rviz/
│   └── isaac_slam_test.rviz             # Pre-configured RViz displays
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Launch Files

### isaac_slam_with_zed.launch.xml

Main launch file that integrates all Isaac SLAM components (without ZED camera).

**Arguments:**
- `enable_imu_fusion` (default: true): Enable IMU fusion in visual SLAM
- `enable_visualization` (default: true): Enable SLAM visualization topics
- `camera_namespace` (default: /sensing/camera/zedxm/zedxm): ZED camera namespace

**Nodes launched:**
1. **left_image_converter**: Converts left image from rgb8 to mono8
2. **right_image_converter**: Converts right image from rgb8 to mono8
3. **left_camera_info_relay**: Relays left camera info to Isaac SLAM
4. **right_camera_info_relay**: Relays right camera info to Isaac SLAM
5. **imu_relay**: Relays IMU data to Isaac SLAM
6. **visual_slam_node**: Isaac ROS Visual SLAM node
7. **isaac_slam_bridge**: Converts odometry to PoseWithCovarianceStamped

All nodes run in the `isaac_slam` namespace.

### standalone_test.launch.py

Complete standalone testing launch for Isaac Visual SLAM with ZED camera and visualization.

**Arguments:**
- `enable_imu_fusion` (default: true): Enable IMU fusion in visual SLAM
- `enable_visualization` (default: true): Enable SLAM visualization topics
- `enable_rviz` (default: true): Launch RViz with pre-configured displays
- `camera_model` (default: zedxm): ZED camera model

**Components launched:**
1. **ZED camera driver**: Launches ZED X Mini with optimal settings
2. **Isaac SLAM pipeline**: All converters, relays, and SLAM node
3. **RViz**: Pre-configured with camera views, paths, landmarks, and odometry displays

This is the **recommended way** to test Isaac Visual SLAM independently.

## Configuration

### isaac_slam_params.yaml

Key parameters:

```yaml
# Camera configuration
num_cameras: 2
camera_optical_frames: ['zedxm_left_camera_optical_frame', 'zedxm_right_camera_optical_frame']

# Frame IDs
base_frame: 'base_link'
map_frame: 'map'
odom_frame: 'odom'

# IMU fusion
enable_imu_fusion: true
imu_frame: 'zedxm_imu_link'

# TF publishing (disabled - let EKF handle TF)
publish_tf: false
publish_map_to_odom_tf: false

# Visualization
enable_slam_visualization: true
```

See config file for full parameter list.

## Topic Mapping

### Input Topics (from ZED camera)
- `/sensing/camera/zedxm/zedxm/left/image_rect_color` (sensor_msgs/Image, rgb8)
- `/sensing/camera/zedxm/zedxm/right/image_rect_color` (sensor_msgs/Image, rgb8)
- `/sensing/camera/zedxm/zedxm/left/camera_info` (sensor_msgs/CameraInfo)
- `/sensing/camera/zedxm/zedxm/right/camera_info` (sensor_msgs/CameraInfo)
- `/sensing/camera/zedxm/zedxm/imu/data` (sensor_msgs/Imu)

### Internal Topics (Isaac SLAM)
- `/visual_slam/image_0` (sensor_msgs/Image, mono8) - Converted left image
- `/visual_slam/image_1` (sensor_msgs/Image, mono8) - Converted right image
- `/visual_slam/camera_info_0` (sensor_msgs/CameraInfo) - Left camera info
- `/visual_slam/camera_info_1` (sensor_msgs/CameraInfo) - Right camera info
- `/visual_slam/imu` (sensor_msgs/Imu) - IMU data
- `/isaac_slam/visual_slam_node/tracking/odometry` (nav_msgs/Odometry) - SLAM output

### Output Topics (to Autoware)
- `/localization/pose_estimator/pose_with_covariance` (geometry_msgs/PoseWithCovarianceStamped)

### Visualization Topics
- `/isaac_slam/visual_slam_node/tracking/slam_path` (nav_msgs/Path)
- `/isaac_slam/visual_slam_node/vis/landmarks_cloud` (sensor_msgs/PointCloud2)
- `/isaac_slam/visual_slam_node/status` (isaac_ros_visual_slam_interfaces/VisualSlamStatus)

## Usage

### Standalone Testing (Recommended)

**Quick Start** - Launch everything with one command:

```bash
source install/setup.bash
ros2 launch autosdv_isaac_slam_launch standalone_test.launch.py
```

This single command launches:
- ZED X Mini camera driver
- Isaac ROS Visual SLAM with all converters and relays
- RViz with pre-configured displays for visualization

**Monitor SLAM status:**
```bash
# In separate terminal
ros2 topic echo /isaac_slam/visual_slam_node/status
# vo_state: 1 = tracking success, 2 = tracking lost

ros2 topic hz /isaac_slam/visual_slam_node/tracking/odometry
# Should show ~30 Hz
```

**Optional arguments:**
```bash
# Test without IMU fusion
ros2 launch autosdv_isaac_slam_launch standalone_test.launch.py enable_imu_fusion:=false

# Launch without RViz (headless)
ros2 launch autosdv_isaac_slam_launch standalone_test.launch.py enable_rviz:=false
```

**For detailed tutorial**, see: `docs/isaac_visual_slam_standalone_test.md`

---

### Manual Step-by-Step Testing

Launch components separately for debugging:

```bash
# Terminal 1: ZED camera
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm

# Terminal 2: Isaac SLAM
ros2 launch autosdv_isaac_slam_launch isaac_slam_with_zed.launch.xml

# Terminal 3: Monitor
ros2 topic echo /isaac_slam/visual_slam_node/status
ros2 topic hz /localization/pose_estimator/pose_with_covariance
```

### Integrated with AutoSDV

Launch via AutoSDV main launch (see Phase 4 integration):

```bash
make launch ARGS="pose_source:=isaac use_gnss:=false camera_model:=zedxm"
```

## Troubleshooting

### No odometry output
- Check ZED camera is publishing images: `ros2 topic hz /sensing/camera/zedxm/zedxm/left/image_rect_color`
- Check image converters running: `ros2 node list | grep converter`
- Check visual SLAM status: `ros2 topic echo /isaac_slam/visual_slam_node/status`
  - `vo_state: 1` = tracking success
  - `vo_state: 2` = tracking lost

### Tracking failures
- Ensure sufficient lighting and texture in environment
- Avoid rapid camera motion during initialization
- Check IMU data is publishing: `ros2 topic hz /sensing/camera/zedxm/zedxm/imu/data`
- Try disabling IMU fusion: `enable_imu_fusion:=false`

### Image format errors
- Verify image converters are creating mono8 images:
  ```bash
  ros2 topic info /visual_slam/image_0 --verbose
  # Should show type: sensor_msgs/msg/Image, encoding: mono8
  ```

### TF errors
- Ensure `publish_tf: false` in config (Autoware EKF handles TF)
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify no duplicate `odom -> base_link` transforms

## Performance

- **Image resolution**: 1280x720 @ 30 Hz (configurable via ZED launch)
- **Processing latency**: ~20-30ms per frame (Jetson AGX Orin)
- **GPU memory**: ~1-2 GB (CUDA operations)
- **CPU usage**: ~15-20% (mostly image conversion)

## References

- **[Standalone Testing Tutorial](../../../docs/isaac_visual_slam_standalone_test.md)** - Complete guide for testing Isaac SLAM with ZED X Mini
- [Isaac ROS 3.2 Documentation](https://nvidia-isaac-ros.github.io/v/release-3.2/) - Official Isaac ROS 3.2 docs (Ubuntu 22.04/Humble)
- [Isaac ROS Visual SLAM Package](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/)
- [AutoSDV Isaac ROS Integration Plan](../../../docs/isaac_ros_visual_slam_integration_plan.md)
- [ZED ROS 2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)

## License

Apache 2.0

## Maintainer

AutoSDV Team <autosdv@newslab.ntu.edu.tw>
