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

- `isaac_ros_visual_slam`: NVIDIA Isaac ROS Visual SLAM package (installed via apt)
- `isaac_ros_image_proc`: Image format conversion (rgb8 → mono8)
- `zed_wrapper`: ZED camera driver
- `odometry_pose_bridge`: Odometry-to-Pose converter
- `topic_tools`: Topic relay nodes

## Package Contents

```
autosdv_isaac_slam_launch/
├── config/
│   └── isaac_slam_params.yaml      # Visual SLAM parameters
├── launch/
│   └── isaac_slam_with_zed.launch.xml  # Main launch file
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Launch Files

### isaac_slam_with_zed.launch.xml

Main launch file that integrates all components.

**Arguments:**
- `enable_imu_fusion` (default: true): Enable IMU fusion in visual SLAM
- `enable_visualization` (default: true): Enable SLAM visualization topics
- `camera_namespace` (default: /sensing/camera/zedxm/zed_node): ZED camera namespace

**Nodes launched:**
1. **left_image_converter**: Converts left image from rgb8 to mono8
2. **right_image_converter**: Converts right image from rgb8 to mono8
3. **left_camera_info_relay**: Relays left camera info to Isaac SLAM
4. **right_camera_info_relay**: Relays right camera info to Isaac SLAM
5. **imu_relay**: Relays IMU data to Isaac SLAM
6. **visual_slam_node**: Isaac ROS Visual SLAM node
7. **isaac_slam_bridge**: Converts odometry to PoseWithCovarianceStamped

All nodes run in the `isaac_slam` namespace.

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
- `/sensing/camera/zedxm/zed_node/left/image_rect_color` (sensor_msgs/Image, rgb8)
- `/sensing/camera/zedxm/zed_node/right/image_rect_color` (sensor_msgs/Image, rgb8)
- `/sensing/camera/zedxm/zed_node/left/camera_info` (sensor_msgs/CameraInfo)
- `/sensing/camera/zedxm/zed_node/right/camera_info` (sensor_msgs/CameraInfo)
- `/sensing/camera/zedxm/zed_node/imu/data` (sensor_msgs/Imu)

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

### Standalone Testing

Launch ZED camera and Isaac SLAM separately:

```bash
# Terminal 1: ZED camera
source install/setup.bash
ros2 launch zed_wrapper zedxm.launch.py

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
- Check ZED camera is publishing images: `ros2 topic hz /sensing/camera/zedxm/zed_node/left/image_rect_color`
- Check image converters running: `ros2 node list | grep converter`
- Check visual SLAM status: `ros2 topic echo /isaac_slam/visual_slam_node/status`
  - `vo_state: 1` = tracking success
  - `vo_state: 2` = tracking lost

### Tracking failures
- Ensure sufficient lighting and texture in environment
- Avoid rapid camera motion during initialization
- Check IMU data is publishing: `ros2 topic hz /sensing/camera/zedxm/zed_node/imu/data`
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

- [Isaac ROS Visual SLAM Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html)
- [AutoSDV Isaac ROS Integration Plan](../../../docs/isaac_ros_visual_slam_integration_plan.md)
- [ZED ROS 2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)

## License

Apache 2.0

## Maintainer

AutoSDV Team <autosdv@newslab.ntu.edu.tw>
