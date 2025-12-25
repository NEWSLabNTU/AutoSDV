# AutoSDV Simulation Testing Guide

This document explains how to configure AutoSDV for simulation testing, including rosbag replay for testing Isaac ROS Visual SLAM integration.

## Simulation Modes

Autoware provides three main simulation modes:

1. **Planning Simulation** - Planning and control only, no sensing/perception
2. **Rosbag Replay Simulation** - Full stack with pre-recorded sensor data
3. **Digital Twin Simulation** - Full stack with CARLA simulator

For testing Isaac SLAM integration, **Rosbag Replay Simulation** is most suitable as it can replay real camera data.

---

## Rosbag Replay Simulation

### Overview

Rosbag replay allows testing the full Autoware stack (sensing, localization, perception, planning, control) using pre-recorded sensor data. This is ideal for:
- Testing localization algorithms with real sensor data
- Reproducing specific scenarios
- Testing without physical hardware

**Key characteristic**: `use_sim_time: true` - all nodes use timestamps from the rosbag instead of wall clock.

### Architecture

```
Rosbag File                    AutoSDV/Autoware
┌─────────────┐               ┌──────────────────────────┐
│ /sensing/   │──────────────>│ Sensing (preprocessing)  │
│   camera/   │               └──────────┬───────────────┘
│   lidar/    │                          │
│   imu/      │                          v
│   gnss/     │               ┌──────────────────────────┐
└─────────────┘               │ Localization             │
                              │  - Isaac SLAM (camera)   │
                              │  - NDT (LiDAR)           │
                              │  - EKF fusion            │
                              └──────────┬───────────────┘
                                         │
                                         v
                              ┌──────────────────────────┐
                              │ Perception/Planning/Ctl  │
                              └──────────────────────────┘
```

### Required Rosbag Topics for Isaac SLAM Testing

For testing Isaac ROS Visual SLAM, the rosbag must contain:

**Essential topics:**
- `/sensing/camera/zedxm/zed_node/left/image_rect_color` (sensor_msgs/Image, rgb8)
- `/sensing/camera/zedxm/zed_node/right/image_rect_color` (sensor_msgs/Image, rgb8)
- `/sensing/camera/zedxm/zed_node/left/camera_info` (sensor_msgs/CameraInfo)
- `/sensing/camera/zedxm/zed_node/right/camera_info` (sensor_msgs/CameraInfo)
- `/sensing/camera/zedxm/zed_node/imu/data` (sensor_msgs/Imu)

**Additional topics (for full system testing):**
- `/sensing/lidar/*/pointcloud_raw` (sensor_msgs/PointCloud2) - for NDT comparison
- `/sensing/gnss/*/nav_sat_fix` (sensor_msgs/NavSatFix) - for GNSS initialization
- `/vehicle/status/velocity_status` (autoware_vehicle_msgs/VelocityReport)
- `/tf` and `/tf_static` (tf2_msgs/TFMessage) - for coordinate transforms

### Configuration

#### Option 1: Using AutoSDV with Simulation Mode

AutoSDV already has a simulation mode parameter. Modify the launch for rosbag replay:

**Create**: `src/launcher/autosdv_launch/launch/autosdv_logging_simulator.launch.yaml`

```yaml
launch:
# Import arguments from autosdv.launch.yaml
- arg:
    name: is_simulation
    default: "true"
    description: "Enable simulation mode (rosbag replay)"

- arg:
    name: lidar_model
    default: "robin-w"
    description: "LiDAR model (cube1, robin-w, or vlp32c)"

- arg:
    name: camera_model
    default: "zedxm"
    description: "Camera model (zedxm, usb, or none)"

- arg:
    name: pose_source
    default: "isaac"
    description: "Pose estimation source: ndt (LiDAR NDT), isaac (Visual SLAM)"

- arg:
    name: use_gnss
    default: "false"
    description: "Enable GNSS for outdoor operation (usually false in simulation)"

# Launch Isaac SLAM when pose_source is isaac
- group:
    if: "$(eval '\"$(var pose_source)\" == \"isaac\"')"
    children:
    - include:
        file: "$(find-pkg-share autosdv_isaac_slam_launch)/launch/isaac_slam_with_zed.launch.xml"
        arg:
        - name: camera_namespace
          value: /sensing/camera/$(var camera_model)/zed_node
        - name: enable_imu_fusion
          value: "true"
        - name: enable_visualization
          value: "true"

# Launch Autoware with logging simulator configuration
- include:
    file: "$(find-pkg-share autoware_launch)/launch/logging_simulator.launch.xml"
    arg:
    - name: vehicle_model
      value: autosdv_vehicle
    - name: sensor_model
      value: autosdv_sensor_kit
    - name: map_path
      value: ./data/COSS-map-planning
    - name: data_path
      value: ./data
    # Modules to launch
    - name: sensing
      value: "true"
    - name: localization
      value: "true"
    - name: perception
      value: "true"
    - name: planning
      value: "false"
    - name: control
      value: "true"
    # System
    - name: launch_system_monitor
      value: "false"
    - name: rviz
      value: "true"

# System Monitor
- include:
    file: "$(find-pkg-share autosdv_system_monitor)/launch/autosdv_system_monitor.launch.yaml"
```

#### Option 2: Using Autoware's logging_simulator.launch.xml Directly

For simpler testing without AutoSDV-specific features:

```bash
cd /home/aeon/repos/AutoSDV/2025.02

# Source ROS and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch Autoware in logging simulation mode
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:=./data/COSS-map-planning \
  vehicle_model:=autosdv_vehicle \
  sensor_model:=autosdv_sensor_kit \
  data_path:=./data
```

Then manually launch Isaac SLAM:
```bash
# In another terminal
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch autosdv_isaac_slam_launch isaac_slam_with_zed.launch.xml
```

### Running Rosbag Replay

1. **Prepare rosbag** with required camera topics

2. **Launch AutoSDV in simulation mode**:
   ```bash
   # Using Option 1 (if created)
   ros2 launch autosdv_launch autosdv_logging_simulator.launch.yaml \
     pose_source:=isaac \
     camera_model:=zedxm

   # Or using Option 2
   ros2 launch autoware_launch logging_simulator.launch.xml \
     map_path:=./data/COSS-map-planning \
     vehicle_model:=autosdv_vehicle \
     sensor_model:=autosdv_sensor_kit
   ```

3. **Play the rosbag**:
   ```bash
   # In another terminal
   ros2 bag play /path/to/rosbag --clock -r 1.0

   # Options:
   # --clock: Publish /clock topic for simulated time
   # -r 1.0: Playback rate (1.0 = real-time, 0.5 = half speed, 2.0 = double speed)
   # --topics: Filter specific topics if needed
   ```

4. **Monitor topics**:
   ```bash
   # Check Isaac SLAM is receiving images
   ros2 topic hz /visual_slam/image_0
   ros2 topic hz /visual_slam/image_1

   # Check visual SLAM output
   ros2 topic echo /isaac_slam/visual_slam_node/status
   ros2 topic hz /localization/pose_estimator/pose_with_covariance

   # Check EKF output
   ros2 topic hz /localization/kinematic_state
   ```

### Expected Behavior

When running correctly:
1. Rosbag publishes sensor topics with historical timestamps
2. All ROS nodes use simulated time from `/clock` topic
3. Isaac SLAM processes stereo images and publishes odometry
4. Bridge converts odometry to pose
5. EKF fuses pose with other sensors
6. Planning and control operate based on localization

### Troubleshooting

#### No image topics from rosbag
- **Symptom**: Isaac SLAM shows no input
- **Check**: `ros2 topic list | grep image`
- **Solution**: Verify rosbag contains image topics with correct names

#### Time synchronization issues
- **Symptom**: "Message has timestamps in the past" warnings
- **Check**: `ros2 topic hz /clock`
- **Solution**: Ensure rosbag is played with `--clock` flag

#### Isaac SLAM not tracking
- **Symptom**: `vo_state: 2` (tracking lost) in status
- **Check**: Image quality, lighting conditions in rosbag
- **Solution**: Try slower playback rate (`-r 0.5`), ensure stereo calibration is correct

#### Transform (TF) errors
- **Symptom**: "Could not transform" errors
- **Check**: `/tf_static` topic in rosbag
- **Solution**: Ensure rosbag contains `/tf_static` with camera-to-base_link transforms

---

## Creating Custom Rosbags for Testing

### Recording with Real Hardware

When hardware becomes available, record a rosbag with required topics:

```bash
# Source workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch AutoSDV normally
make launch ARGS="camera_model:=zedxm lidar_model:=robin-w"

# Record rosbag (in another terminal)
ros2 bag record \
  /sensing/camera/zedxm/zed_node/left/image_rect_color \
  /sensing/camera/zedxm/zed_node/right/image_rect_color \
  /sensing/camera/zedxm/zed_node/left/camera_info \
  /sensing/camera/zedxm/zed_node/right/camera_info \
  /sensing/camera/zedxm/zed_node/imu/data \
  /sensing/lidar/robin_w/pointcloud_raw \
  /tf \
  /tf_static \
  -o autosdv_test_data
```

**Recommended recording scenarios:**
1. **Static scene** (30 seconds) - For verifying basic tracking
2. **Slow forward motion** (1-2 minutes) - For drift analysis
3. **Loop closure** (return to start) - For SLAM map consistency
4. **Indoor corridor** - For challenging low-texture environment

### Rosbag File Size Estimation

Typical data rates for ZED camera:
- Stereo images (1280x720, rgb8, 30 Hz): ~150 MB/min
- Camera info (30 Hz): ~0.1 MB/min
- IMU (200 Hz): ~0.5 MB/min
- LiDAR (10 Hz): ~50 MB/min
- **Total**: ~200 MB/min (12 GB/hour)

**Storage recommendations**:
- Short tests (1-2 min): 500 MB - 1 GB
- Medium tests (5-10 min): 2-5 GB
- Long tests (30+ min): 10+ GB

---

## Available Public Datasets

### Autoware Official Datasets

1. **Bus-ODD Dataset** ✅ **Contains Camera Data**
   - 3 x Lucid Vision Triton 5.4MP cameras (left, right, front)
   - 1 x VLP16 + 2 x VLP32C LiDARs
   - GNSS/INS
   - Download: [Autoware Datasets](https://autowarefoundation.github.io/autoware-documentation/main/datasets/)

2. **Istanbul Open Dataset** ❌ No Camera Data
   - LiDAR and GNSS only
   - Not suitable for visual SLAM testing

### Limitations

⚠️ **Important**: Autoware sample rosbags typically **do not include camera images** for privacy reasons. The Bus-ODD dataset is an exception, but the camera configuration (3 separate monocular cameras) may need topic remapping to work with Isaac SLAM's stereo requirements.

### Using Bus-ODD Dataset (If Compatible)

If the Bus-ODD dataset has suitable stereo camera data:

1. Download dataset from link in Autoware documentation
2. Inspect topics:
   ```bash
   ros2 bag info /path/to/bus_odd_rosbag
   ```
3. Check for left/right stereo image pairs
4. Remap topics when playing:
   ```bash
   ros2 bag play /path/to/bus_odd_rosbag --clock \
     --remap /original/left/image:=/sensing/camera/zedxm/zed_node/left/image_rect_color \
     --remap /original/right/image:=/sensing/camera/zedxm/zed_node/right/image_rect_color \
     --remap /original/left/camera_info:=/sensing/camera/zedxm/zed_node/left/camera_info \
     --remap /original/right/camera_info:=/sensing/camera/zedxm/zed_node/right/camera_info
   ```

---

## Alternative: CARLA Digital Twin Simulation

For generating synthetic camera and LiDAR data, consider using CARLA simulator:
- Full 3D environment
- Synthetic stereo camera data
- Configurable sensors
- Documentation: [CARLA Tutorial](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/digital-twin-simulation/carla-tutorial/)

**Pros**:
- Generate camera data without hardware
- Controlled testing scenarios
- Reproducible conditions

**Cons**:
- More complex setup
- Requires GPU for rendering
- Synthetic data may differ from real-world

---

## Next Steps

### For Simulation Testing (Current Phase)

1. **Option A**: Download Bus-ODD dataset and inspect camera topics
   - Check if stereo pair is available
   - Test topic remapping
   - Validate with Isaac SLAM

2. **Option B**: Wait for real hardware
   - Record custom rosbag with ZED camera
   - Test in controlled environment
   - Use for integration validation

3. **Option C**: Set up CARLA simulator
   - Install CARLA + Autoware integration
   - Generate synthetic stereo data
   - Test Isaac SLAM with synthetic data

### For Hardware Testing (Future Phase)

When real hardware is available:
1. Record test rosbags in different scenarios
2. Validate Isaac SLAM performance
3. Compare with NDT localization
4. Tune parameters based on results
5. Document performance characteristics

---

## Summary

**Current Status**: Isaac SLAM integration is code-complete and ready for testing, but **requires stereo camera data** which is not available in standard Autoware sample rosbags.

**Recommended Path**:
1. **Short term**: Use the simulation configuration documented here to prepare the launch files
2. **When ready**: Either obtain Bus-ODD dataset (if stereo compatible) or record custom rosbag with real ZED camera
3. **Testing**: Follow the rosbag replay workflow to validate Isaac SLAM integration

**Key Insight**: The limitation is **data availability**, not the integration itself. Once suitable camera data is available (either from dataset or real hardware), the system is ready to test.

---

## References

- [Rosbag Replay Simulation](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/rosbag-replay-simulation/)
- [Autoware Datasets](https://autowarefoundation.github.io/autoware-documentation/main/datasets/)
- [Planning Simulation](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/)
- [CARLA Tutorial](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/digital-twin-simulation/carla-tutorial/)
- [Isaac ROS Visual SLAM Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html)

**Created**: 2025-10-10
**Author**: Claude Code (Automated Analysis)
