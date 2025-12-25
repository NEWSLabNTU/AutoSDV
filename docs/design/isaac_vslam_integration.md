# Isaac ROS Visual SLAM Integration Plan for AutoSDV

## Overview

This document outlines the plan to integrate NVIDIA Isaac ROS Visual SLAM (cuVSLAM) into AutoSDV as an alternative localization source. This enables GPU-accelerated stereo visual-inertial odometry for indoor and GNSS-denied environments.

**Target Use Case**: Indoor localization using ZED stereo camera + IMU, complementing or replacing NDT scan matching.

**Repository**: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git (v3.2-12)

### Key Design Principle

**⚠️ IMPORTANT: No Autoware Source Code Modifications**

This integration follows AutoSDV's architecture philosophy:
- **DO NOT** modify Autoware universe packages
- **DO NOT** edit tier4_localization_launch files
- **DO** create new packages in `src/localization/`
- **DO** extend functionality in `src/launcher/autosdv_launch/`

All integration happens through:
1. New AutoSDV-specific packages
2. Topic remapping and bridging
3. Conditional launch in `autosdv.launch.yaml`

This ensures Autoware remains upstream-compatible and updates don't break our integration.

---

## 1. Architecture Overview

### Current AutoSDV Localization Flow
```
┌─────────────┐
│ Sensor Data │ (LiDAR / Camera / GNSS / IMU)
└──────┬──────┘
       │
       v
┌─────────────────────────────────┐
│ Pose Estimator (pose_source)    │
│ - ndt (default)                  │
│ - yabloc                         │
│ - artag                          │
│ - lidar-marker                   │
│ - eagleye                        │
└──────┬──────────────────────────┘
       │ /localization/pose_estimator/pose_with_covariance
       v
┌──────────────────────────┐
│ EKF Localizer            │ (Sensor Fusion)
└──────┬───────────────────┘
       │ /localization/kinematic_state
       v
┌─────────────┐
│ Planning    │
│ Control     │
└─────────────┘
```

### Proposed Isaac SLAM Integration
```
┌────────────────────────────────────────┐
│ ZED Camera (Stereo + IMU)              │
│ - Left/Right rectified images (mono8)  │
│ - IMU data (accel + gyro)              │
└──────┬─────────────────────────────────┘
       │
       v
┌──────────────────────────────────────────┐
│ Isaac ROS Visual SLAM (cuVSLAM)          │
│ - GPU-accelerated stereo VO              │
│ - Visual-inertial odometry (SVIO)        │
│ - Loop closure detection                 │
└──────┬───────────────────────────────────┘
       │ /visual_slam/tracking/odometry (nav_msgs/Odometry)
       │
       v
┌──────────────────────────────────────────┐
│ Odometry to PoseWithCovariance Converter │ (New bridge node)
└──────┬───────────────────────────────────┘
       │ /localization/pose_estimator/pose_with_covariance
       v
┌──────────────────────────┐
│ EKF Localizer            │ (Sensor Fusion with gyro_odom twist)
└──────┬───────────────────┘
       │ /localization/kinematic_state
       v
┌─────────────┐
│ Planning    │
│ Control     │
└─────────────┘
```

**Key Addition**: Isaac ROS Visual SLAM as new `pose_source: isaac` option.

### Integration Strategy Comparison

**Option A: Bypass Autoware Localization Entirely**
- When `pose_source:=isaac`, set `launch_localization: false` in Autoware
- Manually launch EKF and other localization utilities in `autosdv_launch`
- Pros: Clean separation, no interference with NDT
- Cons: Need to replicate Autoware's localization setup (EKF, stop filter, etc.)

**Option B: Use Autoware Localization with External Pose (Recommended)**
- Keep `launch_localization: true` always
- Pass `pose_source: ndt` to Autoware even when using Isaac
- Isaac SLAM publishes to `/localization/pose_estimator/pose_with_covariance`
- NDT runs but has no effect (no pointcloud input or gets overridden)
- EKF consumes Isaac SLAM pose output
- Pros: Simpler, reuses Autoware's EKF and fusion logic
- Cons: NDT nodes still running (minimal overhead)

**This plan uses Option B** for simplicity and better integration with Autoware's sensor fusion.

---

## 2. Required Dependencies

Isaac ROS Visual SLAM has extensive dependencies on Isaac ROS common infrastructure:

### 2.1 Core Isaac ROS Dependencies (Required)
These must be added to the AutoSDV workspace:

```bash
# Isaac ROS Common (base infrastructure)
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# NITROS (zero-copy middleware)
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git

# Image processing
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
```

### 2.2 Dependency Management Strategy

**Option A: Add as Git Submodules** (Recommended)
- Add to `src/sensor_component/external/isaac_ros_*`
- Benefits: Version control, isolated from Autoware
- Drawbacks: Increases workspace build time

**Option B: Use apt packages**
- Install from NVIDIA Isaac ROS apt repository
- Benefits: Faster builds, less workspace clutter
- Drawbacks: Less control over versions, requires NVIDIA apt setup

**Recommended Approach**: Start with submodules for development, transition to apt for production.

### 2.3 Submodule Addition Commands

```bash
cd /home/aeon/repos/AutoSDV/2025.02

# Isaac ROS Common (base)
git submodule add https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git \
  src/sensor_component/external/isaac_ros_common
cd src/sensor_component/external/isaac_ros_common
git checkout release-3.2
cd ../../../..

# NITROS (zero-copy)
git submodule add https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git \
  src/sensor_component/external/isaac_ros_nitros
cd src/sensor_component/external/isaac_ros_nitros
git checkout release-3.2
cd ../../../..

# Image Pipeline (image_proc needed by visual SLAM)
git submodule add https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git \
  src/sensor_component/external/isaac_ros_image_pipeline
cd src/sensor_component/external/isaac_ros_image_pipeline
git checkout release-3.2
cd ../../../..

git add .gitmodules src/sensor_component/external/
```

**Note**: Isaac ROS Visual SLAM already added at `src/sensor_component/external/isaac_ros_visual_slam` (v3.2-12).

---

## 3. Launch File Integration (Without Modifying Autoware)

**Key Principle**: Do NOT modify Autoware source code. All integration happens in AutoSDV packages.

### 3.1 Create AutoSDV Isaac SLAM Launch Package

Create new package: `src/localization/autosdv_isaac_slam_launch/`

**Package Structure**:
```
src/localization/autosdv_isaac_slam_launch/
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── isaac_slam.launch.xml
│   └── isaac_slam_with_zed.launch.xml
├── config/
│   └── isaac_slam_params.yaml
└── README.md
```

### 3.2 Main Isaac SLAM Launch File

Create: `src/localization/autosdv_isaac_slam_launch/launch/isaac_slam_with_zed.launch.xml`

```xml
<?xml version="1.0"?>
<launch>
  <!-- Arguments -->
  <arg name="enable_imu_fusion" default="true"/>
  <arg name="enable_visualization" default="true"/>
  <arg name="camera_namespace" default="/sensing/camera/zedxm/zed_node"/>

  <!-- Image Format Converters (RGB8 -> Mono8) -->
  <node pkg="isaac_ros_image_proc" exec="isaac_ros_image_format_converter_node"
        name="left_image_converter" namespace="isaac_slam">
    <remap from="image_raw" to="$(var camera_namespace)/left/image_rect_color"/>
    <remap from="image" to="/visual_slam/image_0"/>
    <param name="encoding_desired" value="mono8"/>
    <param name="image_width" value="1280"/>
    <param name="image_height" value="720"/>
  </node>

  <node pkg="isaac_ros_image_proc" exec="isaac_ros_image_format_converter_node"
        name="right_image_converter" namespace="isaac_slam">
    <remap from="image_raw" to="$(var camera_namespace)/right/image_rect_color"/>
    <remap from="image" to="/visual_slam/image_1"/>
    <param name="encoding_desired" value="mono8"/>
    <param name="image_width" value="1280"/>
    <param name="image_height" value="720"/>
  </node>

  <!-- Topic Relays for Camera Info and IMU -->
  <node pkg="topic_tools" exec="relay" name="left_camera_info_relay" namespace="isaac_slam"
        args="$(var camera_namespace)/left/camera_info /visual_slam/camera_info_0"/>

  <node pkg="topic_tools" exec="relay" name="right_camera_info_relay" namespace="isaac_slam"
        args="$(var camera_namespace)/right/camera_info /visual_slam/camera_info_1"/>

  <node pkg="topic_tools" exec="relay" name="imu_relay" namespace="isaac_slam"
        args="$(var camera_namespace)/imu/data /visual_slam/imu"/>

  <!-- Isaac ROS Visual SLAM Node -->
  <node pkg="isaac_ros_visual_slam" exec="visual_slam_node" name="visual_slam_node" namespace="isaac_slam">
    <!-- Load parameters -->
    <param from="$(find-pkg-share autosdv_isaac_slam_launch)/config/isaac_slam_params.yaml"/>

    <!-- Override key parameters -->
    <param name="enable_imu_fusion" value="$(var enable_imu_fusion)"/>
    <param name="enable_slam_visualization" value="$(var enable_visualization)"/>
    <param name="publish_tf" value="false"/>  <!-- Don't publish TF, let EKF handle it -->
    <param name="publish_map_tf" value="false"/>

    <!-- Frame IDs -->
    <param name="base_frame" value="base_link"/>
    <param name="map_frame" value="map"/>
    <param name="odom_frame" value="odom"/>
    <param name="camera_optical_frames" value="['zedxm_left_camera_optical_frame', 'zedxm_right_camera_optical_frame']"/>
  </node>

  <!-- Bridge: Odometry -> PoseWithCovarianceStamped -->
  <node pkg="odometry_pose_bridge" exec="odometry_to_pose_bridge"
        name="isaac_slam_bridge" namespace="isaac_slam">
    <remap from="input/odometry" to="/isaac_slam/visual_slam_node/tracking/odometry"/>
    <remap from="output/pose_with_covariance" to="/localization/pose_estimator/pose_with_covariance"/>
  </node>
</launch>
```

### 3.3 Integration into autosdv.launch.yaml

Edit: `src/launcher/autosdv_launch/launch/autosdv.launch.yaml`

**Add pose_source argument** (after line 30):
```yaml
- arg:
    name: pose_source
    default: "ndt"
    description: "Pose estimation source: ndt (LiDAR), isaac (Visual SLAM), or external"
```

**Add conditional Isaac SLAM launch** (before autoware.launch.xml include):
```yaml
# Launch Isaac ROS Visual SLAM if pose_source is isaac
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
```

**Modify Autoware launch arguments**:
```yaml
- include:
    file: "$(find-pkg-share autoware_launch)/launch/autoware.launch.xml"
    arg:
    # ... existing args ...
    -
      name: launch_localization
      value: "$(eval '\"$(var pose_source)\" != \"isaac\"')"  # Disable if using Isaac
    -
      name: pose_source
      value: ndt  # Autoware only needs to know about ndt when localization is enabled
```

**Alternative approach if we still need EKF fusion**:
```yaml
    -
      name: launch_localization
      value: "true"  # Always launch
    -
      name: pose_source
      value: "$(eval '\"$(var pose_source)\" if \"$(var pose_source)\" != \"isaac\" else \"ndt\"')"
```

Then we'll need to prevent NDT from running when using Isaac. This requires launching a minimal localization setup.

---

## 4. Bridge Node: Odometry to PoseWithCovariance Converter

Isaac SLAM outputs `nav_msgs/Odometry`, but Autoware expects `geometry_msgs/PoseWithCovarianceStamped`.

### 4.1 Create New Package: odometry_pose_bridge

Create: `src/localization/odometry_pose_bridge/`

**Package Structure**:
```
odometry_pose_bridge/
├── package.xml
├── CMakeLists.txt
├── src/
│   └── odometry_to_pose_bridge.cpp
└── launch/
    └── odometry_to_pose_bridge.launch.xml
```

### 4.2 Bridge Node Implementation (C++)

**File**: `src/localization/odometry_pose_bridge/src/odometry_to_pose_bridge.cpp`

```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class OdometryToPoseBridge : public rclcpp::Node
{
public:
  OdometryToPoseBridge() : Node("odometry_to_pose_bridge")
  {
    // Subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "input/odometry", 10,
      std::bind(&OdometryToPoseBridge::odometry_callback, this, std::placeholders::_1)
    );

    // Publisher
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "output/pose_with_covariance", 10
    );

    RCLCPP_INFO(this->get_logger(), "Odometry to PoseWithCovariance bridge started");
  }

private:
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Convert Odometry to PoseWithCovarianceStamped
    auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();

    // Copy header
    pose_msg.header = msg->header;

    // Copy pose
    pose_msg.pose.pose = msg->pose.pose;

    // Copy covariance (6x6 pose covariance, first 6x6 of odometry's 6x6 pose covariance)
    for (size_t i = 0; i < 36; ++i) {
      pose_msg.pose.covariance[i] = msg->pose.covariance[i];
    }

    // Publish
    pose_pub_->publish(pose_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryToPoseBridge>());
  rclcpp::shutdown();
  return 0;
}
```

**Alternative**: Use existing ROS 2 topic tools or create Python node for faster prototyping.

---

## 5. Topic Mapping and Integration

### 5.1 ZED Camera Topics

**Current ZED Output** (from `zed-ros2-wrapper`):
- `/sensing/camera/zedxm/zed_node/left/image_rect_color` (sensor_msgs/Image, rgb8)
- `/sensing/camera/zedxm/zed_node/right/image_rect_color` (sensor_msgs/Image, rgb8)
- `/sensing/camera/zedxm/zed_node/left/camera_info` (sensor_msgs/CameraInfo)
- `/sensing/camera/zedxm/zed_node/right/camera_info` (sensor_msgs/CameraInfo)
- `/sensing/camera/zedxm/zed_node/imu/data` (sensor_msgs/Imu)

**Isaac SLAM Input Requirements**:
- `/visual_slam/image_0` (sensor_msgs/Image, **mono8**)
- `/visual_slam/camera_info_0` (sensor_msgs/CameraInfo)
- `/visual_slam/image_1` (sensor_msgs/Image, **mono8**)
- `/visual_slam/camera_info_1` (sensor_msgs/CameraInfo)
- `/visual_slam/imu` (sensor_msgs/Imu)

### 5.2 Image Format Conversion

Isaac SLAM **requires mono8** images. ZED publishes **rgb8**.

**Solution**: Use `isaac_ros_image_proc::ImageFormatConverterNode` (already included in `isaac_ros_visual_slam_core.launch.py`).

Modify the launch file to add topic remapping:

```xml
<!-- Image format converters -->
<node pkg="isaac_ros_image_proc" exec="isaac_ros_image_format_converter_node" name="left_image_converter">
  <remap from="image_raw" to="/sensing/camera/zedxm/zed_node/left/image_rect_color"/>
  <remap from="image" to="/visual_slam/image_0"/>
  <param name="encoding_desired" value="mono8"/>
</node>

<node pkg="isaac_ros_image_proc" exec="isaac_ros_image_format_converter_node" name="right_image_converter">
  <remap from="image_raw" to="/sensing/camera/zedxm/zed_node/right/image_rect_color"/>
  <remap from="image" to="/visual_slam/image_1"/>
  <param name="encoding_desired" value="mono8"/>
</node>

<!-- Camera info and IMU direct remapping -->
<node pkg="topic_tools" exec="relay" name="left_camera_info_relay"
      args="/sensing/camera/zedxm/zed_node/left/camera_info /visual_slam/camera_info_0"/>
<node pkg="topic_tools" exec="relay" name="right_camera_info_relay"
      args="/sensing/camera/zedxm/zed_node/right/camera_info /visual_slam/camera_info_1"/>
<node pkg="topic_tools" exec="relay" name="imu_relay"
      args="/sensing/camera/zedxm/zed_node/imu/data /visual_slam/imu"/>
```

**Better Approach**: Use composable nodes and NITROS for zero-copy (already handled in `isaac_ros_visual_slam_core.launch.py`).

---

## 6. TF Frame Coordination

### 6.1 Current AutoSDV TF Tree
```
map
 └─ odom (published by EKF localizer)
     └─ base_link
         ├─ zedxm_camera_center
         │   ├─ zedxm_left_camera_frame
         │   │   └─ zedxm_left_camera_optical_frame
         │   └─ zedxm_right_camera_frame
         │       └─ zedxm_right_camera_optical_frame
         ├─ imu_link
         └─ robin_w_link
```

### 6.2 Isaac SLAM TF Requirements

Isaac SLAM **publishes**:
- `odom -> base_link` (visual odometry)
- Optionally: `map -> odom` (if using localization in saved map)

**Conflict**: Both EKF Localizer and Isaac SLAM want to publish `odom -> base_link`.

**Solution**:
1. **Option A**: Disable Isaac SLAM TF publishing, use odometry message only
   - Set parameter: `publish_tf: false` in Isaac SLAM config
   - Bridge node publishes pose to EKF
   - EKF publishes TF

2. **Option B**: Use Isaac SLAM TF directly, bypass EKF for visual SLAM mode
   - When `pose_source: isaac`, disable EKF TF
   - Use Isaac SLAM's `odom -> base_link` directly

**Recommended**: Option A (cleaner integration with Autoware's sensor fusion).

---

## 7. Configuration Files

### 7.1 Isaac SLAM Parameter File

Create: `src/localization/autosdv_isaac_slam_launch/config/isaac_slam_params.yaml`

```yaml
/**:
  ros__parameters:
    # Camera configuration
    num_cameras: 2
    camera_optical_frames: ['zedxm_left_camera_optical_frame', 'zedxm_right_camera_optical_frame']

    # Frame IDs
    base_frame: 'base_link'
    map_frame: 'map'
    odom_frame: 'odom'

    # IMU configuration (ZED IMU)
    enable_imu_fusion: true
    imu_frame: 'zedxm_imu_link'

    # ZED IMU noise parameters (from ZED SDK specs)
    gyro_noise_density: 0.0004      # rad/s/√Hz
    gyro_random_walk: 0.00002       # rad/s²/√Hz
    accel_noise_density: 0.002      # m/s²/√Hz
    accel_random_walk: 0.0003       # m/s³/√Hz

    # Image processing
    rectified_images: true
    enable_image_denoising: false   # ZED already has good quality
    image_jitter_threshold_ms: 34.0  # 30 fps = 33.3ms

    # Visualization (disable for performance)
    enable_slam_visualization: true
    enable_landmarks_view: false
    enable_observations_view: false

    # Localization mode
    # For indoor: start fresh each time
    # For outdoor: can load map and localize
    localization_mode: 'tracking'  # 'tracking' or 'localization'

    # Performance tuning
    verbosity: 1  # 0=silent, 1=errors, 2=warnings, 3=info
```

### 7.2 Bridge Node Parameters

Create: `src/localization/odometry_pose_bridge/config/odometry_to_pose_bridge.yaml`

```yaml
/**:
  ros__parameters:
    # Frame override (if needed)
    override_frame_id: ''  # Leave empty to use odometry frame_id

    # Covariance scaling (if Isaac SLAM covariance is too optimistic/pessimistic)
    position_covariance_scale: 1.0
    orientation_covariance_scale: 1.0
```

---

## 8. AutoSDV Launch Modifications

### 8.1 Summary of Changes

**Philosophy**: All changes stay within AutoSDV packages. No Autoware source code modifications.

**Files to modify**:
1. `src/launcher/autosdv_launch/launch/autosdv.launch.yaml` - Add pose_source arg and conditional Isaac SLAM launch

**New packages to create**:
1. `src/localization/autosdv_isaac_slam_launch/` - Isaac SLAM wrapper
2. `src/localization/odometry_pose_bridge/` - Odometry-to-Pose converter

### 8.2 Detailed autosdv.launch.yaml Changes

**Step 1: Add pose_source argument** (after line 30):
```yaml
- arg:
    name: pose_source
    default: "ndt"
    description: "Localization source: ndt (LiDAR NDT), isaac (Visual SLAM)"
```

**Step 2: Conditionally launch Isaac SLAM** (before autoware.launch.xml include, around line 37):
```yaml
# Launch Isaac ROS Visual SLAM when pose_source is isaac
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
```

**Step 3: Modify Autoware localization launch** (around line 71):

**Option A - Disable Autoware localization entirely when using Isaac**:
```yaml
    -
      name: launch_localization
      value: "$(eval '\"$(var pose_source)\" == \"ndt\"')"
```

This means when `pose_source:=isaac`, Autoware's localization module doesn't run at all. Isaac SLAM publishes directly to `/localization/pose_estimator/pose_with_covariance`, and we'd need to launch EKF manually in autosdv_launch.

**Option B - Keep Autoware localization but bypass pose estimators** (Recommended):
```yaml
    -
      name: launch_localization
      value: "true"  # Always launch (includes EKF)
    -
      name: pose_source
      value: "$(eval '\"ndt\" if \"$(var pose_source)\" == \"isaac\" else \"$(var pose_source)\"')"
```

Then rely on topic remapping - when Isaac SLAM is active, it publishes to `/localization/pose_estimator/pose_with_covariance` which gets consumed by EKF. NDT also publishes there but will have no input pointcloud or will be ignored.

**Recommended: Option B** - Simpler, keeps EKF fusion active.

### 8.3 Usage Examples

```bash
# Indoor operation with Isaac SLAM only
make launch ARGS="pose_source:=isaac use_gnss:=false camera_model:=zedxm"

# Outdoor operation with NDT (default)
make launch ARGS="pose_source:=ndt"

# Default: NDT only (existing behavior, no changes needed)
make launch
```

**Note**: Multi-localizer fusion (`ndt_isaac`) would require additional work and is out of scope for initial integration.

---

## 9. Build and Test Plan

### 9.1 Phase 1: Dependency Setup
1. Add Isaac ROS submodules (common, nitros, image_pipeline)
2. Build workspace: `make build`
3. Verify Isaac ROS packages compile

**Test**: `ros2 pkg list | grep isaac_ros`

### 9.2 Phase 2: Bridge Node Development
1. Create `odometry_pose_bridge` package
2. Implement `odometry_to_pose_bridge` node
3. Unit test: manually publish odometry, verify pose output

**Test**:
```bash
# Terminal 1: Run bridge
ros2 run odometry_pose_bridge odometry_to_pose_bridge

# Terminal 2: Publish test odometry
ros2 topic pub /input/odometry nav_msgs/Odometry "..."

# Terminal 3: Check output
ros2 topic echo /output/pose_with_covariance
```

### 9.3 Phase 3: Standalone Isaac SLAM Test
1. Launch ZED camera only
2. Launch Isaac SLAM standalone
3. Verify visual odometry output

**Test**:
```bash
# Terminal 1: ZED camera
ros2 launch zed_wrapper zedxm.launch.py

# Terminal 2: Isaac SLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Terminal 3: Monitor
ros2 topic echo /visual_slam/tracking/odometry
rviz2  # Visualize /visual_slam/tracking/slam_path
```

### 9.4 Phase 4: AutoSDV Integration Test
1. Create `autosdv_isaac_slam_launch` package
2. Modify `autosdv.launch.yaml` to add conditional Isaac SLAM launch
3. Test with `pose_source:=isaac`

**Test**:
```bash
make launch ARGS="pose_source:=isaac use_gnss:=false camera_model:=zedxm"
```

**Verify**:
- Isaac SLAM nodes running in `isaac_slam` namespace
- `/localization/pose_estimator/pose_with_covariance` published by bridge node
- `/localization/kinematic_state` output from EKF (if using Option B)
- TF tree correct (no duplicate `odom -> base_link` transforms)
- Image converters successfully converting rgb8 to mono8

### 9.5 Phase 5: Indoor Validation
1. Drive robot in indoor environment
2. Monitor localization drift
3. Compare with ground truth (if available)

**Metrics**:
- Pose estimation latency
- Tracking success rate (from `/visual_slam/status`)
- Drift over distance traveled

---

## 10. Known Challenges and Mitigations

### 10.1 Challenge: Large Dependency Tree
- **Issue**: Isaac ROS has 10+ dependent packages
- **Mitigation**: Use apt packages in production, submodules for development
- **Status**: Acceptable, one-time setup cost

### 10.2 Challenge: GPU Resource Sharing
- **Issue**: Isaac SLAM and perception both use GPU (TensorRT)
- **Mitigation**: Profile GPU usage, may need to reduce perception load when using Isaac SLAM
- **Status**: Test on target Jetson hardware

### 10.3 Challenge: Loop Closure in Long Corridors
- **Issue**: Repetitive features in hallways → false loop closures
- **Mitigation**: Tune SLAM parameters, use AprilTags at intersections
- **Status**: Test in actual indoor environment

### 10.4 Challenge: ZED IMU Calibration
- **Issue**: Isaac SLAM IMU fusion requires accurate noise parameters
- **Mitigation**: Use ZED SDK's calibration, tune parameters empirically
- **Status**: Start with ZED datasheet values, refine

### 10.5 Challenge: Frame Rate vs Quality Trade-off
- **Issue**: Higher FPS → better tracking, but more GPU load
- **Mitigation**: ZED at 30fps is good balance (KITTI uses 10fps)
- **Status**: Configurable via ZED launch params

---

## 11. Alternative Approaches (Not Recommended)

### 11.1 Use ZED SDK's Internal SLAM
- **Pros**: Simpler integration, ZED-optimized
- **Cons**: Not GPU-accelerated like cuVSLAM, less performant
- **Decision**: Stick with Isaac ROS for best performance

### 11.2 Use RTAB-Map
- **Pros**: Mature, proven, RGB-D SLAM
- **Cons**: Not GPU-accelerated, CPU-heavy
- **Decision**: Isaac ROS is better fit for Jetson

### 11.3 Modify ZED to Output mono8 Directly
- **Pros**: Saves one conversion step
- **Cons**: Modifies upstream package, ZED uses rgb8 for other nodes
- **Decision**: Keep ZED unchanged, use Isaac's image converters

---

## 12. Next Steps (Implementation Order)

**Completed:**
1. ✅ **Study Isaac ROS Visual SLAM usage** (Completed)
2. ✅ **Study Autoware localization structure** (Completed)
3. ✅ **Design integration plan** (This document - Revised to avoid Autoware modifications)

**To Do:**

4. ⬜ **Add Isaac ROS dependencies as submodules** (AutoSDV workspace only)
   ```bash
   cd /home/aeon/repos/AutoSDV/2025.02

   # isaac_ros_common
   git submodule add https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git \
     src/sensor_component/external/isaac_ros_common
   cd src/sensor_component/external/isaac_ros_common && git checkout release-3.2 && cd ../../../..

   # isaac_ros_nitros
   git submodule add https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git \
     src/sensor_component/external/isaac_ros_nitros
   cd src/sensor_component/external/isaac_ros_nitros && git checkout release-3.2 && cd ../../../..

   # isaac_ros_image_pipeline
   git submodule add https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git \
     src/sensor_component/external/isaac_ros_image_pipeline
   cd src/sensor_component/external/isaac_ros_image_pipeline && git checkout release-3.2 && cd ../../../..

   git add .gitmodules src/sensor_component/external/
   ```

5. ⬜ **Create bridge node package**: `src/localization/odometry_pose_bridge/`
   - package.xml, CMakeLists.txt
   - src/odometry_to_pose_bridge.cpp (or Python version)
   - config/odometry_to_pose_bridge.yaml

6. ⬜ **Create Isaac SLAM launch package**: `src/localization/autosdv_isaac_slam_launch/`
   - package.xml, CMakeLists.txt
   - launch/isaac_slam_with_zed.launch.xml
   - config/isaac_slam_params.yaml
   - README.md

7. ⬜ **Modify**: `src/launcher/autosdv_launch/launch/autosdv.launch.yaml`
   - Add `pose_source` argument
   - Add conditional Isaac SLAM launch group
   - Adjust Autoware localization launch (Option B recommended)

8. ⬜ **Build workspace and test**:
   ```bash
   make build
   ```

9. ⬜ **Test standalone**: Isaac SLAM with ZED camera
   ```bash
   # Terminal 1: ZED only
   ros2 launch zed_wrapper zedxm.launch.py

   # Terminal 2: Isaac SLAM
   ros2 launch autosdv_isaac_slam_launch isaac_slam_with_zed.launch.xml
   ```

10. ⬜ **Test integrated**: Full AutoSDV launch with `pose_source:=isaac`
    ```bash
    make launch ARGS="pose_source:=isaac use_gnss:=false camera_model:=zedxm"
    ```

11. ⬜ **Validate**: Indoor driving test
    - Monitor `/visual_slam/status` for tracking state
    - Check pose drift over known distances
    - Verify loop closure detection

12. ⬜ **Document**: Update CLAUDE.md and create user guide
    - Add indoor localization section to CLAUDE.md
    - Document `pose_source:=isaac` usage
    - Troubleshooting guide

---

## 13. References

- [Isaac ROS Visual SLAM Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html)
- [cuVSLAM RealSense Tutorial](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/tutorial_realsense.html)
- [Autoware Localization Design](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/localization/)
- [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- AutoSDV CLAUDE.md (project-specific context)

---

**Document Version**: 1.0
**Last Updated**: 2025-10-08
**Author**: Claude Code (Automated Analysis)
