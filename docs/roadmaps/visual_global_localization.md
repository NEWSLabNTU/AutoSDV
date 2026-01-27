# Visual Global Localization (cuVGL) Integration Roadmap

**Goal**: Enable camera-only global localization for AutoSDV using NVIDIA cuVGL, eliminating the need for LiDAR-based NDT map matching.

**Status**: Planning

---

## Overview

This roadmap outlines the integration of NVIDIA Isaac ROS Visual Global Localization (cuVGL) to provide a complete camera-only localization solution for AutoSDV. This complements the existing cuVSLAM integration by adding global pose initialization capability.

### Why cuVGL?

| Problem                 | NDT Solution              | cuVGL Solution                 |
|-------------------------|---------------------------|--------------------------------|
| Initial pose estimation | Manual RViz click or GNSS | Automatic from visual features |
| Map type                | Point cloud (.pcd, ~GB)   | Visual keyframes (~100MB)      |
| Sensor required         | LiDAR                     | Stereo camera                  |
| Works in dark           | Yes                       | No                             |
| Works indoors (no GNSS) | Yes (with manual init)    | Yes (fully automatic)          |

### Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    Camera-Only Localization Pipeline                     │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌────────────────┐                                                      │
│  │  ZED X Mini    │                                                      │
│  │  (stereo+IMU)  │                                                      │
│  └───────┬────────┘                                                      │
│          │                                                               │
│          ▼                                                               │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                        Startup Phase                               │  │
│  │  ┌──────────────────┐     ┌────────────────────────────────────┐  │  │
│  │  │      cuVGL       │────▶│ /visual_localization/pose          │  │  │
│  │  │ (global localiz.)│     │ (PoseWithCovarianceStamped)        │  │  │
│  │  └──────────────────┘     └─────────────────┬──────────────────┘  │  │
│  │          │                                  │                      │  │
│  │          │ (runs once)                      ▼                      │  │
│  │          │                    ┌─────────────────────────────────┐  │  │
│  │          │                    │ Pose Initializer Bridge         │  │  │
│  │          │                    │ → /localization/initialize      │  │  │
│  │          │                    └─────────────────────────────────┘  │  │
│  └──────────┼────────────────────────────────────────────────────────┘  │
│             │                                                            │
│             ▼                                                            │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                       Tracking Phase                               │  │
│  │  ┌──────────────────┐     ┌────────────────────────────────────┐  │  │
│  │  │     cuVSLAM      │────▶│ /visual_slam/tracking/odometry     │  │  │
│  │  │ (visual odom)    │     │ (nav_msgs/Odometry)                │  │  │
│  │  └──────────────────┘     └─────────────────┬──────────────────┘  │  │
│  │                                             │                      │  │
│  │                                             ▼                      │  │
│  │                           ┌─────────────────────────────────────┐  │  │
│  │                           │ Odometry Pose Bridge                │  │  │
│  │                           │ → /localization/pose_estimator/...  │  │  │
│  │                           └─────────────────┬───────────────────┘  │  │
│  └─────────────────────────────────────────────┼────────────────────┘  │
│                                                │                        │
│                                                ▼                        │
│                              ┌─────────────────────────────────────┐   │
│                              │        ekf_localizer                │   │
│                              │ → /localization/kinematic_state     │   │
│                              └─────────────────────────────────────┘   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Platform Requirements

| Component | Jetson (ARM64) | x86_64 Workstation |
|-----------|----------------|---------------------|
| OS | Ubuntu 22.04 | Ubuntu 24.04 |
| ROS | Humble | Jazzy |
| Isaac ROS | 3.2+ (APT) | 4.0+ (APT) |
| Map Creation | ✅ Supported | ✅ Supported |
| Runtime | ✅ Supported | Build from source |

**Note**: Both map creation and runtime localization are supported on Jetson via APT packages. The `ros-humble-isaac-mapping-ros` package is available for ARM64.

---

## Phase 1: Visual Global Localization Runtime ⏳ In Progress

**Objective**: Install cuVGL runtime packages on Jetson for localization.

### 1.1 Package Overview

The setup script now installs all required packages:
- `ros-humble-isaac-ros-visual-slam` - Visual odometry (cuVSLAM)
- `ros-humble-isaac-ros-visual-global-localization` - Global localization (cuVGL)
- `ros-humble-isaac-ros-image-proc` - Image format conversion

**Note**: cuVGL depends on `ros-humble-isaac-mapping-ros`, which is automatically installed and enables map creation directly on Jetson.

### 1.2 Installation

```bash
# Run the setup script
./setup.sh isaac-ros

# Or install manually
sudo apt-get install -y ros-humble-isaac-ros-visual-global-localization
```

### 1.3 Verification Commands

```bash
# Check packages are installed
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -E "isaac_ros_visual_slam|isaac_ros_visual_global_localization|isaac_mapping"

# Check cuVGL node is available
ros2 pkg executables isaac_ros_visual_global_localization

# Test node starts (will fail without map, but should show usage)
ros2 run isaac_ros_visual_global_localization isaac_ros_visual_global_localization_node --ros-args -p map_dir:=/tmp
```

### Work Items

- [x] **1.1** Update `setup/scripts/install-isaac-ros.sh` to include cuVGL package
- [ ] **1.2** Verify cuVGL node is available after installation
- [ ] **1.3** Test cuVGL node starts without errors

### Success Criteria
- [ ] `ros2 pkg list | grep isaac_ros_visual_global_localization` shows package
- [ ] `ros2 pkg executables isaac_ros_visual_global_localization` lists the node

---

## Phase 2: Map Creation Workflow

**Objective**: Document and automate visual map creation process.

### 2.1 Package Availability

Map creation is supported directly on Jetson via APT:
```bash
# Installed automatically as dependency of cuVGL
ros-humble-isaac-mapping-ros
```

### 2.2 Map Creation Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                   Map Creation Workflow (on Jetson)              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. RECORD ROSBAG                                               │
│     ┌──────────────────────────────────────────────────────┐    │
│     │ ros2 bag record \                                     │    │
│     │   /zedxm/zed_node/left/image_rect_color/h264 \       │    │
│     │   /zedxm/zed_node/right/image_rect_color/h264 \      │    │
│     │   /zedxm/zed_node/left/camera_info \                 │    │
│     │   /zedxm/zed_node/right/camera_info \                │    │
│     │   /zedxm/zed_node/imu/data                           │    │
│     └──────────────────────────────────────────────────────┘    │
│                           │                                      │
│                           ▼                                      │
│  2. CREATE MAPS                                                 │
│     ┌──────────────────────────────────────────────────────┐    │
│     │ ros2 run isaac_mapping_ros create_map_offline.py \   │    │
│     │   --sensor_data_bag=./mapping_bag \                  │    │
│     │   --base_output_folder=~/AutoSDV/data/visual_maps/   │    │
│     └──────────────────────────────────────────────────────┘    │
│                           │                                      │
│                           ▼                                      │
│  3. OUTPUTS (3 map types)                                       │
│     ├── cuvslam_map/     (cuVSLAM landmarks for tracking)       │
│     ├── cuvgl_map/       (cuVGL keyframes for global loc)       │
│     └── occupancy_map/   (2D grid for navigation)               │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.3 Recording Requirements

**Critical requirements for successful map creation:**

1. **H264 compressed images** - Required format for `create_map_offline.py`
2. **10+ seconds stationary at start** - For initialization
3. **Closed loops** - Return to visited areas for loop closure (5-10m segments)
4. **Feature-rich environment** - Furniture, objects (not blank walls)
5. **Consistent lighting** - Avoid drastic light changes
6. **Slow, smooth motion** - Keep trajectory within 1m of future operating path

### Work Items

- [ ] **2.1** Create map recording script (`scripts/record-visual-map.sh`)
- [ ] **2.2** Create map creation script (`scripts/create-visual-map.sh`)
- [ ] **2.3** Document workflow in `docs/guides/visual_map_creation.md`
- [ ] **2.4** Test end-to-end map creation workflow
- [ ] **2.5** Add example maps to `data/visual_maps/` with README

### Success Criteria
- [ ] Recording script captures all required topics with H264 compression
- [ ] Map creation produces all 3 map types without errors
- [ ] cuVGL can load the created maps

---

## Phase 3: Pose Initializer Bridge

**Objective**: Create a bridge node that sends cuVGL global pose to Autoware's pose initializer.

### 3.1 Bridge Node Design

```
┌─────────────────────────────────────────────────────────────────┐
│                  VisualPoseInitializerBridge                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Subscriptions:                                                  │
│  ├── /visual_localization/pose                                  │
│  │   (geometry_msgs/PoseWithCovarianceStamped)                  │
│  │                                                               │
│  └── /visual_localization/status  (optional, for diagnostics)   │
│                                                                  │
│  Service Clients:                                                │
│  └── /localization/initialize                                   │
│      (autoware_internal_localization_msgs/srv/InitializeLocal.) │
│                                                                  │
│  Parameters:                                                     │
│  ├── initialization_method: 1  (direct set, no NDT refinement)  │
│  ├── auto_initialize: true     (call service on first valid pose)│
│  ├── min_confidence: 0.5       (minimum pose confidence)        │
│  └── reinitialize_on_lost: true (re-init if tracking lost)      │
│                                                                  │
│  State Machine:                                                  │
│  ┌──────────┐    pose received    ┌─────────────┐               │
│  │  WAITING │───────────────────▶│ INITIALIZING│               │
│  └──────────┘                     └──────┬──────┘               │
│       ▲                                  │                       │
│       │              service success     ▼                       │
│       │                           ┌─────────────┐               │
│       └───────tracking lost───────│ INITIALIZED │               │
│                                   └─────────────┘               │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 Package Structure

```
src/localization/visual_pose_initializer_bridge/
├── package.xml
├── CMakeLists.txt
├── src/
│   └── visual_pose_initializer_bridge.cpp
├── config/
│   └── visual_pose_initializer.yaml
├── launch/
│   └── visual_pose_initializer.launch.xml
└── README.md
```

### Work Items

- [ ] **3.1** Create `visual_pose_initializer_bridge` package
- [ ] **3.2** Implement bridge node (C++ for low latency)
- [ ] **3.3** Add state machine for initialization lifecycle
- [ ] **3.4** Create unit tests
- [ ] **3.5** Create launch file with parameter configuration

### Success Criteria
- [ ] Bridge receives cuVGL pose and calls Autoware initialize service
- [ ] State machine correctly handles init/tracking/lost states
- [ ] Unit tests pass

---

## Phase 4: AutoSDV Launch Integration

**Objective**: Integrate cuVGL into AutoSDV launch system with new `pose_source:=visual` option.

### 4.1 Launch Parameter Design

```yaml
# New pose_source option: "visual" (camera-only, cuVGL + cuVSLAM)
pose_source:
  - ndt        # LiDAR NDT (default, requires point cloud map)
  - isaac      # cuVSLAM only (relative tracking, manual init)
  - visual     # cuVGL + cuVSLAM (full camera-only solution)
```

### 4.2 Launch File Structure

```xml
<!-- autosdv.launch.yaml additions -->

<!-- Visual Global Localization (cuVGL + cuVSLAM) -->
<group if="$(eval '\"$(var pose_source)\" == \"visual\"')">

  <!-- cuVGL for initial pose -->
  <include file="$(find-pkg-share autosdv_visual_localization_launch)/launch/visual_global_localization.launch.xml">
    <arg name="map_dir" value="$(var visual_map_dir)"/>
    <arg name="camera_namespace" value="/sensing/camera/$(var camera_model)/zed_node"/>
  </include>

  <!-- Pose initializer bridge -->
  <include file="$(find-pkg-share visual_pose_initializer_bridge)/launch/visual_pose_initializer.launch.xml"/>

  <!-- cuVSLAM for continuous tracking (existing) -->
  <include file="$(find-pkg-share autosdv_isaac_slam_launch)/launch/isaac_slam_with_zed.launch.xml">
    <arg name="camera_namespace" value="/sensing/camera/$(var camera_model)/zed_node"/>
    <arg name="enable_imu_fusion" value="true"/>
  </include>

</group>
```

### 4.3 New Launch Arguments

```yaml
# Visual map directory (for cuVGL)
- arg:
    name: visual_map_dir
    default: "$(find-pkg-share autosdv_launch)/data/visual_maps/default"
    description: "Path to cuVGL visual map directory"
```

### Work Items

- [ ] **4.1** Create `autosdv_visual_localization_launch` package
- [ ] **4.2** Add `pose_source:=visual` option to `autosdv.launch.yaml`
- [ ] **4.3** Add `visual_map_dir` argument
- [ ] **4.4** Configure cuVGL node with ZED camera topics
- [ ] **4.5** Wire up pose initializer bridge
- [ ] **4.6** Disable NDT when using visual localization

### Success Criteria
- [ ] `just launch pose_source:=visual` starts all required nodes
- [ ] cuVGL initializes pose automatically on startup
- [ ] cuVSLAM takes over for continuous tracking
- [ ] System works without LiDAR

---

## Phase 5: Setup Script Updates

**Objective**: Update `setup.sh` to install cuVGL dependencies.

### 5.1 Modified Install Script

Update `setup/scripts/install-isaac-ros.sh`:

```bash
#!/usr/bin/env bash
# Install NVIDIA Isaac ROS packages for Visual SLAM and Global Localization
# Requires: ARM64 (Jetson) with JetPack 6.x and ROS 2 Humble

# ... existing checks ...

printf "${YELLOW}→${NC} Installing Isaac ROS Visual SLAM packages...\n"

# Install Isaac ROS Visual SLAM + Global Localization packages
sudo apt-get install -y \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-visual-global-localization \
    ros-humble-isaac-ros-image-proc

# Verify installation
printf "${YELLOW}→${NC} Verifying installation...\n"

for pkg in isaac_ros_visual_slam isaac_ros_visual_global_localization isaac_ros_image_proc; do
    if bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list 2>/dev/null | grep -q \"$pkg\""; then
        printf "${GREEN}✓${NC} $pkg installed\n"
    else
        printf "${RED}✗${NC} $pkg not found\n"
        exit 1
    fi
done

printf "${GREEN}✓${NC} Isaac ROS installation complete\n"
printf "\nUsage:\n"
printf "  pose_source:=isaac   - Visual odometry only (manual init)\n"
printf "  pose_source:=visual  - Full visual localization (auto init + tracking)\n"
```

### Work Items

- [x] **5.1** Update `install-isaac-ros.sh` to include cuVGL package
- [x] **5.2** Update setup.sh interactive question text
- [x] **5.3** Update justfile status display
- [ ] **5.4** Test clean installation on fresh Jetson

### Success Criteria
- [x] `./setup.sh isaac-ros` installs cuVGL package
- [x] `./setup.sh status` shows visual localization status
- [ ] Clean install works on fresh JetPack 6.2.1

---

## Phase 6: Testing and Validation

**Objective**: Validate end-to-end camera-only localization.

### 6.1 Test Scenarios

| Test | Description | Success Criteria |
|------|-------------|------------------|
| T1: Startup Init | Robot starts in mapped area | Pose initialized within 5s |
| T2: Tracking | Robot moves through mapped area | Continuous pose updates, no drift |
| T3: Loop Closure | Robot returns to start | Position error < 0.5m |
| T4: Recovery | Tracking lost and recovered | Re-initializes automatically |
| T5: No LiDAR | System runs without LiDAR | All localization works |

### 6.2 Performance Benchmarks

| Metric | Target | Measured |
|--------|--------|----------|
| Global localization time | < 1.0s | TBD |
| Global localization accuracy | < 0.1m | TBD |
| Tracking update rate | 30 Hz | TBD |
| Tracking drift (100m path) | < 1.0m | TBD |
| GPU memory usage | < 2 GB | TBD |

### Work Items

- [ ] **6.1** Create test map of lab/office environment
- [ ] **6.2** Run startup initialization tests
- [ ] **6.3** Run continuous tracking tests
- [ ] **6.4** Run loop closure tests
- [ ] **6.5** Run recovery tests
- [ ] **6.6** Measure and document performance

### Success Criteria
- [ ] All test scenarios pass
- [ ] Performance meets targets
- [ ] System stable for 30+ minute operation

---

## Phase 7: Documentation

**Objective**: Complete user-facing documentation.

### Work Items

- [ ] **7.1** Update `CLAUDE.md` with visual localization section
- [ ] **7.2** Create `docs/guides/visual_localization.md` user guide
- [ ] **7.3** Create `docs/guides/visual_map_creation.md` map creation guide
- [ ] **7.4** Update `docs/guides/isaac_vslam_testing.md` with cuVGL info
- [ ] **7.5** Add troubleshooting section
- [ ] **7.6** Document known limitations

### Documentation Outline

```
docs/guides/visual_localization.md
├── Overview
│   ├── What is Visual Global Localization?
│   ├── When to use camera-only localization
│   └── Comparison with NDT
├── Quick Start
│   ├── Prerequisites
│   ├── Using existing maps
│   └── Launch commands
├── Map Creation
│   ├── Recording rosbags
│   ├── Jazzy workstation setup
│   ├── Running map creation
│   └── Map management
├── Configuration
│   ├── Launch parameters
│   ├── cuVGL parameters
│   └── cuVSLAM parameters
├── Troubleshooting
│   ├── Initialization failures
│   ├── Tracking lost
│   └── Poor accuracy
└── References
```

### Success Criteria
- [ ] All documentation files created
- [ ] User can follow guides without assistance
- [ ] Troubleshooting covers common issues

---

## Summary

### Dependencies (Jetson Humble via APT)

**Runtime + Map Creation:**
- `ros-humble-isaac-ros-visual-slam` - Visual odometry (cuVSLAM)
- `ros-humble-isaac-ros-visual-global-localization` - Global localization (cuVGL)
- `ros-humble-isaac-ros-image-proc` - Image format conversion
- `ros-humble-isaac-mapping-ros` - Map creation (auto-installed as cuVGL dependency)

### New Packages

1. `visual_pose_initializer_bridge` - Bridges cuVGL to Autoware pose initializer
2. `autosdv_visual_localization_launch` - Launch files for visual localization

### Launch Usage

```bash
# Camera-only localization (full solution)
just launch pose_source:=visual visual_map_dir:=/path/to/map

# Visual odometry only (existing, manual init)
just launch pose_source:=isaac

# LiDAR NDT (default)
just launch pose_source:=ndt
```

---

## References

- [Visual Global Localization Concepts](https://nvidia-isaac-ros.github.io/concepts/visual_global_localization/index.html)
- [isaac_ros_visual_global_localization Package](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mapping_and_localization/isaac_ros_visual_global_localization/index.html)
- [Map Creation Tutorial](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mapping_and_localization/isaac_mapping_ros/tutorial_map_creation.html)
- [Autoware Pose Initializer](https://autowarefoundation.github.io/autoware_core/pr-536/localization/autoware_pose_initializer/)
- [cuVSLAM Paper](https://arxiv.org/abs/2506.04359)
- [Existing Isaac VSLAM Roadmap](./isaac_vslam.md)

---

**Document Version**: 1.0
**Created**: 2026-01-27
**Author**: Claude Code
