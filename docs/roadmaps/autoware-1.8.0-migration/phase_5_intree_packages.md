# Phase 5: In-Tree & Vendored Package Ports

**Objective**: Port the four in-tree (non-submodule) `src/` packages to jazzy / Autoware 1.8.0.

**Status**: ⬜ Not started

**Depends on**: Phase 2 (1.8.0 debs), Phase 3 (repo config)

## Packages

| Package                     | Path                                                             | Risk                                                       |
|-----------------------------|------------------------------------------------------------------|------------------------------------------------------------|
| Blickfeld driver (vendored) | `src/sensor_component/external/ros2_blickfeld_driver_src-v1.5.5` | Medium — C++ driver + patched scanner lib                  |
| Isaac SLAM launch           | `src/localization/autosdv_isaac_slam_launch`                     | Gated — Isaac ROS jazzy availability (Phase 1.2.3)         |
| Control test                | `src/vehicle/control_test`                                       | Low — Python; depends on `tier4_*_msgs`, `autoware_*_msgs` |
| Main launcher               | `src/launcher/autosdv_launch`                                    | Low — launch YAML; Autoware 1.8.0 launch args              |

## Work Items

### 5.1 Blickfeld driver

- [ ] **5.1.1** Rebuild blickfeld-scanner-lib on noble/gcc-13 per Phase 1.1.6 audit (keep `-newslab1` patch only if still needed; check for newer upstream lib and driver than v1.5.5).
- [ ] **5.1.2** Build `ros2_blickfeld_driver` against jazzy; fix rclcpp 28+ / gcc-13 issues.
- [ ] **5.1.3** 📟 *Device-only*: verify point cloud output with a real Cube1 (deferred to Phase 7 checklist).

### 5.2 Isaac SLAM launch (gated)

- [ ] **5.2.1** Apply Phase 1.2.3 decision: if Isaac ROS jazzy/JP7 packages exist, update apt repo + package names (`ros-jazzy-isaac-ros-*`) and rebuild; if not, mark the package `COLCON_IGNORE` with a note and keep the humble instructions in [../isaac_vslam.md](../isaac_vslam.md) as historical.
- [ ] **5.2.2** If ported: verify `odometry_pose_bridge` unit tests still pass (6/6 on humble baseline).

### 5.3 Control test & launcher

- [ ] **5.3.1** `control_test`: verify `tier4_*_msgs` / `autoware_*_msgs` deps exist in the 1.8.0 localrepo; fix any renamed message fields; `make test-control` scripts unaffected.
- [ ] **5.3.2** `autosdv_launch`: cross-check `autosdv_autoware.launch.xml` and perception presets against Autoware 1.8.0 launch arguments (autoware_launch 1.8.0); update renamed/removed args and parameter files.

## Goal (exit criteria)

- [ ] All four packages build green on jazzy (amd64), or are explicitly gated with `COLCON_IGNORE` + documented reason (Isaac only).
- [ ] `colcon test` passes for `control_test` and (if ported) `autosdv_isaac_slam_launch`.
- [ ] `autosdv_launch` dry-launches (`ros2 launch --show-args` / planning-sim smoke) without unknown-argument errors against Autoware 1.8.0.
