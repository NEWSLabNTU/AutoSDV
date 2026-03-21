# Modular Localization Launch Design

## Overview

This document proposes a modular replacement for Autoware's
`tier4_localization_component.launch.xml`, adopting the same plugin pattern
used by `vehicle_model` and `sensor_model`. Each pose estimation method
becomes a self-contained package discoverable at launch time by name.

---

## Part 1: Findings — Autoware 1.5.0 Localization Architecture

### Layer Diagram

```
autoware.launch.xml
└── tier4_localization_component.launch.xml     (autoware_launch)
    └── localization.launch.xml                 (tier4_localization_launch)
        ├── pose_twist_estimator.launch.xml      ← monolith with if/elif per source
        │   ├── ndt_scan_matcher.launch.xml
        │   ├── yabloc.launch.xml
        │   ├── gyro_odometer.launch.xml
        │   ├── eagleye/eagleye_rt.launch.xml
        │   ├── ar_tag_based_localizer.launch.xml
        │   ├── lidar_marker_localizer.launch.xml
        │   └── util/util.launch.xml             (pointcloud downsampling)
        ├── pose_twist_fusion_filter.launch.xml  (EKF + stop filter + twist2accel)
        └── localization_error_monitor.launch.xml
```

### Key Design Characteristics in Upstream

**1. Deep parameter tunneling.** `tier4_localization_component` passes ~25
parameter path args through two layers (`localization.launch.xml` →
`pose_twist_estimator.launch.xml`) even when most params are unused by the
chosen pose source.

**2. Monolithic pose source dispatch.** `pose_twist_estimator.launch.xml` uses
Python `eval` string splitting to parse compound sources like `ndt_yabloc`,
then activates each via `<group if>`. Every new pose source requires modifying
this file.

**3. Common infrastructure mixed with estimator logic.** The same file launches
`pose_initializer`, `gyro_odometer`, `util.launch.xml` (pointcloud
downsampling) alongside the estimator nodes. Pose initializer capability flags
(`ndt_enabled`, `yabloc_enabled`) are hardcoded booleans computed from the
active source.

**4. Pointcloud container.** The container is launched upstream in
`autoware.launch.xml`, not in the localization component. The container name is
passed down as `localization_pointcloud_container_name`. Composable nodes are
loaded into it via `<load_composable_node>`.

**5. Namespace structure.**

```
/localization/
├── pose_estimator/          → scan matcher output
├── twist_estimator/         → gyro or eagleye twist
├── pose_twist_fusion_filter/ → EKF, stop filter, twist2accel
├── kinematic_state          → final odometry (post stop filter)
├── pose_with_covariance     → final pose (post EKF)
├── acceleration             → twist2accel output
└── util/
    └── downsample/pointcloud → NDT input (after 3-stage downsampling)
```

### AutoSDV Current State

`autosdv_autoware.launch.xml` replaces `autoware.launch.xml`. It includes
`autosdv_launch/launch/components/tier4_localization_component.launch.xml`
instead of the upstream version.

The AutoSDV override currently dispatches via if/elif chains:

```
autosdv tier4_localization_component.launch.xml
├── if pose_source == cuda_ndt  →  cuda_localization.launch.xml
├── if pose_source == ndt       →  autoware_localization.launch.xml
└── else (yabloc, eagleye, …)   →  upstream localization.launch.xml
                                    (requires ~25 param path args)
```

`cuda_localization.launch.xml` and `autoware_localization.launch.xml` are
**full-stack duplicates** — each independently launches the twist estimator,
EKF, pose initializer, error monitor, and pointcloud downsampling. They share
no common infrastructure with each other or with the upstream fallback path.

**Known issues with this approach:**

| Issue                                                     | Impact                                                                                           |
|-----------------------------------------------------------|--------------------------------------------------------------------------------------------------|
| Full stack duplicated per source                          | Adding a new source means ~100-line copy-paste                                                   |
| `cuda_localization` and `autoware_localization` can drift | EKF config, stop filter, error monitor may diverge silently                                      |
| Three incompatible code paths in one file                 | `cuda_ndt`/`ndt` use their own stacks; `eagleye`/`yabloc` use upstream with a 25-arg passthrough |
| Isaac visual sources not yet integrated                   | `pose_source:=visual` and `pose_source:=isaac` require manual additions                          |
| util.launch.xml local copy in cuda_ndt_matcher            | Required to fix a composable node name collision; not visible to the other paths                 |

---

## Part 2: Proposed Design

### Core Principle

Same plugin pattern as `vehicle_model` and `sensor_model`:

```
# Discovery at launch time — no if/elif needed
<let name="pose_estimator_pkg"
     value="$(find-pkg-share $(var pose_source)_pose_estimator_launch)"/>
```

Adding a new pose source = creating a new ROS 2 package. Zero changes to the
orchestrator.

### Package Inventory

| Package                          | Location                                                                     | Role                                                      |
|----------------------------------|------------------------------------------------------------------------------|-----------------------------------------------------------|
| `autosdv_localization_launch`    | `src/launcher/autosdv_localization_launch/`                                  | Orchestrator: EKF, twist, error monitor, pose initializer |
| `ndt_pose_estimator_launch`      | `src/launcher/ndt_pose_estimator_launch/`                                    | Thin wrapper: Autoware NDT + pointcloud downsampling      |
| `cuda_ndt_pose_estimator_launch` | `src/localization/cuda_ndt_matcher/src/cuda_ndt_pose_estimator_launch/`      | CUDA NDT + pointcloud downsampling                        |
| `eagleye_pose_estimator_launch`  | `src/launcher/eagleye_pose_estimator_launch/`                                | Eagleye (pose + twist)                                    |
| `visual_pose_estimator_launch`   | `src/localization/autoware_isaac_localization/visual_pose_estimator_launch/` | Isaac cuVGL + cuVSLAM                                     |
| `isaac_pose_estimator_launch`    | `src/localization/autoware_isaac_localization/isaac_pose_estimator_launch/`  | cuVSLAM odometry only                                     |

Twist estimators are simpler and kept as internal sub-launches within the
orchestrator (only two exist: `gyro_odom` and `eagleye`). If a third is added,
they can be extracted to plugin packages using the same pattern.

### Responsibility Split

```
autosdv_localization_launch (orchestrator)        pose estimator plugin
──────────────────────────────────────────────    ──────────────────────────────
EKF localizer (pose_twist_fusion_filter)          Scan matcher / pose algorithm
Stop filter                                       Pointcloud preprocessing (if needed)
Twist2accel                                       Algorithm-specific params
Pose instability detector                         pose_initializer.launch.xml
Localization error monitor
Twist estimator (gyro_odom or eagleye)
Global parameter loader
Pointcloud container (pass-through arg)
```

The orchestrator owns everything that runs regardless of pose source. The
plugin owns everything that is specific to its algorithm.

---

## Part 3: Package Layouts

### `autosdv_localization_launch`

```
src/launcher/autosdv_localization_launch/
├── package.xml
├── CMakeLists.txt
└── launch/
    ├── localization.launch.xml              ← main entry point
    ├── common/
    │   ├── pose_twist_fusion_filter.launch.xml
    │   ├── localization_error_monitor.launch.xml
    │   └── util/
    │       └── pointcloud_downsample.launch.xml   ← shared by NDT-based plugins
    └── twist_estimator/
        ├── gyro_odom.launch.xml
        └── eagleye.launch.xml
```

### Pose Estimator Plugin (general layout)

```
${pose_source}_pose_estimator_launch/
├── package.xml
├── CMakeLists.txt
└── launch/
    ├── pose_estimator.launch.xml      ← REQUIRED: algorithm + preprocessing
    └── pose_initializer.launch.xml    ← REQUIRED: declares initializer capabilities
```

Plugins **may** also carry their own `config/` directory for params that are
not shared with other pose sources.

---

## Part 4: Interface Contracts

### `pose_estimator.launch.xml` — Required Arguments

Every plugin **must** accept all of the following. The orchestrator always
passes them:

```xml
<!-- Topics -->
<arg name="input_pointcloud"
     description="Raw concatenated LiDAR pointcloud from sensing"/>
<arg name="output_pose_with_covariance"
     default="/localization/pose_estimator/pose_with_covariance"/>

<!-- Paths -->
<arg name="config_dir"
     description="Absolute path to localization config directory"/>
<arg name="map_path"
     description="Absolute path to map directory"/>

<!-- Infrastructure -->
<arg name="localization_pointcloud_container_name"/>
<arg name="use_sim_time" default="false"/>
```

Plugins may declare additional optional args (e.g. `visual_map_dir` for Isaac)
with sensible defaults.

### `pose_initializer.launch.xml` — Required Arguments

The orchestrator always passes these. The plugin launches `pose_initializer`
with the capability flags appropriate for its algorithm:

```xml
<arg name="config_file"         description="pose_initializer.param.yaml path"/>
<arg name="gnss_enabled"        description="GNSS available for auto-initialization"/>
<arg name="initial_pose"        description="Optional: [x,y,z,qx,qy,qz,qw] or []"/>
<arg name="system_run_mode"     description="online | logging_simulation"/>
```

---

## Part 5: Launch File Contents

### `autosdv_localization_launch/launch/localization.launch.xml`

```xml
<?xml version="1.0"?>
<launch>
  <!-- ── Config ─────────────────────────────────────────────────────── -->
  <arg name="loc_config_path"
       default="$(find-pkg-share autosdv_launch)/config/localization"/>
  <arg name="map_path"
       description="Path to map directory (pointcloud map, visual map, etc.)"/>

  <!-- ── Pose / Twist source selection ─────────────────────────────── -->
  <arg name="pose_source"  default="ndt"
       description="Pose estimator key: ndt, cuda_ndt, eagleye, visual, isaac"/>
  <arg name="twist_source" default="gyro_odom"
       description="Twist estimator: gyro_odom, eagleye"/>

  <!-- ── Topics ────────────────────────────────────────────────────── -->
  <arg name="input_pointcloud"
       default="/sensing/lidar/concatenated/pointcloud"/>
  <arg name="localization_pointcloud_container_name"
       default="/pointcloud_container"/>

  <!-- ── System ────────────────────────────────────────────────────── -->
  <arg name="use_sim_time"     default="false"/>
  <arg name="system_run_mode"  default="online"
       description="online | logging_simulation"/>
  <arg name="gnss_enabled"     default="true"
       description="GNSS available for automatic pose initialization"/>
  <arg name="initial_pose"     default="[]"
       description="Optional fixed initial pose [x,y,z,qx,qy,qz,qw]"/>
  <arg name="vehicle_model"    default="sample_vehicle"/>

  <!-- ── Global parameters ─────────────────────────────────────────── -->
  <group scoped="false">
    <include file="$(find-pkg-share autoware_global_parameter_loader)/launch/global_params.launch.py">
      <arg name="use_sim_time"  value="$(var use_sim_time)"/>
      <arg name="vehicle_model" value="$(var vehicle_model)"/>
    </include>
  </group>

  <!-- ── Plugin resolution ─────────────────────────────────────────── -->
  <let name="pose_estimator_pkg"
       value="$(find-pkg-share $(var pose_source)_pose_estimator_launch)"/>

  <!-- ── Localization namespace ─────────────────────────────────────── -->
  <group>
    <push-ros-namespace namespace="localization"/>

    <!-- Pose estimator — fully delegated to plugin -->
    <group>
      <push-ros-namespace namespace="pose_estimator"/>
      <include file="$(var pose_estimator_pkg)/launch/pose_estimator.launch.xml">
        <arg name="input_pointcloud"
             value="$(var input_pointcloud)"/>
        <arg name="output_pose_with_covariance"
             value="/localization/pose_estimator/pose_with_covariance"/>
        <arg name="config_dir"
             value="$(var loc_config_path)"/>
        <arg name="map_path"
             value="$(var map_path)"/>
        <arg name="localization_pointcloud_container_name"
             value="$(var localization_pointcloud_container_name)"/>
        <arg name="use_sim_time"
             value="$(var use_sim_time)"/>
      </include>
    </group>

    <!-- Twist estimator — internal sub-launch keyed by twist_source -->
    <group>
      <push-ros-namespace namespace="twist_estimator"/>
      <include file="$(find-pkg-share autosdv_localization_launch)/launch/twist_estimator/$(var twist_source).launch.xml"/>
    </group>

    <!-- EKF + stop filter + twist2accel + pose instability detector -->
    <group>
      <push-ros-namespace namespace="pose_twist_fusion_filter"/>
      <include file="$(find-pkg-share autosdv_localization_launch)/launch/common/pose_twist_fusion_filter.launch.xml">
        <arg name="ekf_localizer_param_path"
             value="$(var loc_config_path)/ekf_localizer.param.yaml"/>
        <arg name="stop_filter_param_path"
             value="$(var loc_config_path)/stop_filter.param.yaml"/>
        <arg name="twist2accel_param_path"
             value="$(var loc_config_path)/twist2accel.param.yaml"/>
        <arg name="pose_instability_detector_param_path"
             value="$(var loc_config_path)/pose_instability_detector.param.yaml"/>
      </include>
    </group>

    <!-- Localization error monitor -->
    <include file="$(find-pkg-share autosdv_localization_launch)/launch/common/localization_error_monitor.launch.xml">
      <arg name="localization_error_monitor_param_path"
           value="$(var loc_config_path)/localization_error_monitor.param.yaml"/>
    </include>

    <!-- Pose initializer — plugin declares its own capability flags -->
    <group>
      <push-ros-namespace namespace="util"/>
      <include file="$(var pose_estimator_pkg)/launch/pose_initializer.launch.xml">
        <arg name="config_file"       value="$(var loc_config_path)/pose_initializer.param.yaml"/>
        <arg name="gnss_enabled"      value="$(var gnss_enabled)"/>
        <arg name="initial_pose"      value="$(var initial_pose)"/>
        <arg name="system_run_mode"   value="$(var system_run_mode)"/>
      </include>
    </group>

  </group>
</launch>
```

---

### `autosdv_localization_launch/launch/common/pose_twist_fusion_filter.launch.xml`

Extracted verbatim from `cuda_localization.launch.xml` (which itself mirrors
upstream). Centralizes what was previously duplicated in every full-stack
launch file.

```xml
<?xml version="1.0"?>
<launch>
  <arg name="ekf_localizer_param_path"/>
  <arg name="stop_filter_param_path"/>
  <arg name="twist2accel_param_path"/>
  <arg name="pose_instability_detector_param_path"/>

  <include file="$(find-pkg-share autoware_ekf_localizer)/launch/ekf_localizer.launch.xml">
    <arg name="input_initial_pose_name"              value="/initialpose3d"/>
    <arg name="input_pose_with_cov_name"             value="/localization/pose_estimator/pose_with_covariance"/>
    <arg name="input_twist_with_cov_name"            value="/localization/twist_estimator/twist_with_covariance"/>
    <arg name="output_odom_name"                     value="kinematic_state"/>
    <arg name="output_pose_name"                     value="pose"/>
    <arg name="output_pose_with_covariance_name"     value="/localization/pose_with_covariance"/>
    <arg name="output_biased_pose_name"              value="biased_pose"/>
    <arg name="output_biased_pose_with_covariance_name" value="biased_pose_with_covariance"/>
    <arg name="output_twist_name"                    value="twist"/>
    <arg name="output_twist_with_covariance_name"    value="twist_with_covariance"/>
    <arg name="param_file"                           value="$(var ekf_localizer_param_path)"/>
  </include>

  <include file="$(find-pkg-share autoware_stop_filter)/launch/stop_filter.launch.xml">
    <arg name="use_twist_with_covariance"            value="True"/>
    <arg name="input_odom_name"                      value="/localization/pose_twist_fusion_filter/kinematic_state"/>
    <arg name="input_twist_with_covariance_name"     value="/localization/pose_twist_fusion_filter/twist_with_covariance"/>
    <arg name="output_odom_name"                     value="/localization/kinematic_state"/>
    <arg name="param_path"                           value="$(var stop_filter_param_path)"/>
  </include>

  <include file="$(find-pkg-share autoware_twist2accel)/launch/twist2accel.launch.xml">
    <arg name="in_odom"   value="/localization/kinematic_state"/>
    <arg name="in_twist"  value="/localization/twist_estimator/twist_with_covariance"/>
    <arg name="out_accel" value="/localization/acceleration"/>
    <arg name="param_file" value="$(var twist2accel_param_path)"/>
  </include>

  <include file="$(find-pkg-share autoware_pose_instability_detector)/launch/pose_instability_detector.launch.xml">
    <arg name="input_odometry" value="/localization/kinematic_state"/>
    <arg name="input_twist"    value="/localization/twist_estimator/twist_with_covariance"/>
    <arg name="param_file"     value="$(var pose_instability_detector_param_path)"/>
  </include>
</launch>
```

---

### `autosdv_localization_launch/launch/common/util/pointcloud_downsample.launch.xml`

Shared by all LiDAR-based pose estimators (NDT, CUDA NDT). Uses the renamed
`localization_voxel_grid_downsample_filter` node to avoid the composable
container name collision documented in `cuda_ndt_matcher/CLAUDE.md`.

```xml
<?xml version="1.0"?>
<launch>
  <arg name="input_pointcloud"/>
  <arg name="localization_pointcloud_container_name"/>
  <arg name="crop_box_param_path"/>
  <arg name="voxel_grid_param_path"/>
  <arg name="random_downsample_param_path"/>
  <arg name="use_intra_process" default="true"/>

  <load_composable_node target="$(var localization_pointcloud_container_name)">
    <composable_node
      pkg="autoware_pointcloud_preprocessor"
      plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent"
      name="crop_box_filter_measurement_range">
      <param from="$(var crop_box_param_path)"/>
      <remap from="input"  to="$(var input_pointcloud)"/>
      <remap from="output" to="measurement_range/pointcloud"/>
      <extra_arg name="use_intra_process_comms" value="$(var use_intra_process)"/>
    </composable_node>

    <!-- Renamed to avoid collision with perception's voxel_grid_downsample_filter -->
    <composable_node
      pkg="autoware_pointcloud_preprocessor"
      plugin="autoware::pointcloud_preprocessor::VoxelGridDownsampleFilterComponent"
      name="localization_voxel_grid_downsample_filter">
      <param from="$(var voxel_grid_param_path)"/>
      <remap from="input"  to="measurement_range/pointcloud"/>
      <remap from="output" to="voxel_grid_downsample/pointcloud"/>
      <extra_arg name="use_intra_process_comms" value="$(var use_intra_process)"/>
    </composable_node>

    <composable_node
      pkg="autoware_pointcloud_preprocessor"
      plugin="autoware::pointcloud_preprocessor::RandomDownsampleFilterComponent"
      name="random_downsample_filter">
      <param from="$(var random_downsample_param_path)"/>
      <remap from="input"  to="voxel_grid_downsample/pointcloud"/>
      <remap from="output" to="downsample/pointcloud"/>
      <extra_arg name="use_intra_process_comms" value="$(var use_intra_process)"/>
    </composable_node>
  </load_composable_node>
</launch>
```

---

### `autosdv_localization_launch/launch/twist_estimator/gyro_odom.launch.xml`

```xml
<?xml version="1.0"?>
<launch>
  <include file="$(find-pkg-share autoware_gyro_odometer)/launch/gyro_odometer.launch.xml">
    <arg name="input_vehicle_twist_with_covariance_topic"
         value="/sensing/vehicle_velocity_converter/twist_with_covariance"/>
    <arg name="output_twist_with_covariance_topic"
         value="/localization/twist_estimator/twist_with_covariance"/>
    <arg name="output_twist_with_covariance_raw_topic"
         value="/localization/twist_estimator/twist_with_covariance_raw"/>
  </include>
</launch>
```

---

### `ndt_pose_estimator_launch/launch/pose_estimator.launch.xml`

```xml
<?xml version="1.0"?>
<!-- Pose estimator plugin: Autoware NDT (ndt_pose_estimator_launch)
     Wraps autoware_ndt_scan_matcher with the shared pointcloud downsampler. -->
<launch>
  <!-- Standard interface (required by all plugins) -->
  <arg name="input_pointcloud"/>
  <arg name="output_pose_with_covariance"
       default="/localization/pose_estimator/pose_with_covariance"/>
  <arg name="config_dir"/>
  <arg name="map_path"/>
  <arg name="localization_pointcloud_container_name"/>
  <arg name="use_sim_time" default="false"/>

  <!-- Pointcloud downsampling (shared utility) -->
  <include file="$(find-pkg-share autosdv_localization_launch)/launch/common/util/pointcloud_downsample.launch.xml">
    <arg name="input_pointcloud"
         value="$(var input_pointcloud)"/>
    <arg name="localization_pointcloud_container_name"
         value="$(var localization_pointcloud_container_name)"/>
    <arg name="crop_box_param_path"
         value="$(var config_dir)/ndt_scan_matcher/pointcloud_preprocessor/crop_box_filter_measurement_range.param.yaml"/>
    <arg name="voxel_grid_param_path"
         value="$(var config_dir)/ndt_scan_matcher/pointcloud_preprocessor/voxel_grid_filter.param.yaml"/>
    <arg name="random_downsample_param_path"
         value="$(var config_dir)/ndt_scan_matcher/pointcloud_preprocessor/random_downsample_filter.param.yaml"/>
  </include>

  <!-- NDT scan matcher -->
  <include file="$(find-pkg-share autoware_ndt_scan_matcher)/launch/ndt_scan_matcher.launch.xml">
    <arg name="input_pointcloud"
         value="/localization/util/downsample/pointcloud"/>
    <arg name="input_initial_pose_topic"
         value="/localization/pose_twist_fusion_filter/biased_pose_with_covariance"/>
    <arg name="input_regularization_pose_topic"
         value="/sensing/gnss/pose_with_covariance"/>
    <arg name="input_service_trigger_node"
         value="/localization/pose_estimator/trigger_node"/>
    <arg name="output_pose_topic"
         value="/localization/pose_estimator/pose"/>
    <arg name="output_pose_with_covariance_topic"
         value="$(var output_pose_with_covariance)"/>
    <arg name="client_map_loader"
         value="/map/get_differential_pointcloud_map"/>
    <arg name="param_file"
         value="$(var config_dir)/ndt_scan_matcher/ndt_scan_matcher.param.yaml"/>
  </include>
</launch>
```

---

### `ndt_pose_estimator_launch/launch/pose_initializer.launch.xml`

Each plugin's `pose_initializer.launch.xml` declares what capability flags the
underlying algorithm supports. The orchestrator passes the standard 4 args.

```xml
<?xml version="1.0"?>
<!-- Pose initializer for NDT.
     ndt_enabled=true: pose_initializer will call ndt_align_srv for localization init. -->
<launch>
  <arg name="config_file"/>
  <arg name="gnss_enabled"/>
  <arg name="initial_pose"/>
  <arg name="system_run_mode"/>

  <let name="stop_check_enabled"
       if="$(eval &quot;'$(var system_run_mode)' == 'online'&quot;)"
       value="true"/>
  <let name="stop_check_enabled"
       unless="$(eval &quot;'$(var system_run_mode)' == 'online'&quot;)"
       value="false"/>
  <let name="initial_pose_enable"
       value="$(eval &quot;len($(var initial_pose)) == 7&quot;)"/>
  <let name="initial_pose_value"
       value="$(var initial_pose)"
       if="$(var initial_pose_enable)"/>
  <let name="initial_pose_value"
       value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]"
       unless="$(var initial_pose_enable)"/>

  <include file="$(find-pkg-share autoware_pose_initializer)/launch/pose_initializer.launch.xml">
    <arg name="user_defined_initial_pose/enable" value="$(var initial_pose_enable)"/>
    <arg name="user_defined_initial_pose/pose"   value="$(var initial_pose_value)"/>
    <arg name="ndt_enabled"         value="true"/>   <!-- NDT supports ndt_align_srv -->
    <arg name="yabloc_enabled"      value="false"/>
    <arg name="gnss_enabled"        value="$(var gnss_enabled)"/>
    <arg name="ekf_enabled"         value="true"/>
    <arg name="stop_check_enabled"  value="$(var stop_check_enabled)"/>
    <arg name="config_file"         value="$(var config_file)"/>
    <arg name="sub_gnss_pose_cov"   value="/sensing/gnss/pose_with_covariance"/>
  </include>

  <group if="$(var gnss_enabled)">
    <include file="$(find-pkg-share autoware_automatic_pose_initializer)/launch/automatic_pose_initializer.launch.xml"/>
  </group>
</launch>
```

---

### `cuda_ndt_pose_estimator_launch/launch/pose_estimator.launch.xml`

Identical structure to the NDT plugin. Only the scan matcher include changes.
Lives inside `src/localization/cuda_ndt_matcher/` alongside the implementation.

```xml
<?xml version="1.0"?>
<!-- Pose estimator plugin: CUDA NDT (cuda_ndt_pose_estimator_launch) -->
<launch>
  <!-- Standard interface -->
  <arg name="input_pointcloud"/>
  <arg name="output_pose_with_covariance"
       default="/localization/pose_estimator/pose_with_covariance"/>
  <arg name="config_dir"/>
  <arg name="map_path"/>
  <arg name="localization_pointcloud_container_name"/>
  <arg name="use_sim_time" default="false"/>

  <!-- Plugin-specific: override NDT params with CUDA-tuned values -->
  <arg name="ndt_param_file"
       default="$(find-pkg-share cuda_ndt_pose_estimator_launch)/config/ndt_scan_matcher.param.yaml"
       description="CUDA NDT params; defaults to plugin's own tuned config"/>

  <!-- Pointcloud downsampling (shared utility) -->
  <include file="$(find-pkg-share autosdv_localization_launch)/launch/common/util/pointcloud_downsample.launch.xml">
    <arg name="input_pointcloud"
         value="$(var input_pointcloud)"/>
    <arg name="localization_pointcloud_container_name"
         value="$(var localization_pointcloud_container_name)"/>
    <arg name="crop_box_param_path"
         value="$(var config_dir)/ndt_scan_matcher/pointcloud_preprocessor/crop_box_filter_measurement_range.param.yaml"/>
    <arg name="voxel_grid_param_path"
         value="$(var config_dir)/ndt_scan_matcher/pointcloud_preprocessor/voxel_grid_filter.param.yaml"/>
    <arg name="random_downsample_param_path"
         value="$(var config_dir)/ndt_scan_matcher/pointcloud_preprocessor/random_downsample_filter.param.yaml"/>
  </include>

  <!-- CUDA NDT scan matcher -->
  <include file="$(find-pkg-share cuda_ndt_matcher_launch)/launch/cuda_ndt_scan_matcher.launch.xml">
    <arg name="input_pointcloud"
         value="/localization/util/downsample/pointcloud"/>
    <arg name="input_initial_pose_topic"
         value="/localization/pose_twist_fusion_filter/biased_pose_with_covariance"/>
    <arg name="input_regularization_pose_topic"
         value="/sensing/gnss/pose_with_covariance"/>
    <arg name="input_service_trigger_node"
         value="/localization/pose_estimator/trigger_node"/>
    <arg name="output_pose_topic"
         value="/localization/pose_estimator/pose"/>
    <arg name="output_pose_with_covariance_topic"
         value="$(var output_pose_with_covariance)"/>
    <arg name="client_map_loader"
         value="/map/get_differential_pointcloud_map"/>
    <arg name="param_file"
         value="$(var ndt_param_file)"/>
  </include>
</launch>
```

The `pose_initializer.launch.xml` for `cuda_ndt` is identical to the `ndt`
version (`ndt_enabled=true`) — both support the same `ndt_align_srv` service.

---

### `eagleye_pose_estimator_launch/launch/pose_estimator.launch.xml`

Eagleye is unique: it can provide pose, twist, or both. When used as
`pose_source:=eagleye`, the twist is also handled by Eagleye, so the standard
`twist_source:=gyro_odom` should be disabled. The orchestrator handles this by
checking if `pose_source == twist_source` (both eagleye) and skipping the
separate twist estimator.

```xml
<?xml version="1.0"?>
<!-- Pose estimator plugin: Eagleye GNSS/INS (eagleye_pose_estimator_launch)
     Eagleye provides both pose and twist; set twist_source:=eagleye as well. -->
<launch>
  <!-- Standard interface -->
  <arg name="input_pointcloud"/>   <!-- unused by Eagleye; accepted for interface compliance -->
  <arg name="output_pose_with_covariance"
       default="/localization/pose_estimator/pose_with_covariance"/>
  <arg name="config_dir"/>
  <arg name="map_path"/>           <!-- unused by Eagleye -->
  <arg name="localization_pointcloud_container_name"/>  <!-- unused -->
  <arg name="use_sim_time" default="false"/>

  <include file="$(find-pkg-share tier4_localization_launch)/launch/pose_twist_estimator/eagleye/eagleye_rt.launch.xml">
    <arg name="output_pose_with_cov_name"   value="$(var output_pose_with_covariance)"/>
    <arg name="output_twist_with_cov_name"  value="/localization/twist_estimator/twist_with_covariance"/>
    <arg name="use_eagleye_pose"            value="true"/>
    <arg name="use_eagleye_twist"           value="true"/>
    <arg name="eagleye_param_path"          value="$(var config_dir)/eagleye_config.param.yaml"/>
  </include>
</launch>
```

The Eagleye `pose_initializer.launch.xml` sets `ndt_enabled=false`,
`gnss_enabled` per arg, and wires `sub_gnss_pose_cov` to the Eagleye pose
output instead of raw GNSS.

---

## Part 6: Integration with `autosdv_autoware.launch.xml`

Replace the current localization group:

```xml
<!-- Before -->
<group if="$(var launch_localization)">
  <include file="$(find-pkg-share autosdv_launch)/launch/components/tier4_localization_component.launch.xml"/>
</group>

<!-- After -->
<group if="$(var launch_localization)">
  <include file="$(find-pkg-share autosdv_localization_launch)/launch/localization.launch.xml">
    <arg name="pose_source"    value="$(var pose_source)"/>
    <arg name="twist_source"   value="$(var twist_source)"/>
    <arg name="map_path"       value="$(var map_path)"/>
    <arg name="gnss_enabled"   value="$(var use_gnss)"/>
    <arg name="initial_pose"   value="$(var initial_pose)"/>
    <arg name="system_run_mode" value="$(var system_run_mode)"/>
    <arg name="use_sim_time"   value="$(var use_sim_time)"/>
    <arg name="localization_pointcloud_container_name"
                               value="$(var pointcloud_container_name)"/>
  </include>
</group>
```

`autosdv_launch` retains the `pose_source` and `twist_source` args it already
exposes. No other changes needed in `autosdv.launch.yaml` or
`logging_simulation.launch.yaml`.

---

## Part 7: Migration Path

### Phase 1 — Create the orchestrator (no behavior change)

1. Create `src/launcher/autosdv_localization_launch/` package
2. Implement `localization.launch.xml` with the full plugin dispatch
3. Implement `common/` sub-launches extracted from `cuda_localization.launch.xml`
4. Implement `twist_estimator/gyro_odom.launch.xml`
5. Wire `autosdv_autoware.launch.xml` to the new orchestrator
6. Verify `pose_source:=cuda_ndt` behavior is unchanged

### Phase 2 — Migrate NDT and CUDA NDT to plugins

1. Create `src/launcher/ndt_pose_estimator_launch/` with `pose_estimator.launch.xml` + `pose_initializer.launch.xml`
2. Create `cuda_ndt_pose_estimator_launch/` inside `cuda_ndt_matcher/`
3. Delete `cuda_localization.launch.xml` and `autoware_localization.launch.xml` (now superseded)
4. Delete `cuda_ndt_matcher_launch/launch/util/util.launch.xml` (now in the shared orchestrator)
5. Remove the if/elif dispatch from `autosdv_launch/components/tier4_localization_component.launch.xml`
6. Verify `pose_source:=ndt` and `pose_source:=cuda_ndt` both work

### Phase 3 — Eagleye plugin

1. Create `src/launcher/eagleye_pose_estimator_launch/`
2. Drop the upstream `localization.launch.xml` fallthrough from the old component
3. Verify `pose_source:=eagleye twist_source:=eagleye`

### Phase 4 — Isaac visual plugins (future)

1. Create `visual_pose_estimator_launch/` within `autoware_isaac_localization/`
2. Wire Isaac `visual_localization.launch.xml` behind the standard interface
3. Provides `pose_initializer.launch.xml` that wires in the Isaac pose bridge
   instead of `autoware_pose_initializer`

---

## Summary of Benefits

| Before                                                 | After                                                       |
|--------------------------------------------------------|-------------------------------------------------------------|
| Adding a pose source = modify orchestrator             | Adding a pose source = new package, no orchestrator changes |
| EKF/EKM duplicated in every full-stack launch          | Single canonical EKF/EKM in orchestrator                    |
| 3 incompatible code paths (cuda/ndt/upstream)          | 1 orchestrator, N uniform plugins                           |
| 25-arg passthrough to upstream localization.launch.xml | 6-arg standard interface per plugin                         |
| `util.launch.xml` local copy in cuda_ndt_matcher       | Shared utility in orchestrator, single source of truth      |
| Isaac sources not integrated                           | Isaac = standard plugin, no special casing                  |
