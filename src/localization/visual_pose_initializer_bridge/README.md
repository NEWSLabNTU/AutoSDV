# Visual Pose Initializer Bridge

Bridge node that connects NVIDIA cuVGL (Visual Global Localization) to Autoware's pose initialization system.

## Overview

This node:
1. Subscribes to cuVGL's global pose output
2. Calls Autoware's `/localization/initialize` service to set the initial pose
3. Enables camera-only localization without manual RViz pose input or GNSS

## Architecture

```
┌─────────────────┐     ┌──────────────────────────┐     ┌─────────────────────┐
│     cuVGL       │────▶│ Visual Pose Initializer  │────▶│ Autoware Pose       │
│ (Global Locali- │     │        Bridge            │     │   Initializer       │
│    zation)      │     │                          │     │                     │
└─────────────────┘     └──────────────────────────┘     └─────────────────────┘
        │                         │                               │
        │                         │                               │
/visual_localization/pose    ~/trigger_initialization    /localization/initialize
(PoseWithCovarianceStamped)      (Trigger service)       (InitializeLocalization)
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `auto_initialize` | bool | true | Auto-initialize on first cuVGL pose |
| `initialization_method` | int | 1 | 0=AUTO (NDT refine), 1=DIRECT |
| `reinitialize_on_trigger` | bool | true | Allow re-init via trigger service |
| `service_timeout_sec` | double | 5.0 | Timeout for service calls |
| `pose_topic` | string | /visual_localization/pose | cuVGL pose topic |
| `initialize_service` | string | /localization/initialize | Autoware init service |

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/visual_localization/pose` | geometry_msgs/PoseWithCovarianceStamped | Global pose from cuVGL |

## Services

### Provided

| Service | Type | Description |
|---------|------|-------------|
| `~/trigger_initialization` | std_srvs/Trigger | Manually trigger re-initialization |

### Called

| Service | Type | Description |
|---------|------|-------------|
| `/localization/initialize` | autoware_internal_localization_msgs/InitializeLocalization | Set Autoware initial pose |

## Usage

### Standalone Launch

```bash
ros2 launch visual_pose_initializer_bridge visual_pose_initializer.launch.xml
```

### With Parameters

```bash
ros2 launch visual_pose_initializer_bridge visual_pose_initializer.launch.xml \
    auto_initialize:=true \
    initialization_method:=1
```

### Manual Re-initialization

```bash
ros2 service call /visual_pose_initializer_bridge/trigger_initialization std_srvs/srv/Trigger
```

## State Machine

```
┌──────────┐    first pose received    ┌─────────────┐
│  WAITING │──────────────────────────▶│ INITIALIZING│
└──────────┘                           └──────┬──────┘
     ▲                                        │
     │              service success           ▼
     │                               ┌─────────────┐
     └────── trigger service ────────│ INITIALIZED │
                                     └─────────────┘
```

## Integration with AutoSDV

This node is automatically launched when using `pose_source:=visual`:

```bash
just launch pose_source:=visual visual_map_dir:=/path/to/map
```
