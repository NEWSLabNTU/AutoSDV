# MRM (Minimum Risk Maneuver) Configuration Guide

**Date**: 2025-12-28
**Status**: Active Configuration

## Overview

The Minimum Risk Maneuver (MRM) system is Autoware's safety mechanism that triggers emergency stops when autonomous operation becomes unsafe. This guide documents the MRM system behavior, configuration, and AutoSDV-specific modifications to prevent false triggers during outdoor testing.

## Problem: False Emergency Stops During Outdoor Testing

### Symptoms
- Vehicle suddenly enters emergency stop during normal autonomous operation
- Hard braking with -2.5 m/s² deceleration
- `/system/fail_safe/mrm_state` shows emergency state
- RViz shows localization uncertainty warnings

### Root Cause
The default MRM configuration triggers emergency stops when localization uncertainty exceeds strict thresholds:
- Position error > **1.5m**
- Lateral error > **0.3m**

With VLP-32C LiDAR and NDT localization on the COSS map, these thresholds are too conservative and cause false triggers even during good localization performance.

## MRM System Architecture

### Emergency Trigger Chain

```
Localization Error Monitor
  ├─ Calculates error ellipse from odometry covariance
  ├─ Checks: ellipse size > 1.5m OR lateral > 0.3m
  └─ Publishes: /autoware/localization/accuracy (diagnostic)
           ↓
Diagnostic Aggregator
  ├─ Checks ALL localization diagnostics pass
  ├─ /autoware/localization requires:
  │   ├─ state (component state)
  │   ├─ topic_rate_check/transform
  │   ├─ topic_rate_check/pose_twist_fusion
  │   ├─ scan_matching_status (NDT score)
  │   ├─ accuracy ← DISABLED IN AUTOSDV
  │   └─ sensor_fusion_status (EKF)
  └─ Publishes: /autoware/modes/autonomous (availability)
           ↓
MRM Handler
  ├─ Monitors autonomous mode availability
  ├─ Detects: !autonomous || timeout || holding
  └─ Triggers: EMERGENCY_STOP behavior
           ↓
MRM Emergency Stop Operator
  ├─ Publishes control command:
  │   ├─ target_acceleration: -2.5 m/s²
  │   └─ target_jerk: -1.5 m/s³
  └─ Output: /system/mrm/emergency_stop/control_cmd
```

### MRM State Machine

```
NORMAL → EMERGENCY → RECOVERY → NORMAL
  ↑                      ↓
  └──────────────────────┘
```

**States:**
1. **NORMAL**: Autonomous mode available, normal operation
2. **EMERGENCY**: Autonomous unavailable, executing emergency behavior
3. **RECOVERY**: Autonomous restored, waiting for recovery timeout (5s default)

**Emergency Behaviors (priority order):**
1. **PULL_OVER**: Attempt to pull over safely (disabled by default)
2. **COMFORTABLE_STOP**: Gentle deceleration -1.0 m/s² (disabled by default)
3. **EMERGENCY_STOP**: Hard braking -2.5 m/s² (always active)

## AutoSDV Configuration Changes

### Change 1: Disable Localization Accuracy Check (IMPLEMENTED)

**File**: `src/launcher/autosdv_launch/config/system/diagnostics/localization.yaml`

**Modification:**
```yaml
/autoware/localization:
  type: short-circuit-and
  list:
    - type: link
      link: /autoware/localization/state
    - type: and
      list:
        - { type: link, link: /autoware/localization/topic_rate_check/transform }
        - { type: link, link: /autoware/localization/topic_rate_check/pose_twist_fusion }
        - { type: link, link: /autoware/localization/scan_matching_status }
        # AutoSDV: DISABLED to prevent false MRM triggers
        # - { type: link, link: /autoware/localization/accuracy }
        - { type: link, link: /autoware/localization/sensor_fusion_status }
```

**Impact:**
- ✅ MRM no longer triggered by localization uncertainty alone
- ✅ Localization quality still monitored via NDT scan_matching_status
- ⚠️ Operator must monitor localization quality via RViz and topics

**Rationale:**
- VLP-32C with NDT on COSS map has 5.4% pose rejection rate (acceptable with EKF fusion)
- Brief uncertainty spikes are normal and handled by EKF sensor fusion
- NDT score threshold (2.2) already provides localization quality control

### Change 2: Alternative - Increase Error Thresholds (NOT IMPLEMENTED)

**File**: `autoware/src/universe/autoware.universe/localization/autoware_localization_error_monitor/config/localization_error_monitor.param.yaml`

**Proposed Override (if needed in future):**
```yaml
/**:
  ros__parameters:
    scale: 3.0
    error_ellipse_size: 3.0              # Increased from 1.5m
    warn_ellipse_size: 2.0               # Increased from 1.2m
    error_ellipse_size_lateral_direction: 0.6   # Increased from 0.3m
    warn_ellipse_size_lateral_direction: 0.4    # Increased from 0.25m
```

**To implement:**
1. Copy file to `src/launcher/autosdv_launch/config/localization/`
2. Update launch file to override default config
3. Re-enable `/autoware/localization/accuracy` in diagnostics

**When to use:**
- If accuracy check provides useful safety margin
- If localization uncertainty is well-calibrated
- If false triggers can be eliminated with higher thresholds

## Monitoring and Debugging

### Check Current MRM State

```bash
# Autonomous mode availability (OK/ERROR)
ros2 topic echo /system/operation_mode/availability

# MRM state machine (NORMAL/EMERGENCY/RECOVERY)
ros2 topic echo /system/fail_safe/mrm_state

# Localization error ellipse (current uncertainty)
ros2 topic echo /localization/localization_error_monitor/debug/ellipse_marker

# NDT scan matching status
ros2 topic echo /localization/pose_estimator/ndt_scan_matcher/exe_time_ms
ros2 topic echo /localization/pose_estimator/ndt_scan_matcher/diagnostics
```

### View All Diagnostics in Real-Time

```bash
# GUI diagnostic monitor (recommended)
ros2 run rqt_robot_monitor rqt_robot_monitor

# Command-line diagnostics
ros2 topic echo /diagnostics
ros2 topic echo /diagnostics_agg
```

### Check Diagnostic Tree Status

```bash
# Check localization diagnostics
ros2 topic echo /system/system_diagnostic_monitor/diagnostics | grep -A 10 localization

# Check autonomous mode availability
ros2 topic echo /system/operation_mode/availability | grep autonomous
```

## Configuration Files Reference

### AutoSDV Configuration (Our Repo)
```
src/launcher/autosdv_launch/config/system/
├── diagnostics/
│   ├── localization.yaml           # MODIFIED: Disabled accuracy check
│   ├── autoware-main.yaml          # Defines autonomous mode requirements
│   └── ...
├── mrm_handler/
│   └── mrm_handler.param.yaml      # MRM behavior selection
├── mrm_emergency_stop_operator/
│   └── mrm_emergency_stop_operator.param.yaml   # Emergency stop params
└── mrm_comfortable_stop_operator/
    └── mrm_comfortable_stop_operator.param.yaml # Gentle stop params
```

### Autoware Default Configuration (Submodule - DO NOT MODIFY)
```
autoware/src/universe/autoware.universe/
├── localization/autoware_localization_error_monitor/
│   └── config/localization_error_monitor.param.yaml  # Error thresholds
├── system/mrm_handler/
│   ├── config/mrm_handler.param.yaml
│   └── src/mrm_handler/mrm_handler_core.cpp    # MRM logic
└── system/mrm_emergency_stop_operator/
    └── config/mrm_emergency_stop_operator.param.yaml
```

## MRM Handler Parameters

**File**: `autoware/src/universe/autoware.universe/system/mrm_handler/config/mrm_handler.param.yaml`

**Key Parameters:**
```yaml
use_pull_over: false                 # Enable pull-over behavior
use_comfortable_stop: false          # Enable gentle stop (-1.0 m/s²)
use_emergency_holding: true          # Hold emergency after timeout
timeout_operation_mode_availability: 0.5  # Timeout before emergency (seconds)
timeout_emergency_recovery: 5.0      # Recovery wait time (seconds)
turning_hazard_on.emergency: true    # Hazard lights during emergency
```

**To enable gentler emergency stops (if needed):**
1. Set `use_comfortable_stop: true`
2. Rebuild with `make build` (config changes only, fast)
3. Test emergency stop behavior

## Emergency Stop Parameters

**File**: `autoware/src/universe/autoware.universe/system/mrm_emergency_stop_operator/config/mrm_emergency_stop_operator.param.yaml`

**Default Values:**
```yaml
update_rate: 30                      # Hz
target_acceleration: -2.5            # m/s² (hard braking)
target_jerk: -1.5                    # m/s³ (aggressive deceleration)
```

**To reduce braking aggressiveness (if needed):**
```yaml
target_acceleration: -1.5            # Gentler braking
target_jerk: -1.0                    # Smoother deceleration
```

## Testing Recommendations

### Before Outdoor Testing

1. **Monitor diagnostics in RViz:**
   - Check `/diagnostics` panel for warnings
   - Verify `/autoware/localization` shows OK
   - Watch for `/autoware/localization/accuracy` errors (now disabled)

2. **Check localization uncertainty:**
   ```bash
   ros2 topic echo /localization/localization_error_monitor/debug/ellipse_marker
   ```
   - Monitor ellipse size during normal operation
   - Typical values should be < 2.0m for good localization

3. **Verify NDT performance:**
   ```bash
   ros2 topic echo /localization/pose_estimator/ndt_scan_matcher/diagnostics
   ```
   - Check score > 2.2 threshold
   - Verify execution time < 15ms
   - Monitor rejection rate

### During Outdoor Testing

1. **Have emergency stop button ready** (E-stop hardware)

2. **Monitor MRM state continuously:**
   ```bash
   ros2 topic echo /system/fail_safe/mrm_state
   ```

3. **Watch for unexpected transitions:**
   - NORMAL → EMERGENCY (investigate trigger cause)
   - EMERGENCY → RECOVERY → NORMAL (check what recovered)

4. **Log all emergency events** for post-analysis

### If Emergency Stop Occurs

1. **Immediately check diagnostics:**
   ```bash
   ros2 run rqt_robot_monitor rqt_robot_monitor
   ```

2. **Check which diagnostic failed:**
   - Localization (unlikely with accuracy check disabled)
   - Perception (object detection timeout)
   - Planning (path planning failure)
   - Control (control command timeout)
   - Vehicle (vehicle interface timeout)

3. **Review rosbag for analysis:**
   ```bash
   ros2 bag play <emergency_rosbag> --topics /diagnostics /system/fail_safe/mrm_state
   ```

## Future Improvements

### Option 1: Dynamic Threshold Adjustment
- Implement adaptive error thresholds based on environment
- Lower thresholds in open areas (highway)
- Higher thresholds in complex areas (urban canyon)

### Option 2: Multi-Level MRM
- Implement graduated response (warning → comfortable stop → emergency stop)
- Allow operator intervention before full emergency stop

### Option 3: Localization Diversity
- Add visual odometry fusion for redundant localization
- Use IMU/wheel odometry to bridge brief localization gaps
- Implement map-based localization fallback

### Option 4: Better Uncertainty Estimation
- Calibrate localization_error_monitor for VLP-32C
- Tune covariance estimation in NDT and EKF
- Validate error ellipse correlates with actual error

## Related Documentation

- **MRM Troubleshooting**: `docs/guides/mrm_troubleshooting.md` (if exists)
- **NDT Parameter Tuning**: `docs/research/localization/ndt_parameter_tuning_coss_map.md`
- **Control Testing**: `docs/guides/control_testing.md`
- **Configuration README**: `src/launcher/autosdv_launch/config/README.md`

## References

**Autoware Documentation:**
- [Fail-safe Design](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/fail-safe/)
- [MRM Handler](https://github.com/autowarefoundation/autoware.universe/tree/main/system/mrm_handler)

**Source Code:**
- MRM Handler: `autoware/src/universe/autoware.universe/system/mrm_handler/`
- Localization Error Monitor: `autoware/src/universe/autoware.universe/localization/autoware_localization_error_monitor/`
- Diagnostic Aggregator: `autoware/src/universe/autoware.universe/system/system_diagnostic_monitor/`

---

**Last Updated**: 2025-12-28
**Tested On**: COSS map (NTU Campus), VLP-32C LiDAR, NDT localization
**Status**: Active - Accuracy check disabled to prevent false emergency stops
