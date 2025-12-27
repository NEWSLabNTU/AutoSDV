# MRM (Minimum Risk Maneuver) Troubleshooting Guide

This guide documents common causes of MRM triggers in AutoSDV and how to diagnose them.

## Overview

MRM (Minimum Risk Maneuver) is Autoware's safety mechanism that triggers emergency actions when the system detects hazardous conditions. When MRM activates, the vehicle will attempt to stop safely and hazard lights will be enabled.

### MRM States
- `NORMAL` - System operating normally
- `MRM_OPERATING` - MRM is actively executing
- `MRM_SUCCEEDED` - MRM completed successfully

## Common MRM Trigger: Localization Pose Timeout

### Symptoms
- MRM oscillates rapidly between `NORMAL` and `MRM_OPERATING` states
- Hazard lights flash on and off
- Log shows repeated messages:
  ```
  MRM State changed: NORMAL -> MRM_OPERATING
  MRM behavior is None. Do nothing.
  EMERGENCY_STOP is operated.
  MRM State changed: MRM_OPERATING -> NORMAL
  EMERGENCY_STOP is canceled.
  ```

### Root Cause

The EKF localization pose topic (`/localization/pose_twist_fusion_filter/pose`) intermittently times out. The topic_state_monitor detects this and sets ERROR status in diagnostics, triggering MRM.

### Evidence in Logs

Check the topic state monitor log:
```bash
cat play_log/latest/node/topic_state_monitor_pose_twist_fusion_filter_pose/err
```

Look for timeout warnings:
```
[WARN] /localization/pose_twist_fusion_filter/pose topic is timeout. Set ERROR in diagnostics.
```

### Cascade of Failures

When pose times out, it causes:
1. **Tracking errors grow too large**:
   ```
   [ERROR] Emergency Stop since the tracking error is too large
   ```
2. **MPC controller fails**:
   ```
   [ERROR] MPC failed due to getting MPC Data (too large yaw error)
   [ERROR] MPC failed due to getting MPC Data (too large position error)
   ```
3. **Planning validator detects invalid trajectory**:
   ```
   [ERROR] Caution! Invalid Trajectory published.
   [WARN] planning trajectory is too far from ego in longitudinal direction!!
   ```

### Possible Causes

1. **NDT Localization Instability**
   - NDT scan matching failing or taking too long
   - Poor pointcloud map quality
   - Insufficient LiDAR points for matching

2. **CPU/GPU Overload**
   - Heavy computational load causing EKF to miss publishing deadline
   - Check system resources during operation

3. **Transform Timing Issues**
   - Warnings like: `no transform found for no_ground_pointcloud: Lookup would require extrapolation into the past`
   - Indicates timestamp synchronization problems between sensors

4. **Sensor Data Delays**
   - LiDAR or IMU data arriving late
   - Network latency issues with sensors

### Diagnostic Commands

```bash
# Check MRM handler logs
cat play_log/latest/node/mrm_handler/err

# Check pose topic state
cat play_log/latest/node/topic_state_monitor_pose_twist_fusion_filter_pose/err

# Check control container for tracking errors
grep -i "tracking error\|yaw error\|position error" play_log/latest/node/control_container/err

# Check planning validator
grep -i "invalid\|too far" play_log/latest/node/planning_validator/err

# Check EKF localizer status
cat play_log/latest/node/autoware_ekf_localizer_node-*/err
```

### Solutions

1. **Increase Topic State Monitor Timeout**
   - If poses are slightly delayed but still valid
   - Modify timeout in system configuration

2. **Reduce System Load**
   - Disable non-essential perception components
   - Use `perception_mode:=lidar` to reduce camera processing

3. **Improve NDT Localization**
   - Ensure good initial pose estimate
   - Use higher quality pointcloud map
   - Check NDT score threshold settings

4. **Check Sensor Configuration**
   - Verify LiDAR timestamp synchronization
   - Ensure IMU data rate is sufficient
   - Check for network latency to sensors

## Common MRM Trigger: No Route/Trajectory

### Symptoms
- MRM triggers immediately when entering Auto mode
- Control command topics show continuous ERROR status

### Evidence in Logs

```bash
cat play_log/latest/node/topic_state_monitor_mission_planning_route/err
```

Shows:
```
/planning/mission_planning/route has not received. Set ERROR in diagnostics.
```

### Root Cause

Vehicle was switched to Auto mode without a valid route:
1. Initial pose not set, OR
2. Goal pose not set

### Solution

Before entering Auto mode:
1. Set initial pose using "2D Pose Estimate" in RViz
2. Wait for localization to converge (NDT activation succeeded)
3. Set goal using "2D Goal Pose" in RViz
4. Wait for route and trajectory to be computed
5. Then switch to Auto mode

## Common MRM Trigger: Control Command Timeout

### Symptoms
- `/control/command/control_cmd` topic not receiving messages
- `/control/trajectory_follower/control_cmd` not publishing

### Evidence in Logs

```bash
cat play_log/latest/node/topic_state_monitor_control_command_control_cmd/err
cat play_log/latest/node/topic_state_monitor_trajectory_follower_control_cmd/err
```

### Root Cause

The trajectory follower controller is not receiving valid input:
1. No trajectory from planning
2. Trajectory is too far from current position
3. Controller initialization not complete

### Solution

1. Ensure trajectory is being published:
   ```bash
   ros2 topic hz /planning/scenario_planning/trajectory
   ```
2. Check planning validator for trajectory issues
3. Verify localization is accurate

## Log Analysis Workflow

### Step 1: Check MRM Handler
```bash
cat play_log/latest/node/mrm_handler/err | grep -E "MRM State|EMERGENCY"
```

### Step 2: Check Topic State Monitors
```bash
for monitor in play_log/latest/node/topic_state_monitor_*/err; do
  echo "=== $(basename $(dirname $monitor)) ==="
  grep -E "timeout|ERROR" "$monitor" | tail -5
done
```

### Step 3: Check Control Pipeline
```bash
grep -ri "error\|ERROR\|fail" play_log/latest/node/control_container/err | tail -20
```

### Step 4: Check Planning Pipeline
```bash
grep -ri "invalid\|error\|ERROR" play_log/latest/node/planning_validator/err | tail -20
```

### Step 5: Check Localization
```bash
grep -ri "error\|warn" play_log/latest/node/autoware_ekf_localizer_node-*/err | tail -20
```

## Real-Time Monitoring

During operation, monitor these topics for issues:

```bash
# MRM state
ros2 topic echo /system/fail_safe/mrm_state

# Localization pose rate (should be ~50Hz)
ros2 topic hz /localization/pose_twist_fusion_filter/pose

# Trajectory rate
ros2 topic hz /planning/scenario_planning/trajectory

# Control command rate
ros2 topic hz /control/command/control_cmd

# Diagnostics aggregator
ros2 topic echo /diagnostics_agg
```

## References

- [Autoware MRM Handler Documentation](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/system/#mrm-handler)
- [Autoware Diagnostics System](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/system/system-diagnostics/)
