# Phase 5: EKF Fusion Configuration

**Status:** ⏸️ Pending
**Duration:** 2-3 days
**Dependencies:** Phase 4

[← Previous: VSLAM Modification](phase_4_vslam.md) | [Back to Overview](README.md) | [Next: Testing →](phase_6_testing.md)

---

## Objectives

Configure EKF Localizer to fuse AR tag pose (global reference) with Isaac VSLAM twist (local odometry) for drift-free localization.

---

## Tasks

### Task 5.1: Configure EKF Localizer for Multi-Source Fusion

**Description:**
Configure EKF Localizer parameters to properly fuse global pose from AR tags with local twist from Isaac VSLAM.

**File:** `src/launcher/autoware_launch/autoware_launch/config/localization/ekf_localizer/ekf_localizer.param.yaml`

**Key Configuration:**

```yaml
/**:
  ros__parameters:
    # Pose measurement from AR tags (global reference)
    pose_frame_id: "map"

    # Enable pose and twist measurements
    pose_additional_delay: 0.0
    twist_additional_delay: 0.0

    # Process noise (system dynamics uncertainty)
    proc_stddev_vx_c: 5.0       # Longitudinal velocity
    proc_stddev_wz_c: 1.0       # Yaw rate
    proc_stddev_yaw_c: 0.005    # Yaw angle drift

    # Measurement smoothing (prevent sudden jumps)
    pose_smoothing_steps: 5     # Smooth AR tag corrections over 5 steps
    twist_smoothing_steps: 2    # Smooth twist over 2 steps

    # Measurement gates (reject outliers)
    pose_gate_dist: 10000.0     # Large value (trust AR tags)
    twist_gate_dist: 10000.0    # Large value (trust Isaac VSLAM)

    # Automatic yaw bias estimation
    enable_yaw_bias_estimation: true
    extend_state_step: 50

    # Debug output
    show_debug_info: true
    publish_tf: true
```

**Topic Subscriptions:**

The EKF automatically subscribes to:
```
/localization/pose_estimator/pose_with_covariance     # AR tag pose
/localization/twist_estimator/twist_with_covariance   # Isaac VSLAM twist
```

**Covariance Tuning Guide:**

**AR Tag Pose Covariance** (from AR tag localizer config):
- Close range (<5m): Low uncertainty (0.2m standard deviation)
- Far range (>5m): Higher uncertainty (scales cubically with distance)
- Position (x,y,z): 0.2m² base covariance
- Orientation (roll, pitch, yaw): 0.02 rad² base covariance

**Isaac VSLAM Twist Covariance** (from odometry bridge):
- Linear velocity: 0.1 m/s standard deviation
- Angular velocity: 0.05 rad/s standard deviation

**Process Noise Tuning:**
- `proc_stddev_vx_c`: How much velocity can change per time step
  - Higher value → EKF responds faster to measurements
  - Lower value → Smoother but slower response
- `proc_stddev_yaw_c`: Expected yaw drift rate
  - Lower value → Trust predicted heading more
  - Higher value → Accept larger heading corrections

**Acceptance Criteria:**
- [ ] EKF parameters configured for AR tag + VSLAM fusion
- [ ] Covariances tuned for sensor characteristics
- [ ] Configuration documented with rationale

---

### Task 5.2: Test EKF Fusion Behavior

**Description:**
Test EKF fusion with both AR tag and VSLAM inputs to verify correct behavior and stability.

**Test Scenarios:**

**Test 1: AR Tag Correction**

```bash
# Launch system with AR tag + VSLAM fusion
ros2 launch autosdv_launch autosdv.launch.yaml \
  pose_source:=artag \
  map_path:=./data/ar_tag_test_map \
  use_gnss:=false

# Monitor topics
ros2 topic echo /localization/pose_estimator/pose_with_covariance  # AR tag pose
ros2 topic echo /localization/twist_estimator/twist_with_covariance  # VSLAM twist
ros2 topic echo /localization/pose_with_covariance  # EKF fused output

# Expected: When AR tag detected, pose should converge to corrected value
```

**Test 2: Smooth Tracking Between Tags**

```bash
# Move vehicle between AR tag detections
# Monitor EKF output continuity

# Expected: Smooth interpolation using Isaac VSLAM twist
# No jumps or discontinuities in pose
```

**Test 3: No Tag Scenario**

```bash
# Move to area without visible AR tags
# Monitor drift accumulation

# Expected:
# - EKF continues using last known pose + VSLAM twist integration
# - Gradual drift accumulation (document characteristics)
```

**Test 4: Tag Re-Detection**

```bash
# Move away from tag, return to same tag
# Compare poses on re-detection

# Expected: Pose should snap back to correct value (<20cm difference)
```

**Monitoring Commands:**

```bash
# View TF tree
ros2 run tf2_tools view_frames.py

# Echo map→base_link transform
ros2 run tf2_ros tf2_echo map base_link

# Monitor EKF diagnostics
ros2 topic echo /diagnostics | grep -A 10 "ekf_localizer"

# Check fusion weights
ros2 topic echo /localization/ekf_localizer/ekf_twist_with_covariance --field covariance
```

**Acceptance Criteria:**
- [ ] AR tag corrections applied to EKF
- [ ] Smooth tracking between tag detections
- [ ] No filter instabilities or crashes
- [ ] Drift characteristics documented

---

## Deliverables

- [ ] Configured EKF parameters (`ekf_localizer.param.yaml`)
- [ ] Tuned covariance values
- [ ] Fusion test results documented
- [ ] Known limitations and drift characteristics documented

---

## Testing

**Validation Checklist:**

- [ ] EKF receives pose from AR tags (~10 Hz when visible)
- [ ] EKF receives twist from Isaac VSLAM (~30 Hz continuous)
- [ ] Fused pose published on `/localization/pose_with_covariance`
- [ ] TF `map → base_link` published correctly
- [ ] No divergence or instabilities during 5+ minutes of operation

**Performance Metrics:**

- **Pose update rate:** >10 Hz
- **Position accuracy with tag visible:** <0.5m
- **Drift rate without tags:** Document (e.g., <1m per 10 meters traveled)
- **Correction time on tag re-detection:** <1 second

---

## Next Phase

Once EKF fusion is stable and validated, proceed to [Phase 6: Integration Testing](phase_6_testing.md) for comprehensive system testing.
