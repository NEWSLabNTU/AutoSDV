# Robin-W LiDAR Field of View Considerations for AutoSDV

## Robin-W Specifications

### Field of View (FOV)
- **Horizontal**: 120° (±60° from center)
- **Vertical**: 70° (±35° from center)
- **Total Coverage**: 120° × 70° solid-state scan

### Key Specifications
- **Points per second**: 1.6 million
- **Scan lines**: 192 lines
- **Angular resolution**: 0.15° × 0.36° (H × V)
- **Minimum distance**: 0.1m
- **Range**: Up to 200m (varies by target reflectivity)

### Comparison with 360° LiDARs

| Feature | Robin-W (Solid-State) | Velodyne VLP-32C (Spinning) |
|---------|----------------------|------------------------------|
| **Horizontal FOV** | 120° | 360° |
| **Coverage** | Forward-facing | Full surround |
| **Moving parts** | None | Rotating mechanism |
| **Reliability** | Higher | Lower (mechanical) |
| **Loop closure** | Limited | Excellent |
| **Best use** | Forward navigation | Full SLAM |

---

## Impact on Localization

### 1. Limited Loop Closure Detection

**Problem**: Loop closure requires recognizing previously visited locations. With 120° FOV:
- ❌ Cannot see behind (240° blind spot)
- ❌ Limited side coverage
- ❌ Difficult to detect when returning to same location from different angle

**Solution Options**:
1. **Multi-sensor fusion**: Add camera for visual loop closure
2. **Odometry reliance**: Use high-quality wheel encoders + IMU
3. **External landmarks**: AprilTags or reflective markers at key locations
4. **UWB beacons**: Provide absolute position independent of FOV

---

### 2. Scan Matching Limitations

**Problem**: Point cloud matching works best with overlapping scans. Limited FOV means:
- ❌ Poor matching during sharp turns (>60°)
- ❌ Reduced feature overlap in open spaces
- ❌ Drift accumulation without 360° reference

**Mitigation Strategies**:

#### A. Use Forward-Optimized SLAM
```yaml
# Example: KISS-ICP with higher odometry weight
kiss_icp:
  deskewing: true  # Important for solid-state
  max_range: 100.0
  voxel_size: 0.5
  initial_threshold: 2.0  # Larger for limited FOV
```

#### B. Tune NDT Parameters
```yaml
# Autoware NDT with limited FOV considerations
ndt_scan_matcher:
  trans_epsilon: 0.01
  step_size: 0.1
  resolution: 2.0
  max_iterations: 30

  # Critical for limited FOV:
  converged_param_transform_probability: 2.5  # Lower threshold
  initial_pose_timeout_sec: 1.5  # Longer timeout
```

#### C. Increase EKF Odometry Weight
```yaml
# robot_localization EKF configuration
ekf_filter_node:
  odom0: /wheel_odometry
  odom0_config: [true, true, false,  # x, y, z
                 false, false, true,  # roll, pitch, yaw
                 true, true, false,   # vx, vy, vz
                 false, false, true,  # vroll, vpitch, vyaw
                 false, false, false] # ax, ay, az
  odom0_queue_size: 10
  odom0_differential: false

  # Higher weight for odometry due to limited LiDAR FOV
  process_noise_covariance: [0.05, 0.0, ...]  # Lower = trust odometry more
```

---

### 3. Environment-Specific Challenges

#### Open Spaces (Large Rooms, Parking Lots)
**Challenge**: Fewer features in FOV, poor convergence
**Solutions**:
- Use visual SLAM (camera has wider effective FOV with features)
- Add UWB for absolute positioning
- Avoid pure LiDAR-only localization

#### Hallways and Corridors
**Challenge**: Symmetrical features, aliasing
**Solutions**:
- Robin-W works WELL here (forward-facing is ideal)
- Use ceiling/wall features if available
- Add unique landmarks at regular intervals

#### Intersections and Turns
**Challenge**: Limited overlap during turns, blind spots
**Solutions**:
- ⭐ **AprilTags at intersections** (highly recommended)
- Reduce speed during turns
- Use IMU gyro for turn estimation

#### Tunnels and GPS-Denied
**Challenge**: Repetitive structure, no GPS correction
**Solutions**:
- Install reflective markers at regular intervals
- Use LiDAR marker localization (see lidar_marker_localization.md)
- Consider UWB beacon system

---

## Recommended Sensor Fusion Architectures

### Architecture 1: Visual-LiDAR Fusion (⭐ Best)

```
                    ┌─────────────────┐
                    │  ZED Camera     │
                    │  (110° stereo)  │
                    └────────┬────────┘
                             │ Visual Features
                             ↓
┌─────────────────┐    ┌──────────────────┐
│  Robin-W LiDAR  │───→│   RTAB-Map       │
│  (120°×70°)     │    │   (RGB-D SLAM)   │
└─────────────────┘    └────────┬─────────┘
                                │ Pose + Covariance
                                ↓
        ┌──────────────────────────────────┐
        │   robot_localization (EKF)       │
        │   ← IMU (gyro, accel)            │
        │   ← Wheel Odometry               │
        └────────────┬─────────────────────┘
                     │ Fused Pose
                     ↓
            Planning & Control
```

**Benefits**:
- Visual features provide 360° awareness (camera can turn)
- Stereo depth + LiDAR depth fusion
- Loop closure from visual features
- Robust to LiDAR FOV limitations

**Configuration**:
```bash
# Launch RTAB-Map with ZED + Robin-W
ros2 launch rtabmap_ros rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start" \
  depth_topic:=/zed/depth/depth_registered \
  rgb_topic:=/zed/rgb/image_rect_color \
  camera_info_topic:=/zed/rgb/camera_info \
  approx_sync:=true \
  subscribe_scan_cloud:=true \
  scan_cloud_topic:=/robin_w/points
```

---

### Architecture 2: LiDAR + AprilTags (⭐ Practical)

```
┌─────────────────┐    ┌──────────────────┐
│  Robin-W LiDAR  │───→│   KISS-ICP       │
│  (120°×70°)     │    │   (Odometry)     │
└─────────────────┘    └────────┬─────────┘
                                │ Continuous Pose
┌─────────────────┐             ↓
│  ZED Camera     │    ┌──────────────────┐
│  (AprilTag det) │───→│   Pose Graph     │
└─────────────────┘    │   Optimization   │
                       └────────┬─────────┘
        ┌──────────────────────┴──────────┐
        │   robot_localization (EKF)      │
        │   ← IMU, Wheel Odometry         │
        └────────────┬────────────────────┘
                     │ Corrected Pose
                     ↓
            Planning & Control
```

**AprilTag Placement Strategy** (for 120° FOV):
1. **At turns/intersections**: Where LiDAR FOV is most limited
2. **Regular intervals**: Every 5-10 meters in hallways
3. **Key landmarks**: Doors, elevators, decision points
4. **Height**: 1.5-2.0m for camera visibility
5. **Orientation**: Face expected approach direction

**Benefits**:
- Low cost (printed markers)
- Corrects drift at critical locations
- Works with existing sensors
- Minimal computation overhead

---

### Architecture 3: LiDAR + UWB (Future)

```
┌─────────────┐         ┌──────────────────┐
│ UWB Beacons │────────→│  UWB Positioning │
│ (4+ anchors)│         │  (Trilateration) │
└─────────────┘         └────────┬─────────┘
                                 │ Absolute Position
┌─────────────────┐              ↓
│  Robin-W LiDAR  │    ┌──────────────────┐
│  (120°×70°)     │───→│  KISS-ICP        │
└─────────────────┘    │  (Relative)      │
                       └────────┬─────────┘
        ┌──────────────────────┴──────────┐
        │   robot_localization (EKF)      │
        │   ← IMU, Wheel Odometry         │
        └────────────┬────────────────────┘
                     │ Global Pose
                     ↓
            Planning & Control
```

**Benefits**:
- UWB provides absolute position (no drift)
- Independent of LiDAR FOV
- Through-wall capability
- High update rate (100 Hz)

**UWB Setup Requirements**:
- 4+ anchors for 3D positioning (6+ for redundancy)
- Anchor positions surveyed accurately
- Tag mounted on robot
- Coverage area planning

---

## FOV-Specific SLAM Algorithm Selection

### ✅ Works Well with 120° FOV

1. **KISS-ICP**
   - Point-to-point ICP adapts to available data
   - Good performance with partial scans
   - Recommended ⭐

2. **LOAM/LEGO-LOAM**
   - Feature-based, works with partial scans
   - Extracts edge and planar features
   - Good for structured environments

3. **RTAB-Map (with camera)**
   - Visual features compensate for limited LiDAR FOV
   - Best overall choice ⭐

### ⚠️ Challenging with 120° FOV

1. **Gmapping**
   - Designed for 2D 360° laser scans
   - Poor performance with limited FOV
   - Not recommended

2. **Hector SLAM**
   - Expects 360° coverage
   - Can work but with reduced accuracy
   - Use only in hallways

3. **NDT (Autoware default)**
   - Works but requires good odometry
   - Limited loop closure capability
   - Needs parameter tuning (see above)

### ❌ Not Suitable with 120° FOV

1. **Pure particle filter methods** (without good odometry)
   - Require full surround view
   - Fail with limited FOV

2. **Graph SLAM without loop closure aids**
   - Accumulates drift without 360° matching
   - Needs visual or marker-based loop closure

---

## Movement Pattern Recommendations

### Optimal Driving Patterns for 120° FOV

#### ✅ Good Patterns
1. **Forward motion in hallways**
   - Ideal use case for Robin-W
   - Maximize feature overlap

2. **Wide, slow turns**
   - Maintain feature overlap during rotation
   - Better scan matching convergence

3. **Regular pauses at landmarks**
   - Stop at AprilTags or known features
   - Allow localization update

#### ❌ Problematic Patterns
1. **Sharp turns (>90°)**
   - Lose feature overlap
   - Increased drift

2. **Reversing/backing up**
   - No rear coverage
   - Pure dead reckoning

3. **Spinning in place**
   - Minimal feature correspondence
   - Odometry-only localization

### Planning Integration

```python
# Example: Constrain path planner for 120° FOV LiDAR

# In Nav2 parameters
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

      # FOV-aware constraints
      cost_travel_multiplier: 2.0  # Prefer forward motion
      minimum_turning_radius: 1.5  # Wider turns
```

---

## Testing and Validation

### Recommended Tests for Limited FOV

#### Test 1: Straight Hallway
**Purpose**: Baseline performance
**Setup**: 50m straight corridor
**Expected**: <5cm drift

#### Test 2: 90° Turn
**Purpose**: Validate turn handling
**Setup**: L-shaped path, 90° corner
**Expected**: <20cm error after turn

#### Test 3: Loop Closure
**Purpose**: Test if loop detection works
**Setup**: Rectangular path, return to start
**Expected**:
- With visual: Loop detected, <10cm error
- Without visual: May not detect loop, >50cm drift

#### Test 4: Open Space
**Purpose**: Worst-case scenario
**Setup**: Large open area (10m × 10m)
**Expected**: Higher drift, rely on odometry/visual

### Metrics to Monitor

```bash
# View localization quality
ros2 topic echo /diagnostics | grep localization

# Check covariance (uncertainty)
ros2 topic echo /localization/pose_with_covariance | grep covariance

# Monitor transform delays
ros2 run tf2_ros tf2_monitor map base_link
```

**Red flags** (indicating FOV issues):
- Covariance increasing rapidly (>0.5m)
- Large jumps in pose estimates (>1m)
- High transform latency (>100ms)
- Frequent SLAM failures at turns

---

## Conclusion

The Robin-W's **120° × 70° FOV** is a significant constraint that requires:

1. **Multi-sensor fusion** (camera + LiDAR) for best results
2. **Strategic landmark placement** (AprilTags or reflective markers)
3. **Algorithm selection** (KISS-ICP or RTAB-Map, avoid Gmapping)
4. **Parameter tuning** (higher odometry weight in EKF)
5. **Movement patterns** (avoid sharp turns and reversing)

**Primary Recommendation for AutoSDV**:
- Implement **Visual-LiDAR Fusion** (RTAB-Map + KISS-ICP + robot_localization)
- Add **AprilTags at intersections** as backup
- Consider **UWB system** for large-scale deployment

The limited FOV is not a showstopper, but it does require thoughtful system design to achieve robust indoor localization.
