# LiDAR Marker Localization for AutoSDV

## Overview

LiDAR Marker Localization is a reflector-based localization method that uses high-intensity reflective markers detected by LiDAR to provide accurate position estimation. This method is particularly useful in GPS-denied environments such as tunnels, parking garages, and indoor facilities.

**Key Features**:
- No point cloud map required (only vector map with marker positions)
- Works in GPS-denied environments
- Uses LiDAR intensity data to detect retroreflective markers
- Provides accurate localization when markers are visible

**Best Use Cases**:
- Tunnels and underground facilities
- Indoor parking garages
- Areas with poor GNSS reception
- Controlled environments with installed infrastructure

## Physical Marker Specifications

### Size and Dimensions

| Parameter                | Default Value    | Description                          |
|--------------------------|------------------|--------------------------------------|
| **Marker Width**         | 0.8 meters       | Physical width of reflective marker  |
| **Height from Ground**   | 1.075 meters     | Center height of marker installation |
| **Shape**                | Flat rectangular | Defined as 4-vertex polygon          |
| **Detection Resolution** | 5 cm (0.05m)     | Grid size for pattern matching       |
| **Pattern Width**        | ~0.55 meters     | 11 grid cells × 0.05m                |

### Reflection and Intensity Requirements

#### Material Specifications
- **Type**: High-intensity retroreflective surface
- **Examples**:
  - 3M Diamond Grade reflective sheeting
  - Retroreflective tape (similar to road signs)
  - Reflective panels with intensity ratio > 20:1

#### Intensity Pattern Detection
The detection algorithm uses intensity pattern matching:

```
Default Pattern: [-1, -1, 0, 1, 1, 1, 1, 1, 0, -1, -1]

Where:
  1  = High intensity (positive match) - Reflective area
  0  = Don't consider (neutral)
 -1  = Low intensity (negative match) - Non-reflective area
```

**Pattern Layout** (viewed from LiDAR):
```
┌─────────────────────────────────┐
│  Low  │ Low │ X │ High Intensity│ X │ Low │ Low  │
│ (bg)  │(bg) │   │  (reflector)  │   │(bg) │ (bg) │
└─────────────────────────────────┘
  ←─────────── ~0.55m ───────────→
```

#### Detection Thresholds
| Parameter | Default Value | Purpose |
|-----------|---------------|---------|
| **Intensity Difference** | 20 units | Minimum difference between high/low areas |
| **Positive Match Threshold** | 3 matches | Required high-intensity matches |
| **Negative Match Threshold** | 3 matches | Required low-intensity matches |
| **Ring Vote Threshold** | 20 rings | LiDAR rings that must detect pattern |

### Physical Installation Guidelines

**Installation Requirements**:
1. **Height**: Install at consistent height (default 1.075m from ground level)
2. **Orientation**: Face markers perpendicular to expected vehicle trajectory
3. **Spacing**: Ensure markers are within 2m detection range when needed
4. **Visibility**: Clear line of sight from vehicle LiDAR position
5. **Environment**: Protect from dirt/debris that could reduce reflectivity

**Marker Placement Strategy**:
- Install at decision points (intersections, lane changes)
- Space markers to ensure continuous coverage in critical areas
- Avoid placing markers near highly reflective objects (mirrors, metal surfaces)
- Consider LiDAR vertical field of view when determining height

## Hardware Requirements

### LiDAR Specifications

**Required Features**:
- ✅ **Intensity channel**: Must provide intensity data per point
- ✅ **Ring-based scanning**: Multi-ring scanning pattern (e.g., 32, 64, 128 rings)
- ✅ **Range**: Sufficient to detect markers within 2-10 meters
- ✅ **Resolution**: Adequate point density to capture pattern

**Compatible LiDAR Models** (verified):
- Velodyne VLP-32C ✓
- Velodyne HDL-64E ✓
- Seyond Robin-W ✓ (AutoSDV default)
- Other 3D LiDARs with intensity output

**AutoSDV LiDAR Configuration**:
```yaml
# Robin-W LiDAR provides:
- Point format: PointXYZIRC (includes intensity)
- Ring information: Required for pattern matching
- Intensity range: Sufficient for reflector detection
```

### Map Infrastructure

**Physical Infrastructure**:
- Retroreflective markers installed at known locations
- Markers accessible to LiDAR scan area
- Maintenance plan to keep markers clean and functional

**Digital Infrastructure**:
- Lanelet2 vector map with precise marker positions
- Survey-grade accuracy for marker coordinates (cm-level preferred)

### Supporting Sensors (for Sensor Fusion)

| Sensor | Purpose | Required? |
|--------|---------|-----------|
| **IMU** | EKF fusion, orientation estimation | Yes |
| **Wheel Odometry** | Speed estimation, dead reckoning | Recommended |
| **GNSS** | Initial pose, outdoor transitions | Optional |

## Software Requirements

### Autoware Packages

**Core Packages** (already included in AutoSDV):
```
autoware_lidar_marker_localizer       # Main detection node
autoware_landmark_based_localizer     # Landmark management
autoware_ekf_localizer                # Sensor fusion
autoware_pointcloud_preprocessor      # Point cloud filtering
```

### Lanelet2 Map Format

Markers must be defined in the Lanelet2 map following this specification:

#### Marker Definition Example

```xml
<?xml version='1.0' encoding='UTF-8'?>
<osm version='0.6' generator='JOSM'>

  <!-- Define 4 vertices for the marker (counter-clockwise) -->
  <node id='1001' lat='35.8xxxxx' lon='139.6xxxxx'>
    <tag k='ele' v='1.075'/>  <!-- Height from ground -->
    <tag k='local_x' v='10.0'/>
    <tag k='local_y' v='5.0'/>
  </node>
  <node id='1002' lat='35.8xxxxx' lon='139.6xxxxx'>
    <tag k='ele' v='1.075'/>
    <tag k='local_x' v='10.8'/>  <!-- 0.8m width -->
    <tag k='local_y' v='5.0'/>
  </node>
  <node id='1003' lat='35.8xxxxx' lon='139.6xxxxx'>
    <tag k='ele' v='2.075'/>  <!-- 1m height -->
    <tag k='local_x' v='10.8'/>
    <tag k='local_y' v='5.0'/>
  </node>
  <node id='1004' lat='35.8xxxxx' lon='139.6xxxxx'>
    <tag k='ele' v='2.075'/>
    <tag k='local_x' v='10.0'/>
    <tag k='local_y' v='5.0'/>
  </node>

  <!-- Create polygon for the marker -->
  <way id='2001'>
    <nd ref='1001'/>
    <nd ref='1002'/>
    <nd ref='1003'/>
    <nd ref='1004'/>
    <nd ref='1001'/>  <!-- Close the polygon -->
    <tag k='type' v='pose_marker'/>
    <tag k='subtype' v='reflector'/>
    <tag k='marker_id' v='marker_001'/>
    <tag k='area' v='yes'/>
  </way>

</osm>
```

#### Vertex Ordering Requirements

**IMPORTANT**: Vertices must be defined in **counter-clockwise** order:

```
Marker viewed from LiDAR side:

    4 ──────────── 3
    │              │
    │   Reflector  │  ↑ Z (height)
    │              │  │
    1 ──────────── 2  │
                      └──→ X (width)

Vertex order: 1 → 2 → 3 → 4 → 1
```

**Coordinate System**:
- X-axis: Parallel to vector from vertex 1 to vertex 2
- Y-axis: Parallel to vector from vertex 2 to vertex 3
- Z-axis: Perpendicular to marker surface

### Configuration Files

The following configuration files are required (paths relative to Autoware launch):

**Main Configuration**:
```
config/localization/lidar_marker_localizer/
  ├── lidar_marker_localizer.param.yaml
  └── pointcloud_preprocessor/
      ├── crop_box_filter_measurement_range.param.yaml
      └── ring_filter.param.yaml
```

## Enabling LiDAR Marker Localization in AutoSDV

### Method 1: Launch Argument (Recommended)

The simplest way to enable LiDAR marker localization:

```bash
# Enable lidar-marker localization
make launch ARGS="pose_source:=lidar-marker"

# Combine with other configuration
make launch ARGS="pose_source:=lidar-marker lidar_model:=robin-w"

# Indoor operation without GNSS
make launch ARGS="pose_source:=lidar-marker use_gnss:=false"
```

### Method 2: Modify Launch File

Edit `src/launcher/autosdv_launch/launch/autosdv.launch.yaml`:

```yaml
-
  name: pose_source
  value: lidar-marker  # Changed from 'ndt'
```

Then launch normally:
```bash
make launch
```

### Method 3: Multi-Localizer Mode (Advanced)

Combine multiple localization methods for redundancy:

```bash
# Use both NDT and lidar-marker
make launch ARGS="pose_source:=ndt_lidar-marker"

# NDT + LiDAR marker + YabLoc (requires camera)
make launch ARGS="pose_source:=ndt_lidar-marker_yabloc"
```

**Note**: When multiple pose sources are used, the `pose_estimator_arbiter` automatically selects the most reliable source.

## Configuration and Tuning

### Parameter File Location

Create or edit the configuration file:
```
src/param/autoware_individual_params/individual_params/config/default/autosdv_sensor_kit/lidar_marker_localizer.param.yaml
```

### Key Parameters

```yaml
/**:
  ros__parameters:
    # Marker identification
    marker_name: "reflector"  # Must match Lanelet2 map subtype

    # Detection algorithm parameters
    resolution: 0.05  # Grid size for pattern matching [m]

    # Intensity pattern: [-1=low, 0=ignore, 1=high]
    intensity_pattern: [-1, -1, 0, 1, 1, 1, 1, 1, 0, -1, -1]

    # Thresholds
    match_intensity_difference_threshold: 20  # Min intensity difference
    positive_match_num_threshold: 3           # Required high matches
    negative_match_num_threshold: 3           # Required low matches
    vote_threshold_for_detect_marker: 20      # Min LiDAR rings

    # Physical marker dimensions
    marker_height_from_ground: 1.075  # Center height [m]
    marker_width: 0.8                 # Physical width [m]

    # Detection range limits
    limit_distance_from_self_pose_to_nearest_marker: 2.0  # [m]
    limit_distance_from_self_pose_to_marker: 2.0          # [m]

    # Pose interpolation
    self_pose_timeout_sec: 1.0
    self_pose_distance_tolerance_m: 1.0

    # Output covariance (6x6 matrix: x, y, z, roll, pitch, yaw)
    base_covariance: [
      0.04, 0.0,  0.0,  0.0,        0.0,        0.0,
      0.0,  0.04, 0.0,  0.0,        0.0,        0.0,
      0.0,  0.0,  0.01, 0.0,        0.0,        0.0,
      0.0,  0.0,  0.0,  0.00007569, 0.0,        0.0,
      0.0,  0.0,  0.0,  0.0,        0.00007569, 0.0,
      0.0,  0.0,  0.0,  0.0,        0.0,        0.00030625
    ]
```

### Tuning Guide

#### If markers are not detected:

1. **Reduce vote threshold**:
   ```yaml
   vote_threshold_for_detect_marker: 15  # From default 20
   ```

2. **Adjust intensity threshold**:
   ```yaml
   match_intensity_difference_threshold: 15  # From default 20
   ```

3. **Increase detection range**:
   ```yaml
   limit_distance_from_self_pose_to_marker: 3.0  # From default 2.0
   ```

#### If false positives occur:

1. **Increase vote threshold**:
   ```yaml
   vote_threshold_for_detect_marker: 25  # More strict
   ```

2. **Require more pattern matches**:
   ```yaml
   positive_match_num_threshold: 4  # From default 3
   negative_match_num_threshold: 4  # From default 3
   ```

#### For different marker sizes:

```yaml
marker_width: 1.0              # For 1m wide markers
marker_height_from_ground: 1.5 # For markers at 1.5m height

# Adjust pattern if needed (wider marker = more high-intensity cells)
intensity_pattern: [-1, -1, 0, 1, 1, 1, 1, 1, 1, 1, 0, -1, -1]
```

## System Integration

### Architecture Overview

```
┌─────────────────┐
│  LiDAR (Robin-W)│
└────────┬────────┘
         │ PointCloud2 (XYZIRC)
         ↓
┌─────────────────────────────┐
│  Pointcloud Preprocessor     │
│  - Crop Box Filter           │
│  - Ring Filter               │
└────────┬────────────────────┘
         │ Filtered PointCloud
         ↓
┌─────────────────────────────┐
│  LiDAR Marker Localizer     │
│  - Detect intensity pattern │
│  - Match with map markers   │
│  - Calculate vehicle pose   │
└────────┬────────────────────┘
         │ PoseWithCovariance
         ↓
┌─────────────────────────────┐
│  EKF Localizer              │◄── IMU, Wheel Odometry
│  - Fuse multiple sources    │
│  - Smooth estimation        │
└────────┬────────────────────┘
         │ Final Pose
         ↓
    Planning & Control
```

### Topic Flow

**Inputs**:
- `/sensing/lidar/concatenated/pointcloud` → LiDAR point cloud
- `/map/vector_map` → Lanelet2 map with markers
- `/localization/pose_twist_fusion_filter/biased_pose_with_covariance` → EKF pose

**Outputs**:
- `/localization/pose_estimator/pose_with_covariance` → Estimated pose
- `/localization/pose_estimator/lidar_marker_localizer/debug/marker_detected` → Debug: detected markers
- `/localization/pose_estimator/lidar_marker_localizer/debug/marker_pointcloud` → Debug: marker points
- `/diagnostics` → System diagnostics

## Monitoring and Debugging

### ROS 2 Topics for Debugging

```bash
# View detected markers (poses)
ros2 topic echo /localization/pose_estimator/lidar_marker_localizer/debug/marker_detected

# View marker point clouds
ros2 topic echo /localization/pose_estimator/lidar_marker_localizer/debug/marker_pointcloud

# Check diagnostics
ros2 topic echo /diagnostics

# View estimated pose
ros2 topic echo /localization/pose_estimator/pose_with_covariance
```

### Visualization in RViz

Add these topics to RViz for visual debugging:

1. **Marker Poses** (`/localization/pose_estimator/lidar_marker_localizer/debug/marker_detected`)
   - Type: PoseArray
   - Shows detected marker positions

2. **Marker Point Cloud** (`/localization/pose_estimator/lidar_marker_localizer/debug/marker_pointcloud`)
   - Type: PointCloud2
   - Shows LiDAR points classified as markers

3. **Mapped Markers** (`/localization/pose_estimator/lidar_marker_localizer/debug/marker_mapped`)
   - Type: MarkerArray
   - Shows marker positions from Lanelet2 map

4. **Estimated Pose** (`/localization/pose_estimator/pose_with_covariance`)
   - Type: PoseWithCovarianceStamped
   - Shows current vehicle position estimate

### Diagnostic Checks

**System Status**:
```bash
# Check if lidar_marker_localizer is running
ros2 node list | grep lidar_marker

# View node info
ros2 node info /localization/pose_estimator/lidar_marker_localizer/lidar_marker_localizer
```

**Common Issues**:

| Issue | Diagnostic | Solution |
|-------|------------|----------|
| No markers detected | Check debug topics, verify LiDAR intensity data | Verify marker reflectivity, adjust thresholds |
| Intermittent detection | Check marker_detected topic frequency | Clean markers, check LiDAR alignment |
| Poor localization accuracy | Check marker_mapped vs detected alignment | Re-survey marker positions in map |
| High covariance values | Check pose_with_covariance values | Increase marker density, check EKF parameters |

### Enable Logging (Optional)

For detailed intensity analysis:

```yaml
/**:
  ros__parameters:
    enable_save_log: true
    save_file_directory_path: /home/jetson/reflector_logs
    save_file_name: detected_reflector_intensity
    save_frame_id: velodyne_top  # Or robin_w_frame
```

This saves detected intensity data for offline analysis.

## Deployment Workflow

### Step 1: Install Physical Markers

1. Purchase retroreflective panels (e.g., 0.8m × 1.0m, 3M Diamond Grade)
2. Mount at consistent height (e.g., 1.075m from ground)
3. Ensure perpendicular orientation to expected vehicle path
4. Verify visibility from vehicle LiDAR height

### Step 2: Survey Marker Positions

1. Use RTK-GNSS or total station to measure marker positions
2. Record precise 3D coordinates (cm-level accuracy)
3. Note marker orientation and dimensions
4. Assign unique IDs to each marker

### Step 3: Create Lanelet2 Map

1. Edit `data/COSS-map-planning/lanelet2_map.osm` (or create new map)
2. Add marker definitions using surveyed coordinates
3. Validate map using `autoware_lanelet2_validation`
4. Test map loading in AutoSDV

### Step 4: Configure and Test

1. Configure `lidar_marker_localizer.param.yaml` with marker dimensions
2. Launch AutoSDV with `pose_source:=lidar-marker`
3. Drive to marker location and verify detection
4. Check debug topics in RViz
5. Tune parameters as needed

### Step 5: Integration Testing

1. Test localization accuracy at each marker
2. Verify smooth transitions between markers
3. Test sensor fusion with EKF
4. Validate in target environment (tunnel, garage, etc.)

## Performance Considerations

### Detection Range
- **Optimal**: 1-5 meters from marker
- **Maximum**: ~10 meters (depends on marker size and LiDAR)
- **Minimum**: 0.5 meters (too close may miss pattern)

### Update Rate
- **Detection rate**: Depends on LiDAR frequency (10-20 Hz typical)
- **Localization update**: Matches LiDAR scan rate
- **EKF fusion**: 50-100 Hz (fused with IMU)

### Computational Load
- **CPU usage**: Low-moderate (pattern matching is efficient)
- **Memory**: Minimal (only processes local point cloud)
- **Suitable for**: Embedded platforms (Jetson, etc.)

## Limitations

1. **Line of Sight**: Requires clear view of markers from LiDAR
2. **Marker Maintenance**: Reflectivity degrades with dirt/damage
3. **Infrastructure Dependency**: Requires pre-installed markers
4. **Limited Coverage**: Only works in areas with markers
5. **No Orientation Correction**: Primarily corrects position (x, y, z)
6. **Detection Range**: Limited to ~2-10m depending on configuration

## Best Practices

### Marker Installation
- ✅ Install at decision points (turns, intersections)
- ✅ Maintain consistent height across installation
- ✅ Use high-quality retroreflective materials
- ✅ Schedule regular cleaning and inspection
- ✅ Install redundant markers in critical areas

### Map Creation
- ✅ Survey markers with RTK-GNSS or total station
- ✅ Verify coordinates before final map deployment
- ✅ Use descriptive marker IDs (e.g., "tunnel_entrance_01")
- ✅ Document marker locations in separate reference file

### System Configuration
- ✅ Start with default parameters, tune incrementally
- ✅ Test in actual environment before deployment
- ✅ Monitor diagnostics regularly
- ✅ Use multi-localizer mode for redundancy in transitions

## References

### Sample Dataset
- **Location**: National Institute for Land and Infrastructure Management, Full-scale tunnel experiment facility
- **Download**: [Sample rosbag and map](https://drive.google.com/file/d/1FuGKbkWrvL_iKmtb45PO9SZl1vAaJFVG/view?usp=sharing)
- **Installation**: Taisei Corporation

### Documentation Links
- [Autoware LiDAR Marker Localizer](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/autoware_landmark_based_localizer/autoware_lidar_marker_localizer)
- [Lanelet2 Format Extension](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/docs/lanelet2_format_extension.md#localization-landmarks)
- [AutoSDV Repository](https://github.com/aeon-labs/AutoSDV)

### Contributors
- TIER IV
- Taisei Corporation
- Yuri Shimizu

## Appendix: Quick Reference

### Launch Commands

```bash
# Basic lidar-marker mode
make launch ARGS="pose_source:=lidar-marker"

# Indoor without GNSS
make launch ARGS="pose_source:=lidar-marker use_gnss:=false"

# Combined with NDT
make launch ARGS="pose_source:=ndt_lidar-marker"

# With specific LiDAR model
make launch ARGS="pose_source:=lidar-marker lidar_model:=robin-w"
```

### Key Parameters Quick Reference

| Parameter | Default | Adjust For |
|-----------|---------|------------|
| `marker_width` | 0.8m | Different marker sizes |
| `marker_height_from_ground` | 1.075m | Different installation heights |
| `vote_threshold_for_detect_marker` | 20 | Detection sensitivity |
| `match_intensity_difference_threshold` | 20 | Intensity sensitivity |
| `limit_distance_from_self_pose_to_marker` | 2.0m | Detection range |

### Marker Specification Summary

- **Size**: 0.8m × 1.0m (default)
- **Material**: Retroreflective (3M Diamond Grade or equivalent)
- **Height**: 1.075m from ground (center)
- **Intensity ratio**: Minimum 20:1 (reflective:background)
- **Pattern**: High-intensity center with low-intensity borders
