# LIO-SAM Point Cloud Mapping for AutoSDV

This guide explains how to create point cloud maps for AutoSDV using LIO-SAM (Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping).

## Overview

**LIO-SAM** is a real-time LiDAR-inertial odometry and mapping framework that:
- Fuses LiDAR, IMU, and optionally GNSS data
- Performs graph-based SLAM with loop closure
- Generates high-quality point cloud maps for Autoware localization
- Works well in outdoor environments with GNSS integration

## Prerequisites

1. **LIO-SAM installed** at `/home/aeon/repos/LIO-SAM/external/LIO-SLAM`
2. **ROS 2 Humble** workspace sourced
3. **Recorded rosbag** with sensor data (LiDAR + IMU + GNSS)
4. **AutoSDV sensors configured** (see sensor configuration guide)

## Quick Start

### 1. Record Mapping Data

Drive your vehicle slowly through the area you want to map while recording sensor data:

```bash
# Start AutoSDV sensors only (no localization/planning)
ros2 launch autosdv_launch logging_simulation.launch.yaml \
  sensor_suite:=robin_zed \
  launch_perception:=false \
  launch_planning:=false \
  launch_control:=false

# In another terminal, record rosbag
ros2 bag record \
  /sensing/lidar/concatenated/pointcloud \
  /sensing/imu/imu_data \
  /sensing/gnss/garmin/fix \
  /tf /tf_static \
  -o mapping_run
```

**Recording Tips:**
- Drive at **5-10 km/h** (slow and steady)
- Make **multiple passes** for better coverage
- Ensure **good loop closure** by returning to start point
- Record for **10-30 minutes** depending on area size
- Avoid rapid accelerations and sharp turns

### 2. Run LIO-SAM Mapping

#### Option A: Real-time mapping (while driving)

```bash
# Terminal 1: Launch LIO-SAM
ros2 launch autosdv_launch lio_sam_mapping.launch.xml \
  lidar_model:=robin-w \
  imu_source:=mpu9250 \
  use_gnss:=true \
  gnss_receiver:=garmin \
  save_pcd:=true

# Terminal 2: Launch sensor drivers
ros2 launch autosdv_launch logging_simulation.launch.yaml \
  sensor_suite:=robin_zed \
  launch_perception:=false \
  launch_planning:=false \
  launch_control:=false
```

#### Option B: Offline mapping (from rosbag)

```bash
# Terminal 1: Launch LIO-SAM (with use_sim_time)
ros2 launch autosdv_launch lio_sam_mapping.launch.xml \
  lidar_model:=robin-w \
  imu_source:=mpu9250 \
  use_gnss:=true \
  save_pcd:=true \
  --ros-args -p use_sim_time:=true

# Terminal 2: Play rosbag
ros2 bag play mapping_run/ --clock
```

### 3. Verify and Use the Map

After mapping completes, the PCD file will be saved:

```bash
# Default location
ls ~/Downloads/LOAM/*.pcd

# Copy to your map directory
mkdir -p data/my_new_map
cp ~/Downloads/LOAM/GlobalMap.pcd data/my_new_map/pointcloud_map.pcd

# View the map
pcl_viewer data/my_new_map/pointcloud_map.pcd
```

## Configuration

### Sensor Configurations

The launch file automatically configures parameters based on your sensor suite:

| Sensor Suite | LiDAR Model | Beams | Horizontal Res | Range |
|-------------|-------------|-------|----------------|-------|
| `robin_zed` | robin-w | 128 | 1024 | 200m |
| `vlp32c_zed` | vlp32c | 32 | 1800 | 150m |
| `cube1_usb` | cube1 | 64 | 1024 | 100m |

### Launch Arguments

```bash
ros2 launch autosdv_launch lio_sam_mapping.launch.xml \
  lidar_model:=robin-w         # LiDAR type: robin-w, vlp32c, cube1
  imu_source:=mpu9250          # IMU source: mpu9250, zed
  use_gnss:=true               # Enable GNSS fusion
  gnss_receiver:=garmin        # GNSS type: garmin, ublox, septentrio
  save_pcd:=true               # Save map to PCD file
  pcd_output_dir:=~/maps/      # Output directory
  rviz:=true                   # Launch RViz visualization
  params_file:=/path/to/custom_params.yaml  # Custom params (optional)
```

### Custom Parameters

To customize LIO-SAM parameters, copy and edit the config file:

```bash
# Copy default config
cp src/launcher/autosdv_launch/config/localization/lio_sam.param.yaml my_custom_params.yaml

# Edit parameters (see Parameter Tuning section below)
vim my_custom_params.yaml

# Use custom config
ros2 launch autosdv_launch lio_sam_mapping.launch.xml \
  params_file:=$(pwd)/my_custom_params.yaml
```

## Parameter Tuning

### LiDAR Parameters

Adjust these based on your LiDAR model:

```yaml
sensor: velodyne              # velodyne, ouster, livox
N_SCAN: 128                   # Number of scan lines (128 for Robin-W)
Horizon_SCAN: 1024            # Horizontal resolution
lidarMinRange: 0.5            # Minimum range (m)
lidarMaxRange: 200.0          # Maximum range (m)
downsampleRate: 1             # Downsample factor (increase if too many points)
```

### IMU Parameters

**Important:** These should be calibrated for your specific IMU!

```yaml
# MPU9250 (typical values)
imuAccNoise: 3.99e-03         # Accelerometer noise
imuGyrNoise: 1.56e-03         # Gyroscope noise
imuAccBiasN: 6.44e-05         # Accelerometer bias noise
imuGyrBiasN: 3.56e-05         # Gyroscope bias noise

# ZED IMU (typically higher noise)
imuAccNoise: 0.01
imuGyrNoise: 0.01
imuAccBiasN: 0.0001
imuGyrBiasN: 0.0001
```

To calibrate your IMU, use the Allan Variance method or run LIO-SAM and adjust until odometry is stable.

### Mapping Quality Parameters

```yaml
# Feature thresholds (tune based on environment)
edgeThreshold: 1.0            # Lower = more edge features
surfThreshold: 0.1            # Lower = more surface features

# Voxel filter sizes (larger = faster but less detail)
odometrySurfLeafSize: 0.4     # Outdoor: 0.4, Indoor: 0.2
mappingCornerLeafSize: 0.2    # Outdoor: 0.2, Indoor: 0.1
mappingSurfLeafSize: 0.4      # Outdoor: 0.4, Indoor: 0.2

# Keyframe parameters (affect map size and quality)
surroundingkeyframeAddingDistThreshold: 1.0   # Distance between keyframes (m)
surroundingkeyframeAddingAngleThreshold: 0.2  # Angular threshold (rad)
```

### Loop Closure

For large outdoor maps, loop closure is critical:

```yaml
loopClosureEnableFlag: true              # Enable loop closure
loopClosureFrequency: 1.0                # Check frequency (Hz)
historyKeyframeSearchRadius: 15.0        # Search radius (m)
historyKeyframeSearchTimeDiff: 30.0      # Time difference (s)
historyKeyframeFitnessScore: 0.3         # ICP threshold (lower = stricter)
```

### GNSS Integration

For outdoor mapping with GNSS:

```yaml
useImuHeadingInitialization: true        # Use GPS heading for initialization
useGpsElevation: true                    # Use GPS altitude (disable if poor vertical accuracy)
gpsCovThreshold: 2.0                     # GPS covariance threshold (m^2)
poseCovThreshold: 25.0                   # Pose covariance threshold (m^2)
```

## Topic Remapping

The launch file automatically remaps AutoSDV topics to LIO-SAM topics:

| LIO-SAM Topic | AutoSDV Topic | Description |
|---------------|---------------|-------------|
| `/points` | `/sensing/lidar/concatenated/pointcloud` | LiDAR point cloud |
| `/imu/data` | `/sensing/imu/imu_data` | IMU data (corrected) |
| `odometry/gpsz` | `/sensing/gnss/<receiver>/nav_sat_fix` | GNSS position |

## Published Topics

LIO-SAM publishes the following topics during mapping:

```bash
# Odometry and pose
/lio_sam/mapping/odometry                    # Current odometry
/lio_sam/mapping/odometry_incremental        # Incremental odometry

# Point clouds
/lio_sam/mapping/map_local                   # Local map around vehicle
/lio_sam/mapping/map_global                  # Full global map (published periodically)
/lio_sam/mapping/cloud_registered            # Registered point cloud

# Loop closure
/lio_sam/mapping/loop_closure_detection      # Loop closure events

# Path
/lio_sam/mapping/path                        # Trajectory path
```

## Troubleshooting

### Map Quality Issues

**Symptom:** Blurry or misaligned map

**Solutions:**
1. **Drive slower** (< 10 km/h) for better scan quality
2. **Increase loop closure frequency**: `loopClosureFrequency: 2.0`
3. **Tighter ICP threshold**: `historyKeyframeFitnessScore: 0.2`
4. **Reduce voxel sizes** for more detail (but slower processing)
5. **Calibrate IMU parameters** - incorrect noise params cause drift

### Odometry Drift

**Symptom:** Odometry diverges, map accumulates error

**Solutions:**
1. **Check IMU calibration** - most common cause
2. **Enable GNSS** if available: `use_gnss:=true`
3. **Increase feature thresholds** to extract more features
4. **Check IMU-LiDAR extrinsic calibration**

### GNSS Not Working

**Symptom:** GNSS data not fused, still drifting

**Solutions:**
1. **Check GNSS topic** is publishing: `ros2 topic echo /sensing/gnss/garmin/fix`
2. **Verify GNSS quality**: Check `fix.status.status` (should be >=1)
3. **Adjust covariance thresholds**:
   ```yaml
   gpsCovThreshold: 5.0      # Increase if GPS accuracy is poor
   useGpsElevation: false    # Disable if vertical accuracy is bad
   ```
4. **Enable heading initialization**: `useImuHeadingInitialization: true`

### Out of Memory / Slow Performance

**Symptom:** System runs out of memory or is very slow

**Solutions:**
1. **Downsample point cloud**: `downsampleRate: 2` (or higher)
2. **Increase voxel sizes**:
   ```yaml
   odometrySurfLeafSize: 0.6
   mappingSurfLeafSize: 0.6
   ```
3. **Reduce loop closure frequency**: `loopClosureFrequency: 0.5`
4. **Increase CPU cores**: `numberOfCores: 8` (if available)
5. **Reduce visualization radius**: `globalMapVisualizationSearchRadius: 500.0`

### RViz Not Showing Map

**Symptom:** RViz launches but no map visible

**Solutions:**
1. **Check fixed frame**: Set to `map` in RViz
2. **Add PointCloud2 display** and subscribe to `/lio_sam/mapping/map_global`
3. **Increase visualization radius** in params:
   ```yaml
   globalMapVisualizationSearchRadius: 1000.0
   ```
4. **Check topic publishing**: `ros2 topic hz /lio_sam/mapping/map_global`

## Best Practices

### For Outdoor Mapping
- ✅ Enable GNSS fusion
- ✅ Drive at 5-10 km/h
- ✅ Make multiple passes
- ✅ Return to start point for loop closure
- ✅ Use larger voxel sizes (0.4-0.6m)
- ✅ Enable loop closure

### For Indoor Mapping
- ❌ Disable GNSS (`use_gnss:=false`)
- ✅ Drive very slowly (< 5 km/h)
- ✅ Use smaller voxel sizes (0.2-0.3m)
- ✅ More frequent keyframes
- ✅ Ensure good feature-rich environment

### For Large Maps (> 1 km)
- ✅ Enable GNSS
- ✅ Enable loop closure
- ✅ Increase loop search radius: `historyKeyframeSearchRadius: 20.0`
- ✅ More CPU cores: `numberOfCores: 8`
- ✅ Consider splitting into multiple smaller maps

## Advanced Usage

### Multi-Session Mapping

To extend an existing map:

```bash
# TODO: LIO-SAM doesn't natively support this
# Consider using map stitching tools or SLAM Toolbox instead
```

### Map Post-Processing

After generating the map, you may want to:

1. **Filter outliers**:
   ```bash
   pcl_outlier_removal GlobalMap.pcd filtered_map.pcd
   ```

2. **Downsample for Autoware**:
   ```bash
   pcl_voxel_grid GlobalMap.pcd downsampled_map.pcd -leaf 0.2,0.2,0.2
   ```

3. **Convert to multiple tiles** (for very large maps):
   ```bash
   # Use Autoware map tools
   ros2 run map_tools pcd_tile_generator \
     --input GlobalMap.pcd \
     --output tiles/ \
     --tile_size 100
   ```

## Using the Map with Autoware

Once you have the PCD map:

```bash
# 1. Copy to your map directory
mkdir -p data/my_map
cp ~/Downloads/LOAM/GlobalMap.pcd data/my_map/pointcloud_map.pcd

# 2. Create a lanelet2 map (for planning)
# Use Vector Map Builder: https://tools.tier4.jp/

# 3. Launch Autoware with your map
make launch ARGS="map_path:=$(pwd)/data/my_map"
```

## References

- [LIO-SAM Paper](https://github.com/TixiaoShan/LIO-SAM)
- [AutoSDV Sensor Configuration](sensor_configuration.md)
- [Autoware Map Creation Guide](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-maps/)
