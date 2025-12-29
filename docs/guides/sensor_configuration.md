# Sensor Configuration Guide

This guide covers sensor configuration options for AutoSDV.

## Sensor Suites (Recommended)

Predefined sensor suites provide convenient configurations for common hardware combinations:

```bash
# Robin-W LiDAR + ZED X Mini + ZED IMU (integrated)
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=robin_zed

# Robin-W LiDAR + ZED X Mini + MPU9250 IMU
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=robin_zed_mpu

# Velodyne VLP-32C + ZED X Mini + ZED IMU (integrated)
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed

# Velodyne VLP-32C + ZED X Mini + MPU9250 IMU
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed_mpu

# Velodyne VLP-32C + ZED IMU only (no camera streams, lightweight)
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed_imu

# Blickfeld Cube1 + USB Cameras + MPU9250 IMU
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=cube1_usb

# Custom configuration (use individual parameters)
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=custom lidar_model:=robin-w camera_model:=zedxm imu_source:=mpu9250
```

### Suite Configuration Matrix

| Suite | LiDAR | Camera | IMU | GNSS | ZED OD |
|-------|-------|--------|-----|------|--------|
| robin_zed | Robin-W | ZED X Mini | ZED built-in | u-blox | Yes |
| robin_zed_mpu | Robin-W | ZED X Mini | MPU9250 | u-blox | Yes |
| vlp32c_zed | Velodyne 32C | ZED X Mini | ZED built-in | u-blox | Yes |
| vlp32c_zed_mpu | Velodyne 32C | ZED X Mini | MPU9250 | u-blox | Yes |
| vlp32c_zed_imu | Velodyne 32C | None | ZED built-in | u-blox | No |
| cube1_usb | Cube1 | USB cameras | MPU9250 | u-blox | No |
| custom | (manual) | (manual) | (manual) | (manual) | (manual) |

### Overriding Suite Defaults

```bash
# Use robin_zed suite but disable GNSS for indoor testing
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=robin_zed use_gnss:=false

# Use vlp32c_zed suite but switch to Septentrio GNSS
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed gnss_receiver:=septentrio
```

## Individual Sensor Parameters

When using `sensor_suite:=custom` (default), configure sensors individually:

### IMU Sources

```bash
# MPU9250 external I2C IMU (default for custom suite)
ros2 launch autosdv_launch autosdv.launch.yaml imu_source:=mpu9250

# ZED camera built-in IMU (factory-calibrated, more stable)
ros2 launch autosdv_launch autosdv.launch.yaml imu_source:=zed camera_model:=zedxm
```

**Note:** When using ZED IMU (`imu_source:=zed`), the ZED camera must be used (`camera_model:=zedxm`). The system automatically relays IMU data from the camera node to avoid launching duplicate ZED drivers.

### LiDAR Models

```bash
make launch ARGS="lidar_model:=robin-w"   # Robin-W Solid-State (default)
make launch ARGS="lidar_model:=vlp32c"    # Velodyne VLP-32C
make launch ARGS="lidar_model:=cube1"     # Blickfeld Cube1
```

### Camera Models

```bash
make launch ARGS="camera_model:=zedxm"    # ZED stereo camera (default)
make launch ARGS="camera_model:=usb"      # USB cameras
make launch ARGS="camera_model:=none"     # No camera
```

### GNSS Receivers

```bash
make launch ARGS="gnss_receiver:=garmin"      # Garmin (default)
make launch ARGS="gnss_receiver:=ublox"       # u-blox
make launch ARGS="gnss_receiver:=septentrio"  # Septentrio
```

## NTRIP/RTK Configuration (u-blox only)

AutoSDV supports RTK positioning for centimeter-level accuracy using NTRIP.

### Hardware Requirements

- u-blox ZED-F9R GNSS receiver (e.g., SimpleRTK2B Fusion board)
- Connected via USB (udev rules create `/dev/ublox-gps` symlink)
- Clear sky view for optimal satellite reception

### NTRIP Service

**Default**: e-GNSS Taiwan VRS (Virtual Reference Station)
- Server: 210.241.63.193:81
- Mountpoint: Taiwan
- Credentials: `src/sensor_kit/autosdv_sensor_kit_launch/launch/ntrip.launch.xml`

### Enable NTRIP

```bash
# Basic RTK setup with u-blox + NTRIP
make launch ARGS="gnss_receiver:=ublox use_ntrip:=true"

# Full outdoor autonomous setup with RTK
make launch ARGS="gnss_receiver:=ublox use_ntrip:=true lidar_model:=robin-w camera_model:=zedxm"
```

### How NTRIP Works

1. NTRIP client connects to e-GNSS Taiwan VRS server
2. Client subscribes to `/sensing/gnss/ublox/nmea_sentence` (rover position)
3. Server sends RTCM corrections via `/sensing/gnss/ntrip/rtcm` topic
4. u-blox driver applies corrections to achieve RTK fix
5. Output: cm-level accuracy on `/sensing/gnss/ublox/nav_sat_fix`

### Monitor RTK Status

```bash
ros2 topic hz /sensing/gnss/ntrip/rtcm              # RTCM corrections
ros2 topic echo /sensing/gnss/ublox/nmea_sentence   # NMEA sentences
ros2 topic echo /sensing/gnss/ublox/nav_sat_fix     # RTK fix quality
ros2 topic echo /sensing/gnss/ublox/rxmrtcm         # RTCM reception status
```

### Configuration Files

- NTRIP client: `src/sensor_kit/autosdv_sensor_kit_launch/launch/ntrip.launch.xml`
- ZED-F9R config: `src/sensor_kit/autosdv_sensor_kit_launch/config/zed_f9r_rover.yaml`

### Troubleshooting

- **No RTCM data**: Check internet connection and e-GNSS credentials
- **Poor accuracy**: Ensure clear sky view (avoid buildings/trees)
- **Recommended location**: Open areas like NTU College of Social Sciences

## Indoor Operation (No GPS)

For indoor testing without GNSS:

```bash
make launch ARGS="use_gnss:=false"
```

When running indoors:
1. The system uses NDT localization instead of GNSS
2. Use RViz's "2D Pose Estimate" tool to set initial vehicle position
3. Click and drag on the map to set pose and orientation
4. The `/initialpose` topic receives the manual pose input

## Localization Sources

### NDT Scan Matching (Default)

```bash
make launch ARGS="pose_source:=ndt"
```

- Tuned for VLP-32C LiDAR on COSS map (NTU Campus)
- Resolution: 2.0m, Score threshold: 2.2, Voxel size: 0.5m, Points: 3000
- See `docs/research/localization/ndt_parameter_tuning_coss_map.md` for tuning details

### Isaac ROS Visual SLAM (GPU-Accelerated)

```bash
make launch ARGS="pose_source:=isaac use_gnss:=false camera_model:=zedxm"
```

- GPU-accelerated stereo visual-inertial odometry (cuVSLAM)
- Suitable for indoor/GNSS-denied environments
- Requires ZED X Mini (GMSL connection) + IMU + ZED SDK 5.x
- See `docs/design/isaac_vslam_integration.md` and `docs/guides/isaac_vslam_testing.md`
