# Localization Presets

Localization presets define the localization configuration for AutoSDV. They control which twist estimation method is used.

## Available Presets

### `default` (Default)
Default localization preset using gyro odometry for twist estimation.

**Configuration**:
- `twist_source`: `gyro_odom`

**Use cases**:
- Standard operation with IMU
- Most outdoor and indoor scenarios
- When Eagleye is not needed

**Requirements**:
- IMU sensor (MPU9250 or ZED IMU)

**Launch**:
```bash
make launch ARGS="localization_preset:=default"
# Or omit (it's the default)
make launch
```

### `eagleye`
Uses Eagleye for twist estimation. Eagleye provides GNSS-based odometry.

**Configuration**:
- `twist_source`: `eagleye`

**Use cases**:
- When high-accuracy GNSS-based odometry is needed
- Outdoor environments with good GNSS reception
- Research on GNSS-IMU fusion

**Requirements**:
- GNSS receiver (u-blox, Septentrio, or Garmin)
- IMU sensor
- Good GNSS satellite visibility
- `use_gnss:=true` must be set

**Launch**:
```bash
make launch ARGS="localization_preset:=eagleye use_gnss:=true gnss_receiver:=ublox"
```

## Usage

Specify the preset when launching AutoSDV:

```bash
# Real hardware mode
make launch ARGS="localization_preset:=<preset_name>"

# Logging simulation mode
ros2 launch autosdv_launch logging_simulation.launch.yaml localization_preset:=<preset_name>
```

## Twist Source Comparison

| Twist Source | Input Sensors | Accuracy | Latency | Use Case |
|--------------|---------------|----------|---------|----------|
| `gyro_odom` | IMU | Medium | Low | Standard operation |
| `eagleye` | GNSS + IMU | High (outdoor) | Medium | GNSS-based odometry |

## Creating Custom Presets

To create a custom preset:

1. Copy an existing preset file (e.g., `default.yaml`)
2. Rename it (e.g., `custom.yaml`)
3. Modify the `twist_source` parameter
4. Add conditional logic in `autosdv.launch.yaml` to support the new preset
5. Update this README with the new preset documentation

## Parameter Reference

| Parameter | Type | Options | Description |
|-----------|------|---------|-------------|
| `twist_source` | string | `gyro_odom`, `eagleye` | Twist (velocity + angular rate) estimation source |

## Relationship with Other Parameters

Localization presets work together with other localization parameters:

- **`pose_source`**: Selects pose estimation method (`cuda_ndt`, `ndt`, `isaac`, `visual`)
  - `cuda_ndt`: CUDA-accelerated NDT scan matching (default, 1.3-1.6x faster)
  - `ndt`: Autoware NDT scan matching (OpenMP CPU, fallback)
  - `isaac`: Visual SLAM using Isaac ROS

- **`use_gnss`**: Enables GNSS subsystem
  - Required for `eagleye` twist source
  - Optional for `gyro_odom`

- **`use_ntrip`**: Enables RTK corrections for GNSS
  - Improves GNSS accuracy
  - Only works with u-blox receivers

- **`use_mapless_mode`**: Disables localization for indoor operation
  - Overrides pose/twist estimation
  - Uses odometry only

## Example Configurations

### Outdoor with RTK
```bash
make launch ARGS="localization_preset:=default use_gnss:=true use_ntrip:=true"
```

### Indoor without GNSS
```bash
make launch ARGS="localization_preset:=default use_gnss:=false use_mapless_mode:=true"
```

### Visual SLAM
```bash
make launch ARGS="localization_preset:=default pose_source:=isaac use_gnss:=false"
```

### Eagleye with GNSS
```bash
make launch ARGS="localization_preset:=eagleye use_gnss:=true use_ntrip:=true"
```

## Notes

- Localization presets are independent of sensor suite configuration
- The `twist_source` parameter is passed to the localization component
- Eagleye requires proper GNSS configuration (see `docs/guides/sensor-integration/gnss.md`)
- For mapless operation, consider using `use_mapless_mode:=true` instead of changing localization preset
