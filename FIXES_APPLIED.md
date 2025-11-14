# Fixes Applied to Resolve Node Crashes in Docker

## Date: November 14, 2025

## Problem Summary
When running `make launch` in the Docker container on an x86 machine, multiple nodes were crashing repeatedly:
- `shape_estimation_node` (exit code -6: SIGABRT)
- `autosdv_system_monitor_node` (exit code 1)
- `nmea_reader` and `nmea_topic_driver` (exit code 1)
- RViz2 and rqt_runtime_monitor (previously fixed)

## Root Causes

### 1. Perception Module (shape_estimation_node)
**Error**: `Could not load library dlopen error: libnvdla_compiler.so`
- **Cause**: Missing NVIDIA Deep Learning Accelerator compiler library
- **Why**: TensorRT/CUDA libraries are only available with real GPU hardware
- **Impact**: Object detection and perception nodes cannot run in Docker on x86

### 2. System Monitor
**Error**: Jetson.GPIO import failure
- **Cause**: `Jetson.GPIO` library requires real Jetson hardware
- **Why**: Attempts to access GPIO pins that don't exist in Docker/x86
- **Impact**: System monitor crashes immediately on startup

### 3. GPS Drivers
**Error**: Serial port `/dev/ttyUSB0` not available
- **Cause**: No GPS hardware connected in Docker environment
- **Why**: Hardware serial ports don't exist in test environment
- **Impact**: GPS reading nodes crash on startup

### 4. Topic State Monitor "ERROR" Messages
**Status**: NOT AN ERROR - These are normal INFO logs
- **Message**: `/map/vector_map has not received. Set ERROR in diagnostics.`
- **Explanation**: Just diagnostic messages indicating topics are waiting for data
- **Action**: No fix needed - completely normal behavior

## Solutions Applied

### 1. Added Launch Parameters (autosdv.launch.yaml)
Added explicit arguments to control hardware-dependent modules:

```yaml
- arg:
    name: launch_perception
    default: "false"
    description: "Launch perception modules (requires GPU/TensorRT)"

- arg:
    name: launch_vehicle
    default: "false"
    description: "Launch vehicle interface (requires Jetson.GPIO)"

- arg:
    name: launch_sensing_driver
    default: "false"
    description: "Launch sensor drivers (requires real hardware)"

- arg:
    name: use_gnss
    default: "false"
    description: "Enable GNSS for outdoor operation (disable for indoor/Docker testing)"
```

### 2. Made Parameters Dynamic (autosdv.launch.yaml)
Updated `let` section to use argument values instead of hardcoded strings:

```yaml
let:
  - name: launch_vehicle
    value: $(var launch_vehicle)  # Use argument value
  - name: launch_sensing_driver
    value: $(var launch_sensing_driver)  # Use argument value
  - name: launch_perception
    value: $(var launch_perception)  # Use argument value
```

### 3. Conditionalized System Monitor (autosdv.launch.yaml)
Made system monitor conditional on vehicle interface:

```yaml
# System Monitor (requires Jetson.GPIO - only launch with real hardware)
- group:
    if: "$(var launch_vehicle)"
    children:
    - include:
        file: "$(find-pkg-share autosdv_system_monitor)/launch/autosdv_system_monitor.launch.yaml"
```

### 4. Updated Makefile Launch Modes
Enhanced both simulation and hardware launch targets:

**Simulation Mode** (`make launch-sim`):
```makefile
ros2 launch autosdv_launch autosdv.launch.yaml \
    launch_rviz:=false \
    launch_vehicle:=false \
    launch_sensing_driver:=false \
    launch_perception:=false \
    use_gnss:=false
```

**Hardware Mode** (`make launch-hw`):
```makefile
ros2 launch autosdv_launch autosdv.launch.yaml \
    launch_rviz:=true \
    launch_vehicle:=true \
    launch_sensing_driver:=true \
    launch_perception:=true \
    use_gnss:=true
```

## Testing Instructions

### 1. Clean Rebuild
```bash
rm -rf build/autosdv_launch install/autosdv_launch
colcon build --packages-select autosdv_launch
```

### 2. Launch
```bash
make launch  # Auto-detects environment
# OR
make launch-sim  # Explicit simulation mode
```

### 3. Expected Results
✅ **Should NOT see**:
- RViz2 crashes
- rqt_runtime_monitor crashes
- shape_estimation_node crashes
- autosdv_system_monitor crashes
- GPS driver crashes

✅ **Should see** (NORMAL):
- Topic state monitor INFO messages
- Nodes starting successfully
- System running stably

## Module Control Summary

| Module                  | launch-sim | launch-hw | Controls                           |
|------------------------|------------|-----------|-------------------------------------|
| **RViz/RQT (GUI)**     | ❌ false   | ✅ true   | Visualization tools                |
| **Vehicle Interface**  | ❌ false   | ✅ true   | Jetson.GPIO, System Monitor        |
| **Sensor Drivers**     | ❌ false   | ✅ true   | LiDAR, Camera, IMU drivers         |
| **Perception**         | ❌ false   | ✅ true   | Object detection, TensorRT modules |
| **GNSS (GPS)**         | ❌ false   | ✅ true   | GPS drivers and positioning        |

## Files Modified

1. `/home/misuhsieh/.../AutoSDV/src/launcher/autosdv_launch/launch/autosdv.launch.yaml`
   - Added launch parameters for perception, vehicle, sensing drivers, and GNSS
   - Made system monitor conditional on launch_vehicle
   - Made RQT conditional on launch_rviz
   - Updated let section to use argument values
   - Changed use_gnss default from "true" to "false"
   - Added use_gnss parameter to Autoware launch include

2. `/home/misuhsieh/.../AutoSDV/Makefile`
   - Enhanced launch-sim with perception:=false and use_gnss:=false
   - Enhanced launch-hw with perception:=true and use_gnss:=true
   - Added clear echo messages showing which parameters are used

## Key Learnings

1. **Conditional Launch** is better than commenting out nodes
   - Allows same launch file to work in different environments
   - Single source of truth for all configurations

2. **Hardware Detection** needs multiple fallbacks
   - Different files exist in different environments
   - Prioritize device tree and platform-specific checks

3. **Topic State Monitor** messages are not errors
   - INFO level logs that happen to contain the word "ERROR"
   - Normal diagnostic behavior when topics haven't received data yet

4. **Module Dependencies** need to be understood:
   - Perception → GPU/TensorRT
   - Vehicle Interface → Jetson.GPIO
   - Sensor Drivers → Physical hardware
   - GNSS/GPS → Serial port and GPS receiver
   - GUI → X11 forwarding and non-headless environment

5. **Multiple Parameters** may control the same hardware:
   - GPS drivers require BOTH `launch_sensing_driver` AND `use_gnss` to be true
   - Understanding the launch file hierarchy is crucial
   - Check sensor kit launch files for conditional logic

## Previous Fixes (For Reference)

1. **RViz2 Crashes**: Made conditional on `launch_rviz` parameter
2. **RQT Runtime Monitor**: Made conditional on `launch_rviz` parameter
3. **Docker Device Access**: Added `-v /dev:/dev:rw` to docker run
4. **Autoware RViz Parameter**: Added both `rviz` and `launch_rviz` parameters

## Status
✅ **All known crashes fixed** (including GPS drivers)
✅ **Launch modes properly configured**
✅ **Five parameters controlling hardware modules**
✅ **Ready for testing**

## Last Updated
November 14, 2025 - Added `use_gnss` parameter to fix GPS driver crashes

