# AutoSDV Launch Error Analysis & Solutions

**Environment:** Clean Jetson AGX Orin (no sensors connected) running in Docker container  
**Launch Mode:** Hardware mode (`make launch-hw`)  
**Date:** November 19, 2025

---

## Summary

The system attempted to launch in **hardware mode** but encountered multiple errors because:
1. Physical sensors are not connected (LiDAR, GPS, IMU)
2. Hardware interfaces are unavailable (I2C for PWM, GPIO for motor control)
3. Missing perception model files
4. Running inside Docker container (limited hardware access)

**Result:** System partially launched but critical hardware-dependent nodes failed.

---

## Critical Errors (System Cannot Start)

### 1. **Missing Perception Model File**
```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): 
[Errno 2] No such file or directory: 
'data/lidar_centerpoint/centerpoint_tiny_ml_package.param.yaml'
```

**Cause:** Perception model files not downloaded/installed  
**Impact:** Entire launch process terminated  
**Solution:**
```bash
# Download perception models (if available)
cd /AutoSDV/data
# Or disable perception in launch
make launch-sim  # Disables perception
```

**Status:** 🔴 **BLOCKING** - Prevents system startup

---

## Hardware Interface Errors (Expected Without Hardware)

### 2. **Actuator Node - PCA9685 I2C PWM Driver**
```
[ERROR] [actuator-3]: process has died [pid 7586, exit code 1]
OSError: [Errno 121] Remote I/O error
File "Adafruit_PCA9685/PCA9685.py", line 75, in __init__
  self.set_all_pwm(0, 0)
```

**Cause:** PCA9685 PWM chip not connected on I2C bus  
**Impact:** Motor/servo control unavailable  
**Expected:** ✅ Normal without hardware  
**Solution:** Use `launch_vehicle:=false` to disable

---

### 3. **Velocity Report Node - Jetson.GPIO Module**
```
[ERROR] [velocity_report-4]: process has died [pid 7588, exit code 1]
ModuleNotFoundError: No module named 'Jetson'
```

**Cause:** Jetson.GPIO Python module not installed in container  
**Impact:** Cannot read wheel encoder/velocity from GPIO  
**Expected:** ✅ Normal in Docker (GPIO not accessible)  
**Solution:** 
- Install in Dockerfile: `pip install Jetson.GPIO`
- OR use `launch_vehicle:=false`

---

### 4. **GPS Drivers - Serial Port Missing**
```
[nmea_reader-44] Traceback (most recent call last):
  serial.serialutil.SerialException: [Errno 2] could not open port /dev/ttyUSB0: 
  [Errno 2] No such file or directory: '/dev/ttyUSB0'
```

**Cause:** No GPS device connected at `/dev/ttyUSB0`  
**Impact:** GNSS positioning unavailable  
**Expected:** ✅ Normal without GPS hardware  
**Solution:** Use `use_gnss:=false`

---

### 5. **NMEA Topic Driver - NumPy 2.0 Compatibility**
```
[nmea_topic_driver-45] AttributeError: `np.maximum_sctype` was removed in the NumPy 2.0 release.
File "transforms3d/quaternions.py", line 26, in <module>
  _MAX_FLOAT = np.maximum_sctype(np.float)
```

**Cause:** transforms3d library incompatible with NumPy 2.0  
**Impact:** GPS topic processing fails  
**Status:** 🟡 **SOFTWARE BUG**  
**Solution:**
```bash
# Downgrade NumPy in container
pip install "numpy<2.0"
# OR update transforms3d
pip install --upgrade transforms3d
```

---

### 6. **MPU9250 IMU - I2C Communication**
```
[mpu9250driver-41] Error waking sensor
Error enabling bypass
Error setting gyroscope range
OSError: Remote I/O error
```

**Cause:** MPU9250 IMU not connected on I2C bus  
**Impact:** IMU data unavailable for localization  
**Expected:** ✅ Normal without hardware  
**Solution:** Use `launch_sensing_driver:=false`

---

### 7. **Seyond LiDAR - Network Connection**
```
[seyond_node-38] [ERROR] strerror: 'Operation now in progress' 
get_connection timeout for 5.00000s
Error opening HTTP connection (172.168.1.10:8010)
```

**Cause:** Seyond LiDAR not connected at IP 172.168.1.10:8010  
**Impact:** LiDAR point cloud unavailable  
**Expected:** ✅ Normal without LiDAR  
**Solution:** Use `launch_sensing_driver:=false`

---

## System Monitor Warnings (Non-Critical)

### 8. **Network Monitor - Socket Connection**
```
[component_container_mt-9] [ERROR] [system.system_monitor.net_monitor]: 
Failed to connect socket. No such file or directory
```

**Cause:** Network monitoring service unavailable in container  
**Impact:** Network diagnostics not available  
**Status:** ⚠️ **Warning** - System continues

---

### 9. **HDD Monitor - Device Unmount**
```
[component_container_mt-9] [ERROR] [system.system_monitor.hdd_monitor]: 
socket connect error. Connection refused
Failed to unmount device : overlay
```

**Cause:** Docker overlay filesystem cannot be unmounted  
**Impact:** Disk monitoring limited  
**Status:** ⚠️ **Warning** - System continues

---

## Topic State Monitor Messages (Informational)

### 10. **Topic Monitoring - Not Received**
```
[topic_state_monitor_node-14] [INFO]: /map/vector_map has not received. Set ERROR in diagnostics.
[topic_state_monitor_node-17] [INFO]: /localization/pose_twist_fusion_filter/pose has not received.
[topic_state_monitor_node-25] [INFO]: /vehicle/status/velocity_status has not received.
```

**Cause:** Topics waiting for data from sensors/processing nodes that haven't started  
**Impact:** None - just diagnostic status  
**Status:** ℹ️ **Informational** - Normal on startup  
**Note:** These are INFO level logs, NOT errors

---

## RCLPy Shutdown Errors (Cleanup Related)

### 11. **Double Shutdown Attempts**
```
rclpy._rclpy_pybind11.RCLError: failed to shutdown: 
rcl_shutdown already called on the given context
```

**Cause:** Nodes attempting to shutdown after context already destroyed  
**Impact:** None - happens during cleanup  
**Status:** ⚠️ **Warning** - Cosmetic issue only

---

## Solutions by Scenario

### Scenario 1: Testing in Docker Without Hardware (Recommended)
```bash
cd /AutoSDV
make launch-sim
```

**Disables:**
- ❌ RViz GUI
- ❌ Vehicle interface (GPIO/I2C)
- ❌ Sensor drivers (LiDAR/Camera/GPS/IMU)
- ❌ Perception (TensorRT)

**Enables:**
- ✅ Autoware core nodes
- ✅ Planning and control logic
- ✅ Map server
- ✅ Localization (without sensors)

---

### Scenario 2: Real Jetson With Full Hardware
```bash
cd /AutoSDV
make launch-hw
```

**Prerequisites:**
- ✅ Seyond LiDAR at 172.168.1.10:8010
- ✅ ZED camera connected
- ✅ GPS at /dev/ttyUSB0
- ✅ MPU9250 IMU on I2C
- ✅ PCA9685 PWM on I2C
- ✅ Perception models downloaded
- ✅ Running on bare metal (not Docker)

---

### Scenario 3: Auto-Detection
```bash
cd /AutoSDV
make launch
```

**Behavior:**
- Detects Docker → uses `launch-sim`
- Detects bare Jetson → uses `launch-hw`

---

## Required Fixes for Full Functionality

### Fix 1: Download Perception Models (CRITICAL)
```bash
cd /AutoSDV/data
# Download lidar_centerpoint model
# (URL/method depends on Autoware/AutoSDV setup)
```

### Fix 2: Install Jetson.GPIO in Container
Add to `docker/Dockerfile`:
```dockerfile
RUN pip install Jetson.GPIO
```

### Fix 3: Fix NumPy Compatibility
Add to `docker/Dockerfile`:
```dockerfile
RUN pip install "numpy<2.0" "transforms3d>=0.4.0"
```

### Fix 4: Enable X11 for RViz (If Needed)
On Jetson host:
```bash
xhost +local:docker
```

---

## Hardware Checklist

When running on **real Jetson with sensors**, verify:

- [ ] **LiDAR**: Seyond at 172.168.1.10, ping successful
- [ ] **Camera**: ZED camera `/dev/video*` exists
- [ ] **GPS**: Serial device `/dev/ttyUSB0` exists
- [ ] **IMU**: MPU9250 detected on I2C (`i2cdetect -y 1`)
- [ ] **PWM**: PCA9685 detected on I2C (`i2cdetect -y 1`)
- [ ] **Perception**: Model files in `data/lidar_centerpoint/`
- [ ] **Display**: X11 configured for RViz

---

## Error Severity Classification

| Severity | Count | Description |
|----------|-------|-------------|
| 🔴 **BLOCKING** | 1 | Missing perception model - prevents startup |
| 🟡 **SOFTWARE BUG** | 1 | NumPy compatibility issue |
| ⚠️ **WARNING** | 4 | System monitors, cleanup errors |
| ✅ **EXPECTED** | 5 | Hardware not connected (normal) |
| ℹ️ **INFO** | 13 | Topic state monitors (normal on startup) |

---

## Recommended Actions

**For Current Setup (Clean Jetson, No Hardware):**
1. ✅ Use `make launch-sim` - **WORKS NOW**
2. Fix perception model issue if needed for future testing
3. Install Jetson.GPIO and fix NumPy for vehicle interface

**For Future Hardware Deployment:**
1. Download perception models before deployment
2. Connect all sensors and verify with diagnostic tools
3. Run `make launch-hw` or `make launch` (auto-detects)

---

**Document Created:** November 19, 2025  
**Status:** Clean Jetson without hardware - Use `make launch-sim` ✅
