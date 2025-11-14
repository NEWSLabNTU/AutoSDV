# AutoSDV Launch Modes

This document explains the different launch modes for AutoSDV and when to use each one.

## Quick Reference

| Command | Environment | Hardware Required | Use Case |
|---------|-------------|-------------------|----------|
| `make launch` | Auto-detect | Depends on detection | Smart default (recommended) |
| `make launch-sim` | Docker/x86 PC | ❌ None | Testing without sensors |
| `make launch-hw` | Jetson Orin | ✅ Full hardware | Production deployment |

---

## Mode 1: Simulation Mode (`make launch-sim`)

**Use this when:**
- Testing in Docker on x86 PC (QEMU emulation)
- Development without physical hardware
- CI/CD pipelines
- Initial software testing

**What's disabled:**
- ❌ RViz2 (`launch_rviz:=false`) - GUI crashes in emulated environment
- ❌ Vehicle Interface (`launch_vehicle:=false`) - No Jetson.GPIO available
- ❌ Sensor Drivers (`launch_sensing_driver:=false`) - No real sensors connected

**What still runs:**
- ✅ Planning modules (path planning, behavior planning)
- ✅ Perception algorithms (object detection, tracking)
- ✅ Localization (waiting for sensor data)
- ✅ Control logic (trajectory following)
- ✅ Map loading and processing
- ✅ System state monitors

**Expected behavior:**
- System starts and stays running
- Nodes wait peacefully for sensor data
- No crashes or restarts
- Log messages show: `[INFO] waiting for data...` (this is NORMAL)

---

## Mode 2: Hardware Mode (`make launch-hw`)

**Use this when:**
- Running on **real Jetson Orin hardware**
- All sensors physically connected and powered
- Vehicle is ready for testing/operation
- Production deployment

**What's enabled:**
- ✅ RViz2 (`launch_rviz:=true`) - Full visualization
- ✅ Vehicle Interface (`launch_vehicle:=true`) - Actuator control via GPIO/I2C
- ✅ Sensor Drivers (`launch_sensing_driver:=true`) - LiDAR, cameras, GPS, IMU

**Additional components that activate:**
- Vehicle actuator control (steering servo, motor ESC)
- LiDAR drivers (Velodyne, Seyond Robin-W, Blickfeld)
- Camera drivers (ZED stereo camera)
- GPS/GNSS drivers (NMEA over serial)
- IMU drivers (MPU9250 over I2C)
- CAN bus interface
- PCA9685 PWM controller for servos

**Expected behavior:**
- All sensors streaming data
- Vehicle responds to control commands
- RViz shows live visualization
- Full autonomous driving capability

---

## Mode 3: Auto-Detection (`make launch`)

**How it works:**
The Makefile uses a multi-level detection strategy to distinguish real Jetson hardware from Docker containers:

1. **Check `/proc/device-tree/model`** for "NVIDIA" (GOLD STANDARD ⭐)
   - **If found** → Real Jetson hardware → runs `make launch-hw`
   - Most reliable method - only exists on actual ARM hardware
   
2. **Check `/sys/devices/platform/`** for Tegra/NVIDIA devices
   - **If found** → Real Jetson hardware → runs `make launch-hw`
   - Detects Jetson-specific platform devices
   
3. **Check `/sys/class/gpio/` + `/etc/nv_tegra_release`**
   - **If both found** → Likely real Jetson → runs `make launch-hw`
   - GPIO hardware interfaces indicate real hardware
   
4. **Check `/etc/nv_tegra_release` only**
   - **If found** → L4T Docker image (not real hardware) → runs `make launch-sim`
   - This file exists in Docker images, so insufficient alone
   
5. **Default fallback**
   - → Unknown environment → runs `make launch-sim` (safe default)

**Why this matters:**
The L4T Docker base image contains `/etc/nv_tegra_release` even when running on x86 in Docker. We must check for actual Jetson hardware interfaces (device tree, platform devices, GPIO) to make the correct decision.

**Recommended usage:**
```bash
# Just use this everywhere, it auto-detects!
make launch
```

---

## Usage Examples

### On your x86 PC (Docker):
```bash
cd /AutoSDV
make launch        # Auto-detects → simulation mode
# Or explicitly:
make launch-sim
```

### On Jetson Orin (Real hardware):
```bash
cd /AutoSDV
make launch        # Auto-detects → hardware mode
# Or explicitly:
make launch-hw
```

### Custom configuration (advanced):
```bash
ros2 launch autosdv_launch autosdv.launch.yaml \
    launch_rviz:=true \
    launch_vehicle:=false \
    launch_sensing_driver:=true
# Example: Enable RViz and sensors, but disable vehicle control
```

---

## Pre-Launch Checklist (Hardware Mode)

Before running `make launch-hw` on real Jetson:

### Hardware Connections:
- [ ] LiDAR powered on and connected (check network: `ping 192.168.1.201`)
- [ ] ZED camera detected (`ls /dev/video*` should show devices)
- [ ] GPS connected (`ls /dev/ttyUSB*` should show serial port)
- [ ] IMU detected on I2C (`i2cdetect -y 1` shows device at 0x68)
- [ ] PCA9685 PWM board detected (`i2cdetect -y 1` shows device at 0x40)
- [ ] Servos and motors connected to PCA9685

### Network Configuration:
- [ ] LiDAR network interface configured (192.168.1.x subnet)
- [ ] CAN interface up if using CAN (`ip link show can0`)

### Safety:
- [ ] Vehicle is in safe testing area
- [ ] Emergency stop is accessible
- [ ] All personnel aware of test
- [ ] Battery voltage checked

### Software:
- [ ] Workspace built: `make build`
- [ ] Workspace sourced: `source install/setup.bash`
- [ ] Map files present in `./data/`

---

## Troubleshooting

### Issue: Crash loop in Docker
**Solution:** You're probably using hardware mode in Docker. Use `make launch-sim` instead.

### Issue: Sensors not working on Jetson
**Solution:** Check hardware connections and permissions:
```bash
# Check I2C permissions
sudo usermod -a -G i2c $USER
sudo usermod -a -G dialout $USER

# Check device permissions
ls -l /dev/i2c-*
ls -l /dev/ttyUSB*
```

### Issue: RViz crashes on Jetson
**Solution:** Make sure X11 is properly configured:
```bash
export DISPLAY=:0
xhost +local:
```

---

## What Each Parameter Controls

### `launch_rviz` (true/false)
- **true**: Launch RViz2 for 3D visualization
- **false**: Headless mode (no GUI)
- **Requires**: X11 display, GPU support

### `launch_vehicle` (true/false)
- **true**: Launch vehicle interface nodes (actuator control)
- **false**: Skip vehicle control (software-only testing)
- **Requires**: Jetson.GPIO, I2C devices (PCA9685), physical actuators

### `launch_sensing_driver` (true/false)
- **true**: Launch all sensor driver nodes
- **false**: Skip sensor drivers (use for bag playback or pure simulation)
- **Requires**: Physical sensors connected and powered

---

## Architecture Compatibility

| Environment | Architecture | GPU | Jetson.GPIO | Recommended Mode |
|-------------|--------------|-----|-------------|------------------|
| x86 PC Docker | ARM64 (QEMU) | Emulated | ❌ No | `launch-sim` |
| Jetson Orin | ARM64 (native) | NVIDIA | ✅ Yes | `launch-hw` |
| Jetson Xavier | ARM64 (native) | NVIDIA | ✅ Yes | `launch-hw` |
| Raspberry Pi | ARM64 (native) | VideoCore | ⚠️ Maybe | `launch-sim` + custom |

---

## Questions?

- **Q: Can I test with ROS bags in simulation mode?**
  - A: Yes! Use `launch-sim` and play a rosbag with sensor data.

- **Q: Can I enable just sensors without vehicle control?**
  - A: Yes! Use custom launch: `launch_sensing_driver:=true launch_vehicle:=false`

- **Q: Will the Docker image work on real Jetson?**
  - A: Yes! The same image runs on both x86 (via QEMU) and native ARM64 Jetson.

- **Q: How do I know which mode is running?**
  - A: The Makefile prints a message: "🤖 Jetson detected" or "🐳 Docker detected"

---

Last updated: 2025-01-13

