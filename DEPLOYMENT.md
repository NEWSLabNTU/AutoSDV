# AutoSDV Deployment Guide

## Overview

This guide explains how to deploy the same AutoSDV Docker image across two environments:
- **Testing**: x86 PC with QEMU emulation
- **Production**: Jetson Orin with real hardware

The system **automatically detects** which environment it's running in and configures itself accordingly.

---

## Deployment Architecture

### Phase 1: Testing (Current)
- **Hardware**: x86_64 PC (Intel/AMD)
- **Docker**: ARM64 container via QEMU emulation
- **Sensors**: None (simulation only)
- **Auto-detection**: → Simulation mode

### Phase 2: Production (Final)
- **Hardware**: Jetson Orin (native ARM64)
- **Docker**: Same ARM64 container (native, no emulation!)
- **Sensors**: All connected (LiDAR, Camera, GPS, IMU)
- **Auto-detection**: → Hardware mode

---

## Key Insight: Same Image, Different Behavior

The **same Docker image** will:
- ✅ Run in **simulation mode** on x86 (auto-detected via device checks)
- ✅ Run in **hardware mode** on Jetson Orin (auto-detected via device tree)
- ✅ Require **no manual configuration** between environments

---

## How Auto-Detection Works

### On x86 + Docker + QEMU:
```
/proc/device-tree/model        → ❌ Not present (no real ARM hardware)
/sys/devices/platform/tegra*   → ❌ Not present (no Jetson-specific chips)
/sys/class/gpio/               → ❌ No GPIO hardware interfaces
/etc/nv_tegra_release          → ✅ Present (from L4T Docker image)

Result: 🐳 Simulation mode
Action: launch_vehicle=false, launch_sensing_driver=false
```

### On Jetson Orin + Docker:
```
/proc/device-tree/model        → ✅ Present: "NVIDIA Orin Nano Developer Kit"
                                 └─ MATCH! Real Jetson detected!

Result: 🤖 Hardware mode
Action: launch_vehicle=true, launch_sensing_driver=true
```

---

## Deployment Workflow

### Step 1: Build and Test on x86 PC

```bash
# On your x86 development machine:
cd docker
make build              # Build ARM64 image (slow with QEMU, be patient!)
make run                # Run container

# Inside container:
cd /AutoSDV
make build              # Build ROS workspace
make launch             # Auto-detects → simulation mode

# Expected output:
# Detecting hardware environment...
# 🐳 L4T Docker image detected (no real Jetson hardware) - using simulation mode
# 🐳 Launching in SIMULATION mode (no hardware required)
```

### Step 2: Transfer Image to Jetson

**Option A: Save and Load Image**
```bash
# On x86 PC:
docker save autosdv_image:latest | gzip > autosdv_image.tar.gz
scp autosdv_image.tar.gz jetson@jetson-ip:/home/jetson/

# On Jetson:
docker load < autosdv_image.tar.gz
```

**Option B: Docker Registry**
```bash
# On x86 PC:
docker tag autosdv_image:latest myregistry/autosdv:latest
docker push myregistry/autosdv:latest

# On Jetson:
docker pull myregistry/autosdv:latest
```

**Option C: Rebuild on Jetson (Recommended - Fastest!)**
```bash
# Copy source code to Jetson
scp -r /path/to/AutoSDV jetson@jetson-ip:/home/jetson/

# On Jetson:
cd AutoSDV/docker
make build              # Much faster - native ARM64 build, no QEMU!
```

### Step 3: Run on Jetson Orin

```bash
# On Jetson Orin:
cd docker
make run                # Runs with --privileged and full device access

# Inside container:
cd /AutoSDV
make launch             # Auto-detects → hardware mode!

# Expected output:
# Detecting hardware environment...
# 🤖 Real Jetson hardware detected (device tree found)!
# 🚗 Launching on HARDWARE (Jetson Orin with sensors)
```

---

## Docker Configuration

The `docker/Makefile` is configured with the following flags to ensure hardware access on Jetson:

```bash
--privileged              # Full hardware access
--gpus all                # GPU access for TensorRT, CUDA
--net host                # Network access for LiDAR sensors
-v /dev:/dev:rw           # Device access (GPIO, I2C, serial, cameras)
-v /tmp/.X11-unix:...     # X11 for RViz visualization
-v /AutoSDV:rw            # Mount workspace for development
```

These flags are **safe on both x86 and Jetson**:
- On x86: Privileged but no hardware → no effect
- On Jetson: Privileged + hardware present → full access

---

## What's Accessible in Each Environment

| Component | x86 Docker | Jetson Docker | Notes |
|-----------|------------|---------------|-------|
| Device Tree | ❌ Absent | ✅ Present | Key detection method |
| GPIO (`/dev/gpiochip*`) | ❌ Absent | ✅ Present | Servo/motor control |
| I2C (`/dev/i2c-*`) | ❌ Absent | ✅ Present | IMU, PWM board |
| Serial (`/dev/ttyUSB*`) | ❌ Absent | ✅ Present | GPS/GNSS |
| LiDAR (Network) | ❌ No HW | ✅ Works | Via `--net host` |
| ZED Camera | ❌ No HW | ✅ Works | Via `/dev` mount |
| CUDA/GPU | ⚠️ Emulated | ✅ Native | Via `--gpus all` |
| Planning Logic | ✅ Works | ✅ Works | CPU-only, no hardware needed |
| Perception Algorithms | ✅ Works | ✅ Works | Can run without sensor data |

---

## Testing Checklist

### Before Deploying to Jetson (x86 Testing):
- [ ] `make build` completes successfully
- [ ] `make launch` runs without crashes
- [ ] Auto-detection shows simulation mode
- [ ] Core modules start and stay running
- [ ] No crash loops

### On Jetson (Production Validation):
- [ ] Docker runs with `--privileged` and `-v /dev:/dev`
- [ ] Auto-detection shows hardware mode
- [ ] Sensors detected: `ls /dev/tty*`, `/dev/i2c-*`, `/dev/video*`
- [ ] GPIO accessible: `ls /sys/class/gpio/`
- [ ] LiDAR pingable: `ping 192.168.1.201`
- [ ] ZED camera streams: `v4l2-ctl --list-devices`

---

## Important Notes

### 1. QEMU Performance
Building on x86 with QEMU is **5-10x slower** than native:
- Docker build may take 1-2 hours on x86
- Same build takes 10-15 minutes on Jetson
- Consider building directly on Jetson for faster iteration

### 2. Device Permissions on Jetson
You may need to add your user to device groups:
```bash
sudo usermod -aG i2c,dialout,gpio $USER
# Log out and back in for changes to take effect
```

Or use Docker with `--privileged` (already configured).

### 3. Network Configuration for LiDAR
Ensure the host network interface is configured for the LiDAR subnet:
```bash
# Check your network interface name (e.g., eth0, enp0s1)
ip link show

# Configure for LiDAR subnet (192.168.1.x)
sudo ip addr add 192.168.1.100/24 dev eth0
```

### 4. RViz on Jetson
RViz works but may be slow on Jetson. Alternatives:
- Run RViz on a separate PC connected to the same ROS network
- Use web-based visualization (Foxglove Studio)
- Use headless mode and visualize data in post-processing

---

## Troubleshooting

### Issue: Auto-detection fails on Jetson
**Solution**: Verify device tree is accessible:
```bash
cat /proc/device-tree/model
# Should show: NVIDIA Orin Nano Developer Kit (or similar)
```

### Issue: Sensors not working on Jetson
**Solution**: Check device permissions and mounts:
```bash
# Inside container:
ls -la /dev/i2c-*
ls -la /dev/ttyUSB*
ls -la /dev/gpiochip*
```

### Issue: RViz crashes on Jetson
**Solution**: Ensure X11 forwarding is configured:
```bash
# On Jetson (before running container):
export DISPLAY=:0
xhost +local:
```

---

## Summary

✅ **Same Docker image** works on x86 (testing) and Jetson (production)  
✅ **Auto-detection** handles both environments without manual configuration  
✅ **No code changes** needed between environments  
✅ **Hardware access** properly configured via `--privileged` and device mounts  
✅ **Safe on both platforms** (simulation when no hardware, full mode when hardware present)

**Simple Workflow:**
1. Build and test on x86
2. Transfer image to Jetson
3. Run with same commands
4. System auto-detects and configures itself!

---

Last updated: 2025-01-13

