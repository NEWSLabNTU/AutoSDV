# AutoSDV Quick Start Guide 🚀

**Last Updated:** November 19, 2025  
**Status:** ARM64 Docker images available on DockerHub  
**Version:** 2025.11-v1.0

---

## Quick Commands for Tomorrow

### On ARM64 Jetson (Container Already Running)

```bash
# Inside container, just run:
cd /AutoSDV
make launch-sim

# This launches Autoware core in simulation mode
# (No hardware dependencies, no sensor_kit_launch needed)
```

### If Container Exited

```bash
# Restart the existing container
cd ~/Container_Test/AutoSDV-containerization/docker
make run

# Then inside: cd /AutoSDV && make launch-sim
```

### Using Pre-built DockerHub Image (Recommended)

```bash
# Pull the latest image from DockerHub
docker pull misuhsieh001/autosdv:2025.11-latest

# Run it
cd ~/AutoSDV-containerization/docker
make run

# Inside container:
cd /AutoSDV
make build         # Build workspace once
make launch-sim    # Launch!
```

### If Need to Build from Source

```bash
cd ~/AutoSDV-containerization
git pull
docker rm -f autosdv-container
cd docker && make build-force  # 20-30 minutes
make run

# Inside container:
cd /AutoSDV
make build         # 15-20 minutes
make launch-sim    # Launch!
```

---

## Important Notes

### ⚠️ Always Use `launch-sim` 

**DO NOT USE:**
- `make launch` - Auto-detects hardware, tries launch-hw (will fail)
- `make launch-hw` - Needs autosdv_sensor_kit_launch (we skipped it)

**ALWAYS USE:**
- `make launch-sim` - Works without sensor_kit_launch ✅

### Why We Skip sensor_kit_launch

`autosdv_sensor_kit_launch` depends on `seyond` package, which contains x86-64 binaries incompatible with ARM64. We can't modify the submodule (no access to NEWSLabNTU repo).

**Workaround:** Launch sensors individually if needed:
```bash
# In separate terminals:
ros2 launch zed_wrapper zed2i.launch.py
ros2 launch nmea_reader nmea_serial_driver.launch.py
ros2 run mpu9250driver mpu9250driver_node
```

---

## What's Working ✅

- Docker container on Jetson Orin
- ~38-40 ROS packages built
- Core Autoware system
- Individual sensor drivers (ZED, GPS, IMU, Blickfeld)
- Hardware auto-detection
- GPU runtime auto-configuration

## What's Not Working ❌

- Unified sensor_kit_launch (needs Seyond ARM64 SDK)
- Seyond LiDAR driver (x86-64 only)
- Vehicle interface in container (Jetson.GPIO access issues)

---

## Deployment to Additional Jetsons

```bash
# On each new Jetson:
git clone --recurse-submodules -b 2025.11-containerization \
  https://github.com/misuhsieh001/AutoSDV-containerization.git

cd AutoSDV-containerization
docker pull misuhsieh001/autosdv:2025.02-latest
cd docker && make run

# Inside: cd /AutoSDV && make build && make launch-sim
```

---

## Troubleshooting

### If Only 4 Packages Built
```bash
# Git submodules weren't initialized
git submodule update --init --recursive
```

### If GPU Runtime Error
```bash
# Fixed - Makefile auto-detects now
# Jetson uses: --runtime=nvidia
# x86 uses: --gpus all
```

### If "autosdv_sensor_kit_launch not found"
```bash
# Expected - we skipped it
# Use: make launch-sim
```

---

## Repository Info

- **GitHub:** https://github.com/misuhsieh001/AutoSDV-containerization
- **Branch:** 2025.11-containerization
- **DockerHub:** misuhsieh001/autosdv:2025.02-latest

---

## Documentation Files

- `LAUNCH_MODES.md` - Launch mode details
- `DEPLOYMENT.md` - Full deployment guide
- `DOCKERHUB_DEPLOYMENT.md` - DockerHub workflow
- `FIXES_APPLIED.md` - All errors and solutions
- `DEPLOYMENT_COMMANDS.md` - Copy-paste commands

---

## Contact for ARM64 SDK

**Seyond Inc.** - Request ARM64 version of their LiDAR SDK to enable `autosdv_sensor_kit_launch`

---

**Good luck tomorrow! The hard part is done! 🎉**

