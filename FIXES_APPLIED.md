# AutoSDV Containerization: Issues and Solutions

## Overview
This document tracks all issues encountered and solutions applied while containerizing the AutoSDV project for ARM64 architecture, building cross-platform Docker images on x86 host machines.

---

## Phase 1: Cross-Platform Build Setup

### Issue 1.1: Cannot Build ARM64 Image on x86 Host
**Date**: November 13-18, 2025  
**Error**: Default Docker cannot build ARM64 images on x86_64  
**Root Cause**: Standard Docker build doesn't support cross-architecture compilation  
**Solution**:
1. Installed QEMU for ARM64 emulation:
   ```bash
   sudo apt-get install qemu binfmt-support qemu-user-static
   docker run --privileged --rm tonistiigi/binfmt --install all
   ```
2. Created Docker buildx builder:
   ```bash
   docker buildx create --name autosdv-builder --use
   ```
3. Modified `docker/Makefile` to use `docker buildx build --platform linux/arm64`

**Files Modified**:
- `docker/Makefile`: Added `buildx` support and `--platform linux/arm64` flag

---

### Issue 1.2: Git Submodule Repository Mismatch
**Error**: `fatal: reference is not a tree: 798e7c415a310a85576d1c21e62ed37ca0a0bcd5`  
**Root Cause**: Dockerfile was cloning from upstream `NEWSLabNTU/AutoSDV` but trying to checkout a commit from the forked repository `misuhsieh001/AutoSDV-containerization`  
**Solution**:
1. Added `REPO_URL` build argument to Dockerfile
2. Modified Makefile to auto-detect the correct repository URL from git config
3. Changed to clone from the user's fork and checkout the branch name instead of commit hash

**Files Modified**:
- `docker/Dockerfile`: Added `ARG REPO_URL` and use `${REPO_URL}` in git clone
- `docker/Makefile`: Added `REPO_URL` and `BRANCH_NAME` variables, pass them as build args

---

## Phase 2: Dependency Installation Issues

### Issue 2.1: ROS 2 Packages Not Found During Build
**Error**: `E: Unable to locate package ros-humble-cv-bridge`  
**Root Cause**: ROS 2 packages were being installed before the ROS 2 repository was configured by `setup-dev-env.sh`  
**Solution**: Split package installation into two steps:
1. Install non-ROS packages first (OpenCV, I2C tools, wget)
2. Source ROS environment, then install ROS packages in a separate RUN command

**Files Modified**:
- `docker/Dockerfile`: Separated ROS package installation and added `source /opt/ros/humble/setup.bash` before installing ROS packages

---

### Issue 2.2: Nested Git Submodules Not Initialized
**Error**: `fatal: destination path 'zed-ros2-interfaces' already exists`  
**Root Cause**: `git clone --recursive` only initializes one level of submodules, not nested ones  
**Solution**: Added explicit nested submodule initialization in Dockerfile

**Files Modified**:
- `docker/Dockerfile`: Added `git submodule update --init --recursive` after cloning

---

## Phase 3: Build Failures Inside Container

### Issue 3.1: Seyond Driver Build Failure - Architecture Mismatch
**Error**: 
```
/usr/bin/ld: libinnolidarsdkclient.a: error adding symbols: file in wrong format
Relocations in generic ELF (EM: 62)
```
**Root Cause**: The `inno-lidar-sdk` submodule contained precompiled x86-64 static libraries (`.a` files). The seyond driver's build script checked if the library existed but didn't verify architecture compatibility, so it tried to link x86-64 binaries into an ARM64 build.

**Why It Works on ARM Host**: When building directly on a Jetson/ARM host, the SDK was likely compiled from source for ARM64, or an ARM64 precompiled version was available.

**Solution**: Force rebuild the SDK from source for ARM64 by:
1. Removing precompiled x86-64 libraries
2. Running the SDK's build script to compile for ARM64

**Files Modified**:
- `docker/Dockerfile`: Added step to delete precompiled libs and run `build_unix.sh` to compile SDK from source

---

### Issue 3.2: ZED SDK Not Found During Build
**Error**: `CMake Error: Could not find a package configuration file provided by "ZED"`  
**Root Cause**: The ZED SDK was installed but CMake couldn't find it because `CMAKE_PREFIX_PATH` wasn't properly set  
**Solution**: Pass ZED SDK path directly to colcon via `--cmake-args -DCMAKE_PREFIX_PATH="/usr/local/zed"`

**Files Modified**:
- `Makefile` (top-level): Modified `build` target to pass ZED path as CMake argument

---

## Phase 4: Runtime Issues (Previously Fixed - November 14, 2025)

### Issue 4.1: Perception Nodes Crashing
**Error**: `Could not load library dlopen error: libnvdla_compiler.so`  
**Root Cause**: TensorRT/CUDA libraries require real GPU hardware, not available in emulated Docker on x86  
**Solution**: Added `launch_perception` parameter to disable perception modules in test environments

### Issue 4.2: System Monitor Crashing
**Error**: Jetson.GPIO import failure  
**Root Cause**: `Jetson.GPIO` library requires real Jetson hardware GPIO pins  
**Solution**: Added `launch_vehicle` parameter to disable vehicle interface in test environments

### Issue 4.3: GPS Driver Crashes
**Error**: Serial port `/dev/ttyUSB0` not available  
**Root Cause**: No GPS hardware connected in Docker environment  
**Solution**: Added `launch_gnss` parameter to disable GPS drivers in test environments

### Issue 4.4: DDS Multicast Failures
**Error**: `IP_MULTICAST_IF failed: No such device`  
**Root Cause**: Docker container's network doesn't support multicast properly  
**Solution**: Modified `cyclonedds.xml` to disable multicast and use loopback interface only

### Issue 4.5: Launch File Configuration Path Errors
**Error**: `sensors_calibration.yaml not found`  
**Root Cause**: Launch file couldn't resolve relative paths to config files  
**Solution**: Added explicit `config_dir` path in `autosdv.launch.yaml`

**Files Modified**:
- `src/launch/autosdv_launch/launch/autosdv.launch.yaml`: Added launch parameters and config paths
- `cyclonedds.xml`: Disabled multicast, set loopback interface
- `Makefile`: Changed `. install/setup.bash` to `source install/setup.bash`

---

## Summary of All Modified Files

### Docker Configuration
1. **`docker/Dockerfile`**:
   - Added ZED SDK installation for JetPack 6.0 (L4T 36.3)
   - Split ROS package installation (non-ROS first, then ROS with sourcing)
   - Added nested submodule initialization
   - Added seyond SDK compilation from source for ARM64
   - Added `git config --global --add safe.directory '*'`

2. **`docker/Makefile`**:
   - Added `docker buildx` support with `--platform linux/arm64`
   - Added `REPO_URL` and `BRANCH_NAME` auto-detection
   - Pass repository URL and branch name as build arguments
   - Made container persistent (removed `--rm`, added create/start/attach logic)

### Build Configuration
3. **`Makefile` (top-level)**:
   - Modified `build` target to pass ZED SDK path via `--cmake-args`
   - Changed launch target to use `source` instead of `.`

### Runtime Configuration
4. **`cyclonedds.xml`**:
   - Disabled multicast: `<AllowMulticast>false</AllowMulticast>`
   - Set loopback interface: `<NetworkInterface name="lo" multicast="false"/>`

5. **`src/launch/autosdv_launch/launch/autosdv.launch.yaml`**:
   - Added `launch_perception`, `launch_vehicle`, `launch_gnss` parameters
   - Added `config_dir` path for vehicle launch

---

## Workflow Summary

### On Host (x86)
```bash
cd docker/
make bootstrap          # Install buildx, QEMU (one-time setup)
make build-force        # Build ARM64 image (20-30 min)
make run               # Create and enter container
```

### Inside Container (ARM64 emulated)
```bash
make setup             # Install dependencies (first time only)
make build             # Build ROS workspace (~15-20 min)
make launch            # Launch AutoSDV system
```

---

## Current Status
✅ Cross-platform ARM64 image build working  
✅ All dependencies installed correctly  
✅ Seyond SDK compiles from source for ARM64  
✅ ZED SDK properly configured  
✅ ROS 2 workspace builds successfully  
✅ System launches without hardware-dependent crashes  

---

## Appendix: Detailed Module Control (From November 14 Fixes)

For reference, the following hardware-dependent modules are controlled by launch parameters:

| Module                  | Parameter                  | Default | Required Hardware        |
|------------------------|----------------------------|---------|--------------------------|
| RViz/RQT (GUI)         | `launch_rviz`             | false   | Display/X11              |
| Vehicle Interface      | `launch_vehicle`          | false   | Jetson.GPIO              |
| Sensor Drivers         | `launch_sensing_driver`   | false   | LiDAR, Camera, IMU       |
| Perception             | `launch_perception`       | false   | GPU, TensorRT            |
| GNSS (GPS)             | `use_gnss`                | false   | GPS hardware             |

**Simulation Mode**: All hardware modules disabled  
**Hardware Mode**: All modules enabled for Jetson deployment

---

**Document Last Updated**: November 18, 2025  
**Status**: Ready for ARM64 Jetson deployment1. **RViz2 Crashes**: Made conditional on `launch_rviz` parameter
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

