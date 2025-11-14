# AutoSDV Evaluation Checklist

## Pre-Evaluation Status

### ✅ Completed on x86 PC:
- [x] Docker image built successfully (autosdv:4af9537, 19.4GB)
- [x] Container runs with QEMU emulation
- [x] Simulation mode tested (no crashes)
- [x] Auto-detection mechanism implemented
- [x] Multi-architecture support configured
- [x] NVIDIA container toolkit configured

### ⚠️ Pending on ARM64 Server:
- [ ] Deploy to ARM64 server/Jetson
- [ ] Test hardware auto-detection
- [ ] Verify perception modules launch
- [ ] Test lidar_centerpoint specifically
- [ ] Measure GPU utilization
- [ ] Measure object detection FPS

---

## Evaluation Requirement 0: Build

**Command:**
```bash
cd AutoSDV/docker
make build
```

**Status:** ✅ COMPLETE
- Image: `autosdv:4af9537`
- Size: 19.4GB
- Architecture: ARM64
- Base: NVIDIA L4T + ROS 2 Humble

**Evidence:** Docker image exists and runs successfully on x86 via QEMU

---

## Evaluation Requirement 1: Run on ARM64 Server

### Step 1: Deploy Container to ARM64 Server

**Option A: Via DockerHub (Recommended for multiple devices)**
```bash
# On x86 PC:
docker login
docker tag autosdv:4af9537 yourusername/autosdv:2025.02-latest
docker push yourusername/autosdv:2025.02-latest

# On ARM64 server:
docker pull yourusername/autosdv:2025.02-latest
```

**Option B: Direct Image Transfer**
```bash
# On x86 PC:
docker save autosdv:4af9537 | gzip > autosdv_image.tar.gz
scp autosdv_image.tar.gz user@arm64-server:/tmp/

# On ARM64 server:
gunzip -c /tmp/autosdv_image.tar.gz | docker load
```

### Step 2: Run Container on ARM64

```bash
# On ARM64 server:
docker run -it --name autosdv_container \
    --gpus all \
    --net host \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev:rw \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    autosdv:4af9537 /bin/bash
```

### Step 3: Run make launch

```bash
# Inside container:
cd /AutoSDV
make launch
```

### Expected Output:

```
Detecting hardware environment...
🤖 Real Jetson hardware detected (device tree found)!
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🚗 Launching on HARDWARE (Jetson Orin with sensors)
   launch_rviz:=true (GUI enabled)
   launch_vehicle:=true (GPIO hardware available)
   launch_sensing_driver:=true (real sensors connected)
   launch_perception:=true (GPU/TensorRT available)
   use_gnss:=true (GPS hardware connected)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### Verification Checklist:

- [ ] Container starts without errors
- [ ] Auto-detection shows "Real Jetson hardware detected"
- [ ] launch_perception:=true is shown
- [ ] No crash loops
- [ ] All nodes start successfully

**Status:** ⚠️ READY FOR TESTING

---

## Evaluation Requirement 2: Perception Modules (lidar_centerpoint)

### Verify Perception is Enabled

```bash
# Inside container, check if perception is launching
ros2 node list | grep perception
```

**Expected nodes:**
```
/perception/object_recognition/detection/centerpoint/lidar_centerpoint
/perception/object_recognition/detection/clustering/...
/perception/object_recognition/tracking/...
```

### Verify lidar_centerpoint Specifically

```bash
# Check if lidar_centerpoint node is running
ros2 node list | grep lidar_centerpoint

# Check topics published by lidar_centerpoint
ros2 topic list | grep centerpoint

# Check node info
ros2 node info /perception/object_recognition/detection/centerpoint/lidar_centerpoint
```

**Expected output:**
```
Publishers:
  /perception/object_recognition/detection/centerpoint/objects
  
Subscribers:
  /perception/obstacle_segmentation/pointcloud
  /tf
```

### Check TensorRT Model Loading

```bash
# Look for TensorRT messages in the logs
ros2 topic echo /diagnostics | grep -i tensorrt
```

### Verify Object Detection Output

```bash
# Check if objects are being detected
ros2 topic echo /perception/object_recognition/objects
```

**Should show DetectedObjects messages with bounding boxes, classifications, etc.**

### Verification Checklist:

- [ ] lidar_centerpoint node is running
- [ ] No TensorRT errors in logs
- [ ] Object detection topics are publishing
- [ ] DetectedObjects messages contain valid data
- [ ] Objects are being detected from LiDAR data

**Status:** ⚠️ NEEDS TESTING ON ARM64

---

## Evaluation Requirement 3: GPU Utilization & FPS

### Method 1: Using jtop (Recommended on Jetson)

```bash
# On Jetson host (not inside container):
sudo apt install python3-pip -y
sudo pip3 install jetson-stats
sudo jtop
```

**Look for:**
- GPU utilization % (should be >0% when perception is running)
- GPU memory usage
- TensorRT engine loading
- CPU usage per core

### Method 2: Using nvtop

```bash
# Install nvtop
sudo apt install nvtop

# Run nvtop
nvtop
```

### Method 3: Using nvidia-smi (Basic)

```bash
nvidia-smi -l 1  # Update every 1 second
```

### Measure Object Detection FPS

```bash
# Inside container:

# Method 1: Using ros2 topic hz
ros2 topic hz /perception/object_recognition/objects

# Method 2: Using ros2 topic echo with timestamps
ros2 topic echo --once /perception/object_recognition/objects | grep stamp

# Method 3: Check node performance
ros2 topic hz /perception/object_recognition/detection/centerpoint/objects
```

**Expected FPS:**
- lidar_centerpoint: 5-10 Hz (typical for LiDAR perception)
- Final objects output: 5-10 Hz

### Measure End-to-End Latency

```bash
# Check latency from sensor input to detection output
ros2 topic delay /perception/object_recognition/objects
```

### Performance Metrics to Record:

```
GPU Utilization:      ____%
GPU Memory Usage:     ____MB / ____MB
Object Detection FPS: ____Hz
Average Latency:      ____ms
CPU Usage:            ____%
```

### Verification Checklist:

- [ ] GPU utilization is >0% (confirms GPU is being used)
- [ ] Object detection topics publish at reasonable FPS (>1 Hz)
- [ ] No dropped frames or timeouts
- [ ] Performance metrics documented

**Status:** ⚠️ NEEDS TESTING ON ARM64

---

## Troubleshooting Guide

### Issue: Auto-detection fails (shows simulation mode on ARM64)

**Debug:**
```bash
# Check device tree
cat /proc/device-tree/model

# Check Tegra platform
ls /sys/devices/platform/ | grep tegra

# Force hardware mode
make launch-hw
```

### Issue: Perception modules don't start

**Debug:**
```bash
# Check if perception is enabled
ros2 param get /autoware_launch launch_perception

# Check for errors
ros2 topic echo /diagnostics | grep -i error

# Check TensorRT logs
journalctl -f | grep tensorrt
```

### Issue: lidar_centerpoint crashes

**Debug:**
```bash
# Check TensorRT model files exist
ls -lh /AutoSDV/data/lidar_centerpoint/

# Check CUDA availability
nvidia-smi

# Check node logs
ros2 run rqt_console rqt_console
```

### Issue: Low or zero GPU utilization

**Possible causes:**
1. TensorRT model not loaded
2. No LiDAR data input
3. Perception disabled in launch config
4. GPU drivers not installed

**Debug:**
```bash
# Check CUDA
nvidia-smi

# Check LiDAR input
ros2 topic hz /sensing/lidar/points_raw

# Check perception mode
grep perception_mode /AutoSDV/src/launcher/autosdv_launch/launch/autosdv.launch.yaml
```

### Issue: Zero FPS on detection topics

**Possible causes:**
1. No sensor data (LiDAR not connected)
2. Perception not started
3. TensorRT engine compilation in progress (wait 1-2 minutes)

**Debug:**
```bash
# Check all topics
ros2 topic list

# Check node status
ros2 node list

# Check if nodes are still starting
ros2 topic echo /diagnostics
```

---

## Summary Checklist

### Pre-Evaluation (x86 PC):
- [x] Docker image built
- [x] Multi-architecture support configured
- [x] Auto-detection implemented
- [x] Simulation mode tested (no crashes)

### Evaluation on ARM64 Server:
- [ ] **Req 0**: make build completes
- [ ] **Req 1**: Container runs, make launch works
- [ ] **Req 2**: Perception modules run, lidar_centerpoint detected
- [ ] **Req 3**: GPU utilization measured, FPS recorded

### Final Deliverables:
- [ ] Screenshot of jtop showing GPU utilization
- [ ] Output of `ros2 topic hz` showing detection FPS
- [ ] Screenshot of running system with perception
- [ ] Performance metrics documented

---

## Quick Test Script

Save this as `test_perception.sh` inside the container:

```bash
#!/bin/bash

echo "=== AutoSDV Perception Test ==="

echo "1. Checking if lidar_centerpoint is running..."
ros2 node list | grep lidar_centerpoint
if [ $? -eq 0 ]; then
    echo "   ✅ lidar_centerpoint node found"
else
    echo "   ❌ lidar_centerpoint node NOT found"
fi

echo ""
echo "2. Checking object detection topic..."
ros2 topic list | grep "perception/object_recognition/objects"
if [ $? -eq 0 ]; then
    echo "   ✅ Detection topic exists"
    echo "   Measuring FPS (10 seconds)..."
    timeout 10 ros2 topic hz /perception/object_recognition/objects
else
    echo "   ❌ Detection topic NOT found"
fi

echo ""
echo "3. Checking GPU usage (requires nvidia-smi on host)..."
nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader

echo ""
echo "=== Test Complete ==="
```

---

**Last Updated**: November 14, 2025
**Current Status**: Ready for ARM64 evaluation
**Next Step**: Deploy to ARM64 server and run evaluation

