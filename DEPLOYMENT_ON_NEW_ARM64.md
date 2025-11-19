# AutoSDV Deployment on Brand New ARM64 Computer

**Target:** Fresh ARM64 device (Jetson Orin, Raspberry Pi 5, etc.)  
**Version:** 2025.11-v1.0  
**Last Updated:** November 19, 2025

---

## Prerequisites

- ARM64 device with Ubuntu 22.04 or JetPack 6.0 (L4T 36.3)
- Internet connection
- At least 50GB free disk space
- Root/sudo access

---

## Step 1: Install Docker

### On Jetson (with JetPack 6.0)

```bash
# Docker is usually pre-installed, verify:
docker --version

# If not installed:
sudo apt update
sudo apt install -y docker.io

# Add your user to docker group
sudo usermod -aG docker $USER

# Log out and back in for group changes to take effect
# Or run: newgrp docker
```

### On Generic ARM64 Ubuntu 22.04

```bash
# Install Docker
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release

sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io

# Add user to docker group
sudo usermod -aG docker $USER

# Log out and back in, or run:
newgrp docker

# Verify installation
docker run hello-world
```

---

## Step 2: Install NVIDIA Container Toolkit (Jetson Only)

```bash
# Configure the repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install the package
sudo apt update
sudo apt install -y nvidia-container-toolkit

# Configure Docker to use NVIDIA runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Verify
docker run --rm --runtime=nvidia nvcr.io/nvidia/l4t-base:r36.3.0 nvidia-smi
```

---

## Step 3: Pull AutoSDV Docker Image

```bash
# Pull the latest ARM64 image from DockerHub
docker pull misuhsieh001/autosdv:2025.11-latest

# Verify the image
docker images | grep autosdv
# Should show: misuhsieh001/autosdv  2025.11-latest  ...
```

---

## Step 4: Clone Repository (for Scripts and Configs)

```bash
# Create working directory
mkdir -p ~/autosdv-workspace
cd ~/autosdv-workspace

# Clone the repository
git clone https://github.com/misuhsieh001/AutoSDV-containerization.git
cd AutoSDV-containerization

# Checkout the containerization branch
git checkout 2025.11-containerization
```

---

## Step 5: Run the Container

### First Time - Create Container

```bash
cd ~/autosdv-workspace/AutoSDV-containerization/docker

# Run the container (will auto-detect DockerHub image)
make run
```

**What this does:**
- Detects the DockerHub image automatically
- Creates a persistent container named `autosdv-container`
- Mounts the repository directory into `/AutoSDV` inside container
- Sets up GPU access, X11 forwarding, and device access
- Drops you into a bash shell inside the container

### Subsequent Runs

```bash
cd ~/autosdv-workspace/AutoSDV-containerization/docker
make run
```

The Makefile will:
- Start the existing container if stopped
- Attach to the running container if already running
- Create a new one if none exists

---

## Step 6: Build ROS 2 Workspace (Inside Container)

Once inside the container:

```bash
# You should be in /AutoSDV directory automatically
cd /AutoSDV

# Build the workspace (takes 15-20 minutes)
make build
```

**What happens during build:**
1. Checks for seyond SDK and rebuilds if needed for ARM64
2. Sources ROS 2 Humble environment
3. Detects ZED SDK at `/usr/local/zed`
4. Builds all ROS 2 packages with colcon
5. Creates `install/`, `build/`, and `log/` directories

**Expected output:**
```
--- Rebuilding seyond SDK for ARM64 (if needed) ---
--- Sourcing ROS environment ---
--- Starting colcon build with ZED SDK path ---
ZED SDK found at /usr/local/zed
Building with ZED SDK 5.0 (compatible with zed-ros2-wrapper humble-v5.0.0)
...
Summary: X packages finished [time]
```

---

## Step 7: Launch AutoSDV

### For Testing Without Hardware (Simulation Mode)

```bash
# Inside container at /AutoSDV
make launch-sim
```

**Simulation mode disables:**
- ❌ RViz (no GUI)
- ❌ Sensor drivers (no LiDAR/Camera/GPS)
- ❌ Vehicle interface (no GPIO/motors)
- ❌ Perception (no TensorRT/GPU)

**What runs:**
- ✅ Autoware core nodes
- ✅ Planning and control
- ✅ Localization (without sensors)
- ✅ Map server

### For Real Hardware (Hardware Mode)

```bash
# Inside container at /AutoSDV
make launch-hw
```

**Hardware mode enables:**
- ✅ RViz visualization
- ✅ All sensor drivers (ZED camera, LiDAR, GPS, IMU)
- ✅ Vehicle interface (Jetson.GPIO)
- ✅ Perception (TensorRT object detection)

### Auto-Detection Mode

```bash
# Let the system detect if you're on real Jetson hardware
make launch
```

This checks for:
- NVIDIA device tree (`/proc/device-tree/model`)
- Tegra platform devices
- GPIO hardware
- L4T release file

---

## Step 8: Verify System is Running

### Check Running Nodes

```bash
# Open another terminal on host
cd ~/autosdv-workspace/AutoSDV-containerization/docker
make run  # This will attach to running container

# Inside container, in new shell:
source install/setup.bash
ros2 node list
```

### Check Topics

```bash
source install/setup.bash
ros2 topic list
ros2 topic echo /tf  # Check transforms
```

### Monitor Logs

```bash
# Watch all logs
ros2 topic echo /diagnostics

# Or check specific nodes
ros2 node info /autoware_state_machine
```

---

## Troubleshooting

### Container Won't Start

```bash
# Check if old container exists
docker ps -a | grep autosdv

# Remove old container
docker rm -f autosdv-container

# Try again
cd docker && make run
```

### Build Fails

```bash
# Clean and rebuild
cd /AutoSDV
rm -rf build install log
make build
```

### GPU Not Accessible (Jetson)

```bash
# Verify NVIDIA runtime
docker run --rm --runtime=nvidia nvcr.io/nvidia/l4t-base:r36.3.0 nvidia-smi

# If fails, reconfigure toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### Permission Denied Errors

```bash
# On host
sudo chown -R $USER:$USER ~/autosdv-workspace/AutoSDV-containerization

# Ensure user in docker group
groups | grep docker
# If not present:
sudo usermod -aG docker $USER
newgrp docker
```

---

## Quick Reference Commands

### On Host Machine

```bash
# Navigate to docker directory
cd ~/autosdv-workspace/AutoSDV-containerization/docker

# Enter container
make run

# Pull latest image
docker pull misuhsieh001/autosdv:2025.11-latest

# Remove container
docker rm -f autosdv-container

# Remove image
docker rmi misuhsieh001/autosdv:2025.11-latest
```

### Inside Container

```bash
# Build workspace
cd /AutoSDV && make build

# Launch simulation
make launch-sim

# Launch with hardware
make launch-hw

# Auto-detect and launch
make launch

# Clean build
rm -rf build install log
```

---

## System Requirements

### Minimum
- **CPU:** ARM64 quad-core 1.5 GHz
- **RAM:** 8 GB
- **Storage:** 50 GB free
- **OS:** Ubuntu 22.04 ARM64

### Recommended (Jetson Orin)
- **CPU:** ARM Cortex-A78AE (8-core)
- **RAM:** 32 GB
- **GPU:** NVIDIA Ampere (1024-2048 CUDA cores)
- **Storage:** 128 GB NVMe SSD
- **OS:** JetPack 6.0 (L4T 36.3)

---

## Network Configuration

### For Multi-Device Setup (Optional)

If running multiple devices (Jetson + workstation):

```bash
# Inside container, edit cyclonedds.xml
nano /AutoSDV/cyclonedds.xml

# Change from loopback to actual network interface
# Replace:
#   <NetworkInterface name="lo" multicast="false"/>
# With:
#   <NetworkInterface name="eth0" multicast="false"/>
# Or your actual interface (check with: ip addr)
```

---

## Data Persistence

### Important Directories

| Directory | Location | Purpose |
|-----------|----------|---------|
| Source Code | `/AutoSDV/src/` | ROS 2 packages (volume-mounted) |
| Build Artifacts | `/AutoSDV/build/` | Compiled code (volume-mounted) |
| Install Space | `/AutoSDV/install/` | Installed packages (volume-mounted) |
| Logs | `/AutoSDV/log/` | Build and runtime logs (volume-mounted) |
| ZED SDK | `/usr/local/zed` | Inside container only |

**Note:** All `/AutoSDV` content is volume-mounted from host, so changes persist.

---

## Updating to New Version

```bash
# On host
cd ~/autosdv-workspace/AutoSDV-containerization

# Pull latest code
git pull

# Pull latest Docker image
docker pull misuhsieh001/autosdv:2025.11-latest

# Remove old container
docker rm -f autosdv-container

# Create new container
cd docker && make run

# Inside new container, rebuild
cd /AutoSDV
rm -rf build install log
make build
```

---

## Support

- **Repository:** https://github.com/misuhsieh001/AutoSDV-containerization
- **DockerHub:** https://hub.docker.com/r/misuhsieh001/autosdv
- **Issues:** https://github.com/misuhsieh001/AutoSDV-containerization/issues
- **Documentation:** https://newslabntu.github.io/autosdv-book/

---

**Last Updated:** November 19, 2025  
**Version:** 2025.11-v1.0  
**Platform:** ARM64 (Jetson Orin, Generic ARM64)
