# GitHub + Docker Hybrid Deployment Guide

This guide explains how to deploy AutoSDV to multiple Jetson Orin devices using **GitHub for source code** and **DockerHub for the runtime environment**.

## Overview

**Strategy:** Separate concerns
- **GitHub**: Source code distribution and version control
- **DockerHub**: Pre-configured runtime environment (dependencies, ROS, ZED SDK, etc.)
- **Volume Mount**: Source code mounted into container at runtime

**Benefits:**
- ✅ Easy code updates (git pull)
- ✅ Version control for collaboration
- ✅ Smaller Docker image (~19 GB vs ~25 GB)
- ✅ No Docker rebuild for code changes
- ✅ Good balance for course projects

---

## Prerequisites

### On x86 PC:
- Docker with buildx (multi-architecture support)
- Git with GitHub account
- ~50 GB free disk space

### On Each Jetson:
- Docker installed
- Git installed
- NVIDIA Container Toolkit
- Internet connection
- ~50 GB free disk space

---

## Part 1: Setup on x86 PC

### Step 1: Fork Repository on GitHub

**⚠️ IMPORTANT:** The repository is currently pointing to the course's original repo (`NEWSLabNTU/AutoSDV`). You should NOT push your changes there!

1. **Fork the repository:**
   - Go to https://github.com/NEWSLabNTU/AutoSDV
   - Click "Fork" button (top right)
   - Your fork: https://github.com/YOUR_USERNAME/AutoSDV

2. **Set up your local repository (AUTOMATED):**

```bash
cd /home/misuhsieh/Documents/Graduate_Courses/04-Middleware_for_Software_Defined_Vehicle/Lab3/AutoSDV
./setup_git_fork.sh
```

This creates a new branch `lab3-deployment` and points your local repo to YOUR fork.

**Or manually:**
```bash
git checkout -b lab3-deployment
git remote rename origin upstream
git remote add origin https://github.com/YOUR_USERNAME/AutoSDV.git
```

### Step 2: Prepare Git Repository (Optional)

```bash
cd /home/misuhsieh/Documents/Graduate_Courses/04-Middleware_for_Software_Defined_Vehicle/Lab3/AutoSDV

# Create .gitignore if not exists
cat > .gitignore << 'EOF'
# Build artifacts
build/
install/
log/

# IDE files
.vscode/
.idea/

# Core dumps
*.core

# Temporary files
*.swp
*.swo
*~
.DS_Store

# Data files (too large for git)
*.bag
*.db3
*.pcd

# Keep empty directories
!.gitkeep
EOF

# Add all files
git add .

# Commit
git commit -m "Lab3: AutoSDV containerization with all fixes"

# Push to YOUR fork on the lab3-deployment branch
git push -u origin lab3-deployment
```

**Your changes are now safely in YOUR fork, not the course repo!**

### Step 2: Build Docker Image

```bash
cd docker
make build
```

**Expected time:** 45-60 minutes  
**Expected size:** ~19 GB

### Step 3: Test on x86 (Optional but Recommended)

```bash
cd docker
make run

# Inside container:
cd /AutoSDV
make build    # Build workspace
make launch   # Test in simulation mode
```

Verify:
- ✅ No crashes
- ✅ Simulation mode detected
- ✅ System runs stably

### Step 4: Push to DockerHub

```bash
cd ..  # Back to AutoSDV root
./push_to_dockerhub.sh
```

Or manually:
```bash
docker login
docker tag autosdv:HASH YOUR_USERNAME/autosdv:2025.02-latest
docker push YOUR_USERNAME/autosdv:2025.02-latest
```

**Expected time:** 25-60 minutes (depending on upload speed)

---

## Part 2: Setup on Each Jetson

### Option A: Automated Setup (Recommended)

**On x86 PC**, copy the setup script to Jetson:
```bash
scp jetson_setup.sh jetson@JETSON_IP:~/
```

**On Jetson**, run the setup:
```bash
chmod +x jetson_setup.sh
./jetson_setup.sh
```

The script will:
1. Clone the AutoSDV repository
2. Pull the Docker image
3. Create a run script

Then follow the on-screen instructions.

### Option B: Manual Setup

#### 1. Clone Repository

```bash
cd ~
git clone -b lab3-deployment https://github.com/YOUR_USERNAME/AutoSDV.git
cd AutoSDV
```

**Note:** We're cloning the `lab3-deployment` branch, which has all your fixes!

#### 2. Pull Docker Image

```bash
docker pull YOUR_USERNAME/autosdv:2025.02-latest
```

**Expected time:** 10-20 minutes (downloading ~19 GB)

#### 3. Create Run Script

```bash
cat > run_container.sh << 'EOF'
#!/bin/bash
DOCKER_IMAGE="YOUR_USERNAME/autosdv:2025.02-latest"
CONTAINER_NAME="autosdv_container"

# Check if container exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Starting existing container..."
    docker start -ai "${CONTAINER_NAME}"
    exit 0
fi

# Create new container
echo "Creating new container..."
docker run -it --name "${CONTAINER_NAME}" \
    --gpus all \
    --net host \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd):/AutoSDV:rw \
    -v /dev:/dev:rw \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    "${DOCKER_IMAGE}" /bin/bash
EOF

chmod +x run_container.sh
```

**Replace** `YOUR_USERNAME` with your actual DockerHub username!

#### 4. Start Container

```bash
./run_container.sh
```

#### 5. Build Workspace (First Time Only!)

```bash
# Inside container:
cd /AutoSDV
make build
```

**Expected time:** 15-20 minutes

#### 6. Launch AutoSDV

```bash
# Inside container:
cd /AutoSDV
make launch
```

The system should:
- ✅ Auto-detect Jetson hardware
- ✅ Enable perception modules
- ✅ Enable vehicle interface
- ✅ Launch RViz and all drivers

---

## Daily Usage

### Starting AutoSDV

```bash
# On Jetson host:
cd ~/AutoSDV
./run_container.sh

# Inside container (auto-sources workspace):
cd /AutoSDV
make launch
```

### Stopping AutoSDV

```bash
# Inside container:
Ctrl+C  # Stop ROS 2

# Exit container:
exit

# Container persists and can be restarted
```

---

## Updating Code

### Update on All Jetsons

```bash
# On Jetson host (outside container):
cd ~/AutoSDV
git pull

# If container is running, restart it:
docker restart autosdv_container
docker attach autosdv_container

# Inside container: Rebuild if code changed
cd /AutoSDV
make build  # Only if needed
make launch
```

### Push Updates from x86 PC

```bash
# Make your changes
git add .
git commit -m "Description of changes"
git push origin main

# No need to rebuild/repush Docker image!
# Unless you changed dependencies in Dockerfile
```

---

## Troubleshooting

### Issue: "Container already exists"

```bash
# Remove old container
docker rm -f autosdv_container

# Run again
./run_container.sh
```

### Issue: "Workspace not sourced"

```bash
# Inside container:
source /opt/ros/humble/setup.bash
source /AutoSDV/install/setup.bash
```

### Issue: "Permission denied on /dev"

```bash
# Ensure container runs with --privileged flag
# Check run_container.sh includes: --privileged
```

### Issue: Git pull fails on Jetson

```bash
# Set up Git credentials
git config --global credential.helper store
git pull  # Enter credentials once, then stored
```

### Issue: Need to rebuild Docker image

If you changed **dependencies** (Dockerfile), you need to:

```bash
# On x86 PC:
cd docker
make build
cd ..
./push_to_dockerhub.sh

# On each Jetson:
docker pull YOUR_USERNAME/autosdv:2025.02-latest
docker rm -f autosdv_container  # Remove old
./run_container.sh              # Create new
```

---

## Comparison with Other Approaches

### GitHub + Docker (This Approach)

**Pros:**
- Easy code updates
- Version control
- Smaller Docker images
- Good for teams

**Cons:**
- Per-device workspace build (15-20 min)
- Two-step setup

**Best for:** Course projects, development, teams

### Build-in-Docker

**Pros:**
- Pull and run immediately
- No per-device build
- Self-contained

**Cons:**
- Larger Docker image (~25 GB)
- Code changes need full rebuild
- Longer Docker push/pull

**Best for:** Production, many devices (10+)

### Volume-Mount Only (Current)

**Pros:**
- Simplest Dockerfile
- Very flexible

**Cons:**
- Need source on every device
- Need to remember to mount
- Not a true deployment

**Best for:** Single-device development

---

## Evaluation Checklist

After deploying to Jetson, verify:

- [ ] Container starts without errors
- [ ] Workspace sourced automatically
- [ ] Hardware auto-detection shows "Real Jetson hardware"
- [ ] Perception modules launch
- [ ] GPU utilization >0% (check with `jtop`)
- [ ] lidar_centerpoint node running
- [ ] Object detection topics publishing
- [ ] RViz displays correctly

See `EVALUATION_CHECKLIST.md` for detailed testing procedures.

---

## Summary

**This approach gives you:**
1. **Easy deployment**: One-time setup per Jetson (~30 min)
2. **Version control**: Git for source code management
3. **Fast updates**: Just `git pull`, no Docker rebuild
4. **Team friendly**: Everyone uses same environment
5. **Reasonable tradeoff**: Small per-device build time acceptable

**Perfect for your course project!** 🚀

---

**Created:** November 14, 2025  
**For:** AutoSDV Multi-Jetson Deployment  
**Approach:** GitHub (source) + DockerHub (environment)

