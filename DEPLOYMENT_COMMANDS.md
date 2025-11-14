# AutoSDV Deployment - Quick Reference

**Your Repository:** https://github.com/misuhsieh001/AutoSDV-containerization.git  
**Branch:** `lab3-deployment`  
**DockerHub:** `misuhsieh001/autosdv:2025.02-latest`

---

## Step 1: Push Code to GitHub (x86 PC)

```bash
cd /home/misuhsieh/Documents/Graduate_Courses/04-Middleware_for_Software_Defined_Vehicle/Lab3/AutoSDV

# Add your repository as remote
git remote add misuhsieh https://github.com/misuhsieh001/AutoSDV-containerization.git

# Create and switch to deployment branch
git checkout -b lab3-deployment

# Stage all changes
git add .

# Commit with message
git commit -m "Lab3: AutoSDV containerization with all fixes applied"

# Push to your repository
git push -u misuhsieh lab3-deployment
```

**Verification:**  
Visit: https://github.com/misuhsieh001/AutoSDV-containerization/tree/lab3-deployment

---

## Step 2: Build Docker Image (x86 PC)

```bash
cd /home/misuhsieh/Documents/Graduate_Courses/04-Middleware_for_Software_Defined_Vehicle/Lab3/AutoSDV/docker

# Build the image (~45-60 minutes)
make build

# Check the image
docker images | grep autosdv
```

---

## Step 3: Test in Simulation (x86 PC - Optional)

```bash
cd /home/misuhsieh/Documents/Graduate_Courses/04-Middleware_for_Software_Defined_Vehicle/Lab3/AutoSDV/docker

# Run container
make run

# Inside container:
cd /AutoSDV
make build    # Build workspace (15-20 min)
make launch   # Test (should auto-detect simulation mode)
```

---

## Step 4: Push to DockerHub (x86 PC)

```bash
cd /home/misuhsieh/Documents/Graduate_Courses/04-Middleware_for_Software_Defined_Vehicle/Lab3/AutoSDV

# Run the push script
./push_to_dockerhub.sh

# Enter: misuhsieh001 (when prompted for username)
```

**Estimated time:** 25-60 minutes (uploading ~19 GB)

**Verification:**  
Visit: https://hub.docker.com/r/misuhsieh001/autosdv

---

## Step 5: Deploy to Each Jetson

### Option A: Automated (Recommended)

```bash
# Copy setup script to Jetson
scp jetson_setup.sh jetson@JETSON_IP:~/

# On Jetson, run:
./jetson_setup.sh
# Press Enter to use defaults
# Follow on-screen instructions
```

### Option B: Manual

```bash
# On Jetson:

# 1. Clone repository
cd ~
git clone -b lab3-deployment https://github.com/misuhsieh001/AutoSDV-containerization.git
cd AutoSDV-containerization

# 2. Pull Docker image
docker pull misuhsieh001/autosdv:2025.02-latest

# 3. Create run script
cat > run_container.sh << 'SCRIPT'
#!/bin/bash
docker run -it --name autosdv_container \
    --gpus all --net host --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd):/AutoSDV:rw \
    -v /dev:/dev:rw \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    misuhsieh001/autosdv:2025.02-latest /bin/bash
SCRIPT

chmod +x run_container.sh

# 4. Run container
./run_container.sh

# 5. Inside container: Build workspace (FIRST TIME ONLY)
cd /AutoSDV
make build    # 15-20 minutes

# 6. Launch AutoSDV
make launch   # Auto-detects hardware!
```

---

## Daily Usage (Jetson)

```bash
# Start container
cd ~/AutoSDV-containerization
./run_container.sh

# Inside container (auto-sources workspace):
make launch
```

---

## Update Code (All Devices)

### Update on x86 PC:

```bash
cd /home/misuhsieh/.../AutoSDV

# Make changes...

git add .
git commit -m "Description of changes"
git push misuhsieh lab3-deployment
```

### Update on Jetsons:

```bash
# On Jetson host:
cd ~/AutoSDV-containerization
git pull

# If container is running:
docker restart autosdv_container
docker attach autosdv_container

# Inside container: Rebuild if needed
cd /AutoSDV
make build
make launch
```

---

## Troubleshooting

### Check Docker Image

```bash
docker images | grep misuhsieh001
```

### Check Repository Status

```bash
cd /home/misuhsieh/.../AutoSDV
git remote -v
git branch -a
git status
```

### Remove and Recreate Container

```bash
docker rm -f autosdv_container
./run_container.sh
```

### Verify Hardware Detection

```bash
# Inside container:
make launch

# Should show:
# - x86: "Non-Jetson environment detected - using simulation mode"
# - Jetson: "Real Jetson hardware detected"
```

---

## Key URLs

- **GitHub Repository:** https://github.com/misuhsieh001/AutoSDV-containerization
- **GitHub Branch:** https://github.com/misuhsieh001/AutoSDV-containerization/tree/lab3-deployment
- **DockerHub:** https://hub.docker.com/r/misuhsieh001/autosdv
- **Docker Image:** `misuhsieh001/autosdv:2025.02-latest`

---

**Last Updated:** November 14, 2025  
**For:** AutoSDV Lab3 - Containerization & Multi-Jetson Deployment
