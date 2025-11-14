# AutoSDV DockerHub Deployment Guide

## Overview
Deploy AutoSDV to multiple Jetson Orin devices using DockerHub as the central registry.

---

## 📋 Prerequisites

1. **DockerHub Account**: Create one at https://hub.docker.com if you don't have one
2. **Current Image**: `autosdv:4af9537` (19.4GB, built on x86 with ARM64 support)
3. **Target Devices**: Multiple Jetson Orin ARM64 machines

---

## 🚀 Part 1: Push Image to DockerHub (On x86 PC)

### Step 1: Login to DockerHub

```bash
docker login
# Enter your DockerHub username and password
```

### Step 2: Tag Your Image

```bash
# Replace 'yourusername' with your actual DockerHub username
docker tag autosdv:4af9537 yourusername/autosdv:2025.02-latest

# Optional: Also tag with specific version for rollback capability
docker tag autosdv:4af9537 yourusername/autosdv:2025.02-v1.0
```

**Tag naming convention:**
- `latest` - Always points to the newest stable version
- `2025.02-latest` - Latest for Autoware 2025.02 branch
- `2025.02-v1.0` - Specific version for rollback

### Step 3: Push to DockerHub

```bash
# Push the latest tag (this will take time - 19.4GB upload!)
docker push yourusername/autosdv:2025.02-latest

# Push the versioned tag (reuses layers, much faster)
docker push yourusername/autosdv:2025.02-v1.0
```

**Expected upload time:**
- Upload speed 10 Mbps: ~4-5 hours
- Upload speed 50 Mbps: ~50-60 minutes
- Upload speed 100 Mbps: ~25-30 minutes

**💡 Tip**: Start the upload before leaving for the day!

---

## 📥 Part 2: Pull and Run on Jetson Devices

### On Each Jetson Orin Device:

#### Step 1: Login to DockerHub (if image is private)

```bash
docker login
```

#### Step 2: Pull the Image

```bash
# Pull the latest version
docker pull yourusername/autosdv:2025.02-latest

# Or pull a specific version
docker pull yourusername/autosdv:2025.02-v1.0
```

**Expected download time on Jetson:**
- Download speed 10 Mbps: ~4-5 hours
- Download speed 50 Mbps: ~50-60 minutes
- Download speed 100 Mbps: ~25-30 minutes

#### Step 3: Create and Run Container

```bash
# Create a run script for convenience
cat > ~/run_autosdv.sh << 'EOF'
#!/bin/bash

CONTAINER_NAME="autosdv_container"
IMAGE_NAME="yourusername/autosdv:2025.02-latest"  # Replace with your username

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "Attaching to running container ${CONTAINER_NAME}..."
        docker attach ${CONTAINER_NAME}
    else
        echo "Starting existing container ${CONTAINER_NAME}..."
        docker start -ai ${CONTAINER_NAME}
    fi
else
    echo "Creating new container ${CONTAINER_NAME}..."
    docker run -it --name ${CONTAINER_NAME} \
        --gpus all \
        --net host \
        --privileged \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v $HOME/.Xauthority:/root/.Xauthority:rw \
        -v /dev:/dev:rw \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        ${IMAGE_NAME} /bin/bash
fi
EOF

chmod +x ~/run_autosdv.sh
```

**Replace `yourusername` with your actual DockerHub username!**

#### Step 4: Launch AutoSDV

```bash
# Run the container
~/run_autosdv.sh

# Inside container - it will auto-detect Jetson hardware!
cd /AutoSDV
make launch

# Expected output:
# Detecting hardware environment...
# 🤖 Real Jetson hardware detected (device tree found)!
# 🚗 Launching on HARDWARE (Jetson Orin with sensors)
```

---

## 🔄 Updating Multiple Devices

### When you make changes and want to update all Jetsons:

#### On x86 PC (Build & Push):
```bash
# 1. Rebuild the image with changes
cd docker
make build

# 2. Tag with new version
docker tag autosdv:NEW_HASH yourusername/autosdv:2025.02-latest
docker tag autosdv:NEW_HASH yourusername/autosdv:2025.02-v1.1  # Increment version

# 3. Push updates
docker push yourusername/autosdv:2025.02-latest
docker push yourusername/autosdv:2025.02-v1.1
```

#### On Each Jetson (Pull & Restart):
```bash
# 1. Stop and remove old container
docker stop autosdv_container
docker rm autosdv_container

# 2. Pull latest image
docker pull yourusername/autosdv:2025.02-latest

# 3. Run new container
~/run_autosdv.sh
```

---

## 🎯 Quick Deploy Script for New Jetson Devices

Save this on your x86 PC to quickly set up new Jetsons:

```bash
# File: setup_new_jetson.sh
#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: ./setup_new_jetson.sh <jetson-ip>"
    exit 1
fi

JETSON_IP=$1
DOCKERHUB_USER="yourusername"  # Replace with your username

echo "🚀 Setting up AutoSDV on Jetson at ${JETSON_IP}..."

# Copy run script to Jetson
ssh jetson@${JETSON_IP} "cat > ~/run_autosdv.sh" << 'EOF'
#!/bin/bash
CONTAINER_NAME="autosdv_container"
IMAGE_NAME="yourusername/autosdv:2025.02-latest"

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        docker attach ${CONTAINER_NAME}
    else
        docker start -ai ${CONTAINER_NAME}
    fi
else
    docker run -it --name ${CONTAINER_NAME} \
        --gpus all --net host --privileged \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v $HOME/.Xauthority:/root/.Xauthority:rw \
        -v /dev:/dev:rw \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        ${IMAGE_NAME} /bin/bash
fi
EOF

# Make script executable
ssh jetson@${JETSON_IP} "chmod +x ~/run_autosdv.sh"

# Pull Docker image
echo "📥 Pulling Docker image on Jetson (this may take a while)..."
ssh jetson@${JETSON_IP} "docker pull ${DOCKERHUB_USER}/autosdv:2025.02-latest"

echo "✅ Setup complete! To launch AutoSDV on Jetson:"
echo "   ssh jetson@${JETSON_IP}"
echo "   ~/run_autosdv.sh"
```

**Usage:**
```bash
chmod +x setup_new_jetson.sh
./setup_new_jetson.sh 192.168.1.10   # Replace with Jetson IP
./setup_new_jetson.sh 192.168.1.11   # Another Jetson
./setup_new_jetson.sh 192.168.1.12   # And another...
```

---

## 🔒 Making Your Image Private (Optional)

If your code is proprietary:

1. Go to https://hub.docker.com
2. Navigate to your repository: `yourusername/autosdv`
3. Click **Settings** → **Make Private**
4. All devices will need to `docker login` before pulling

---

## 📊 Image Size Optimization Tips

Your current image is **19.4GB**. To reduce it (optional):

### Option 1: Multi-stage Build
```dockerfile
# Use builder pattern to exclude build tools from final image
FROM autosdv_base AS builder
# ... build steps ...

FROM autosdv_base
COPY --from=builder /AutoSDV/install /AutoSDV/install
# Final image only has runtime dependencies
```

### Option 2: Clean Up Build Artifacts
```bash
# Add to Dockerfile before final layer:
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* && \
    rm -rf /AutoSDV/build
```

---

## 🛠️ Troubleshooting

### Issue: "unauthorized: authentication required"
**Solution:**
```bash
docker login
# Enter credentials and try again
```

### Issue: "no matching manifest for linux/arm64"
**Solution:** Verify image was built for ARM64:
```bash
docker inspect autosdv:4af9537 | grep Architecture
# Should show: "Architecture": "arm64"
```

### Issue: Push/Pull is very slow
**Solutions:**
- Use compression: `docker save autosdv:4af9537 | gzip | docker load` (local transfer)
- Consider private registry on local network (faster than DockerHub)
- Use `--compress` flag with docker push (enabled by default)

### Issue: "Image too large" error on DockerHub
**Solution:**
- Free tier has limits - upgrade to paid plan if needed
- Or use alternative registry: AWS ECR, Google GCR, Harbor (self-hosted)

---

## 📝 Best Practices

1. **Always tag versions**: Never rely only on `latest` - version your images!
2. **Test before pushing**: Verify image works in simulation before pushing
3. **Document changes**: Use git tags that match Docker image tags
4. **Backup**: Keep local copies of working images
5. **Security**: Use secrets for sensitive data, not ENV vars in Dockerfile

---

## 🎯 Summary of Commands

```bash
# On x86 PC (One-time setup):
docker login
docker tag autosdv:4af9537 yourusername/autosdv:2025.02-latest
docker push yourusername/autosdv:2025.02-latest

# On each Jetson (One-time setup):
docker login
docker pull yourusername/autosdv:2025.02-latest
~/run_autosdv.sh

# Inside container (every time):
cd /AutoSDV
make launch  # Auto-detects hardware mode!
```

---

## 📚 Additional Resources

- DockerHub Documentation: https://docs.docker.com/docker-hub/
- Docker ARM64 Guide: https://docs.docker.com/build/building/multi-platform/
- Jetson Docker Best Practices: https://docs.nvidia.com/deeplearning/frameworks/user-guide/

---

**Last Updated**: November 14, 2025
**Image Version**: 2025.02-v1.0 (includes all fixes for crash-free operation)

