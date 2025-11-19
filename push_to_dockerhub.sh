#!/bin/bash
#
# AutoSDV DockerHub Push Script
# This script helps you push your AutoSDV image to DockerHub
#

set -e

# Configuration
IMAGE_NAME="autosdv"
IMAGE_TAG="be3c163"
VERSION="2025.11-v1.0"
LATEST_TAG="2025.11-latest"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "╔══════════════════════════════════════════════════════════════════════════════╗"
echo "║                  AutoSDV DockerHub Push Script                               ║"
echo "╚══════════════════════════════════════════════════════════════════════════════╝"
echo ""

# Get DockerHub username
read -p "Enter your DockerHub username: " DOCKERHUB_USERNAME

if [ -z "$DOCKERHUB_USERNAME" ]; then
    echo -e "${RED}Error: DockerHub username cannot be empty${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}Using DockerHub username: $DOCKERHUB_USERNAME${NC}"
echo ""

# Step 1: Login
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "STEP 1: Login to DockerHub"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

docker login
if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Docker login failed${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}✓ Login successful${NC}"
echo ""

# Step 2: Check if source image exists
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "STEP 2: Verify source image"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

if ! docker image inspect ${IMAGE_NAME}:${IMAGE_TAG} > /dev/null 2>&1; then
    echo -e "${RED}Error: Source image ${IMAGE_NAME}:${IMAGE_TAG} not found${NC}"
    echo "Available images:"
    docker images | grep autosdv
    exit 1
fi

IMAGE_SIZE=$(docker image inspect ${IMAGE_NAME}:${IMAGE_TAG} --format='{{.Size}}' | awk '{print $1/1024/1024/1024}')
echo -e "${GREEN}✓ Source image found: ${IMAGE_NAME}:${IMAGE_TAG}${NC}"
echo -e "  Size: ${IMAGE_SIZE} GB"
echo ""

# Step 3: Tag images
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "STEP 3: Tag images for DockerHub"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

echo "Tagging: ${DOCKERHUB_USERNAME}/autosdv:${LATEST_TAG}"
docker tag ${IMAGE_NAME}:${IMAGE_TAG} ${DOCKERHUB_USERNAME}/autosdv:${LATEST_TAG}

echo "Tagging: ${DOCKERHUB_USERNAME}/autosdv:${VERSION}"
docker tag ${IMAGE_NAME}:${IMAGE_TAG} ${DOCKERHUB_USERNAME}/autosdv:${VERSION}

echo ""
echo -e "${GREEN}✓ Images tagged successfully${NC}"
echo ""

# Step 4: Push images
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "STEP 4: Push to DockerHub"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo -e "${YELLOW}⚠️  This will upload ~${IMAGE_SIZE} GB to DockerHub${NC}"
echo -e "${YELLOW}⚠️  Estimated time: 25-60 minutes (depending on your upload speed)${NC}"
echo ""
read -p "Continue with push? (y/n) " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Push cancelled."
    exit 0
fi

echo ""
echo "Pushing: ${DOCKERHUB_USERNAME}/autosdv:${LATEST_TAG}"
docker push ${DOCKERHUB_USERNAME}/autosdv:${LATEST_TAG}

echo ""
echo "Pushing: ${DOCKERHUB_USERNAME}/autosdv:${VERSION}"
docker push ${DOCKERHUB_USERNAME}/autosdv:${VERSION}

# Step 5: Success
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${GREEN}✅ SUCCESS! Images pushed to DockerHub${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Your images are now available at:"
echo "  • https://hub.docker.com/r/${DOCKERHUB_USERNAME}/autosdv"
echo ""
echo "To pull on your Jetson devices:"
echo "  docker pull ${DOCKERHUB_USERNAME}/autosdv:${LATEST_TAG}"
echo ""
echo "Next steps:"
echo "  1. SSH to each Jetson Orin"
echo "  2. Pull the image (command above)"
echo "  3. Run the container (see DOCKERHUB_DEPLOYMENT.md)"
echo "  4. Execute: cd /AutoSDV && make launch"
echo ""

