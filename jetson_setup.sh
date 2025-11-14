#!/bin/bash
#
# AutoSDV Jetson Setup Script
# Use this script on each Jetson Orin to set up AutoSDV
#

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "╔══════════════════════════════════════════════════════════════════════════════╗"
echo "║              AutoSDV Jetson Setup Script                                    ║"
echo "╚══════════════════════════════════════════════════════════════════════════════╝"
echo ""

# Configuration
REPO_URL="https://github.com/misuhsieh001/AutoSDV-containerization.git"
REPO_BRANCH="2025.11-containerization"

echo "Repository: $REPO_URL"
echo "Branch: $REPO_BRANCH"
echo ""

read -p "Enter DockerHub username (default: misuhsieh001): " DOCKERHUB_USERNAME
DOCKERHUB_USERNAME="${DOCKERHUB_USERNAME:-misuhsieh001}"

if [ -z "$REPO_URL" ] || [ -z "$DOCKERHUB_USERNAME" ]; then
    echo -e "${RED}Error: Repository URL and DockerHub username are required${NC}"
    exit 1
fi

DOCKER_IMAGE="${DOCKERHUB_USERNAME}/autosdv:2025.02-latest"
WORKSPACE_DIR="$HOME/AutoSDV"

echo ""
echo -e "${GREEN}Configuration:${NC}"
echo "  Repository: $REPO_URL"
echo "  Docker Image: $DOCKER_IMAGE"
echo "  Workspace: $WORKSPACE_DIR"
echo ""

# Step 1: Clone repository
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${BLUE}Step 1: Cloning AutoSDV repository...${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ -d "$WORKSPACE_DIR" ]; then
    echo -e "${YELLOW}Warning: $WORKSPACE_DIR already exists${NC}"
    read -p "Remove and re-clone? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$WORKSPACE_DIR"
    else
        echo "Keeping existing directory"
    fi
fi

if [ ! -d "$WORKSPACE_DIR" ]; then
    git clone -b "$REPO_BRANCH" "$REPO_URL" "$WORKSPACE_DIR"
    echo -e "${GREEN}✓ Repository cloned (branch: $REPO_BRANCH)${NC}"
else
    echo -e "${GREEN}✓ Using existing repository${NC}"
fi

cd "$WORKSPACE_DIR"
echo ""

# Step 2: Pull Docker image
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${BLUE}Step 2: Pulling Docker image...${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${YELLOW}This will download ~19 GB. Please be patient...${NC}"
echo ""

docker pull "$DOCKER_IMAGE"
echo ""
echo -e "${GREEN}✓ Docker image pulled${NC}"
echo ""

# Step 3: Create run script
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${BLUE}Step 3: Creating run script...${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

cat > "$WORKSPACE_DIR/run_container.sh" << 'RUNSCRIPT'
#!/bin/bash
# AutoSDV Container Run Script

DOCKER_IMAGE="DOCKER_IMAGE_PLACEHOLDER"
CONTAINER_NAME="autosdv_container"

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container '${CONTAINER_NAME}' already exists."
    read -p "Remove and recreate? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker rm -f "${CONTAINER_NAME}"
    else
        echo "Starting existing container..."
        docker start -ai "${CONTAINER_NAME}"
        exit 0
    fi
fi

# Run new container
echo "Creating and starting container..."
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
RUNSCRIPT

# Replace placeholder with actual image name
sed -i "s|DOCKER_IMAGE_PLACEHOLDER|$DOCKER_IMAGE|g" "$WORKSPACE_DIR/run_container.sh"
chmod +x "$WORKSPACE_DIR/run_container.sh"

echo -e "${GREEN}✓ Run script created: $WORKSPACE_DIR/run_container.sh${NC}"
echo ""

# Step 4: Instructions
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${GREEN}✅ SETUP COMPLETE!${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Next steps:"
echo ""
echo "1. Start the container:"
echo -e "   ${BLUE}cd $WORKSPACE_DIR${NC}"
echo -e "   ${BLUE}./run_container.sh${NC}"
echo ""
echo "2. Inside the container, build the workspace (FIRST TIME ONLY):"
echo -e "   ${BLUE}cd /AutoSDV${NC}"
echo -e "   ${BLUE}make build${NC}     ${YELLOW}(15-20 minutes)${NC}"
echo ""
echo "3. Launch AutoSDV:"
echo -e "   ${BLUE}make launch${NC}"
echo ""
echo "For subsequent uses, just run:"
echo -e "   ${BLUE}./run_container.sh${NC}"
echo -e "   ${BLUE}cd /AutoSDV && make launch${NC}"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

