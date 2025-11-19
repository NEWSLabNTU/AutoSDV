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

DOCKER_IMAGE="${DOCKERHUB_USERNAME}/autosdv:2025.11-latest"
WORKSPACE_DIR="$HOME/AutoSDV-containerization"

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
    read -p "Update existing repository? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        cd "$WORKSPACE_DIR"
        git fetch --all
        git checkout "$REPO_BRANCH"
        git pull
        echo "Updating submodules..."
        git submodule update --init --recursive
        echo -e "${GREEN}✓ Repository updated to latest $REPO_BRANCH${NC}"
    else
        cd "$WORKSPACE_DIR"
        echo -e "${GREEN}✓ Using existing repository${NC}"
    fi
else
    git clone --recursive -b "$REPO_BRANCH" "$REPO_URL" "$WORKSPACE_DIR"
    cd "$WORKSPACE_DIR"
    echo "Initializing all submodules (including nested)..."
    git submodule update --init --recursive
    echo -e "${GREEN}✓ Repository cloned (branch: $REPO_BRANCH) with all submodules${NC}"
fi
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

# Step 3: Verify Makefile setup
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${BLUE}Step 3: Verifying setup...${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ -f "$WORKSPACE_DIR/docker/Makefile" ]; then
    echo -e "${GREEN}✓ Docker Makefile found${NC}"
else
    echo -e "${RED}✗ Docker Makefile not found!${NC}"
    exit 1
fi

echo ""

# Step 4: Instructions
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${GREEN}✅ SETUP COMPLETE!${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Next steps:"
echo ""
echo "1. Start the container:"
echo -e "   ${BLUE}cd $WORKSPACE_DIR/docker${NC}"
echo -e "   ${BLUE}make run${NC}"
echo ""
echo "2. Inside the container, build the workspace (FIRST TIME ONLY):"
echo -e "   ${BLUE}cd /AutoSDV${NC}"
echo -e "   ${BLUE}make build${NC}     ${YELLOW}(15-20 minutes)${NC}"
echo ""
echo "3. Launch AutoSDV (auto-detects hardware):"
echo -e "   ${BLUE}make launch${NC}     ${YELLOW}(or 'make launch-sim' / 'make launch-hw')${NC}"
echo ""
echo "For subsequent uses, just run:"
echo -e "   ${BLUE}cd $WORKSPACE_DIR/docker && make run${NC}"
echo -e "   ${BLUE}cd /AutoSDV && make launch${NC}"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

