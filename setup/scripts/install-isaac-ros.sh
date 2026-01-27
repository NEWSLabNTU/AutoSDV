#!/usr/bin/env bash
# Install NVIDIA Isaac ROS packages for Visual SLAM
# Requires: ARM64 (Jetson) with JetPack 6.x and ROS 2 Humble

set -eo pipefail

# Colors
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m'

# Check architecture
if [[ "$(uname -m)" != "aarch64" ]]; then
    printf "${RED}Error:${NC} Isaac ROS APT packages only available for ARM64 (Jetson)\n"
    printf "For x86_64, build from source: https://nvidia-isaac-ros.github.io\n"
    exit 1
fi

# Check ROS 2 is installed
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    printf "${RED}Error:${NC} ROS 2 Humble not found at /opt/ros/humble\n"
    printf "Install ROS 2 first: ./setup.sh ros2\n"
    exit 1
fi

printf "${YELLOW}→${NC} Setting up NVIDIA Isaac ROS repository...\n"

# Add or fix NVIDIA Isaac ROS repository configuration
REPO_FILE="/etc/apt/sources.list.d/nvidia-isaac-ros.list"
EXPECTED_REPO="deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-3 jammy/"

if [[ ! -f "$REPO_FILE" ]] || ! grep -qF "$EXPECTED_REPO" "$REPO_FILE"; then
    sudo apt-get update
    sudo apt-get install -y curl gnupg

    # Add GPG key if not present
    if [[ ! -f /usr/share/keyrings/nvidia-isaac-ros.gpg ]]; then
        curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | \
            sudo gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg
    fi

    # Add repository (release-3 for Isaac ROS 3.x compatible with JetPack 6.x)
    # Note: No component specified - the repo uses flat structure
    echo "$EXPECTED_REPO" | sudo tee "$REPO_FILE"

    sudo apt-get update
    printf "${GREEN}✓${NC} Isaac ROS repository configured\n"
else
    printf "${GREEN}✓${NC} Isaac ROS repository already configured\n"
fi

printf "${YELLOW}→${NC} Installing Isaac ROS Visual SLAM packages...\n"

# Install Isaac ROS Visual SLAM packages
sudo apt-get install -y \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-image-proc

# Verify installation
printf "${YELLOW}→${NC} Verifying installation...\n"

# Verify packages using a subshell to avoid environment issues
if bash -c 'source /opt/ros/humble/setup.bash && ros2 pkg list 2>/dev/null | grep -q "isaac_ros_visual_slam"'; then
    printf "${GREEN}✓${NC} isaac_ros_visual_slam installed\n"
else
    printf "${RED}✗${NC} isaac_ros_visual_slam not found\n"
    exit 1
fi

if bash -c 'source /opt/ros/humble/setup.bash && ros2 pkg list 2>/dev/null | grep -q "isaac_ros_image_proc"'; then
    printf "${GREEN}✓${NC} isaac_ros_image_proc installed\n"
else
    printf "${RED}✗${NC} isaac_ros_image_proc not found\n"
    exit 1
fi

printf "${GREEN}✓${NC} Isaac ROS Visual SLAM installation complete\n"
printf "\nUsage: just launch ARGS=\"pose_source:=isaac\"\n"
