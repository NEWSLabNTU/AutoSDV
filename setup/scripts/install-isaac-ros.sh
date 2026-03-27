#!/usr/bin/env bash
# Install NVIDIA Isaac ROS packages for Visual SLAM and Global Localization
# Requires: NVIDIA GPU + ROS 2 Humble (supports both x86_64 and ARM64/Jetson)

set -eo pipefail

# Colors
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m'

# Check ROS 2 is installed
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    printf "${RED}Error:${NC} ROS 2 Humble not found at /opt/ros/humble\n"
    printf "Install ROS 2 first: ./setup.sh ros2\n"
    exit 1
fi

printf "${YELLOW}→${NC} Setting up NVIDIA Isaac ROS repository...\n"

# Repository configuration
REPO_FILE="/etc/apt/sources.list.d/nvidia-isaac-ros.list"
KEYRING="/usr/share/keyrings/nvidia-isaac-ros.gpg"
CODENAME=$(lsb_release -cs)
EXPECTED_REPO="deb [signed-by=${KEYRING}] https://isaac.download.nvidia.com/isaac-ros/release-3 ${CODENAME} release-3.0"

# Check for conflicting repository configurations
CONFLICTING_FILES=()
for f in /etc/apt/sources.list.d/isaac*.list /etc/apt/sources.list.d/*isaac*.list; do
    [[ -f "$f" ]] && [[ "$f" != "$REPO_FILE" ]] && CONFLICTING_FILES+=("$f")
done

# Check if existing repo file has wrong format
if [[ -f "$REPO_FILE" ]] && ! grep -qF "$EXPECTED_REPO" "$REPO_FILE"; then
    CONFLICTING_FILES+=("$REPO_FILE (wrong format)")
fi

if [[ ${#CONFLICTING_FILES[@]} -gt 0 ]]; then
    printf "${RED}Error:${NC} Found conflicting Isaac ROS repository configurations:\n"
    for f in "${CONFLICTING_FILES[@]}"; do
        printf "  - %s\n" "$f"
    done
    printf "\nPlease review and remove conflicting files:\n"
    printf "  sudo rm -f /etc/apt/sources.list.d/isaac*.list\n"
    printf "\nThen run this script again.\n"
    exit 1
fi

# Install prerequisites
sudo apt-get install -y curl gnupg lsb-release

# Add GPG key to keyring (modern method)
printf "${YELLOW}→${NC} Configuring GPG key...\n"
if [[ ! -f "${KEYRING}" ]]; then
    curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | \
        sudo gpg --dearmor -o "${KEYRING}"
    sudo chmod 644 "${KEYRING}"
fi

# Add repository with correct format: deb [signed-by=...] URL CODENAME COMPONENT
printf "${YELLOW}→${NC} Configuring APT repository...\n"
echo "${EXPECTED_REPO}" | sudo tee "${REPO_FILE}" > /dev/null

printf "${YELLOW}→${NC} Updating package lists...\n"
sudo apt-get update
printf "${GREEN}✓${NC} Isaac ROS repository configured\n"

printf "${YELLOW}→${NC} Installing Isaac ROS packages...\n"

# Install Isaac ROS Visual SLAM and image processing packages.
# Note: ros-humble-isaac-ros-visual-global-localization is NOT installed here
# because it pulls in ros-humble-isaac-ros-data-replayer which depends on
# ros-humble-foxglove-bridge (not available for Humble). The visual global
# localization package is built from source via the autoware_isaac_localization
# submodule in src/localization/ instead.
sudo apt-get install -y \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-image-proc

# Verify installation
printf "${YELLOW}→${NC} Verifying installation...\n"

PACKAGES=(
    "isaac_ros_visual_slam"
    "isaac_ros_image_proc"
)

for pkg in "${PACKAGES[@]}"; do
    if bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list 2>/dev/null | grep -q \"$pkg\""; then
        printf "${GREEN}✓${NC} $pkg installed\n"
    else
        printf "${RED}✗${NC} $pkg not found\n"
        exit 1
    fi
done

printf "${GREEN}✓${NC} Isaac ROS installation complete\n"
printf "\n"
printf "Note: isaac_ros_visual_global_localization is built from source\n"
printf "      (src/localization/autoware_isaac_localization) via 'just build'\n"
printf "\nUsage:\n"
printf "  pose_source:=isaac   - Visual odometry only (requires manual init)\n"
printf "  pose_source:=visual  - Full visual localization (auto init + tracking)\n"
