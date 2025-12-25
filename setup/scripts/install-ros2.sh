#!/usr/bin/env bash
# Install ROS 2 Humble
# Converted from ansible/roles/ros2/tasks/main.yaml

set -e

ROSDISTRO="${ROSDISTRO:-humble}"
ROS2_INSTALLATION_TYPE="${ROS2_INSTALLATION_TYPE:-desktop}"

echo "Installing ROS 2 ${ROSDISTRO} (${ROS2_INSTALLATION_TYPE})..."

# Install locales
sudo apt-get update
sudo apt-get install -y locales
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Install dependencies for setting up apt sources
sudo apt-get install -y software-properties-common curl

# Enable universe repository
sudo add-apt-repository -y universe
sudo apt-get update

# Get latest ros-apt-source release
echo "Fetching latest ros-apt-source version..."
ROS_APT_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -Po '"tag_name": "\K[^"]+')
UBUNTU_CODENAME=$(source /etc/os-release && echo "$VERSION_CODENAME")

echo "Using ros-apt-source version: ${ROS_APT_VERSION}"

# Download and install ros-apt-source
ROS_APT_DEB="/tmp/ros2-apt-source.deb"
curl -fSL -o "${ROS_APT_DEB}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VERSION}/ros2-apt-source_${ROS_APT_VERSION}.${UBUNTU_CODENAME}_all.deb"

sudo apt-get install -y "${ROS_APT_DEB}"
sudo apt-get update

# Check if package is held
PACKAGE_NAME="ros-${ROSDISTRO}-${ROS2_INSTALLATION_TYPE}"
if apt-mark showhold | grep -q "${PACKAGE_NAME}"; then
    echo "Warning: ${PACKAGE_NAME} is apt-mark hold. Skipping installation."
else
    echo "Installing ${PACKAGE_NAME}..."
    sudo apt-get install -y "${PACKAGE_NAME}"
fi

echo "ROS 2 ${ROSDISTRO} installation complete."
