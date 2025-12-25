#!/usr/bin/env bash
# Install ROS 2 development tools
# Converted from ansible/roles/ros2_dev_tools/tasks/main.yaml

set -e

echo "Installing ROS 2 development tools..."

sudo apt-get update
sudo apt-get install -y \
    python3-colcon-mixin \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    ros-dev-tools \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures

# Initialize rosdep if not already done
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    echo "Initializing rosdep..."
    sudo rosdep init
fi

echo "ROS 2 development tools installation complete."
