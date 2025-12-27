#!/bin/bash
# Play rosbag for logging simulation testing
# Usage: ./scripts/play_rosbag.sh [delay] [rosbag_path]

set -e

DELAY="${1:-15}"
ROSBAG_PATH="${2:-rosbags/outdoor_20251226_153115/}"

sleep "$DELAY"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play "$ROSBAG_PATH" --clock -l -r 1.5
