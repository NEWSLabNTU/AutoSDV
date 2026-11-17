#!/bin/bash
# Play rosbag for logging simulation testing
# Usage: ./scripts/rosbag/play_rosbag.sh [rosbag_path]

set -e

ROSBAG_PATH="${1:-rosbags/outdoor_20251226_153115/}"

source scripts/env.sh; set +u   # the selected Autoware and its workspace
ros2 bag play "$ROSBAG_PATH" --clock -l -r 1.5
