#!/bin/bash
# Record localization-related topics for logging simulation testing
# Usage: ./scripts/rosbag/record_localization.sh

set -e

OUTPUT_DIR="rosbags/localization_test_$(date +%Y%m%d_%H%M%S)"

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Recording localization topics to: $OUTPUT_DIR"
ros2 bag record -o "$OUTPUT_DIR" \
    --regex "/localization/pose_estimator/.*" \
    --max-bag-duration 60
