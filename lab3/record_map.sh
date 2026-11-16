#!/bin/bash
# Step 1: record the mapping bag into lab3/bags/rosbag2_<date>-<time>.
cd "$(git rev-parse --show-toplevel)" && mkdir -p lab3/bags && cd lab3/bags
ros2 bag record $(cat ../../src/lab/autosdv_lab3/config/record/map_topics.txt)
