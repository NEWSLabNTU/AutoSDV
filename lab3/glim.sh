#!/bin/bash
# Step 2: build the point cloud map. Usage: lab3/glim.sh lab3/bags/<bag> [config]
# config: robin_w_ready (default, NVIDIA GPU) or robin_w_ready_cpu (no GPU).
cd "$(git rev-parse --show-toplevel)"
ros2 run glim_ros glim_rosbag "$1" --ros-args \
    -p config_path:=$PWD/src/lab/autosdv_lab3/config/glim/${2:-robin_w_ready} \
    -p dump_path:=$PWD/lab3/glim -p auto_quit:=true
