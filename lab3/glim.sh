#!/bin/bash
# Step 2: build the point cloud map. Usage: lab3/glim.sh lab3/bags/<bag>
# Without an NVIDIA GPU, use robin_w_ready_cpu.
cd "$(git rev-parse --show-toplevel)"
ros2 run glim_ros glim_rosbag "$1" --ros-args \
    -p config_path:=$PWD/src/lab/autosdv_lab3/config/glim/robin_w_ready \
    -p dump_path:=$PWD/lab3/glim -p auto_quit:=true
