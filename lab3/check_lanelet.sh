#!/bin/bash
# Step 3: the four map checks and a lap in the planning simulator.
cd "$(git rev-parse --show-toplevel)"
ros2 run autosdv_lab3 lab3_check_lanelet lab3/map "$@"
