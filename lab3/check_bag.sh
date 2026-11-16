#!/bin/bash
# Step 1: is the bag usable for mapping? Usage: lab3/check_bag.sh lab3/bags/<bag>
ros2 run autosdv_lab3 lab3_check_bag "$1"
