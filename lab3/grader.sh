#!/bin/bash
# Part 2: record and grade one lap into lab3/bags; Ctrl-C after the car stops.
cd "$(git rev-parse --show-toplevel)"
ros2 launch autosdv_lab3 lab3_grader.launch.xml out_dir:=$PWD/lab3/bags
