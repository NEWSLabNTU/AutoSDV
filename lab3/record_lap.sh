#!/bin/bash
# Part 2: record a lap for grading into lab3/bags/rosbag2_<date>-<time>: the sensors, the
# speed, the control output, NDT's pose and scores (NVTL), the car's path, the plan and
# the operation mode. Start it after run.sh is up and before the goal; once the car has
# stopped, Ctrl-C here first, then run.sh (stopping the stack first cuts the bag's end).
# Grade: grade.sh.
cd "$(git rev-parse --show-toplevel)" && mkdir -p lab3/bags && cd lab3/bags
ros2 bag record $(cat ../../src/lab/autosdv_lab3/config/record/run_topics.txt)
