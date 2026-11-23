#!/bin/bash
# Part 2: record a lap for grading into lab3/bags/rosbag2_<date>-<time>: the car's path,
# NDT's scores (NVTL), the operation mode, the speed and the sensors. Start it after
# run.sh is up and before the goal; Ctrl-C once the car has stopped. Grade: grade.sh.
cd "$(git rev-parse --show-toplevel)" && mkdir -p lab3/bags && cd lab3/bags
ros2 bag record $(cat ../../src/lab/autosdv_lab3/config/record/run_topics.txt)
