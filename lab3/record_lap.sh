#!/bin/bash
# Part 2: record a practice lap into lab3/bags (never beside the grader).
cd "$(git rev-parse --show-toplevel)" && mkdir -p lab3/bags && cd lab3/bags
ros2 bag record $(cat ../../src/lab/autosdv_lab3/config/record/run_topics.txt)
