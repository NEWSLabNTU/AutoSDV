#!/bin/bash
# Step 4: record the localization output of a replay into lab3/bags.
cd "$(git rev-parse --show-toplevel)" && mkdir -p lab3/bags && cd lab3/bags
ros2 bag record $(cat ../../src/lab/autosdv_lab3/config/record/localization_topics.txt)
