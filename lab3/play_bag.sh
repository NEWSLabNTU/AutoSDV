#!/bin/bash
# Step 4: play a bag's sensor inputs. Usage: lab3/play_bag.sh lab3/bags/<bag>
ros2 bag play "$1" --clock --topics $(cat "$(git rev-parse --show-toplevel)"/src/lab/autosdv_lab3/config/record/replay_topics.txt)
