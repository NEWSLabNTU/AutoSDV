#!/bin/bash
# Step 4: play a bag's sensor inputs. Usage: lab3/play_bag.sh lab3/bags/<bag> [--rate 0.5]
# Options after the bag go to ros2 bag play (--rate 0.5 doubles the time to set the pose).
topics=$(git rev-parse --show-toplevel)/src/lab/autosdv_lab3/config/record/replay_topics.txt
ros2 bag play "$1" --clock --topics $(cat "$topics") "${@:2}"
