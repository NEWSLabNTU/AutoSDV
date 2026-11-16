#!/bin/bash
# Step 4: the stack without sensors or motor, for a bag replay. Teams 4-6: car_c.
cd "$(git rev-parse --show-toplevel)"
play_launch launch autosdv_lab3 lab3_replay.launch.xml vehicle_id:=car_b map_path:=$PWD/lab3/map
