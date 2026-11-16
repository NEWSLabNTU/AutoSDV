#!/bin/bash
# Step 3: practise the lap in the planning simulator.
cd "$(git rev-parse --show-toplevel)"
play_launch launch autosdv_lab3 lab3_planning_sim.launch.xml map_path:=$PWD/lab3/map
