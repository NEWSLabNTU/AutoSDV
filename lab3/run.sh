#!/bin/bash
# Part 2: Autoware on your map, on the car. Teams 4-6: car_c.
cd "$(git rev-parse --show-toplevel)"
unset DISPLAY LD_PRELOAD   # the ZED does not open from a VNC terminal otherwise
play_launch launch autosdv_lab3 lab3_run.launch.xml vehicle_id:=car_b map_path:=$PWD/lab3/map
