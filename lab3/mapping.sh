#!/bin/bash
# Step 1: the sensors and the vehicle interface, for the mapping drive. Teams 4-6: car_c.
cd "$(git rev-parse --show-toplevel)"
unset DISPLAY LD_PRELOAD   # the ZED does not open from a VNC terminal otherwise
play_launch launch autosdv_lab3 lab3_mapping.launch.xml vehicle_id:=car_b
