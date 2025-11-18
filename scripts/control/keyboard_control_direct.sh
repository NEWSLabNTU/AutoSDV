#!/bin/bash
# Keyboard controller with topic remapping for direct vehicle control
# Remaps from /external/selected/* to /control/command/*

cd "$(dirname "$0")/../.."
source install/setup.bash

ros2 run autoware_manual_control keyboard_control \
  --ros-args \
  -r /external/selected/control_cmd:=/control/command/control_cmd \
  -r /external/selected/gear_cmd:=/control/command/gear_cmd
