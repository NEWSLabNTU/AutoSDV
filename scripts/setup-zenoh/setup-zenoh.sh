#!/usr/bin/env bash

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

export AUTOWARE_HOME=/opt/autoware
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/0.0.0.0:0"];scouting/multicast/enabled=true'

# sudo apt install ros-humble-rmw-zenoh-cpp
pkill -9 -f ros && ros2 daemon stop
"$script_dir/zenoh-daemon.sh" start
