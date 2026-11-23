#!/bin/bash
# Part 2: Autoware on your map, on the car. One setting per line: edit the values here
# (teams 4-6: vehicle_id:=car_c), commit and push. RViz opens, and reopens if closed, when
# this terminal has a display (the car's VNC desktop).
cd "$(git rev-parse --show-toplevel)"
cfg=$PWD/src/lab/autosdv_lab3/config
if [ -n "$DISPLAY" ]; then rviz=true; else rviz=false; fi
play_launch launch autosdv_launch autosdv.launch.yaml \
    vehicle_id:=car_b \
    map_path:=$PWD/lab3/map \
    use_gnss:=false \
    use_ntrip:=false \
    pose_source:=ndt \
    ndt_param_file:=$cfg/ndt_indoor_robin_w.param.yaml \
    ndt_voxel_grid_param_file:=$cfg/localization/voxel_grid_robin_w.param.yaml \
    ndt_random_downsample_param_file:=$cfg/localization/random_downsample_robin_w.param.yaml \
    pointcloud_map_filter_param_path:=$cfg/pointcloud_map_filter_indoor.param.yaml \
    planning_profile:=rc \
    use_speed_limit:=true \
    speed_limit_config:=$PWD/src/planning/autosdv_speed_limit/config/speed_limit.yaml \
    rviz:=$rviz \
    rviz_respawn:=true
