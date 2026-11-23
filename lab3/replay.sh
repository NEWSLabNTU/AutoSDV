#!/bin/bash
# Step 4: the stack of run.sh without sensor drivers or motor, on the bag's clock, for a
# replay (lab3/play_bag.sh in another terminal). Keep the settings as in run.sh.
cd "$(git rev-parse --show-toplevel)"
cfg=$PWD/src/lab/autosdv_lab3/config
play_launch launch autosdv_launch autosdv.launch.yaml \
    vehicle_id:=car_b \
    map_path:=$PWD/lab3/map \
    use_gnss:=false \
    use_ntrip:=false \
    pose_source:=cuda_ndt \
    cuda_ndt_param_file:=$cfg/ndt_indoor_robin_w.param.yaml \
    ndt_voxel_grid_param_file:=$cfg/localization/voxel_grid_robin_w.param.yaml \
    ndt_random_downsample_param_file:=$cfg/localization/random_downsample_robin_w.param.yaml \
    pointcloud_map_filter_param_path:=$cfg/pointcloud_map_filter_indoor.param.yaml \
    planning_profile:=rc \
    use_speed_limit:=true \
    speed_limit_config:=$PWD/src/planning/autosdv_speed_limit/config/speed_limit.yaml \
    launch_sensing_driver:=false \
    launch_vehicle_interface:=false \
    is_simulation:=true \
    use_sim_time:=true \
    rviz:=true \
    rviz_respawn:=false
