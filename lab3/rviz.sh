#!/bin/bash
# Part 2: RViz with the AutoSDV layout.
rviz2 -d $(ros2 pkg prefix autosdv_launch)/share/autosdv_launch/rviz/autosdv.rviz --ros-args \
    --params-file $(ros2 pkg prefix autosdv_vehicle_description)/share/autosdv_vehicle_description/config/vehicle_info.param.yaml
