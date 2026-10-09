# Lab 2, coach-board pursuit: the simulator bench (Part 1) and the vehicle
# (Part 2). Arguments after the recipe name go to the launch file, e.g.
#
#   just lab2 sim scenario:=dropout
#   just lab2 sim controller_class:=my_lab2.cruise_controller:CruiseController
#   just lab2 vehicle lidar_model:=robin-w imu_yaw_sign:=-1.0
#
# Then, in a second terminal: `just lab2 engage`.
#
# Recipes here run from the REPO ROOT; see the note in just/tool.just.
set working-directory := '..'

# The web UI address. Override for a shared host:
#   LAB2_WEB_ADDR=127.0.0.1:8091 just lab2 sim
web_addr := env_var_or_default('LAB2_WEB_ADDR', '0.0.0.0:8081')

# `just <module>` runs the module's FIRST recipe, so it has to be harmless.
[private]
default:
    @just --list lab2

# Part 1: pursuit stack on the planning simulator with a scripted board
sim *ARGS:
    #!/usr/bin/env bash
    args=(map_path:="$PWD/data/COSS-map-planning")
    [ -n "${DISPLAY:-}" ] || args+=(rviz:=false)
    cmd=(play_launch launch --web-addr '{{ web_addr }}'
         autosdv_launch coach_pursuit_sim.launch.xml "${args[@]}" {{ ARGS }})
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# Part 2: pursuit stack on the vehicle (LiDAR detector, dead reckoning)
vehicle *ARGS:
    #!/usr/bin/env bash
    cmd=(play_launch launch --web-addr '{{ web_addr }}'
         autosdv_launch coach_pursuit.launch.xml {{ ARGS }})
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# Engage autonomous mode (the car starts following the board)
engage:
    #!/usr/bin/env bash
    source scripts/env.sh
    ros2 service call /api/operation_mode/change_to_autonomous \
        autoware_adapi_v1_msgs/srv/ChangeOperationMode {}

# Leave autonomous mode (the car stops under Autoware's stop mode)
disengage:
    #!/usr/bin/env bash
    source scripts/env.sh
    ros2 service call /api/operation_mode/change_to_stop \
        autoware_adapi_v1_msgs/srv/ChangeOperationMode {}

# Record the topics the Lab 2 grading reads, to rosbags/lab2_<time>
record NAME="":
    #!/usr/bin/env bash
    source scripts/env.sh
    name='{{ NAME }}'
    out="rosbags/lab2_${name:-$(date +%Y%m%d_%H%M%S)}"
    mkdir -p rosbags
    topics=(
        /perception/coach/board /perception/coach/state
        /planning/trajectory /planning/coach_pursuit/debug
        /localization/kinematic_state /localization/acceleration
        /vehicle/status/velocity_status /vehicle/status/steering_status
        /control/trajectory_follower/control_cmd /control/command/control_cmd
        /api/operation_mode/state /system/fail_safe/mrm_state
        /sensing/imu/imu_data /virtual_coach/truth
        /tf /tf_static
    )
    echo "recording to $out"
    exec ros2 bag record -o "$out" "${topics[@]}"
