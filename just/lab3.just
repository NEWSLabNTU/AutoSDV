# Lab 3, indoor map building and NDT localization: the mapping drive, GLIM, the lanelet2
# loop, and the autonomous laps.
#
# Students use stock ROS commands (ros2 launch, ros2 bag, ros2 run, GLIM's own tools,
# PCL's command-line tools); docs/guides/lab3.md, "Student commands", lists them. These
# recipes are the TA's shortcuts and the scripted paths the TA simulation runs:
#
#   just lab3 stack car_b                      week 1: sensors + dry-run actuator (play_launch)
#   just lab3 record --car car_b --team 3      the mapping bag, checked on Ctrl-C
#   just lab3 map BAG                          GLIM headless -> BAG_glim/raw_map.pcd
#   just lab3 downsample BAG_glim/raw_map.pcd ~/lab3/team3_map --voxel 0.1
#   just lab3 check-lanelet ~/lab3/team3_map   loop, clockwise, speed limits; simulator lap
#   just lab3 planning-sim ~/lab3/team3_map    practise the lap (RViz)
#   just lab3 run car_b ~/lab3/team3_map       week 2: the full stack
#   just lab3 replay car_b ~/lab3/team3_map    a recorded lap against the map, at home
#   just lab3 grade BAG                        grade a recorded lap again
#
# Settings are on the command line; nothing here reads a config file.

set working-directory := '..'

web_addr := env_var_or_default('LAB3_WEB_ADDR', '0.0.0.0:8081')

[private]
default:
    @just --list lab3

#   just lab3 stack car_b
# Week 1: the mapping stack (sensors, actuator as a dry run)
[positional-arguments]
stack VEHICLE_ID *ARGS:
    #!/usr/bin/env bash
    vid=$1; shift
    case "$vid" in car_a|car_b|car_c|sim) ;; *) echo "usage: just lab3 stack VEHICLE_ID [KEY:=VALUE ...] (car_a, car_b, car_c)" >&2; exit 2 ;; esac
    # The ZED never gets a DISPLAY (camera_display.sh says why)
    source scripts/camera_display.sh
    camera_display
    cmd=(play_launch launch --web-addr '{{ web_addr }}' autosdv_lab3 lab3_mapping.launch.xml
         "vehicle_id:=$vid" "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

#   just lab3 record --car car_b --team 3                   the mapping bag, checked on Ctrl-C
#   just lab3 record --car car_b --team 3 --kind run --map ~/lab3/team3_map
# Record a Lab 3 bag beside the running stack (second terminal)
[positional-arguments]
record *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_record "$@"

#   just lab3 check-bag ~/lab3/bags/team3_map
# Can GLIM use this mapping bag? PASS/FAIL per requirement
[positional-arguments]
check-bag BAG *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_check_bag "$@"

#   just lab3 map BAG                  writes BAG_glim/raw_map.pcd and glim_report.yaml
#   just lab3 map BAG -o DIR --cpu
# Build the point-cloud map with GLIM (headless, the bag's own TF), keeping intensity
[positional-arguments]
map BAG *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_map "$@"

#   just lab3 downsample RAW_CLOUD MAP_DIR --voxel 0.1 [--max-mb 50]
# Downsample a raw cloud (.pcd, or GLIM's exported .ply) into MAP_DIR/pointcloud_map.pcd
[positional-arguments]
downsample CLOUD MAP_DIR *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_downsample "$@"

#   just lab3 check-lanelet MAP_DIR             the requirements, then the simulator lap
#   just lab3 check-lanelet MAP_DIR --no-sim    the requirements only
# Check the lanelet2 loop: closed, clockwise, speed limits; then a one-goal simulator lap
[positional-arguments]
check-lanelet MAP_DIR *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_check_lanelet "$@"

#   just lab3 sim-lap MAP_DIR [--goal-behind 0.7] [--timeout S] [--keep-log]
# The planning-simulator lap alone
[positional-arguments]
sim-lap MAP_DIR *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_sim_lap "$@"

#   just lab3 planning-sim ~/lab3/team3_map
# Practise the lap in the RC planning simulator on a team's map (RViz)
[positional-arguments]
planning-sim MAP_DIR *ARGS:
    #!/usr/bin/env bash
    map=$1; shift
    cmd=(play_launch launch --web-addr '{{ web_addr }}' autosdv_lab3 lab3_planning_sim.launch.xml
         "map_path:=$(realpath "$map")" "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

#   just lab3 run car_b ~/lab3/team3_map
# Week 2: the full stack on the team's map; set the pose and goal in RViz
[positional-arguments]
run VEHICLE_ID MAP_DIR *ARGS:
    #!/usr/bin/env bash
    vid=$1; map=$2; shift 2
    case "$vid" in car_a|car_b|car_c|sim) ;; *) echo "usage: just lab3 run VEHICLE_ID MAP_DIR [KEY:=VALUE ...]" >&2; exit 2 ;; esac
    [ -f "$map/pointcloud_map.pcd" ] || { echo "error: $map has no pointcloud_map.pcd" >&2; exit 2; }
    [ -f "$map/lanelet2_map.osm" ] || { echo "error: $map has no lanelet2_map.osm" >&2; exit 2; }
    source scripts/camera_display.sh
    camera_display
    cmd=(play_launch launch --web-addr '{{ web_addr }}' autosdv_lab3 lab3_run.launch.xml
         "vehicle_id:=$vid" "map_path:=$(realpath "$map")" "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

#   just lab3 replay car_b ~/lab3/team3_map     then: ros2 bag play BAG --clock --topics ...
# The stack on a recorded lap at home: no drivers, no vehicle interface, sim time
[positional-arguments]
replay VEHICLE_ID MAP_DIR *ARGS:
    #!/usr/bin/env bash
    vid=$1; map=$2; shift 2
    cmd=(play_launch launch --web-addr '{{ web_addr }}' autosdv_lab3 lab3_replay.launch.xml
         "vehicle_id:=$vid" "map_path:=$(realpath "$map")" "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

#   just lab3 lanelet-speed ~/lab3/team3_map                  list each lanelet's limit
#   just lab3 lanelet-speed ~/lab3/team3_map --corners 4 --straights 8
# The per-lanelet (segment) speed limits; restart the stack to load them
[positional-arguments]
lanelet-speed MAP_DIR *ARGS:
    #!/usr/bin/env bash
    map=$1; shift
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_lanelet_speed "$map" "$@"

#   just lab3 grade ~/lab3/bags/team3_run_20261120_101500 [-o report.yaml]
# Grade a recorded lap again, from the bag alone
[positional-arguments]
grade *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_grade "$@"
