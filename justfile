# AutoSDV Development Commands
# Use `just --list` to see all available commands

# Demo scenarios live in their own module so the top-level list stays short.
# `just demo` lists them; `just demo run` runs the COSS NDT replay end to end.
mod demo

# ============================================================================
# Core Commands
# ============================================================================

# Default recipe: show all available commands
default:
    @just --list

# Initialize and update all git submodules
checkout:
    git submodule update --init --recursive --checkout

# Run interactive setup (installs ROS 2, dependencies, etc.)
setup:
    ./setup.sh

# Build this project
# --cargo-args --release applies to the Rust packages (cuda_ndt_matcher); without
# it colcon-cargo builds them unoptimized while CMAKE_BUILD_TYPE=Release covers
# only the C++ ones, so pose_source:=cuda_ndt ran a debug binary at ~80 ms per
# scan against the ~5 ms the package documents.
build:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash && \
    colcon build \
        --base-paths src \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_DISABLE_FIND_PACKAGE_isaac_ros_common=TRUE \
        --cargo-args --release

# Run tests for packages in src/ directory
test:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash && \
    colcon test \
        --base-paths src \
        --return-code-on-test-failure; \
    TEST_EXIT_CODE=$?; \
    echo "" && \
    colcon test-result --verbose; \
    exit $TEST_EXIT_CODE

# Clean up built binaries (requires confirmation)
clean:
    #!/usr/bin/env bash
    while true; do \
        read -p 'Are you sure to clean up? (yes/no) ' yn; \
        case $yn in \
            yes ) rm -rf build install log; break;; \
            no ) break;; \
            * ) echo 'Please enter yes or no.';; \
        esac \
    done

# ============================================================================
# Launch Commands - Start systems
# ============================================================================

# Launch AutoSDV system with web UI at http://localhost:8081
launch ARGS="":
    #!/usr/bin/env bash
    source install/setup.bash && \
    if [ -n "$DISPLAY" ]; then \
        play_launch launch \
            --web-addr 0.0.0.0:8081 \
            autosdv_launch autosdv.launch.yaml {{ARGS}}; \
    else \
        play_launch launch \
            --web-addr 0.0.0.0:8081 \
            autosdv_launch autosdv.launch.yaml \
            rviz:=false {{ARGS}}; \
    fi

# Launch Autoware planning simulator with AutoSDV vehicle
launch-sim-planning:
    #!/usr/bin/env bash
    source install/setup.bash && \
    play_launch launch \
        --web-addr 0.0.0.0:8081 \
        autoware_launch planning_simulator.launch.xml \
        map_path:={{justfile_directory()}}/data/COSS-map-planning \
        vehicle_model:=autosdv_vehicle \
        sensor_model:=autosdv_sensor_kit

# Launch logging simulation for rosbag replay testing
launch-sim-logging ARGS="":
    #!/usr/bin/env bash
    source install/setup.bash && \
    if [ -n "$DISPLAY" ]; then \
        play_launch launch \
            --web-addr 0.0.0.0:8081 \
            autosdv_launch logging_simulation.launch.yaml {{ARGS}}; \
    else \
        play_launch launch \
            --web-addr 0.0.0.0:8081 \
            autosdv_launch logging_simulation.launch.yaml \
            rviz:=false {{ARGS}}; \
    fi

# Launch only ZED camera node for testing
launch-zed:
    #!/usr/bin/env bash
    source install/setup.bash && \
    play_launch launch \
        --web-addr 0.0.0.0:8081 \
        zed_wrapper zed_camera.launch.py camera_model:=zedxm

# ============================================================================
# Tool Commands - Development and monitoring tools
# ============================================================================

# Launch RViz with AutoSDV configuration
tool-rviz:
    rviz2 -d ./src/launcher/autosdv_launch/rviz/autosdv.rviz

# Launch PlotJuggler for data visualization
tool-plotjuggler:
    #!/usr/bin/env bash
    source install/setup.bash && \
    ros2 run plotjuggler plotjuggler

# Launch manual keyboard control
tool-controller:
    #!/usr/bin/env bash
    source install/setup.bash && \
    ros2 run control_test keyboard_control

# Launch drive monitor TUI (shows pose, speed, component states)
tool-tui:
    #!/usr/bin/env bash
    source install/setup.bash && \
    python3 ./scripts/testing/drive/run.py

# ============================================================================
# Control Commands - Control system testing
# ============================================================================

# Launch vehicle control test (basic_control.launch.xml)
control-basic:
    #!/usr/bin/env bash
    source install/setup.bash && \
    play_launch launch control_test basic_control.launch.xml

# Run trajectory player with straight_10m.yaml (10m straight line)
control-straight:
    #!/usr/bin/env bash
    source install/setup.bash && \
    ros2 run control_test trajectory_player --ros-args -p trajectory_file:=straight_10m.yaml

# Run trajectory player with circle.yaml (circular path)
control-circle:
    #!/usr/bin/env bash
    source install/setup.bash && \
    ros2 run control_test trajectory_player --ros-args -p trajectory_file:=circle.yaml

# ============================================================================
# Data Commands - Download datasets
# ============================================================================

# Download test rosbag (outdoor_20251226_153115) from Synology Drive
# Automatically installs synology-dl via cargo if not found.
download-data:
    ./scripts/rosbag/download-test-rosbag.sh

# ============================================================================
# Map Commands - Validate map directories
# ============================================================================

# Check a map directory is usable for a given pose_source (default: cuda_ndt).
# Verifies lanelet2/projector/PCD/grid presence, and -- the point of the
# whole check -- that the occupancy grid's frame matches the lanelet2 map's,
# which catches the "grid built in the wrong frame" failure class.
# Further flags pass through, e.g. --grid-yaml NAME to check a grid variant
# other than occupancy_grid.yaml, or --autoware-setup PATH.
map-check MAP_DIR POSE_SOURCE="cuda_ndt" *FLAGS:
    python3 ./scripts/map/check_map.py "{{MAP_DIR}}" --pose-source "{{POSE_SOURCE}}" {{FLAGS}}

# Build MAP_DIR/occupancy_grid.{pgm,yaml} by slicing a height band out of the
# PCD map already in MAP_DIR, record how it was built in autosdv_map.yaml, then
# validate the result for pose_source:=mcl.
#
# The z band is the one judgement you have to make, and it is not forgiving:
# a wrong band yields a valid-looking grid that localizes badly rather than an
# error. Run without FLAGS to see the height distribution, the estimated ground
# level and a suggested band, then re-run with them:
#
#   just map-grid-from-pcd data/COSS-map-planning
#   just map-grid-from-pcd data/COSS-map-planning --z-min 9.1 --z-max 9.4
#
# Further flags pass straight through: --resolution (default 0.05 m/px), and
# --min-points (points needed in the band before a cell counts as occupied).
map-grid-from-pcd MAP_DIR *FLAGS:
    #!/usr/bin/env bash
    set -euo pipefail
    PCD="{{MAP_DIR}}/pointcloud_map.pcd"
    if [ ! -f "$PCD" ]; then
        echo "no $PCD -- this recipe slices an existing PCD map." >&2
        echo "For a site with no PCD, build the grid from a recorded drive:" >&2
        echo "  just map-grid-from-bag <bag> {{MAP_DIR}}" >&2
        exit 1
    fi
    python3 ./scripts/map/pcd_to_pgm.py "$PCD" "{{MAP_DIR}}/occupancy_grid" \
        --sidecar {{FLAGS}}
    echo
    just map-check "{{MAP_DIR}}" mcl

# Build MAP_DIR/occupancy_grid.{pgm,yaml} by accumulating 2-D scans from a
# recorded drive (BAG) at their ground-truth poses, for a site with no PCD map.
# Records provenance in autosdv_map.yaml, then validates for pose_source:=mcl.
#
# The band here is relative to the scan plane rather than to site ground level,
# so its defaults (-0.15..0.15 m) are meaningful; override them, and
# --resolution (default 0.1) / --min-hits (default 3), by passing them through.
map-grid-from-bag BAG MAP_DIR *FLAGS:
    #!/usr/bin/env bash
    set -euo pipefail
    python3 ./scripts/2dlidar/scan_accumulate_grid.py "{{BAG}}" \
        "{{MAP_DIR}}/occupancy_grid" --sidecar {{FLAGS}}
    echo
    just map-check "{{MAP_DIR}}" mcl

# ============================================================================
# Bag Commands - Rosbag recording and playback
# ============================================================================

# Record outdoor sensor topics to rosbags/ directory
bag-record:
    ./scripts/rosbag/record_outdoor.sh

# Play the most recent outdoor recording
bag-play:
    #!/usr/bin/env bash
    LATEST=$(ls -td rosbags/outdoor_* 2>/dev/null | head -1); \
    if [ -z "$LATEST" ]; then \
        echo "No outdoor recordings found in rosbags/"; \
        exit 1; \
    fi; \
    echo "Playing: $LATEST"; \
    ros2 bag play "$LATEST" --clock

# ============================================================================
# Simulation Commands - Full simulation scenarios
# ============================================================================

# Run COSS Park simulation (launch + rosbag feed + localization recording)
# Requires: rosbag data from NTU COSS Park (run ./scripts/rosbag/download-test-rosbag.sh)
sim-coss-park:
    #!/usr/bin/env bash
    source install/setup.bash && \
    parallel --line-buffer ::: \
        "just launch-sim-logging" \
        "sleep 40 && ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock -l -r 1.0" \
        "sleep 45 && ./scripts/rosbag/record_localization.sh"
