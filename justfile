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

# ============================================================================
# Autoware model data and TensorRT engines
# ============================================================================

# just --list shows only the LAST comment line, so the description goes here.
# Link Autoware model data into data/autoware_data (writable, for TensorRT)
setup-autoware-data:
    ./scripts/setup_autoware_data.sh

# Compile the TensorRT engines this stack needs, ahead of the first launch.
#
# Autoware compiles an .onnx into a .engine inside the NODE'S CONSTRUCTOR the
# first time it runs. On the Orin that is minutes per model, during which the
# node is not up and perception is unavailable. Doing it here turns that into a
# provisioning step.
#
# Engines are specific to the TensorRT version AND the GPU, so this must run ON
# THE TARGET BOARD. It cannot be baked into an image built elsewhere, and it
# must be re-run after an Autoware or JetPack upgrade.
#
# The model set below is what the perception presets actually resolve to:
# centerpoint (autosdv_perception_component defaults lidar_detection_model to
# `centerpoint`, not the tiny variant), the yolox-sPlus camera 2D detector, and
# the three traffic-light models that camera_lidar_fusion adds. Re-derive it if
# a preset changes:
#
#     play_launch resolve autosdv_launch autosdv.launch.yaml \
#       launch_perception:=true perception_preset:=camera_lidar_fusion -o ./tmp/m.yaml
#
# `autoware_shape_estimation` is deliberately absent: it uses TensorRT but ships
# no `build_only` argument, so its pointnet engine is still built on first use.
# That is one model rather than five, and it succeeds now that the directory is
# writable.
#
# just --list shows only the LAST comment line, so the description goes here.
# Compile TensorRT engines ahead of time (minutes; run on the target board)
build-engines:
    #!/usr/bin/env bash
    # No `set -u`: ROS's own setup.bash reads unbound variables and dies under
    # it (AMENT_TRACE_SETUP_FILES). No `set -e` either — one model failing must
    # not hide the rest, and the summary at the end reports what actually
    # landed.
    set -o pipefail
    just setup-autoware-data
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/setup.bash
    DATA="${AUTOSDV_DATA_PATH:-{{justfile_directory()}}/data/autoware_data}"

    # Each entry: <package> <launch file> [extra args]. `build_only:=true` makes
    # the node exit as soon as its engine is written — an Autoware-provided
    # argument, so the builder settings match what the node will later expect.
    # Building with trtexec by hand would not guarantee that.
    build() {
        local pkg="$1" launch="$2"; shift 2
        echo "=== ${pkg} ${launch}"
        local start=$SECONDS
        # Stream the TensorRT lines rather than piping into `tail`, which
        # buffers the whole build and shows nothing for minutes — on a step
        # that takes minutes per model, silence is indistinguishable from a
        # hang. `--line-buffered` matters for the same reason.
        #
        # Not fatal on failure: one model failing must not hide the others, and
        # the summary below reports what actually landed.
        ros2 launch "${pkg}" "${launch}" data_path:="${DATA}" build_only:=true "$@" 2>&1 \
            | grep --line-buffered -iE "engine generation|engine build|error|fail" \
            | sed -u 's/^/    /' || true
        echo "    (${pkg}: $((SECONDS - start))s)"
    }

    build autoware_lidar_centerpoint lidar_centerpoint.launch.xml model_name:=centerpoint
    # use_decompress:=false is required: the decompressor node the launch
    # otherwise starts has no build_only and never exits, so `ros2 launch`
    # hangs forever after the engine is written (observed: 3h20m).
    build autoware_tensorrt_yolox yolox_s_plus_opt.launch.xml use_decompress:=false
    build autoware_traffic_light_classifier car_traffic_light_classifier.launch.xml
    build autoware_traffic_light_classifier pedestrian_traffic_light_classifier.launch.xml
    build autoware_traffic_light_fine_detector traffic_light_fine_detector.launch.xml

    echo
    echo "=== engines in ${DATA}"
    find "${DATA}" -name '*.engine' -type f -printf '    %p (%s bytes)\n' 2>/dev/null | sort
    echo "    total: $(find "${DATA}" -name '*.engine' -type f 2>/dev/null | wc -l)"

# --cargo-args --release applies to the Rust packages (cuda_ndt_matcher); without
# it colcon-cargo builds them unoptimized while CMAKE_BUILD_TYPE=Release covers
# only the C++ ones, so pose_source:=cuda_ndt ran a debug binary at ~80 ms per
# scan against the ~5 ms the package documents.

# Build this project
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
