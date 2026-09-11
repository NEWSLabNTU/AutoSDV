# AutoSDV Development Commands
# Use `just --list` to see all available commands

# ============================================================================
# Modules -- grouped commands. `just <module>` lists that module's recipes.
#
# Invoke either way:  `just bag play`  or  `just bag::play`
#
# Only families entered deliberately live here. The daily verbs (build, test,
# clean, launch, checkout, setup) stay at the root -- partly by choice, and
# partly because a module may NOT share a name with a recipe: `mod launch`
# beside a `launch:` recipe is a hard error that kills the whole justfile.
# ============================================================================

# Demo scenarios: `just demo run` runs the COSS NDT replay end to end
mod demo
# Rosbag recording, playback and download
mod bag 'just/bag.just'
# Control system testing: trajectories and the basic control launch
mod control 'just/control.just'
# Map validation and occupancy-grid construction
mod map 'just/map.just'
# Simulation: planning simulator, rosbag replay, full scenarios
mod sim 'just/sim.just'
# Development and monitoring tools: RViz, PlotJuggler, TUI, manual control
mod tool 'just/tool.just'

# ============================================================================
# Core Commands
# ============================================================================

# --list-submodules is not optional. Without it each module collapses to a
# single `bag ...` line and its recipes are invisible, which is the difference
# between a menu and a riddle.
# just --list shows only the LAST comment line, so the description goes here.
# Show all available commands, modules expanded
default:
    @just --list --list-submodules

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
# centerpoint_tiny (the perception stack resolves the centerpoint model to its
# tiny variant at runtime -- verified from a live launch's TRT input filenames,
# not from the launch arg defaults), the yolox-sPlus camera 2D detector, and
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

    build autoware_lidar_centerpoint lidar_centerpoint.launch.xml model_name:=centerpoint_tiny
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
    # No CUDA PATH setup needed: cuda_ffi discovers the toolkit by the
    # ecosystem convention (CUDA_PATH/CUDA_HOME, then /usr/local/cuda-*,
    # then the /usr/local/cuda symlink), and ndt_cuda pins cudarc's version
    # features so nothing runs `nvcc --version`. Verified by building with
    # nvcc absent from PATH.
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
