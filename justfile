# AutoSDV Development Commands
# Use `just --list` to see all available commands

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
build:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash && \
    colcon build \
        --base-paths src \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

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
    play_launch launch \
        --web-addr 0.0.0.0:8081 \
        autoware_launch planning_simulator.launch.xml \
        map_path:={{justfile_directory()}}/data/COSS-map-planning \
        vehicle_model:=autosdv_vehicle \
        sensor_model:=autosdv_sensor_kit

# Launch logging simulation for rosbag replay testing
launch-sim-logging ARGS="":
    #!/usr/bin/env bash
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
