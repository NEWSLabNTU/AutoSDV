# AutoSDV Development Commands
# Use `just --list` to see all available commands

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
launch-planning-simulation:
    play_launch launch \
        --web-addr 0.0.0.0:8081 \
        autoware_launch planning_simulator.launch.xml \
        map_path:={{justfile_directory()}}/data/COSS-map-planning \
        vehicle_model:=autosdv_vehicle \
        sensor_model:=autosdv_sensor_kit

# Launch only ZED camera node for testing
launch-zed-only:
    play_launch launch \
        --web-addr 0.0.0.0:8081 \
        zed_wrapper zed_camera.launch.py camera_model:=zedxm

# Launch logging simulation for rosbag replay testing
launch-logging-simulation ARGS="":
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

# Launch manual keyboard control
run-controller:
    #!/usr/bin/env bash
    source install/setup.bash && \
    ros2 run control_test keyboard_control

# Launch PlotJuggler for data visualization
run-plotjuggler:
    #!/usr/bin/env bash
    source install/setup.bash && \
    ros2 run plotjuggler plotjuggler

# Launch vehicle control test (basic_control.launch.xml)
play-basic-control:
    play_launch launch control_test basic_control.launch.xml

# Run trajectory player with straight_10m.yaml (10m straight line)
run-straight-10m:
    #!/usr/bin/env bash
    source install/setup.bash && \
    ros2 run control_test trajectory_player --ros-args -p trajectory_file:=straight_10m.yaml

# Run trajectory player with circle.yaml (circular path)
run-circle:
    #!/usr/bin/env bash
    source install/setup.bash && \
    ros2 run control_test trajectory_player --ros-args -p trajectory_file:=circle.yaml

# Launch RViz with AutoSDV configuration
run-rviz:
    rviz2 -d ./src/launcher/autosdv_launch/rviz/autosdv.rviz

# Record outdoor sensor topics to rosbags/ directory
record-outdoor:
    ./scripts/record_outdoor.sh

# Play the most recent outdoor recording
play-outdoor:
    #!/usr/bin/env bash
    LATEST=$(ls -td rosbags/outdoor_* 2>/dev/null | head -1); \
    if [ -z "$LATEST" ]; then \
        echo "No outdoor recordings found in rosbags/"; \
        exit 1; \
    fi; \
    echo "Playing: $LATEST"; \
    ros2 bag play "$LATEST" --clock

# Run full logging simulation test (launch + rosbag + drive + record)
test-logging-simulation:
    #!/usr/bin/env bash
    source install/setup.bash && \
    parallel --line-buffer ::: \
        "just launch-logging-simulation" \
        "sleep 40 && ros2 bag play rosbags/outdoor_20251226_153115/ --clock -l -r 1.0" \
        "sleep 45 && ./scripts/record_localization.sh"

# Run autonomous driving with poses from scripts/testing/drive/poses.json
run-drive:
    #!/usr/bin/env bash
    source install/setup.bash && \
    python3 ./scripts/testing/drive/run.py

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
