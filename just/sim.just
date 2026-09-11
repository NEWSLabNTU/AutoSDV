# Simulation: the planning simulator, rosbag replay, and full scenarios.
#
# Recipes here run from the REPO ROOT; see the note in just/tool.just.
set working-directory := '..'

repo := justfile_directory() / '..'

# `just <module>` runs the module's FIRST recipe, so it has to be harmless.
[private]
default:
    @just --list sim

# Launch the Autoware planning simulator with the AutoSDV vehicle (no sensors)
planning:
    #!/usr/bin/env bash
    source install/setup.bash && \
    play_launch launch \
        --web-addr 0.0.0.0:8081 \
        autoware_launch planning_simulator.launch.xml \
        map_path:={{ repo }}/data/COSS-map-planning \
        vehicle_model:=autosdv_vehicle \
        sensor_model:=autosdv_sensor_kit

# Launch the logging simulation, for rosbag replay testing
logging ARGS="":
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

# Requires the test rosbag; get it with `just bag download`.
# just --list shows only the LAST comment line, so the description goes here.
# Run the COSS Park scenario: logging sim + rosbag feed + localization recording
coss-park:
    #!/usr/bin/env bash
    source install/setup.bash && \
    parallel --line-buffer ::: \
        "just sim logging" \
        "sleep 40 && ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock -l -r 1.0" \
        "sleep 45 && ./scripts/rosbag/record_localization.sh"
