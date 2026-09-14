# Simulation: the planning simulator, rosbag replay, and full scenarios.
#
# Recipes here run from the REPO ROOT; see the note in just/tool.just.
set working-directory := '..'

# `just <module>` runs the module's FIRST recipe, so it has to be harmless.
[private]
default:
    @just --list sim

# Launch the Autoware planning simulator with the AutoSDV vehicle (no sensors)
planning:
    #!/usr/bin/env bash
    source scripts/env.sh && \
    play_launch launch \
        --web-addr 0.0.0.0:8081 \
        autoware_launch planning_simulator.launch.xml \
        map_path:="$PWD/data/COSS-map-planning" \
        vehicle_model:=autosdv_vehicle \
        sensor_model:=autosdv_sensor_kit

# Launch the logging simulation, for rosbag replay testing
logging ARGS="":
    #!/usr/bin/env bash
    # `just launch ARGS="x:=y"` is a spelling this project documented for years
    # and which silently does nothing: in just, ARGS="..." AFTER a recipe name
    # is a positional value, not a variable assignment, so this parameter
    # receives the literal text `ARGS=x:=y` and hands it to play_launch, which
    # cannot use it. The launch then runs with DEFAULT arguments.
    #
    # That is invisible on a machine with an NVIDIA driver -- the default
    # pose_source:=cuda_ndt loads and works -- and fatal on one without, where
    # it panics with no CUDA library to load. Fail loudly instead.
    case "{{ARGS}}" in
        ARGS=*)
            echo "error: write the arguments positionally, without 'ARGS='." >&2
            echo >&2
            echo "    just RECIPE \"pose_source:=ndt launch_perception:=false\"" >&2
            echo >&2
            echo "ARGS= after the recipe name is passed through as a literal" >&2
            echo "launch argument, and the launch silently uses its defaults." >&2
            exit 2
            ;;
    esac
    source scripts/env.sh && \
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
    source scripts/env.sh && \
    parallel --line-buffer ::: \
        "just sim logging" \
        "sleep 40 && ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock -l -r 1.0" \
        "sleep 45 && ./scripts/rosbag/record_localization.sh"
