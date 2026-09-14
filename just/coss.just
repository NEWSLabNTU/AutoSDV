# The COSS Park scenario: the map, the recording, and the two simulations the
# tutorial is built on.
#
# These are convenience wrappers. The raw `play_launch` command each one runs is
# echoed before it runs, because the command is the thing worth learning and the
# recipe is only a way to avoid typing it forty times.
#
# Recipes here run from the REPO ROOT; see the note in just/tool.just.
set working-directory := '..'

map := 'data/COSS-map-planning'
rosbag := 'data/rosbags/outdoor_20251226_153115'

# `just <module>` runs the module's FIRST recipe, so it has to be harmless.
[private]
default:
    #!/usr/bin/env bash
    just --list coss --list-heading $'The COSS Park scenario.\n'
    echo
    echo "    The logging simulation needs two terminals, and that is the lesson:"
    echo
    echo "        terminal 1   just coss logging-sim     the stack, waiting on a clock"
    echo "        terminal 2   just coss play-rosbag     the recording, driving that clock"
    echo
    echo "    Without the second, nothing happens and nothing reports why."

# About 1.6 GB downloaded, 2.8 GB unpacked. Re-running verifies and exits.
# just --list shows only the LAST comment line, so the description goes here.
# Download the COSS drive recording (needed by logging-sim only)
download-rosbag:
    ./scripts/rosbag/download-test-rosbag.sh

# The planning component alone: no sensors, no localization, no GPU, no rosbag.
planning-sim:
    #!/usr/bin/env bash
    # Without a display, say so: play_launch supervises rviz2, so a crash on
    # startup is not one crash but a restart loop -- 58 of them in a minute the
    # first time this ran headless.
    args=()
    [ -n "${DISPLAY:-}" ] || args+=(rviz:=false)
    cmd=(play_launch launch --web-addr 0.0.0.0:8081
         autoware_launch planning_simulator.launch.xml
         map_path:="$PWD/{{ map }}"
         vehicle_model:=autosdv_vehicle
         sensor_model:=autosdv_sensor_kit "${args[@]}")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# `gpu` selects cuda_ndt and perception; the default runs on any laptop.
# just --list shows only the LAST comment line, so the description goes here.
# The full stack fed from a recording. Feed it with `just coss play-rosbag`.
logging-sim backend='cpu':
    #!/usr/bin/env bash
    # The CPU path is the default deliberately: pose_source:=ndt needs no CUDA,
    # and launch_perception:=false skips the TensorRT engine build that costs
    # 10-30 minutes on a machine that has never run it.
    case '{{ backend }}' in
        cpu) args=(pose_source:=ndt launch_perception:=false) ;;
        gpu) args=(pose_source:=cuda_ndt) ;;
        *)   echo "unknown backend '{{ backend }}': expected cpu or gpu" >&2; exit 2 ;;
    esac
    [ -n "${DISPLAY:-}" ] || args+=(rviz:=false)
    cmd=(play_launch launch --web-addr 0.0.0.0:8081
         autosdv_launch logging_simulation.launch.yaml
         map_path:="$PWD/{{ map }}" "${args[@]}")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# --clock is not optional: the stack runs on simulated time, and without it the
# clock never advances and nothing happens, with no error anywhere.
# just --list shows only the LAST comment line, so the description goes here.
# Play the COSS recording into a running logging-sim (second terminal)
play-rosbag rate='1.0':
    #!/usr/bin/env bash
    if [ ! -f '{{ rosbag }}/metadata.yaml' ]; then
        echo "no recording at {{ rosbag }} -- run: just coss download-rosbag" >&2
        exit 1
    fi
    cmd=(ros2 bag play '{{ rosbag }}' --clock -r '{{ rate }}')
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# Stack, pose seed, replay and a metrics report, unattended. See `just demo`.
demo:
    @just demo run
