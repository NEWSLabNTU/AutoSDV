# Rosbag recording, playback and download.
#
# Recipes here run from the REPO ROOT; see the note in just/tool.just.
set working-directory := '..'

# `just <module>` runs the module's FIRST recipe, so it has to be the harmless
# one -- without this, `just bag` would start recording.
[private]
default:
    @just --list bag

# Record outdoor sensor topics to rosbags/ directory
record:
    ./scripts/rosbag/record_outdoor.sh

#   just bag map car_b                        the latest rosbags/outdoor_*
#   just bag map car_b rosbags/outdoor_X      a recording of your choosing
#   just bag map car_a rosbags/outdoor_X --cpu -o DIR    further arguments go to lab3_map
# Writes BAG_glim/raw_map.pcd and glim_report.yaml (`just lab3 map`'s GLIM run).
# `record` keeps the LiDAR only as /sensing/lidar/concatenated/pointcloud: on the
# default CPU backend that is the driver's cloud moved into base_link, per-point
# times intact, so GLIM deskews it and takes LiDAR -> IMU from the bag's /tf_static
# (base_link -> the ZED IMU). Nothing in the bag names the LiDAR, since every car's
# calibration lists every frame, so the car's preset (sensor_suite) picks GLIM's
# sensor settings.
# Build the point cloud map from a recording with GLIM (default: the latest)
[positional-arguments]
map VEHICLE_ID *ARGS:
    #!/usr/bin/env bash
    set -euo pipefail
    vid=$1; shift
    preset="src/param/autoware_individual_params/individual_params/config/$vid/autosdv.preset.yaml"
    if [ ! -f "$preset" ]; then
        echo "usage: just bag map VEHICLE_ID [BAG] [lab3_map args] (car_a, car_b, car_c): no $preset" >&2
        exit 2
    fi
    suite=$(python3 -c 'import sys, yaml
    for item in yaml.safe_load(open(sys.argv[1]))["launch"]:
        a = item.get("arg", {})
        if a.get("name") == "sensor_suite":
            print(a.get("default", ""))' "$preset")
    case "$suite" in
        vlp16*) sensor=vlp16 ;;
        robin*) sensor=robin_w ;;
        *) echo "$vid's sensor_suite is '$suite', not a LiDAR GLIM is set up for (vlp16, robin_w)" >&2; exit 2 ;;
    esac
    if [ $# -gt 0 ] && [ -d "$1" ]; then
        bag=$1; shift
    else
        # the newest recording, not a map this recipe wrote beside one (BAG_glim)
        bag=$(ls -td rosbags/outdoor_* 2>/dev/null | grep -v '_glim$' | head -1 || true)
        if [ -z "$bag" ]; then
            echo "No outdoor recordings in rosbags/ (record one with: just bag record)" >&2
            exit 1
        fi
    fi
    if [ ! -f "$bag/metadata.yaml" ]; then
        echo "$bag has no metadata.yaml: the recording is still open or was not closed cleanly" >&2
        exit 1
    fi
    # A recording made with a topic list that keeps the driver's own cloud
    # instead is left to lab3_map's detection
    lidar=()
    if grep -q 'name: /sensing/lidar/concatenated/pointcloud$' "$bag/metadata.yaml"; then
        lidar=(--lidar-topic /sensing/lidar/concatenated/pointcloud)
    fi
    echo "Mapping $bag ($vid, $sensor)"
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_map "$bag" "${lidar[@]}" --sensor "$sensor" "$@"

# Play the most recent outdoor recording
play:
    #!/usr/bin/env bash
    LATEST=$(ls -td rosbags/outdoor_* 2>/dev/null | head -1); \
    if [ -z "$LATEST" ]; then \
        echo "No outdoor recordings found in rosbags/"; \
        exit 1; \
    fi; \
    echo "Playing: $LATEST"; \
    ros2 bag play "$LATEST" --clock

# Moved, because the name promised a generic downloader and delivered one
# specific drive. Kept as a pointer for a release; the recipes that name their
# scenario live in just/coss.just.
# just --list shows only the LAST comment line, so the description goes here.
# Deprecated: use `just coss download-rosbag`
download:
    @echo 'just bag download -> just coss download-rosbag (it fetches the COSS drive).'
    @just coss download-rosbag
