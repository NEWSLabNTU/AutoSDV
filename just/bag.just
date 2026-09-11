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

# Installs synology-dl via cargo if it is not found.
# just --list shows only the LAST comment line, so the description goes here.
# Download the test rosbag from Synology Drive (~2.8 GB)
download:
    ./scripts/rosbag/download-test-rosbag.sh
