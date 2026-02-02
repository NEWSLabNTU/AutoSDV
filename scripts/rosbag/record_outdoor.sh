#!/bin/bash
# Record outdoor sensor topics for AutoSDV
# Usage: ./scripts/rosbag/record_outdoor.sh [output_dir]
#
# This script records sensor topics listed in outdoor_topics.txt plus /clock
# for time synchronization during playback.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
TOPICS_FILE="$SCRIPT_DIR/outdoor_topics.txt"

# Default output directory with timestamp
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
DEFAULT_OUTPUT_DIR="$PROJECT_DIR/rosbags/outdoor_$TIMESTAMP"
OUTPUT_DIR="${1:-$DEFAULT_OUTPUT_DIR}"

# Check if topics file exists
if [[ ! -f "$TOPICS_FILE" ]]; then
    echo "Error: Topics file not found: $TOPICS_FILE"
    exit 1
fi

# Create output directory if it doesn't exist
mkdir -p "$(dirname "$OUTPUT_DIR")"

# Read topics from file (skip empty lines and comments)
TOPICS=()
while IFS= read -r line || [[ -n "$line" ]]; do
    # Skip empty lines and comments
    [[ -z "$line" || "$line" =~ ^[[:space:]]*# ]] && continue
    TOPICS+=("$line")
done < "$TOPICS_FILE"

# Add /clock topic for time synchronization during playback
TOPICS+=("/clock")

echo "Recording ${#TOPICS[@]} topics to: $OUTPUT_DIR"
echo "Topics:"
for topic in "${TOPICS[@]}"; do
    echo "  - $topic"
done
echo ""
echo "Press Ctrl+C to stop recording..."
echo ""

# Build ros2 bag record command
# --use-sim-time is not set since we're recording real sensor data
# /clock topic is included for playback synchronization
ros2 bag record \
    --output "$OUTPUT_DIR" \
    --storage sqlite3 \
    "${TOPICS[@]}"
