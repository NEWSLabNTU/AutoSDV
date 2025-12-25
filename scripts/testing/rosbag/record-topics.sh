#!/usr/bin/env bash

# Script to record ROS2 topics from a file list
# Usage: ./record-topics.sh <topics_file> [optional_output_name]

# Check if argument is provided
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <topics_file> [optional_output_name]"
    echo "  <topics_file>      : Text file containing list of topics to record (one per line)"
    echo "  [optional_output_name] : Optional name for the bag output directory"
    exit 1
fi

TOPICS_FILE="$1"
OUTPUT_NAME="$2"

# Check if topics file exists
if [ ! -f "$TOPICS_FILE" ]; then
    echo "Error: Topics file '$TOPICS_FILE' not found"
    exit 1
fi

# Read topics from file into an array
readarray -t TOPICS < "$TOPICS_FILE"

# Remove empty lines and comment lines (lines starting with #)
FILTERED_TOPICS=()
for topic in "${TOPICS[@]}"; do
    # Skip empty lines and comment lines
    if [[ -n "$topic" && ! "$topic" =~ ^[[:space:]]*# ]]; then
        # Trim whitespace from both ends
        trimmed_topic=$(echo "$topic" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')
        FILTERED_TOPICS+=("$trimmed_topic")
    fi
done

# Check if we have any topics to record
if [ ${#FILTERED_TOPICS[@]} -eq 0 ]; then
    echo "Error: No valid topics found in file '$TOPICS_FILE'"
    exit 1
fi

# Prepare output directory name
if [ -n "$OUTPUT_NAME" ]; then
    OUTPUT_DIR="$OUTPUT_NAME"
else
    OUTPUT_DIR="bag-$(date +%Y%m%d-%H%M%S)"
fi

echo "Recording topics from file: $TOPICS_FILE"
echo "Output directory: $OUTPUT_DIR"
echo "Topics to record:"
for topic in "${FILTERED_TOPICS[@]}"; do
    echo "  - $topic"
done
echo ""
echo "Starting recording..."

# Run ros2 bag record with the topics
ros2 bag record -o "$OUTPUT_DIR" "${FILTERED_TOPICS[@]}"
