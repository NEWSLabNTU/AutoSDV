#!/usr/bin/env bash
# Create visual maps from recorded rosbag
# Usage: ./create-map.sh <rosbag_dir> [output_dir]
#
# This script runs the Isaac ROS mapping pipeline to create:
# - cuVSLAM map (for visual odometry/tracking)
# - cuVGL map (for global localization)
# - Occupancy grid map (for navigation)

set -eo pipefail

# Colors
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m'

# Arguments
ROSBAG_DIR="$1"
OUTPUT_DIR="${2:-$(dirname "$ROSBAG_DIR")/visual_map_$(date +%Y%m%d_%H%M%S)}"

# Validate arguments
if [[ -z "$ROSBAG_DIR" ]]; then
    printf "${RED}Error:${NC} Missing rosbag directory\n"
    printf "\nUsage: %s <rosbag_dir> [output_dir]\n" "$0"
    printf "\nExample:\n"
    printf "  %s ./visual_map_recording_20250127_120000\n" "$0"
    exit 1
fi

if [[ ! -d "$ROSBAG_DIR" ]]; then
    printf "${RED}Error:${NC} Rosbag directory not found: %s\n" "$ROSBAG_DIR"
    exit 1
fi

# Check if ROS 2 is sourced
if ! command -v ros2 &> /dev/null; then
    printf "${RED}Error:${NC} ROS 2 not sourced. Run: source /opt/ros/humble/setup.bash\n"
    exit 1
fi

# Check if isaac_mapping_ros is available
if ! ros2 pkg list 2>/dev/null | grep -q "isaac_mapping_ros"; then
    printf "${RED}Error:${NC} isaac_mapping_ros not installed\n"
    printf "Run: ./setup.sh isaac-ros\n"
    exit 1
fi

# Get absolute paths
ROSBAG_DIR=$(realpath "$ROSBAG_DIR")
OUTPUT_DIR=$(realpath -m "$OUTPUT_DIR")

printf "${YELLOW}Visual Map Creation${NC}\n"
printf "==================\n\n"
printf "Input rosbag:  %s\n" "$ROSBAG_DIR"
printf "Output folder: %s\n" "$OUTPUT_DIR"

# Check rosbag info
printf "\n${YELLOW}→${NC} Checking rosbag...\n"
ros2 bag info "$ROSBAG_DIR" 2>/dev/null | head -20

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Run the mapping pipeline
printf "\n${YELLOW}→${NC} Starting map creation pipeline...\n"
printf "This may take a while depending on the rosbag size.\n\n"

# The create_map_offline.py script handles the full pipeline:
# 1. Extract images and metadata (EDEX format)
# 2. Run cuVSLAM to compute poses
# 3. Optimize poses with loop closure
# 4. Generate cuVGL keyframe database
# 5. Run depth estimation (Foundation Stereo)
# 6. Generate occupancy grid (Nvblox)

ros2 run isaac_mapping_ros create_map_offline.py \
    --sensor_data_bag="$ROSBAG_DIR" \
    --base_output_folder="$OUTPUT_DIR"

# Check outputs
printf "\n${YELLOW}→${NC} Checking outputs...\n"

OUTPUTS=(
    "cuvslam_map:cuVSLAM landmarks for tracking"
    "cuvgl_map:cuVGL keyframes for global localization"
    "occupancy_map:2D occupancy grid for navigation"
)

ALL_OK=true
for output in "${OUTPUTS[@]}"; do
    dir="${output%%:*}"
    desc="${output#*:}"
    if [[ -d "$OUTPUT_DIR/$dir" ]]; then
        printf "${GREEN}✓${NC} %s (%s)\n" "$dir" "$desc"
    else
        printf "${YELLOW}○${NC} %s not created (%s)\n" "$dir" "$desc"
        ALL_OK=false
    fi
done

if $ALL_OK; then
    printf "\n${GREEN}✓ Map creation complete!${NC}\n"
else
    printf "\n${YELLOW}⚠${NC} Some outputs may be missing. Check the logs above.\n"
fi

printf "\nOutput directory: %s\n" "$OUTPUT_DIR"
printf "\nTo use the map for localization:\n"
printf "  just launch pose_source:=visual visual_map_dir:=%s\n" "$OUTPUT_DIR"
