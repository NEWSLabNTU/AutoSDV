#!/usr/bin/env bash
# Record rosbag for visual map creation
# Usage: ./record.sh [output_dir] [duration_seconds]
#
# Requirements:
# - ZED camera running (ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm)
# - H264 encoding enabled in ZED config (for optimal map creation)
#
# Recording tips for good maps:
# 1. Start with 10+ seconds stationary (for initialization)
# 2. Move slowly and smoothly
# 3. Create closed loops (return to start)
# 4. Ensure feature-rich environment (avoid blank walls)
# 5. Maintain consistent lighting

set -eo pipefail

# Colors
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m'

# Default values
OUTPUT_DIR="${1:-$(pwd)/visual_map_recording_$(date +%Y%m%d_%H%M%S)}"
DURATION="${2:-}"
CAMERA_NS="${CAMERA_NS:-/sensing/camera/zedxm/zed_node}"

# Topics to record for visual mapping
# Note: Adjust these based on your ZED SDK version and configuration
TOPICS=(
    # Stereo images (prefer H264 if available, fall back to raw)
    "${CAMERA_NS}/left/image_rect_color"
    "${CAMERA_NS}/right/image_rect_color"
    # Camera info (required for calibration)
    "${CAMERA_NS}/left/camera_info"
    "${CAMERA_NS}/right/camera_info"
    # IMU data (required for visual-inertial odometry)
    "${CAMERA_NS}/imu/data"
)

# Check if ROS 2 is sourced
if ! command -v ros2 &> /dev/null; then
    printf "${RED}Error:${NC} ROS 2 not sourced. Run: source /opt/ros/humble/setup.bash\n"
    exit 1
fi

# Check if ZED camera topics are available
printf "${YELLOW}→${NC} Checking camera topics...\n"
MISSING_TOPICS=()
for topic in "${TOPICS[@]}"; do
    if ! ros2 topic list 2>/dev/null | grep -qF "$topic"; then
        MISSING_TOPICS+=("$topic")
    fi
done

if [[ ${#MISSING_TOPICS[@]} -gt 0 ]]; then
    printf "${RED}Error:${NC} Missing camera topics:\n"
    for topic in "${MISSING_TOPICS[@]}"; do
        printf "  - %s\n" "$topic"
    done
    printf "\nMake sure ZED camera is running:\n"
    printf "  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm\n"
    exit 1
fi

printf "${GREEN}✓${NC} All camera topics available\n"

# Show recording info
printf "\n${YELLOW}Recording Configuration:${NC}\n"
printf "  Output: %s\n" "$OUTPUT_DIR"
printf "  Topics:\n"
for topic in "${TOPICS[@]}"; do
    hz=$(ros2 topic hz "$topic" --window 5 2>/dev/null | head -1 | grep -oE '[0-9]+\.[0-9]+' || echo "?")
    printf "    - %s (~%s Hz)\n" "$topic" "$hz"
done

if [[ -n "$DURATION" ]]; then
    printf "  Duration: %s seconds\n" "$DURATION"
else
    printf "  Duration: Until Ctrl+C\n"
fi

# Recording tips
printf "\n${YELLOW}Recording Tips:${NC}\n"
printf "  1. Keep camera STATIONARY for first 10 seconds\n"
printf "  2. Move SLOWLY (walking speed)\n"
printf "  3. Create CLOSED LOOPS (return to start)\n"
printf "  4. Cover the area you want to localize in\n"

# Confirm start
printf "\n"
read -p "Press Enter to start recording (Ctrl+C to cancel)..."

# Build ros2 bag command
CMD="ros2 bag record -o ${OUTPUT_DIR}"
for topic in "${TOPICS[@]}"; do
    CMD+=" ${topic}"
done

if [[ -n "$DURATION" ]]; then
    CMD+=" --max-bag-duration ${DURATION}"
fi

# Start recording
printf "\n${GREEN}Recording started...${NC}\n"
printf "Press Ctrl+C to stop\n\n"

eval "$CMD"

printf "\n${GREEN}✓${NC} Recording saved to: %s\n" "$OUTPUT_DIR"
printf "\nNext step: Create map with:\n"
printf "  ./scripts/visual-map/create-map.sh %s\n" "$OUTPUT_DIR"
