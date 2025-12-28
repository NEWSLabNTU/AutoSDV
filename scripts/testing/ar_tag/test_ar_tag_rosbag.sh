#!/bin/bash
# Test AR tag localization with rosbag playback
# This script launches the AutoSDV system with AR tag localization and plays test data

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DATA_DIR="${PROJECT_ROOT}/data/ar_tag_test_real/sample_data_for_ar_tag_based_localizer"
ROSBAG_DIR="${DATA_DIR}/ar_tag_based_localizer_sample_bag"
MAP_DIR="${DATA_DIR}/map"

# Default parameters
PLAYBACK_RATE=1.0
RECORD_OUTPUT=false
OUTPUT_DIR="${PROJECT_ROOT}/tmp/ar_tag_test_$(date +%Y%m%d_%H%M%S)"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -r|--rate)
            PLAYBACK_RATE="$2"
            shift 2
            ;;
        --record)
            RECORD_OUTPUT=true
            shift
            ;;
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Test AR tag localization with rosbag playback"
            echo ""
            echo "Options:"
            echo "  -r, --rate RATE    Playback rate (default: 1.0)"
            echo "  --record           Record output to rosbag"
            echo "  -o, --output DIR   Output directory for recordings"
            echo "  -h, --help         Show this help message"
            echo ""
            echo "Example:"
            echo "  $0 --rate 0.5 --record"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

echo "=========================================="
echo "AR Tag Rosbag Testing Script"
echo "=========================================="
echo ""

# Check if test data exists
if [ ! -d "${ROSBAG_DIR}" ]; then
    echo "ERROR: Test rosbag not found at: ${ROSBAG_DIR}"
    echo ""
    echo "Run the download script first:"
    echo "  ./scripts/testing/download_ar_tag_data.sh"
    exit 1
fi

if [ ! -d "${MAP_DIR}" ]; then
    echo "ERROR: Map not found at: ${MAP_DIR}"
    exit 1
fi

echo "✓ Test data found"
echo "  Rosbag: ${ROSBAG_DIR}"
echo "  Map: ${MAP_DIR}"
echo ""
echo "Test parameters:"
echo "  Playback rate: ${PLAYBACK_RATE}x"
echo "  Record output: ${RECORD_OUTPUT}"
if [ "${RECORD_OUTPUT}" = true ]; then
    echo "  Output directory: ${OUTPUT_DIR}"
fi
echo ""

# Check if ROS 2 is sourced
if [ -z "${ROS_DISTRO}" ]; then
    echo "ERROR: ROS 2 environment not sourced"
    echo "Source your workspace:"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  source install/setup.bash"
    exit 1
fi

echo "✓ ROS 2 environment: ${ROS_DISTRO}"
echo ""

# Create output directory if recording
if [ "${RECORD_OUTPUT}" = true ]; then
    mkdir -p "${OUTPUT_DIR}"
    echo "→ Output directory created: ${OUTPUT_DIR}"
    echo ""
fi

# Show rosbag info
echo "→ Rosbag information:"
ros2 bag info "${ROSBAG_DIR}"
echo ""

echo "=========================================="
echo "Starting test..."
echo "=========================================="
echo ""
echo "1. Launch AutoSDV with AR tag localization in separate terminal:"
echo ""
echo "   ros2 launch autosdv_launch autosdv.launch.yaml \\"
echo "     pose_source:=artag \\"
echo "     map_path:=${MAP_DIR} \\"
echo "     use_gnss:=false"
echo ""
echo "2. Wait for system to initialize (30-60 seconds)"
echo ""
echo "3. Press ENTER to start rosbag playback..."
read -r

# Play rosbag
echo ""
echo "→ Playing rosbag at ${PLAYBACK_RATE}x speed..."
echo ""

if [ "${RECORD_OUTPUT}" = true ]; then
    # Record output while playing
    echo "→ Recording output to: ${OUTPUT_DIR}"
    ros2 bag record -o "${OUTPUT_DIR}" \
        /localization/pose_estimator/pose_with_covariance \
        /localization/twist_estimator/twist_with_covariance \
        /localization/pose_with_covariance \
        /localization/ekf_localizer/ekf_pose_with_covariance \
        /tf \
        /tf_static &
    RECORD_PID=$!
    
    # Give recorder time to start
    sleep 2
    
    # Play rosbag
    cd "${ROSBAG_DIR}"
    ros2 bag play . -r "${PLAYBACK_RATE}"
    
    # Stop recording
    kill ${RECORD_PID}
    wait ${RECORD_PID} 2>/dev/null || true
    
    echo ""
    echo "✓ Recording saved to: ${OUTPUT_DIR}"
else
    # Just play
    cd "${ROSBAG_DIR}"
    ros2 bag play . -r "${PLAYBACK_RATE}"
fi

echo ""
echo "=========================================="
echo "Playback complete!"
echo "=========================================="
echo ""
echo "Monitor localization output with:"
echo "  ros2 topic echo /localization/pose_with_covariance"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo ""
if [ "${RECORD_OUTPUT}" = true ]; then
    echo "Analyze recorded data:"
    echo "  ros2 bag info ${OUTPUT_DIR}"
    echo ""
fi
