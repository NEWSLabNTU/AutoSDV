#!/bin/bash
# Download AR tag test data from Google Drive
# This script downloads the sample data for AR tag based localizer

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DATA_DIR="${PROJECT_ROOT}/data/ar_tag_test_real"

echo "=========================================="
echo "AR Tag Test Data Download Script"
echo "=========================================="
echo ""

# Check if data already exists
if [ -d "${DATA_DIR}/sample_data_for_ar_tag_based_localizer/ar_tag_based_localizer_sample_bag" ]; then
    echo "✓ Test data already exists at: ${DATA_DIR}"
    echo ""
    echo "Rosbag: ${DATA_DIR}/sample_data_for_ar_tag_based_localizer/ar_tag_based_localizer_sample_bag"
    echo "Map: ${DATA_DIR}/sample_data_for_ar_tag_based_localizer/map"
    echo ""
    echo "To re-download, delete the directory first:"
    echo "  rm -rf ${DATA_DIR}"
    exit 0
fi

# Create data directory
mkdir -p "${DATA_DIR}"
cd "${DATA_DIR}"

# Download using gdown
echo "→ Downloading AR tag test data from Google Drive..."
echo "  File ID: 1ynGJfm-KHPdhuV4GVYMb1x0KpVBnr_xY"
echo "  Size: ~912 MB"
echo ""

if ! command -v gdown &> /dev/null; then
    echo "ERROR: gdown is not installed"
    echo "Install with: pip3 install gdown"
    exit 1
fi

gdown 1ynGJfm-KHPdhuV4GVYMb1x0KpVBnr_xY -O sample_data_for_ar_tag_based_localizer.zip

# Extract
echo ""
echo "→ Extracting archive..."
unzip -q sample_data_for_ar_tag_based_localizer.zip

# Verify extraction
if [ -d "sample_data_for_ar_tag_based_localizer/ar_tag_based_localizer_sample_bag" ]; then
    echo ""
    echo "✓ Download and extraction complete!"
    echo ""
    echo "Rosbag: ${DATA_DIR}/sample_data_for_ar_tag_based_localizer/ar_tag_based_localizer_sample_bag"
    echo "Map: ${DATA_DIR}/sample_data_for_ar_tag_based_localizer/map"
    echo ""

    # Show rosbag info
    echo "→ Rosbag info:"
    cd sample_data_for_ar_tag_based_localizer/ar_tag_based_localizer_sample_bag
    ros2 bag info . 2>/dev/null || echo "  (ros2 bag info failed - ROS 2 environment may not be sourced)"
else
    echo ""
    echo "ERROR: Extraction failed or incomplete"
    exit 1
fi

echo ""
echo "=========================================="
echo "Test data is ready!"
echo "=========================================="
