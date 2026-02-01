#!/bin/bash
# Download Leo Drive Bus-ODD Dataset (Autoware sample data with camera streams)
#
# Dataset contains:
#   - 3x Lucid Vision Triton 5.4MP cameras (left, right, front)
#   - 1x VLP16 + 2x VLP32C LiDARs
#   - Applanix POS LV 120 GNSS/INS
#
# Size: ~10.9GB
# Source: https://autowarefoundation.github.io/autoware-documentation/main/datasets/
#
# Usage: ./scripts/leodrive-dataset/download.sh [output_dir]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

OUTPUT_DIR="${1:-$REPO_ROOT/data/leodrive-busood}"
S3_BUCKET="s3://autoware-files/collected_data/2022-08-22_leo_drive_isuzu_bags/"

# Check for awscli
if ! command -v aws &> /dev/null; then
    echo "Error: awscli is not installed."
    echo "Install with: sudo apt install awscli"
    exit 1
fi

echo "=== Leo Drive Bus-ODD Dataset Downloader ==="
echo "Source: $S3_BUCKET"
echo "Destination: $OUTPUT_DIR"
echo "Size: ~10.9GB"
echo ""

mkdir -p "$OUTPUT_DIR"

echo "Starting download (existing files will be skipped)..."
echo ""

# aws s3 sync features:
#   - Skips files that already exist with same size/mtime
#   - --no-sign-request: anonymous access (public bucket)
#   - --no-progress: cleaner output for scripts (remove for progress bars)
aws s3 sync "$S3_BUCKET" "$OUTPUT_DIR" \
    --no-sign-request \
    --only-show-errors

echo ""
echo "Download complete: $OUTPUT_DIR"
echo ""
echo "Contents:"
ls -lh "$OUTPUT_DIR"
