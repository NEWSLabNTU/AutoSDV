#!/usr/bin/env bash
# scripts/2dlidar/download-sample-rosbag.sh
# Fetch the official Autoware replay-simulation assets:
#   sample-map-rosbag  (PCD + lanelet2)   id 1A-8BvYRX3DhSzkAnOcGWFw5T30xTlwZI
#   sample-rosbag      (db3, no camera)   id 1VnwJx9tI3kI_cTLzP61ktuAJ1ChgygpG
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$SCRIPT_DIR/../../data/sample-rosbag-replay"
mkdir -p "$OUT"
cd "$OUT"

GDOWN="${GDOWN:-$HOME/.local/bin/gdown}"

if [ ! -d sample-map-rosbag ]; then
    "$GDOWN" 'https://docs.google.com/uc?export=download&id=1A-8BvYRX3DhSzkAnOcGWFw5T30xTlwZI' -O sample-map-rosbag.zip
    unzip -o sample-map-rosbag.zip && rm sample-map-rosbag.zip
fi
if [ ! -d sample-rosbag ]; then
    "$GDOWN" 'https://docs.google.com/uc?export=download&id=1VnwJx9tI3kI_cTLzP61ktuAJ1ChgygpG' -O sample-rosbag.zip
    unzip -o sample-rosbag.zip && rm sample-rosbag.zip
fi

echo "--- checksums (pin these in versions.yaml when stable) ---"
find . -name '*.pcd' -o -name '*.db3' | xargs sha256sum
echo "--- bag contents ---"
set +u
source /opt/ros/humble/setup.bash
set -u
ros2 bag info sample-rosbag
