#!/bin/bash
#
# Download and extract test rosbag for AutoSDV testing
#
# This script downloads the outdoor_20251226_153115 rosbag from Synology Drive
# and extracts it to data/rosbags/
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Configuration
SHARE_URL="https://newslabn.csie.ntu.edu.tw/drive/d/s/16OFeybLzjWqa1S6lryLhYaSNQZ5RQtZ/dHSz8tnPGB0V2njaH5qFdmCo1996Zvlg-zLegeIV83gw"
OUTPUT_DIR="$REPO_ROOT/data/rosbags"
ZIP_NAME="outdoor_20251226_153115.zip"
ROSBAG_NAME="outdoor_20251226_153115"
DB3_FILE="outdoor_20251226_153115_0.db3"

# SHA256 checksum of the db3 file for verification
EXPECTED_SHA256="58aaa368957d5448b416e06c886cda784dc63754d56a2c65b7655ae813278e43"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Verify SHA256 checksum
verify_checksum() {
    local file="$1"
    local expected="$2"

    if [ ! -f "$file" ]; then
        return 1
    fi

    local actual
    actual=$(sha256sum "$file" | cut -d' ' -f1)

    if [ "$actual" = "$expected" ]; then
        return 0
    else
        log_warn "Checksum mismatch:"
        log_warn "  Expected: $expected"
        log_warn "  Actual:   $actual"
        return 1
    fi
}

# Check if rosbag already exists with correct checksum
DB3_PATH="$OUTPUT_DIR/$ROSBAG_NAME/$DB3_FILE"
if [ -f "$DB3_PATH" ]; then
    log_info "Checking existing rosbag..."
    if verify_checksum "$DB3_PATH" "$EXPECTED_SHA256"; then
        log_info "Rosbag already exists with correct checksum, skipping download"
        log_info "Location: $OUTPUT_DIR/$ROSBAG_NAME"
        exit 0
    else
        log_warn "Existing rosbag has incorrect checksum, re-downloading..."
        rm -rf "$OUTPUT_DIR/$ROSBAG_NAME"
    fi
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Download
log_info "Downloading rosbag..."
"$SCRIPT_DIR/utils/synology-drive-download.sh" -f "$SHARE_URL" "$OUTPUT_DIR"

# Extract
ZIP_PATH="$OUTPUT_DIR/$ZIP_NAME"
if [ -f "$ZIP_PATH" ]; then
    log_info "Extracting $ZIP_NAME..."
    unzip -o "$ZIP_PATH" -d "$OUTPUT_DIR"

    # Verify checksum after extraction
    log_info "Verifying checksum..."
    if verify_checksum "$DB3_PATH" "$EXPECTED_SHA256"; then
        log_info "Checksum verified successfully"
    else
        log_error "Checksum verification failed!"
        rm -rf "$OUTPUT_DIR/$ROSBAG_NAME"
        rm -f "$ZIP_PATH"
        exit 1
    fi

    # Clean up zip file
    log_info "Cleaning up zip file..."
    rm -f "$ZIP_PATH"

    log_info "Rosbag extracted to: $OUTPUT_DIR/$ROSBAG_NAME"
else
    log_error "Download failed, zip file not found"
    exit 1
fi

log_info "Done!"
echo ""
echo "To play the rosbag:"
echo "  ros2 bag play $OUTPUT_DIR/$ROSBAG_NAME"
