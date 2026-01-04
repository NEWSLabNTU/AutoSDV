#!/usr/bin/env bash
# Install Blickfeld Scanner Library
# With EULA acceptance prompt

set -e

DATA_DIR="${DATA_DIR:-$(dirname "$0")/../../data}"
ARCH="${ARCH:-$(uname -m)}"

echo "Blickfeld Scanner Library Installation"
echo "======================================="
echo ""

# Determine URL based on architecture
if [[ "$ARCH" == "x86_64" ]]; then
    URL="https://github.com/NEWSLabNTU/blickfeld-scanner-lib/releases/download/v2.20.6-newslab1/blickfeld-scanner-lib_2.20.6-1_amd64.deb"
    DEB="$DATA_DIR/blickfeld-scanner-lib/blickfeld-scanner-lib_2.20.6-1_amd64.deb"
    ARCH_NAME="AMD64"
elif [[ "$ARCH" == "aarch64" ]]; then
    URL="https://github.com/NEWSLabNTU/blickfeld-scanner-lib/releases/download/v2.20.6-newslab1/blickfeld-scanner-lib_2.20.6-1_arm64.deb"
    DEB="$DATA_DIR/blickfeld-scanner-lib/blickfeld-scanner-lib_2.20.6-1_arm64.deb"
    ARCH_NAME="ARM64"
else
    echo "Error: Unsupported architecture: $ARCH"
    exit 1
fi

# Show license information
echo "Package: Blickfeld Scanner Library v2.20.6-newslab1"
echo "Architecture: $ARCH_NAME"
echo "Source: https://github.com/NEWSLabNTU/blickfeld-scanner-lib"
echo ""
echo "This is a modified version maintained by NEWSLab NTU."
echo "Original software by Blickfeld GmbH."
echo ""
echo "License information:"
echo "  https://github.com/NEWSLabNTU/blickfeld-scanner-lib#license"
echo ""

# Check if license acceptance was already provided via environment variable
if [[ "${AUTOSDV_ACCEPT_BLICKFELD_EULA:-0}" == "1" ]]; then
    echo "License accepted via AUTOSDV_ACCEPT_BLICKFELD_EULA environment variable."
elif [[ -t 0 ]]; then
    # Interactive mode: ask for acceptance
    read -p "Do you accept the license terms? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "License not accepted. Skipping Blickfeld installation."
        echo ""
        echo "Note: You can install it later by running: just blickfeld"
        exit 1
    fi
else
    # Non-interactive mode without environment variable
    echo "Error: Non-interactive mode requires AUTOSDV_ACCEPT_BLICKFELD_EULA=1"
    exit 1
fi

echo ""
echo "Installing Blickfeld Scanner Library..."

# Download if not exists
mkdir -p "$DATA_DIR/blickfeld-scanner-lib"
if [[ ! -f "$DEB" ]]; then
    echo "Downloading from GitHub..."
    curl -fSL -o "$DEB" "$URL"
fi

# Install
echo "Installing package..."
sudo apt-get install -y "$DEB"

echo ""
echo "✓ Blickfeld Scanner Library installed successfully!"
echo ""
echo "This package is required for Blickfeld Cube1 LiDAR."
