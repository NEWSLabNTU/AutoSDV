#!/usr/bin/env bash
# Install ZED SDK
# Converted from ansible/roles/zed_sdk/tasks/main.yaml

set -e

DATA_DIR="${DATA_DIR:-$(dirname "$0")/../../data}"
ARCH="${ARCH:-$(uname -m)}"

echo "Installing ZED SDK..."
echo "  Architecture: ${ARCH}"
echo "  Data directory: ${DATA_DIR}"

# Create directory
mkdir -p "${DATA_DIR}/zed-sdk"

# Download and install architecture-specific package
if [[ "${ARCH}" == "x86_64" ]]; then
    DEB_URL="https://github.com/jerry73204/zed-sdk-debian-package/releases/download/4.2-1/zed-sdk_4.2-1_amd64.deb"
    DEB_FILE="${DATA_DIR}/zed-sdk/zed-sdk_4.2-1_amd64.deb"
elif [[ "${ARCH}" == "aarch64" ]]; then
    DEB_URL="https://github.com/jerry73204/zed-sdk-debian-package/releases/download/4.2-1/zed-sdk_4.2-1_arm64.deb"
    DEB_FILE="${DATA_DIR}/zed-sdk/zed-sdk_4.2-1_arm64.deb"
else
    echo "Error: Unsupported architecture: ${ARCH}"
    exit 1
fi

# Download if not exists
if [[ ! -f "${DEB_FILE}" ]]; then
    echo "Downloading ZED SDK..."
    curl -fSL -o "${DEB_FILE}" "${DEB_URL}"
fi

# Install the package
echo "Installing ZED SDK package..."
sudo apt-get install -y "${DEB_FILE}"

# Download AI models
echo "Downloading ZED AI models..."
if command -v zed_download_ai_models &> /dev/null; then
    sudo zed_download_ai_models
else
    echo "Warning: zed_download_ai_models not found, skipping AI models download"
fi

# Fix permissions on the ZED SDK directory if it exists
if [[ -d /usr/local/zed ]]; then
    echo "Fixing ZED SDK directory permissions..."
    sudo chown -R root:root /usr/local/zed
    sudo chmod -R 755 /usr/local/zed
fi

echo "ZED SDK installation complete."
