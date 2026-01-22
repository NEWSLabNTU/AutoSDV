#!/usr/bin/env bash
set -e

echo "→ Installing Autoware Localrepo 1.5.0-1..."

REPO_URL_BASE="https://github.com/NEWSLabNTU/autoware-localrepo/releases/download/1.5.0-1"
DEB_FILE=""
SHA256SUM=""

# Checksums
SHA256SUM_UBUNTU2204="9f433f7ae4642c9501b9b1f53853a3d724627cc2e85088ac6a2e2e21775898ea"
SHA256SUM_JETPACK62="5c50148e9d9ad5426e92fdee68d2ab22ca23f37d9de83fe9e7fd06db330f0ae0"

ARCH=$(uname -m)

if [[ "$ARCH" == "x86_64" ]]; then
    echo "  Detected architecture: amd64 (x86_64)"
    DEB_FILE="autoware-localrepo-1-5-0_1.5.0-1ubuntu2204_all.deb"
    SHA256SUM="${SHA256SUM_UBUNTU2204}"
elif [[ "$ARCH" == "aarch64" ]]; then
    echo "  Detected architecture: arm64 (aarch64) - Assuming JetPack 6.2 compatibility"
    DEB_FILE="autoware-localrepo-1-5-0_1.5.0-1jetpack62_all.deb"
    SHA256SUM="${SHA256SUM_JETPACK62}"
else
    echo "Error: Unsupported architecture: $ARCH"
    exit 1
fi

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
DATA_DIR="${DATA_DIR:-${SCRIPT_DIR}/../../data}"
DEB_DOWNLOAD_DIR="${DATA_DIR}/autoware-debian"
mkdir -p "${DEB_DOWNLOAD_DIR}"

DOWNLOAD_URL="${REPO_URL_BASE}/${DEB_FILE}"
TEMP_DEB="${DEB_DOWNLOAD_DIR}/${DEB_FILE}"

# Install aria2c if not already installed
if ! command -v aria2c &> /dev/null; then
    echo "  Installing aria2c for parallel downloads..."
    sudo apt update
    sudo apt install -y aria2
fi

echo "  Downloading ${DOWNLOAD_URL} to ${DEB_DOWNLOAD_DIR}..."

DOWNLOAD_REQUIRED=false
if [[ -f "$TEMP_DEB" ]]; then
    echo "  File ${DEB_FILE} already exists. Verifying checksum..."
    if [[ -n "$SHA256SUM" ]]; then
        ACTUAL_SHA256SUM=$(sha256sum "$TEMP_DEB" | awk '{print $1}')
        if [[ "$ACTUAL_SHA256SUM" == "$SHA256SUM" ]]; then
            echo "  Checksum matches. Skipping download."
        else
            echo "--------------------------------------------------------------------------------"
            echo "  ERROR: Checksum mismatch for existing file: ${TEMP_DEB}"
            echo "  Expected SHA256: ${SHA256SUM}"
            echo "  Actual SHA256:   ${ACTUAL_SHA256SUM}"
            echo "  This likely means the downloaded file is corrupted or has been modified."
            echo "  Please manually remove or rename the file, or update the script with the"
            echo "  correct checksum if you are using a custom build."
            echo "  Exiting to prevent accidental deletion of a potentially custom file."
            echo "--------------------------------------------------------------------------------"
            exit 1 # Exit with an error
        fi
    else
        echo "  No checksum provided for verification. Skipping download assuming integrity."
    fi
else
    echo "  File ${DEB_FILE} not found. Downloading..."
    DOWNLOAD_REQUIRED=true
fi

if "$DOWNLOAD_REQUIRED"; then
    if [[ -n "$SHA256SUM" ]]; then
        aria2c "${DOWNLOAD_URL}" --dir="${DEB_DOWNLOAD_DIR}" --out="${DEB_FILE}" --checksum=sha-256="${SHA256SUM}" -x 10 -s 10 -k 1M
    else
        aria2c "${DOWNLOAD_URL}" --dir="${DEB_DOWNLOAD_DIR}" --out="${DEB_FILE}" -x 10 -s 10 -k 1M
    fi
else
    echo "  Using existing file: ${TEMP_DEB}"
fi

echo "  Installing Autoware localrepo..."
sudo apt update
sudo apt install -y "$TEMP_DEB"

# Run setup-prerequisites.sh
if [ -f /usr/share/autoware/setup-prerequisites.sh ]; then
    echo "  Running /usr/share/autoware/setup-prerequisites.sh..."
    sudo /usr/share/autoware/setup-prerequisites.sh
else
    echo "  Warning: /usr/share/autoware/setup-prerequisites.sh not found. Skipping."
fi

echo "  Updating apt cache after localrepo installation..."
sudo apt update

# Fix: If time-daemon is not provided by any package, install chrony.
# This is to satisfy the time-daemon dependency of autoware-full-1-5-0.
if ! apt-cache search --names-only '^time-daemon$' | grep -q 'time-daemon'; then
    echo "  'time-daemon' not found in any package. Installing 'chrony' as a replacement."
    sudo apt install -y chrony
fi

echo "  Installing autoware-full-1-5-0..."
sudo apt install -y autoware-full-1-5-0

echo "✓ Autoware Localrepo 1.5.0-1 and Autoware Full 1.5.0 installed successfully."

