#!/usr/bin/env bash
# Install Autoware Debian packages
# Converted from ansible/roles/autoware_debian/tasks/main.yaml

set -e

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
DATA_DIR="${DATA_DIR:-${SCRIPT_DIR}/../../data}"
ARCH="${ARCH:-$(uname -m)}"

# Source version information from versions.yaml
source "${SCRIPT_DIR}/../../scripts/version/export-versions.sh"

echo "Installing Autoware Debian packages..."
echo "  Architecture: ${ARCH}"
echo "  Data directory: ${DATA_DIR}"
echo "  Autoware version: ${AUTOWARE_VERSION}"
echo "  Package version: ${AUTOWARE_PACKAGE_VERSION}"

# Create directory
mkdir -p "${DATA_DIR}/autoware-debian"

# URL-encode the release tag (replace / with %2F)
ROSDEBIAN_RELEASE_ENCODED="${AUTOWARE_ROSDEBIAN_RELEASE//\//%2F}"

# Detect JetPack 6.0 (Jetson Linux 36.3)
IS_JETPACK_6_0=false
if [[ -f /etc/nv_tegra_release ]]; then
    if grep -q 'R36 (release), REVISION: 3\.' /etc/nv_tegra_release 2>/dev/null; then
        IS_JETPACK_6_0=true
        echo "  Detected: JetPack 6.0 (Jetson Linux 36.3)"
    fi
fi

# Download and install architecture-specific package
if [[ "${ARCH}" == "x86_64" ]]; then
    DEB_URL="https://github.com/NEWSLabNTU/autoware/releases/download/${ROSDEBIAN_RELEASE_ENCODED}/autoware-localrepo_${AUTOWARE_PACKAGE_VERSION}_amd64.deb"
    DEB_FILE="${DATA_DIR}/autoware-debian/autoware-localrepo_${AUTOWARE_PACKAGE_VERSION}_amd64.deb"
    DEB_SHA256="${CHECKSUM_AUTOWARE_DEB_AMD64}"
elif [[ "${ARCH}" == "aarch64" ]] && [[ "${IS_JETPACK_6_0}" == "true" ]]; then
    DEB_URL="https://github.com/NEWSLabNTU/autoware/releases/download/${ROSDEBIAN_RELEASE_ENCODED}/autoware-localrepo_${AUTOWARE_PACKAGE_VERSION}_jetpack6.0.deb"
    DEB_FILE="${DATA_DIR}/autoware-debian/autoware-localrepo_${AUTOWARE_PACKAGE_VERSION}_jetpack6.0.deb"
    DEB_SHA256="${CHECKSUM_AUTOWARE_DEB_JETPACK60}"
elif [[ "${ARCH}" == "aarch64" ]]; then
    DEB_URL="https://github.com/NEWSLabNTU/autoware/releases/download/${ROSDEBIAN_RELEASE_ENCODED}/autoware-localrepo_${AUTOWARE_PACKAGE_VERSION}_arm64.deb"
    DEB_FILE="${DATA_DIR}/autoware-debian/autoware-localrepo_${AUTOWARE_PACKAGE_VERSION}_arm64.deb"
    DEB_SHA256="${CHECKSUM_AUTOWARE_DEB_ARM64}"
else
    echo "Error: Unsupported architecture: ${ARCH}"
    exit 1
fi

# Download if not exists or checksum mismatch
NEED_DOWNLOAD=true
if [[ -f "${DEB_FILE}" ]] && [[ -n "${DEB_SHA256}" ]]; then
    ACTUAL_SHA256=$(sha256sum "${DEB_FILE}" | cut -d' ' -f1)
    if [[ "${ACTUAL_SHA256}" == "${DEB_SHA256}" ]]; then
        echo "Package already downloaded and verified."
        NEED_DOWNLOAD=false
    fi
fi

if [[ "${NEED_DOWNLOAD}" == "true" ]]; then
    echo "Downloading Autoware local repository package..."
    curl -fSL -o "${DEB_FILE}" "${DEB_URL}"

    # Verify checksum if available
    if [[ -n "${DEB_SHA256}" ]]; then
        echo "Verifying checksum..."
        echo "${DEB_SHA256}  ${DEB_FILE}" | sha256sum -c -
    fi
fi

# Install the local repository package
echo "Installing Autoware local repository..."
sudo apt-get install -y "${DEB_FILE}"
sudo apt-get update

# Install Autoware full package
echo "Installing autoware-full package..."
sudo apt-get install -y autoware-full

# Run Autoware setup
echo "Running Autoware setup..."
if ! sudo autoware-setup; then
    RC=$?
    # RC 130 = user cancelled with Ctrl+C, which is OK
    if [[ $RC -ne 130 ]]; then
        echo "Warning: autoware-setup exited with code $RC"
    fi
fi

# Add Autoware environment to bashrc
BASHRC_LINE="source /opt/autoware/autoware-env"
if ! grep -qF "${BASHRC_LINE}" ~/.bashrc; then
    echo "Adding Autoware environment to ~/.bashrc..."
    echo "${BASHRC_LINE}" >> ~/.bashrc
fi

echo "Autoware Debian packages installation complete."
