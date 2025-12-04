#!/usr/bin/env bash
# Install Autoware Debian packages
# Converted from ansible/roles/autoware_debian/tasks/main.yaml

set -e

DATA_DIR="${DATA_DIR:-$(dirname "$0")/../../data}"
ARCH="${ARCH:-$(uname -m)}"

echo "Installing Autoware Debian packages..."
echo "  Architecture: ${ARCH}"
echo "  Data directory: ${DATA_DIR}"

# Create directory
mkdir -p "${DATA_DIR}/autoware-debian"

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
    DEB_URL="https://github.com/NEWSLabNTU/autoware/releases/download/rosdebian%2F2025.02-1/autoware-localrepo_2025.2-1_amd64.deb"
    DEB_FILE="${DATA_DIR}/autoware-debian/autoware-localrepo_2025.2-1_amd64.deb"
    DEB_SHA256="bccfd4d0818794f9efdc0cacfb3e229e987b3551e04441159922556fed385891"
elif [[ "${ARCH}" == "aarch64" ]] && [[ "${IS_JETPACK_6_0}" == "true" ]]; then
    DEB_URL="https://github.com/NEWSLabNTU/autoware/releases/download/rosdebian%2F2025.02-1/autoware-localrepo_2025.2-1_jetpack6.0.deb"
    DEB_FILE="${DATA_DIR}/autoware-debian/autoware-localrepo_2025.2-1_jetpack6.0.deb"
    DEB_SHA256="6da04e1f55bedf93b13f4c3b79ab700787a426697d8d260f7b08d45536eb0b3d"
elif [[ "${ARCH}" == "aarch64" ]]; then
    DEB_URL="https://github.com/NEWSLabNTU/autoware/releases/download/rosdebian%2F2025.02-1/autoware-localrepo_2025.2-1_arm64.deb"
    DEB_FILE="${DATA_DIR}/autoware-debian/autoware-localrepo_2025.2-1_arm64.deb"
    DEB_SHA256=""  # TODO: Add checksum when available
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
