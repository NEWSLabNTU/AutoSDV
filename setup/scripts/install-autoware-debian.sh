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
    echo "  Detected architecture: arm64 (aarch64) - JetPack 6.x target (Jetson)"
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

# Download helper: prefer aria2c (parallel download + inline checksum), then
# fall back to wget, then curl. aria2c is only a speed optimisation, so its
# absence — or failure — is not fatal as long as one fetcher is available.
download_deb() {
    local url="$1" dir="$2" out="$3" sha="$4"
    local dest="${dir}/${out}"

    if command -v aria2c &> /dev/null; then
        echo "  Downloading with aria2c (parallel)..."
        local args=(--dir="$dir" --out="$out" -x 10 -s 10 -k 1M)
        [[ -n "$sha" ]] && args+=(--checksum=sha-256="$sha")
        if aria2c "$url" "${args[@]}"; then
            return 0  # aria2c verified the checksum inline
        fi
        echo "  aria2c download failed; falling back to wget/curl..."
        rm -f "$dest"
    fi

    if command -v wget &> /dev/null; then
        echo "  Downloading with wget..."
        wget -O "$dest" "$url" || { echo "  wget failed."; rm -f "$dest"; return 1; }
    elif command -v curl &> /dev/null; then
        echo "  Downloading with curl..."
        curl -fL -o "$dest" "$url" || { echo "  curl failed."; rm -f "$dest"; return 1; }
    else
        echo "Error: need aria2c, wget, or curl to download ${out}, none found."
        return 1
    fi

    # aria2c checks the hash during transfer; the wget/curl path verifies after.
    if [[ -n "$sha" ]]; then
        echo "  Verifying checksum..."
        local actual
        actual=$(sha256sum "$dest" | awk '{print $1}')
        if [[ "$actual" != "$sha" ]]; then
            echo "  ERROR: checksum mismatch for ${dest}"
            echo "  Expected: ${sha}"
            echo "  Actual:   ${actual}"
            rm -f "$dest"
            return 1
        fi
        echo "  Checksum matches."
    fi
}

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

if [[ "$DOWNLOAD_REQUIRED" == "true" ]]; then
    download_deb "${DOWNLOAD_URL}" "${DEB_DOWNLOAD_DIR}" "${DEB_FILE}" "${SHA256SUM}"
else
    echo "  Using existing file: ${TEMP_DEB}"
fi

echo "  Installing Autoware localrepo..."
sudo apt update
sudo apt install -y "$TEMP_DEB"

# Run setup-prerequisites.sh
#
# Left to itself this script asks its own two questions — ROS 2 Humble, and
# SpConv/Cumm — partway through ours, which is a second interactive session
# arriving after the user thought they had answered everything. It accepts
# flags for both, so the answers are collected in our menu and passed through;
# see AUTOWARE_PREREQ_* in setup.sh.
#
# Defaults when the variables are unset (i.e. this script run directly):
#   ROS 2  -> --no-ros, because the ros2 step installs ROS 2 itself, before
#             this step. Letting the nested script install it again is at best
#             redundant and at worst a different configuration.
#   SpConv -> --no-spconv, matching the nested script's own default. It is
#             needed only by perception models this stack does not use
#             (BEVFusion and friends).
if [ -f /usr/share/autoware/setup-prerequisites.sh ]; then
    PREREQ_ARGS=()
    if [ "${AUTOWARE_PREREQ_ROS:-n}" = "y" ]; then
        PREREQ_ARGS+=(--install-ros)
    else
        PREREQ_ARGS+=(--no-ros)
    fi
    if [ "${AUTOWARE_PREREQ_SPCONV:-n}" = "y" ]; then
        PREREQ_ARGS+=(--spconv)
    else
        PREREQ_ARGS+=(--no-spconv)
    fi
    echo "  Running /usr/share/autoware/setup-prerequisites.sh ${PREREQ_ARGS[*]}..."
    sudo /usr/share/autoware/setup-prerequisites.sh "${PREREQ_ARGS[@]}"
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

