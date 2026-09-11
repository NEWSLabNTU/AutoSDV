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
# Left to itself this script asks its own questions partway through ours, which
# is a second interactive session arriving after the user thought they had
# answered everything. What it will and will not accept as flags decides how
# much of that can be avoided, and the answer is narrower than it looks:
#
#   --no-ros / --install-ros    answer the ROS question. Fine.
#   --all-nvidia / --no-nvidia  answer the NVIDIA question AND set
#                               NVIDIA_PROMPTED, so the menu does not open.
#   --cuda --cudnn --tensorrt   set the variables but NOT NVIDIA_PROMPTED, so
#                               the four-item menu opens anyway.
#   --no-spconv                 DOES NOT EXIST. Passing it is a hard error:
#                               "[ERROR] Unknown option: --no-spconv", and the
#                               step dies there. This script used to pass it.
#
# So there is no flag that installs CUDA, cuDNN and TensorRT while declining
# SpConv. The choice is between everything and nothing, and it is made here from
# what the machine already has rather than from a menu answer:
#
#   NVIDIA stack already present -> --no-nvidia. Nothing to install, no menu,
#                                   and no SpConv tarball built for CUDA 12.8
#                                   landing on a box running something else.
#   not present                  -> --all-nvidia, which is the nested script's
#                                   own default selection. SpConv comes with it;
#                                   it is dead weight for this stack (BEVFusion
#                                   and friends), not a hazard.
#
# Override either way with AUTOWARE_PREREQ_NVIDIA=y|n.
#
# The final "Proceed with installation?" prompt is unconditional -- even -y does
# not skip it -- so it is answered on stdin. That is the one question this
# wrapper cannot remove with a flag.
if [ -f /usr/share/autoware/setup-prerequisites.sh ]; then
    PREREQ_ARGS=()

    # ROS 2: the ros2 step installs it before this one. Letting the nested
    # script install it again is at best redundant and at worst a different
    # configuration.
    if [ "${AUTOWARE_PREREQ_ROS:-n}" = "y" ]; then
        PREREQ_ARGS+=(--install-ros)
    else
        # --no-ros is a promise that ROS is already there. Break that promise
        # and the failure surfaces later as a wall of unmet apt dependencies
        # from autoware-full-1-5-0, naming ros-humble packages rather than the
        # step that installs them. (Guard borrowed from the golf cart, which
        # hit exactly that.)
        if [ ! -f /opt/ros/humble/setup.bash ]; then
            echo "ROS 2 Humble is not installed, and this step is configured not to" >&2
            echo "install it (AUTOWARE_PREREQ_ROS is not 'y')." >&2
            echo "Run the ros2 step first:  ./setup.sh --only ros2" >&2
            echo "Or set AUTOWARE_PREREQ_ROS=y to let Autoware's own prerequisite" >&2
            echo "script install it." >&2
            exit 1
        fi
        PREREQ_ARGS+=(--no-ros)
    fi

    if [ -n "${AUTOWARE_PREREQ_NVIDIA:-}" ]; then
        nvidia_wanted="$AUTOWARE_PREREQ_NVIDIA"
    elif command -v nvcc >/dev/null 2>&1 && \
         ls /usr/lib/*-linux-gnu/libnvinfer.so* >/dev/null 2>&1; then
        echo "  CUDA and TensorRT are already installed; skipping the NVIDIA step."
        nvidia_wanted=n
    else
        echo "  CUDA or TensorRT missing; installing the full NVIDIA set."
        nvidia_wanted=y
    fi

    if [ "$nvidia_wanted" = "y" ]; then
        PREREQ_ARGS+=(--all-nvidia)
    else
        PREREQ_ARGS+=(--no-nvidia)
    fi

    echo "  Running /usr/share/autoware/setup-prerequisites.sh ${PREREQ_ARGS[*]}..."
    # `yes` rather than a single y: the prompt loop re-asks on anything it does
    # not recognise, and a closed stdin would spin it.
    yes | sudo /usr/share/autoware/setup-prerequisites.sh "${PREREQ_ARGS[@]}"
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

