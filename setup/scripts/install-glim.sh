#!/usr/bin/env bash
# Install GLIM (LiDAR-IMU mapping) from koide3's PPA, for `just lab3 map`.
#
# Two variants, and they do not coexist -- the CUDA packages conflict with the
# CPU ones of the same name:
#
#   cuda  ros-humble-glim-ros-cudaX.Y + libgtsam-points-cudaX.Y-dev, X.Y one
#         of the CUDA builds the PPA publishes for Humble (12.2, 12.6, 13.1;
#         https://koide3.github.io/glim/installation.html). JetPack 6.2 carries
#         12.6. The installed toolkit picks it, from
#         /usr/local/cuda/version.json; a toolkit with no matching build is
#         refused rather than paired with a mismatched one.
#   cpu   ros-humble-glim-ros + libgtsam-points-dev
#         a machine with no NVIDIA GPU; `lab3_map --cpu` selects the matching
#         modules either way.
#
# Auto-detected (a Jetson, or nvidia-smi on PATH, means cuda); override with
# AUTOSDV_GLIM_VARIANT=cpu|cuda, and the CUDA build with AUTOSDV_GLIM_CUDA=X.Y.
set -euo pipefail

if [[ -f /opt/ros/humble/share/glim_ros/package.xml ]]; then
    echo "GLIM is already installed: $(dpkg-query -W -f='${Package} ${Version}\n' \
        'ros-humble-glim-ros*' 2>/dev/null | head -1)"
    exit 0
fi

variant="${AUTOSDV_GLIM_VARIANT:-}"
if [[ -z "${variant}" ]]; then
    if [[ -f /etc/nv_tegra_release ]] || command -v nvidia-smi >/dev/null 2>&1; then
        variant=cuda
    else
        variant=cpu
    fi
fi

# The CUDA builds the PPA publishes for Humble
GLIM_CUDA_BUILDS=(12.2 12.6 13.1)

if [[ "${variant}" == cuda ]]; then
    cuda="${AUTOSDV_GLIM_CUDA:-}"
    if [[ -z "${cuda}" ]]; then
        cuda=$(python3 -c 'import json; v = json.load(open("/usr/local/cuda/version.json"))["cuda"]["version"]; print(".".join(v.split(".")[:2]))' 2>/dev/null || true)
    fi
    if [[ -z "${cuda}" ]]; then
        echo "No CUDA toolkit found (/usr/local/cuda/version.json). Install it, or set" >&2
        echo "AUTOSDV_GLIM_CUDA=X.Y, or AUTOSDV_GLIM_VARIANT=cpu." >&2
        exit 2
    fi
    if [[ " ${GLIM_CUDA_BUILDS[*]} " != *" ${cuda} "* ]]; then
        echo "CUDA ${cuda} has no GLIM build; the PPA has ${GLIM_CUDA_BUILDS[*]}." >&2
        echo "Set AUTOSDV_GLIM_CUDA to one of them, or AUTOSDV_GLIM_VARIANT=cpu." >&2
        exit 2
    fi
fi

case "${variant}" in
    cuda) pkgs=("libgtsam-points-cuda${cuda}-dev" "ros-humble-glim-ros-cuda${cuda}") ;;
    cpu)  pkgs=(libgtsam-points-dev ros-humble-glim-ros) ;;
    *)    echo "AUTOSDV_GLIM_VARIANT must be cpu or cuda, got '${variant}'" >&2; exit 2 ;;
esac

echo "Installing GLIM (${variant} variant${cuda:+, CUDA ${cuda}})"

# The PPA script needs both
sudo apt-get install -y --no-install-recommends curl gpg

# The PPA script adds the key and the source list, then runs apt-get update.
curl -fsSL https://koide3.github.io/ppa/setup_ppa.sh | sudo bash

sudo apt-get install -y --no-install-recommends \
    libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev "${pkgs[@]}"

# gtsam_points lands in /usr/local/lib, which the loader cache does not know
# about until ldconfig runs; glim_rosbag then fails to start with a missing
# libgtsam_points.so.1.
sudo ldconfig

echo "GLIM installed: $(dpkg-query -W -f='${Package} ${Version}\n' 'ros-humble-glim-ros*' | head -1)"
