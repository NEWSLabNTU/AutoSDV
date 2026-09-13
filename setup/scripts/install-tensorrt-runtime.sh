#!/usr/bin/env bash
# Install the EXACT TensorRT runtime the Autoware amd64 Debians were compiled
# against, into a private prefix, so cached .engine files survive a relaunch.
#
# Why this exists
# ---------------
# `autoware_tensorrt_common` validates an existing engine by comparing the
# plan's TensorRT version (bytes 24-26 of the .engine) against the
# NV_TENSORRT_* macros baked into the shipped library. A host running any other
# patch of TensorRT 10 fails that comparison, discards the engine, and rebuilds
# all five perception engines on EVERY launch -- ~9 minutes on an RTX 3090, with
# perception unavailable until it finishes. `Depends: libnvinfer10` does not
# express this: 10.9 and 10.16 satisfy it and both void the cache.
#
# On a Jetson the question does not arise -- JetPack's TensorRT is the one the
# arm64 Debians were built against. This script is amd64-only.
#
# Why a private prefix rather than `apt install`
# ---------------------------------------------
# Downgrading the system libnvinfer pulls apt into the whole TensorRT dev/python
# package set (observed: a 10.8 runtime downgrade wanted to drag in TensorRT 11
# dev packages), and it would take other CUDA work on the same workstation with
# it. Extracting the three runtime debs into /opt/tensorrt/<version> touches no
# installed package, is undone with `rm -rf`, and is picked up only by processes
# whose LD_LIBRARY_PATH says so -- which is scripts/env.sh's job.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
VERSION="$("${REPO_DIR}/scripts/version/get-version.sh" nvidia_amd64.tensorrt_engine_abi)"
PREFIX="/opt/tensorrt/${VERSION}"
LIB_DIR="${PREFIX}/lib"

if [[ "$(uname -m)" != "x86_64" ]]; then
    echo "install-tensorrt-runtime.sh: not x86_64 -- JetPack already ships the"
    echo "matching TensorRT on arm64. Nothing to do."
    exit 0
fi

# The debs are versioned `<version>.<build>-1+cuda<x.y>`; the build/CUDA suffix
# is not in versions.yaml because it is an artifact of NVIDIA's packaging, not
# something Autoware requires. Resolve it from the repository instead of pinning
# a second copy of it here.
# `awk ... {exit}` would close the pipe while apt-cache is still writing, and
# SIGPIPE plus `set -o pipefail` makes that a fatal exit 141 -- a failure with no
# message at all, which is what this script did on its first run. So read the
# whole listing and pick the first match without exiting early.
candidate() {
    apt-cache madison "$1" 2>/dev/null \
        | awk -v v="${VERSION}." '$3 ~ ("^" v) && !seen { print $3; seen = 1 }'
}

PKGS=(libnvinfer10 libnvinfer-plugin10 libnvonnxparsers10)

if [[ -f "${LIB_DIR}/libnvinfer.so.${VERSION}" ]]; then
    echo "TensorRT ${VERSION} runtime already at ${PREFIX}"
    exit 0
fi

declare -a specs=()
for pkg in "${PKGS[@]}"; do
    ver="$(candidate "${pkg}")"
    if [[ -z "${ver}" ]]; then
        echo "error: no ${pkg} ${VERSION}.* in any configured apt repository." >&2
        echo "       Add NVIDIA's CUDA repository for this distribution, then retry:" >&2
        echo "       https://developer.download.nvidia.com/compute/cuda/repos/" >&2
        exit 1
    fi
    specs+=("${pkg}=${ver}")
done

WORK="$(mktemp -d)"
trap 'rm -rf "${WORK}"' EXIT

echo "=== downloading ${specs[*]}"
(cd "${WORK}" && apt-get download "${specs[@]}")

echo "=== extracting into ${LIB_DIR}"
sudo mkdir -p "${LIB_DIR}"
for deb in "${WORK}"/*.deb; do
    # -x, not -X: the debs lay their libraries out under
    # usr/lib/x86_64-linux-gnu, and only that directory is wanted.
    dpkg-deb -x "${deb}" "${WORK}/unpacked"
done
sudo cp -a "${WORK}/unpacked/usr/lib/x86_64-linux-gnu/." "${LIB_DIR}/"

# Deliberately NOT added to /etc/ld.so.conf.d: this prefix must apply to AutoSDV
# processes and nothing else on the machine. scripts/env.sh prepends it to
# LD_LIBRARY_PATH, which takes precedence over the ldconfig cache.
echo
echo "=== TensorRT ${VERSION} runtime installed at ${PREFIX}"
echo "    $(ls "${LIB_DIR}" | wc -l) file(s). Used by any shell that has sourced"
echo "    scripts/env.sh; nothing else on this machine sees it."
