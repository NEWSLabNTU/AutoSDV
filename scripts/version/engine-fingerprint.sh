#!/usr/bin/env bash
# Print this machine's TensorRT engine cache-fingerprint: the key
# `just engines` looks up (and `just export-engines` publishes under) on a
# NEWSLabNTU/AutoSDV release. See docs/roadmap/11-engine-file-delivery.md.
#
# Both platforms key by exact GPU today -- no shortcut exists on either:
#   orin-{board}-{l4t}-{trt}-autoware{version}    -- JetPack has no hardware-
#                                                     compatibility build mode
#   desktop-{gpu}-{trt}-autoware{version}          -- `autoware_tensorrt_common`
#                                                     1.5.0 never calls
#                                                     setHardwareCompatibilityLevel
#                                                     (checked: headers, params,
#                                                     and `strings` on every
#                                                     shipped .so -- no kAMPERE_PLUS
#                                                     anywhere), so a 3090 build
#                                                     does not run on a 4090/5090
#                                                     yet. Sharing one engine
#                                                     across desktop generations
#                                                     needs an upstream patch to
#                                                     that package first, same as
#                                                     the CUDA pointcloud filters.
set -euo pipefail

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
AUTOWARE_VERSION="$("${SCRIPT_DIR}/get-version.sh" autoware.version)"

trt_version() {
    # Trim the Debian revision (e.g. "10.3.0.30-1+cuda12.5" -> "10.3.0.30"):
    # the upstream TensorRT version is what the engine is tied to.
    dpkg-query -W -f='${Version}' tensorrt 2>/dev/null | cut -d- -f1
}

if [[ -f /etc/nv_tegra_release ]]; then
    # e.g. "# R36 (release), REVISION: 4.4, GCID: ..." -> "R36.4.4"
    l4t=$(sed -n 's/^# R\([0-9]\+\).*REVISION: \([0-9.]\+\).*/R\1.\2/p' /etc/nv_tegra_release)
    model=$(tr -d '\0' </proc/device-tree/model 2>/dev/null || echo "")
    case "$model" in
        *"AGX Orin"*)  board="agx-orin" ;;
        *"Orin NX"*)   board="orin-nx" ;;
        *"Orin Nano"*) board="orin-nano" ;;
        *)             board="" ;;
    esac
    trt="$(trt_version)"
    if [[ -z "$l4t" || -z "$trt" || -z "$board" ]]; then
        echo "error: could not identify this Orin (model='$model' l4t='$l4t' trt='$trt')" >&2
        exit 1
    fi
    echo "orin-${board}-${l4t}-${trt}-autoware${AUTOWARE_VERSION}"
elif command -v nvidia-smi >/dev/null 2>&1; then
    trt="$(trt_version)"
    # e.g. "NVIDIA GeForce RTX 4090" -> "rtx-4090"
    gpu=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 \
        | tr '[:upper:]' '[:lower:]' | sed -E 's/nvidia|geforce//g; s/[^a-z0-9]+/-/g; s/^-+|-+$//g')
    if [[ -z "$trt" || -z "$gpu" ]]; then
        echo "error: could not identify this GPU (gpu='$gpu' trt='$trt')" >&2
        exit 1
    fi
    echo "desktop-${gpu}-${trt}-autoware${AUTOWARE_VERSION}"
else
    echo "error: no Jetson (/etc/nv_tegra_release) and no nvidia-smi -- no GPU detected" >&2
    exit 1
fi
