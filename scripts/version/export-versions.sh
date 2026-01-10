#!/usr/bin/env bash
# Export all versions from versions.yaml as environment variables
#
# Usage:
#   source scripts/version/export-versions.sh
#
# Exported variables:
#   AUTOSDV_VERSION, AUTOSDV_CHANNEL
#   AUTOWARE_VERSION, AUTOWARE_ROSDEBIAN_RELEASE, AUTOWARE_PACKAGE_VERSION
#   ROS_DISTRO, ROS_INSTALLATION_TYPE, RMW_IMPLEMENTATION
#   CUDA_VERSION_AMD64, CUDNN_VERSION_AMD64, TENSORRT_VERSION_AMD64
#   CUDA_VERSION_ARM64, CUDNN_VERSION_ARM64, TENSORRT_VERSION_ARM64
#   JETPACK_VERSION, L4T_VERSION
#   PRE_COMMIT_CLANG_FORMAT_VERSION
#   CHECKSUM_AUTOWARE_DEB_AMD64, CHECKSUM_AUTOWARE_DEB_ARM64, CHECKSUM_AUTOWARE_DEB_JETPACK60

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
VERSIONS_FILE="${SCRIPT_DIR}/../../versions.yaml"

if [[ ! -f "$VERSIONS_FILE" ]]; then
    echo "Error: versions.yaml not found at $VERSIONS_FILE" >&2
    return 1 2>/dev/null || exit 1
fi

# Parse versions.yaml and export environment variables
eval "$(python3 -c "
import yaml

with open('$VERSIONS_FILE') as f:
    v = yaml.safe_load(f)

exports = [
    # AutoSDV
    ('AUTOSDV_VERSION', v['autosdv']['version']),
    ('AUTOSDV_PRERELEASE', v['autosdv']['prerelease']),
    ('AUTOSDV_CHANNEL', v['autosdv']['channel']),

    # Autoware
    ('AUTOWARE_VERSION', v['autoware']['version']),
    ('AUTOWARE_ROSDEBIAN_RELEASE', v['autoware']['rosdebian_release']),
    ('AUTOWARE_PACKAGE_VERSION', v['autoware']['package_version']),

    # ROS
    ('ROS_DISTRO', v['ros']['distro']),
    ('ROS_INSTALLATION_TYPE', v['ros']['installation_type']),
    ('RMW_IMPLEMENTATION', v['ros']['rmw_implementation']),

    # NVIDIA AMD64
    ('CUDA_VERSION_AMD64', v['nvidia_amd64']['cuda']),
    ('CUDNN_VERSION_AMD64', v['nvidia_amd64']['cudnn']),
    ('TENSORRT_VERSION_AMD64', v['nvidia_amd64']['tensorrt']),

    # NVIDIA ARM64
    ('JETPACK_VERSION', v['nvidia_arm64']['jetpack']),
    ('L4T_VERSION', v['nvidia_arm64']['l4t']),
    ('CUDA_VERSION_ARM64', v['nvidia_arm64']['cuda']),
    ('CUDNN_VERSION_ARM64', v['nvidia_arm64']['cudnn']),
    ('TENSORRT_VERSION_ARM64', v['nvidia_arm64']['tensorrt']),

    # Tools
    ('PRE_COMMIT_CLANG_FORMAT_VERSION', v['tools']['pre_commit_clang_format']),

    # Checksums
    ('CHECKSUM_AUTOWARE_DEB_AMD64', v['checksums']['autoware_deb_amd64']),
    ('CHECKSUM_AUTOWARE_DEB_ARM64', v['checksums']['autoware_deb_arm64']),
    ('CHECKSUM_AUTOWARE_DEB_JETPACK60', v['checksums']['autoware_deb_jetpack60']),
]

for name, value in exports:
    # Escape single quotes in values
    escaped = str(value).replace(\"'\", \"'\\\"'\\\"'\")
    print(f\"export {name}='{escaped}'\")
")"
