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
#   AUTOWARE_INSTALL_PREFIX
#   CHECKSUM_AUTOWARE_DEB_AMD64, CHECKSUM_AUTOWARE_DEB_ARM64
#
# The AUTOWARE_* and checksum values are the selected version's
# (scripts/version/autoware-version.sh).

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
VERSIONS_FILE="${SCRIPT_DIR}/../../versions.yaml"

if [[ ! -f "$VERSIONS_FILE" ]]; then
    echo "Error: versions.yaml not found at $VERSIONS_FILE" >&2
    return 1 2>/dev/null || exit 1
fi

# The selected Autoware version (autoware-version.sh); its entry supplies every
# AUTOWARE_* and CHECKSUM_AUTOWARE_* value below.
_autosdv_aw_selected="$("${SCRIPT_DIR}/autoware-version.sh")" || return 1 2>/dev/null || exit 1

# Parse versions.yaml and export environment variables
eval "$(python3 -c "
import yaml

with open('$VERSIONS_FILE') as f:
    v = yaml.safe_load(f)
aw = v['autoware']['versions']['$_autosdv_aw_selected']

exports = [
    # AutoSDV
    ('AUTOSDV_VERSION', v['autosdv']['version']),
    ('AUTOSDV_PRERELEASE', v['autosdv']['prerelease']),
    ('AUTOSDV_CHANNEL', v['autosdv']['channel']),

    # Autoware
    ('AUTOWARE_VERSION', aw['version']),
    ('AUTOWARE_ROSDEBIAN_RELEASE', aw['rosdebian_release']),
    ('AUTOWARE_PACKAGE_VERSION', aw['package_version']),
    ('AUTOWARE_INSTALL_PREFIX', aw['install_prefix']),

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
    ('CHECKSUM_AUTOWARE_DEB_AMD64', aw['sha256_amd64']),
    ('CHECKSUM_AUTOWARE_DEB_ARM64', aw['sha256_jetpack62']),
]

for name, value in exports:
    # Escape single quotes in values
    escaped = str(value).replace(\"'\", \"'\\\"'\\\"'\")
    print(f\"export {name}='{escaped}'\")
")"
unset _autosdv_aw_selected
