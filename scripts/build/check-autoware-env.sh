#!/usr/bin/env bash
# Refuse to build or test unless this shell carries the selected Autoware.
#
# `just build` sources ROS only and relies on the Autoware underlay the shell
# already has (direnv's .envrc, or scripts/env.sh). Selecting another version
# (.autoware-version, AUTOSDV_AUTOWARE_VERSION) without reloading the shell would
# otherwise build the new version's workspace against the old underlay: colcon
# succeeds, and the mismatch surfaces at launch as libraries of one version in
# nodes of the other.
set -euo pipefail
repo="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
want="$("$repo/scripts/version/autoware-version.sh")"
have="${AUTOWARE_VERSION:-}"
if [ "$have" != "$want" ]; then
    if [ -z "$have" ]; then
        echo "error: no Autoware in this shell; this checkout selects $want." >&2
    else
        echo "error: this shell carries Autoware $have; this checkout selects $want." >&2
    fi
    echo "  Reload the environment first: 'direnv reload' in the checkout, or" >&2
    echo "  'source scripts/env.sh' from a shell without another Autoware." >&2
    exit 1
fi
