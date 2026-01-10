#!/usr/bin/env bash
# Get a version value from versions.yaml
#
# Usage:
#   get-version.sh <key> [default]
#
# Examples:
#   get-version.sh autosdv.version           # Returns "1.0.0-dev"
#   get-version.sh nvidia_amd64.cuda         # Returns "12.3"
#   get-version.sh autoware.version          # Returns "2025.02"
#   get-version.sh missing.key "default"     # Returns "default"

set -e

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
VERSIONS_FILE="${SCRIPT_DIR}/../../versions.yaml"

KEY="$1"
DEFAULT="${2:-}"

if [[ -z "$KEY" ]]; then
    echo "Usage: get-version.sh <key> [default]" >&2
    echo "Example: get-version.sh autosdv.version" >&2
    exit 1
fi

if [[ ! -f "$VERSIONS_FILE" ]]; then
    echo "Error: versions.yaml not found at $VERSIONS_FILE" >&2
    exit 1
fi

# Use yq if available (faster), fallback to Python
if command -v yq &>/dev/null; then
    VALUE=$(yq -r ".${KEY} // \"${DEFAULT}\"" "$VERSIONS_FILE" 2>/dev/null)
else
    VALUE=$(python3 -c "
import yaml
import sys

with open('$VERSIONS_FILE') as f:
    data = yaml.safe_load(f)

keys = '$KEY'.split('.')
val = data
try:
    for k in keys:
        val = val[k]
    print(val)
except (KeyError, TypeError):
    default = '$DEFAULT'
    if default:
        print(default)
    else:
        sys.exit(1)
" 2>/dev/null)
fi

if [[ -z "$VALUE" ]] && [[ -z "$DEFAULT" ]]; then
    echo "Error: Key '$KEY' not found in versions.yaml" >&2
    exit 1
fi

echo "$VALUE"
