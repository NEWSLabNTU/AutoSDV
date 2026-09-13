#!/usr/bin/env bash
# Get a version value from versions.yaml
#
# Usage:
#   get-version.sh <key> [default]
#
# Examples:
#   get-version.sh autosdv.version           # Returns "0.1.0-dev"
#   get-version.sh nvidia_amd64.cuda         # Returns "12.3"
#   get-version.sh autoware.version          # Returns "1.5.0"
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

# PyYAML only, deliberately: `yq` exists in two incompatible flavors (mikefarah
# and the jq-wrapping Python one) whose argument forms differ, and a wrong guess
# fails by printing nothing while exiting 0 -- the same silent-empty failure
# export-versions.sh documents. export-versions.sh already reads this file with
# PyYAML, so nothing new is required here.
VALUE=$(python3 -c "
import sys
import yaml

with open('$VERSIONS_FILE') as f:
    data = yaml.safe_load(f)

val = data
for k in '$KEY'.split('.'):
    try:
        val = val[k]
    except (KeyError, TypeError, IndexError):
        default = '$DEFAULT'
        if default:
            print(default)
            sys.exit(0)
        print(\"Error: key '$KEY' not found in $VERSIONS_FILE\", file=sys.stderr)
        sys.exit(1)
print(val)
")

if [[ -z "$VALUE" ]] && [[ -z "$DEFAULT" ]]; then
    echo "Error: Key '$KEY' not found in versions.yaml" >&2
    exit 1
fi

echo "$VALUE"
