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
# autoware.<key> reads the SELECTED version's entry (autoware-version.sh), so a
# caller asking for autoware.install_prefix follows the checkout's choice;
# autoware.default and autoware.versions.<v>.<key> are read as written.
KEY_PATH="$KEY"
case "$KEY" in
    autoware.default|autoware.versions|autoware.versions.*) ;;
    autoware.*)
        AW_SELECTED="$("${SCRIPT_DIR}/autoware-version.sh")" || exit 1
        KEY_PATH="autoware.versions.${AW_SELECTED}.${KEY#autoware.}"
        ;;
esac

# An empty value is a value (autoware.workspace_suffix is "" for 1.5.0), so
# only a missing key is an error, told apart by the exit status.
set +e
VALUE=$(python3 - "$VERSIONS_FILE" "$KEY_PATH" "$DEFAULT" <<'PY'
import sys
import yaml

path, key, default = sys.argv[1:4]
val = yaml.safe_load(open(path))
# Version numbers contain dots, so walk greedily: at each level take the
# longest dotted prefix that is a key.
parts = key.split('.')
while parts:
    for n in range(len(parts), 0, -1):
        k = '.'.join(parts[:n])
        if isinstance(val, dict) and k in val:
            val = val[k]
            parts = parts[n:]
            break
    else:
        if default:
            print(default)
            sys.exit(0)
        print(f"Error: key '{key}' not found in {path}", file=sys.stderr)
        sys.exit(1)
print('' if val is None else val)
PY
)
STATUS=$?
set -e
[ "$STATUS" -eq 0 ] || exit "$STATUS"

echo "$VALUE"
