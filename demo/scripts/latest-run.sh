#!/usr/bin/env bash
# Print the newest demo run that actually holds a recording.
#
# An interrupted run leaves a directory behind with only launch artifacts in it.
# Analysis recipes that just took the newest directory, or a LATEST pointer
# written at startup, would then be pointed at a corpse and fail obscurely.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUNS="$REPO/tmp/demo-runs"

latest=$(cat "$RUNS/LATEST" 2>/dev/null || true)
if [[ -n "$latest" && -f "$latest/bag/metadata.yaml" ]]; then
    echo "$latest"
    exit 0
fi

for d in $(ls -dt "$RUNS"/*/ 2>/dev/null); do
    if [[ -f "$d/bag/metadata.yaml" ]]; then
        echo "${d%/}"
        exit 0
    fi
done

echo "no completed demo run found under $RUNS; try 'just demo run'" >&2
exit 1
