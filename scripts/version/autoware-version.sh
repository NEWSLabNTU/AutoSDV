#!/usr/bin/env bash
# Print the Autoware version this checkout builds and runs against.
#
# First hit wins:
#   1. AUTOSDV_AUTOWARE_VERSION      a one-off, for this shell
#   2. .autoware-version             this checkout's choice (gitignored, one line)
#   3. autoware.default              in versions.yaml
#
# Exits 1, naming the versions versions.yaml knows, when the choice is not one of
# them: a typo must not fall back to the default silently, because building one
# version's workspace against another's underlay is the failure this exists to
# prevent.
#
# Deliberately NOT read: AUTOWARE_VERSION. Autoware's own setup.bash exports it,
# so a shell that sourced some Autoware earlier (a ~/.bashrc line) would pick the
# version by accident. .envrc and scripts/env.sh compare it with this choice
# instead, and refuse a shell that already carries another underlay.
set -euo pipefail
repo="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"

choice="${AUTOSDV_AUTOWARE_VERSION:-}"
from="AUTOSDV_AUTOWARE_VERSION"
if [ -z "$choice" ] && [ -f "$repo/.autoware-version" ]; then
    choice="$(head -n1 "$repo/.autoware-version" | tr -d '[:space:]')"
    from="$repo/.autoware-version"
fi

python3 - "$repo/versions.yaml" "$choice" "$from" <<'PY'
import sys, yaml
path, choice, source = sys.argv[1:4]
aw = yaml.safe_load(open(path))['autoware']
known = list(aw['versions'])
version = choice or aw['default']
if version not in aw['versions']:
    origin = source if choice else 'autoware.default in versions.yaml'
    sys.exit(f"autoware-version.sh: '{version}' (from {origin}) is not in versions.yaml; "
             f"known: {', '.join(known)}")
print(version)
PY
