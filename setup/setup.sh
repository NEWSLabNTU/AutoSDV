#!/usr/bin/env bash
# AutoSDV setup: a launcher, and nothing else.
#
# This was 746 lines of bash, of which ~250 were a hand-written checkbox menu
# re-forking `tput` and `cut` on every keystroke, and the rest a MENU_ITEMS
# table whose entries had to agree by hand with the justfile's `setup:` recipe
# chain and with a `_setup-*` wrapper per option. Steps nobody wrote a wrapper
# for ran unconditionally and never appeared in the menu.
#
# The menu is now Python `curses`, which is in the standard library, so there is
# no environment to build and nothing here to do but hand over. A step is one
# object in setup/autosdv_setup/registry.py: if it is not in that list it does
# not run, and if it is in that list the UI shows it.
#
#   ./setup.sh                pick a profile, then the steps
#   ./setup.sh --status       what is installed
#   ./setup.sh --list         every step, and whether it applies here
#   ./setup.sh --run --profile vehicle --yes
#   ./setup.sh --run --all --skip tensorrt-engines
#   ./setup.sh --rerun opencv
#   ./setup.sh --plain        numbered menu, for a terminal curses cannot drive

set -euo pipefail

# This file lives at setup/setup.sh; the repo root carries a symlink to it, so
# resolve the link before deriving anything from the path.
SETUP_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
MAIN="$SETUP_DIR/main.py"

command -v python3 >/dev/null || {
    printf 'python3 not found (expected on Ubuntu 22.04)\n' >&2
    exit 1
}

# PyYAML, before handing over. registry.py reads versions.yaml at import time,
# so without it setup.sh cannot start at all -- it dies on a traceback from a
# module the user has never heard of, before printing a single word of its own:
#
#   File ".../autosdv_setup/registry.py", line 43, in _versions
#     import yaml
#   ModuleNotFoundError: No module named 'yaml'
#
# Ubuntu 22.04 ships python3 without PyYAML, and nothing in this repository
# installs it: scripts/version/ REQUIRES it and assumes it is there. That
# assumption holds on every machine this was developed on and on none that a
# student brings, which is the same way `install-tensorrt.sh` silently stopped
# pinning its version. Here it is fatal rather than silent.
#
# Installed here rather than as a step, because a step cannot run before the
# registry that declares it has been imported.
if ! python3 -c 'import yaml' 2>/dev/null; then
    printf 'installing python3-yaml (setup reads versions.yaml before it can start)\n'
    # Root needs no sudo, and a root container may not even have it installed.
    if [ "$(id -u)" -eq 0 ]; then
        _sudo=""
    elif command -v sudo >/dev/null 2>&1; then
        _sudo="sudo"
    else
        _sudo=""
    fi
    if [ "$(id -u)" -eq 0 ] || [ -n "$_sudo" ]; then
        $_sudo apt-get update -qq >/dev/null 2>&1 || true
        $_sudo apt-get install -y --no-install-recommends python3-yaml >/dev/null 2>&1 || true
    fi
fi

if ! python3 -c 'import yaml' 2>/dev/null; then
    cat >&2 <<'EOF'

  PyYAML is missing and could not be installed automatically.

  setup reads versions.yaml before it can do anything, so install it first:

      sudo apt-get install -y python3-yaml

  then run this again.

EOF
    exit 1
fi

exec python3 "$MAIN" "$@"
