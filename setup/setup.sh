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

exec python3 "$MAIN" "$@"
