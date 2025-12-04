#!/usr/bin/env bash
# AutoSDV Setup Wrapper
# Handles sudo keep-alive with proper cleanup on exit (normal or abnormal)
#
# Usage: ./setup.sh [recipe]
#   ./setup.sh          - Run full setup
#   ./setup.sh status   - Show setup status
#   ./setup.sh ros2     - Run specific step

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MARKER_DIR="${SCRIPT_DIR}/.markers"
SUDO_PID_FILE="${MARKER_DIR}/.sudo-loop-pid"
SUDO_LOOP_PID=""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m'

# Cleanup function - called on exit (normal or abnormal)
cleanup() {
    if [[ -n "$SUDO_LOOP_PID" ]] && kill -0 "$SUDO_LOOP_PID" 2>/dev/null; then
        kill "$SUDO_LOOP_PID" 2>/dev/null || true
        printf "\n${GREEN}✓${NC} sudo keep-alive stopped (PID: $SUDO_LOOP_PID)\n"
    fi
    rm -f "$SUDO_PID_FILE"
}

# Set trap for cleanup on any exit
trap cleanup EXIT INT TERM

# Recipes that don't need sudo
NO_SUDO_RECIPES="status clean-marker clean-markers default"

# Check if recipe needs sudo
needs_sudo() {
    local recipe="${1:-setup}"
    for r in $NO_SUDO_RECIPES; do
        [[ "$recipe" == "$r" ]] && return 1
    done
    return 0
}

# Start sudo keep-alive loop
start_sudo_loop() {
    mkdir -p "$MARKER_DIR"

    # Check if we already have sudo credentials cached
    if sudo -n true 2>/dev/null; then
        printf "${GREEN}✓${NC} sudo credentials already cached\n"
    else
        printf "${YELLOW}→${NC} This setup requires sudo privileges.\n"
        sudo -v || { printf "${RED}✗${NC} Failed to obtain sudo credentials\n"; exit 1; }
    fi

    # Start background sudo refresh loop
    # - Redirect all output to /dev/null
    # - Disown to detach from shell job control
    (
        while true; do
            sudo -n true
            sleep 50
        done
    ) </dev/null >/dev/null 2>&1 &
    SUDO_LOOP_PID=$!
    disown $SUDO_LOOP_PID 2>/dev/null || true
    echo "$SUDO_LOOP_PID" > "$SUDO_PID_FILE"
    printf "${GREEN}✓${NC} sudo keep-alive started (PID: $SUDO_LOOP_PID)\n"
}

# Main
main() {
    local recipe="${1:-setup}"

    # Check if just is installed
    if ! command -v just &> /dev/null; then
        printf "${RED}✗${NC} 'just' command not found.\n"
        printf "Install it with:\n"
        printf "  curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/.local/bin\n"
        exit 1
    fi

    # Start sudo loop if needed
    if needs_sudo "$recipe"; then
        start_sudo_loop
    fi

    # Run just with all arguments
    cd "$SCRIPT_DIR"
    just "$@"
}

main "$@"
