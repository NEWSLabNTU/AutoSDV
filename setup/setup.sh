#!/usr/bin/env bash
# AutoSDV Setup Wrapper
# Interactive setup with optional components

set -e

# Resolve symlinks to find the actual script location
SCRIPT_PATH="$(readlink -f "$0")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
MARKER_DIR="${SCRIPT_DIR}/.markers"
SUDO_PID_FILE="${MARKER_DIR}/.sudo-loop-pid"
SUDO_LOOP_PID=""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# Show usage
show_usage() {
    cat << 'EOF'
AutoSDV Setup Script

Usage:
  ./setup.sh              Run interactive setup
  ./setup.sh status       Show setup status
  ./setup.sh <recipe>     Run specific recipe (ros2, dev-tools, etc.)
  ./setup.sh --help       Show this help

Examples:
  ./setup.sh              # Interactive full setup
  ./setup.sh status       # Check what's installed
  ./setup.sh ros2         # Install only ROS 2
  ./setup.sh clean-markers # Reset all installation markers

For available recipes, run: just --list
EOF
}

# Cleanup function - called on exit (normal or abnormal)
cleanup() {
    if [[ -n "$SUDO_LOOP_PID" ]] && kill -0 "$SUDO_LOOP_PID" 2>/dev/null; then
        kill "$SUDO_LOOP_PID" 2>/dev/null || true
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
    if ! sudo -n true 2>/dev/null; then
        printf "${YELLOW}→${NC} Requesting sudo privileges...\n"
        sudo -v || { printf "${RED}✗${NC} Failed to obtain sudo credentials\n"; exit 1; }
    fi

    # Start background sudo refresh loop (silent)
    (
        while true; do
            sudo -n true
            sleep 50
        done
    ) </dev/null >/dev/null 2>&1 &
    SUDO_LOOP_PID=$!
    disown $SUDO_LOOP_PID 2>/dev/null || true
    echo "$SUDO_LOOP_PID" > "$SUDO_PID_FILE"
}

# Ask yes/no question
ask_yes_no() {
    local question="$1"
    local default="${2:-y}"
    local prompt

    if [[ "$default" == "y" ]]; then
        prompt="[Y/n]"
    else
        prompt="[y/N]"
    fi

    while true; do
        printf "${BLUE}?${NC} %s %s " "$question" "$prompt"
        read -r response || {
            # Handle Ctrl-C or Ctrl-D
            printf "\n${YELLOW}Cancelled${NC}\n"
            exit 130
        }
        response="${response:-$default}"
        case "${response,,}" in
            y|yes) return 0 ;;
            n|no) return 1 ;;
            *) printf "${RED}Please answer yes or no.${NC}\n" ;;
        esac
    done
}

# Interactive setup configuration
interactive_setup() {
    printf "\n${BLUE}AutoSDV Setup${NC}\n\n"

    printf "Core: ROS 2, dev tools, GeographicLib, Python deps\n\n"

    # Optional: Autoware Debian packages
    INSTALL_AUTOWARE="n"

    printf "${YELLOW}Optional:${NC} Autoware Debian packages (~2-3 GB)\n"
    printf "You can skip and build from source instead.\n"
    if ask_yes_no "Install Autoware Debian packages?" "n"; then
        INSTALL_AUTOWARE="y"
    fi
    printf "\n"

    # Export choice for justfile
    export SKIP_AUTOWARE_DEBIAN="$([[ "$INSTALL_AUTOWARE" == "n" ]] && echo "1" || echo "0")"

    # Summary
    printf "Installing: Core"
    if [[ "$INSTALL_AUTOWARE" == "y" ]]; then
        printf " + Autoware Debian"
    fi
    printf "\n\n"

    if ! ask_yes_no "Continue?" "y"; then
        printf "${YELLOW}Cancelled${NC}\n"
        exit 0
    fi
    printf "\n"
}

# Main
main() {
    # Handle --help/-h
    if [[ "$1" == "--help" ]] || [[ "$1" == "-h" ]]; then
        show_usage
        exit 0
    fi

    local recipe="${1:-setup}"

    # Check if just is installed
    if ! command -v just &> /dev/null; then
        printf "${RED}✗${NC} 'just' not found\n"
        printf "Install: curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/.local/bin\n"
        exit 1
    fi

    # If running full setup, show interactive wizard
    if [[ "$recipe" == "setup" ]]; then
        if [[ ! -t 0 ]]; then
            # Non-interactive mode (piped input)
            printf "${YELLOW}→${NC} Non-interactive mode\n"
        else
            # Interactive mode
            interactive_setup
        fi
    fi

    # Start sudo loop if needed (silently)
    if needs_sudo "$recipe"; then
        start_sudo_loop
    fi

    # Run just with all arguments
    cd "$SCRIPT_DIR"
    if [[ $# -eq 0 ]]; then
        # No arguments, run setup
        just setup
    else
        # Pass through all arguments
        just "$@"
    fi
}

main "$@"
