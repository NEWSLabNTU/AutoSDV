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
    local exit_code=$?
    if [[ -n "$SUDO_LOOP_PID" ]] && kill -0 "$SUDO_LOOP_PID" 2>/dev/null; then
        kill "$SUDO_LOOP_PID" 2>/dev/null || true
    fi
    rm -f "$SUDO_PID_FILE"

    # Show cancellation message if interrupted (but not on normal exit)
    if [[ $exit_code -eq 130 ]]; then
        printf "\n${YELLOW}Cancelled by user${NC}\n" >&2
    fi
}

# Handle interrupt signal (Ctrl-C)
interrupt_handler() {
    printf "\n${YELLOW}Interrupted${NC}\n" >&2
    exit 130
}

# Set trap for cleanup on any exit
trap cleanup EXIT
trap interrupt_handler INT TERM

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
        prompt="[Y/n/q]"
    else
        prompt="[y/N/q]"
    fi

    while true; do
        printf "${BLUE}?${NC} %s %s " "$question" "$prompt"
        read -r response || {
            # Handle Ctrl-D (EOF)
            printf "\n${YELLOW}Cancelled${NC}\n"
            exit 130
        }
        response="${response:-$default}"
        case "${response,,}" in
            y|yes) return 0 ;;
            n|no) return 1 ;;
            q|quit|exit)
                printf "\n${YELLOW}Cancelled${NC}\n"
                exit 0
                ;;
            *) printf "${RED}Please answer yes, no, or q to quit.${NC}\n" ;;
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

    # System-wide CycloneDDS kernel buffer configuration
    CONFIGURE_CYCLONEDDS_SYSCTL="n"
    printf "${YELLOW}System Configuration:${NC} CycloneDDS kernel buffers (recommended)\n"
    printf "This modifies: /etc/sysctl.d/10-cyclone-max.conf (system-wide)\n\n"
    if ask_yes_no "Configure kernel network buffers?" "y"; then
        CONFIGURE_CYCLONEDDS_SYSCTL="y"
    fi
    printf "\n"

    # Blickfeld Scanner Library with EULA
    INSTALL_BLICKFELD="n"
    ACCEPT_BLICKFELD_EULA="0"
    printf "${YELLOW}Sensor Drivers:${NC} Blickfeld Scanner Library (for Cube1 LiDAR)\n"
    printf "License: https://github.com/NEWSLabNTU/blickfeld-scanner-lib\n"
    printf "This is a modified version maintained by NEWSLab NTU.\n"
    printf "Original software by Blickfeld GmbH.\n\n"
    if ask_yes_no "Install Blickfeld Scanner Library?" "y"; then
        if ask_yes_no "Do you accept the Blickfeld license terms?" "y"; then
            INSTALL_BLICKFELD="y"
            ACCEPT_BLICKFELD_EULA="1"
        else
            printf "${YELLOW}License not accepted. Skipping Blickfeld installation.${NC}\n"
            INSTALL_BLICKFELD="n"
        fi
    fi
    printf "\n"

    # ML model artifacts download
    DOWNLOAD_ARTIFACTS="n"
    printf "${YELLOW}Optional:${NC} ML model artifacts (YOLOX, CenterPoint, ~2-5 GB)\n"
    printf "Required for perception features (object detection, tracking).\n"
    printf "You can skip and download later with: just download-artifacts\n\n"
    if ask_yes_no "Download ML model artifacts?" "n"; then
        DOWNLOAD_ARTIFACTS="y"
    fi
    printf "\n"

    # TurboVNC + VirtualGL (for hardware-accelerated VNC)
    INSTALL_TURBOVNC_VIRTUALGL="n"
    printf "${YELLOW}Optional:${NC} TurboVNC + VirtualGL (for hardware-accelerated VNC)\n"
    printf "Required for ZED camera usage in VNC sessions.\n"
    printf "You can skip and install later with: just turbovnc-virtualgl\n\n"
    if ask_yes_no "Install TurboVNC + VirtualGL?" "n"; then
        INSTALL_TURBOVNC_VIRTUALGL="y"
    fi
    printf "\n"

    # Export choices for justfile
    export SKIP_AUTOWARE_DEBIAN="$([[ "$INSTALL_AUTOWARE" == "n" ]] && echo "1" || echo "0")"
    export CONFIGURE_CYCLONEDDS_SYSCTL="$CONFIGURE_CYCLONEDDS_SYSCTL"
    export SKIP_BLICKFELD="$([[ "$INSTALL_BLICKFELD" == "n" ]] && echo "1" || echo "0")"
    export AUTOSDV_ACCEPT_BLICKFELD_EULA="$ACCEPT_BLICKFELD_EULA"
    export DOWNLOAD_ARTIFACTS="$DOWNLOAD_ARTIFACTS"
    export INSTALL_TURBOVNC_VIRTUALGL="$INSTALL_TURBOVNC_VIRTUALGL"

    # Summary
    printf "Installing: Core"
    if [[ "$INSTALL_AUTOWARE" == "y" ]]; then
        printf " + Autoware"
    fi
    if [[ "$CONFIGURE_CYCLONEDDS_SYSCTL" == "y" ]]; then
        printf " + CycloneDDS sysctl"
    fi
    if [[ "$INSTALL_BLICKFELD" == "y" ]]; then
        printf " + Blickfeld"
    fi
    if [[ "$DOWNLOAD_ARTIFACTS" == "y" ]]; then
        printf " + ML artifacts"
    fi
    if [[ "$INSTALL_TURBOVNC_VIRTUALGL" == "y" ]]; then
        printf " + TurboVNC/VirtualGL"
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

    # After successful setup, download artifacts if requested
    if [[ "$recipe" == "setup" ]] && [[ "${DOWNLOAD_ARTIFACTS:-n}" == "y" ]]; then
        printf "\n${YELLOW}→${NC} Downloading ML model artifacts...\n"
        just download-artifacts || {
            printf "${YELLOW}Warning: Artifact download failed. You can retry later with: just download-artifacts${NC}\n"
        }
    fi

    # After successful setup, show direnv instructions
    if [[ "$recipe" == "setup" ]]; then
        if ! command -v direnv &> /dev/null; then
            printf "\n${YELLOW}┌────────────────────────────────────────────────────────────┐${NC}\n"
            printf "${YELLOW}│ IMPORTANT: Install direnv for environment management      │${NC}\n"
            printf "${YELLOW}└────────────────────────────────────────────────────────────┘${NC}\n\n"
            printf "  ${BLUE}# Install direnv${NC}\n"
            printf "  sudo apt install direnv\n\n"
            printf "  ${BLUE}# Add to your shell (bash)${NC}\n"
            printf "  echo 'eval \"\$(direnv hook bash)\"' >> ~/.bashrc\n"
            printf "  source ~/.bashrc\n\n"
            printf "  ${BLUE}# Allow .envrc${NC}\n"
            printf "  direnv allow\n\n"
            printf "See: ${BLUE}https://direnv.net/${NC}\n"
        else
            printf "\n${GREEN}✓ direnv detected!${NC}\n"
            printf "Run this to activate environment: ${BLUE}direnv allow${NC}\n"
        fi
    fi
}

main "$@"
