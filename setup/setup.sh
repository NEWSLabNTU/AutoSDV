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
BOLD='\033[1m'
NC='\033[0m'

# Show usage
show_usage() {
    cat << 'EOF'
AutoSDV Setup Script

Usage:
  ./setup.sh              Run interactive setup
  ./setup.sh --all        Install everything without questions
  ./setup.sh --minimal    Install core only (ROS 2, dev tools, Python deps)
  ./setup.sh status       Show setup status
  ./setup.sh <recipe>     Run specific recipe (ros2, dev-tools, etc.)
  ./setup.sh --help       Show this help

Flags (combine with --all or --minimal):
  --no-autoware           Skip Autoware Debian packages
  --no-isaac              Skip Isaac ROS Visual Localization
  --no-blickfeld          Skip Blickfeld Scanner Library

Examples:
  ./setup.sh                      # Interactive setup
  ./setup.sh --all                # Install everything, no questions
  ./setup.sh --all --no-isaac     # Everything except Isaac ROS
  ./setup.sh --minimal            # Core only
  ./setup.sh status               # Check what's installed
  ./setup.sh ros2                 # Install only ROS 2

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

# Detect NVIDIA GPU presence
has_nvidia_gpu() {
    command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null
}

# Detect Jetson platform
is_jetson() {
    [[ -f /etc/nv_tegra_release ]] || dpkg -l nvidia-l4t-core &>/dev/null 2>&1
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

# Print summary of what will be installed
print_summary() {
    printf "${BOLD}Components:${NC}\n"
    printf "  ${GREEN}✓${NC} Core (ROS 2, dev tools, GeographicLib, Python deps)\n"
    if [[ "$INSTALL_AUTOWARE" == "y" ]]; then
        printf "  ${GREEN}✓${NC} Autoware Debian packages\n"
    else
        printf "  ${YELLOW}⊘${NC} Autoware Debian packages (skipped)\n"
    fi
    if [[ "$INSTALL_ISAAC_ROS" == "y" ]]; then
        printf "  ${GREEN}✓${NC} Isaac ROS Visual Localization\n"
    else
        printf "  ${YELLOW}⊘${NC} Isaac ROS Visual Localization (skipped)\n"
    fi
    if [[ "$INSTALL_BLICKFELD" == "y" ]]; then
        printf "  ${GREEN}✓${NC} Blickfeld Scanner Library\n"
    else
        printf "  ${YELLOW}⊘${NC} Blickfeld Scanner Library (skipped)\n"
    fi
    printf "\n"
}

# Ask to confirm, retry, or quit
# Returns 0 on confirm, restarts caller on retry, exits on quit
ask_confirm_or_retry() {
    while true; do
        printf "${BLUE}?${NC} Continue? [Y/r/q] "
        read -r response || { printf "\n${YELLOW}Cancelled${NC}\n"; exit 130; }
        case "${response,,}" in
            ""|y|yes) return 0 ;;
            r|retry)  return 1 ;;
            q|quit|exit|n|no)
                printf "${YELLOW}Cancelled${NC}\n"
                exit 0
                ;;
            *) printf "${RED}Please answer y (continue), r (retry), or q (quit).${NC}\n" ;;
        esac
    done
}

# Interactive setup configuration
interactive_setup() {
    while true; do
        printf "\n${BOLD}${BLUE}AutoSDV Setup${NC}\n\n"

        # Offer install-all shortcut
        printf "This will install the AutoSDV development environment.\n"
        printf "Core components (ROS 2, dev tools, Python deps) are always installed.\n\n"
        if ask_yes_no "Install all optional components? (Autoware, Isaac ROS, Blickfeld)" "y"; then
            INSTALL_AUTOWARE="y"
            INSTALL_BLICKFELD="y"
            ACCEPT_BLICKFELD_EULA="1"
            if has_nvidia_gpu || is_jetson; then
                INSTALL_ISAAC_ROS="y"
            else
                INSTALL_ISAAC_ROS="n"
                printf "  ${YELLOW}→${NC} Isaac ROS skipped (no NVIDIA GPU detected)\n"
            fi
        else
            printf "\n"

            # Detailed questions
            INSTALL_AUTOWARE="n"
            printf "${YELLOW}Optional:${NC} Autoware Debian packages (~2-3 GB)\n"
            printf "  Pre-built binaries. Skip to build from source instead.\n"
            if ask_yes_no "Install Autoware Debian packages?" "y"; then
                INSTALL_AUTOWARE="y"
            fi
            printf "\n"

            # Isaac ROS — only ask if GPU detected
            INSTALL_ISAAC_ROS="n"
            if has_nvidia_gpu || is_jetson; then
                printf "${YELLOW}Optional:${NC} Isaac ROS Visual Localization\n"
                printf "  Camera-only localization (cuVSLAM + cuVGL). Requires NVIDIA GPU.\n"
                if ask_yes_no "Install Isaac ROS Visual Localization?" "y"; then
                    INSTALL_ISAAC_ROS="y"
                fi
            else
                printf "${YELLOW}Skipping:${NC} Isaac ROS (no NVIDIA GPU detected)\n"
            fi
            printf "\n"

            # Blickfeld — single question with EULA
            INSTALL_BLICKFELD="n"
            ACCEPT_BLICKFELD_EULA="0"
            printf "${YELLOW}Optional:${NC} Blickfeld Scanner Library (for Cube1 LiDAR)\n"
            printf "  License: https://github.com/NEWSLabNTU/blickfeld-scanner-lib\n"
            if ask_yes_no "Install Blickfeld Scanner Library and accept license terms?" "y"; then
                INSTALL_BLICKFELD="y"
                ACCEPT_BLICKFELD_EULA="1"
            fi
        fi

        printf "\n"
        export_choices
        print_summary
        if ask_confirm_or_retry; then
            printf "\n"
            return
        fi
        # retry — loop back to top
    done
}

# Export choices as environment variables for justfile
export_choices() {
    export SKIP_AUTOWARE_DEBIAN="$([[ "$INSTALL_AUTOWARE" == "n" ]] && echo "1" || echo "0")"
    export CONFIGURE_CYCLONEDDS_SYSCTL="n"
    export SKIP_BLICKFELD="$([[ "$INSTALL_BLICKFELD" == "n" ]] && echo "1" || echo "0")"
    export AUTOSDV_ACCEPT_BLICKFELD_EULA="$ACCEPT_BLICKFELD_EULA"
    export INSTALL_ISAAC_ROS="$INSTALL_ISAAC_ROS"
    export INSTALL_TURBOVNC_VIRTUALGL="n"
}

# Parse CLI flags and set defaults for non-interactive modes
parse_flags() {
    INSTALL_AUTOWARE="y"
    INSTALL_ISAAC_ROS="y"
    INSTALL_BLICKFELD="y"
    ACCEPT_BLICKFELD_EULA="1"

    for arg in "$@"; do
        case "$arg" in
            --no-autoware)  INSTALL_AUTOWARE="n" ;;
            --no-isaac)     INSTALL_ISAAC_ROS="n" ;;
            --no-blickfeld) INSTALL_BLICKFELD="n"; ACCEPT_BLICKFELD_EULA="0" ;;
        esac
    done

    # Auto-skip Isaac ROS if no GPU
    if [[ "$INSTALL_ISAAC_ROS" == "y" ]] && ! has_nvidia_gpu && ! is_jetson; then
        INSTALL_ISAAC_ROS="n"
        printf "${YELLOW}→${NC} Isaac ROS skipped (no NVIDIA GPU detected)\n"
    fi
}

# Ensure just is installed
ensure_just() {
    if command -v just &> /dev/null; then
        return
    fi

    local just_dir="$HOME/.local/bin"
    printf "${YELLOW}→${NC} 'just' not found. Installing to %s...\n" "$just_dir"
    mkdir -p "$just_dir"
    if curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to "$just_dir" 2>&1; then
        export PATH="$just_dir:$PATH"
        printf "${GREEN}✓${NC} just $("$just_dir/just" --version) installed\n"
        # Ensure ~/.local/bin is in PATH for future sessions
        if ! grep -qsF "$just_dir" "$HOME/.profile" "$HOME/.bashrc" 2>/dev/null; then
            printf "${YELLOW}→${NC} Adding %s to PATH in ~/.profile\n" "$just_dir"
            printf '\n# Added by AutoSDV setup\nexport PATH="%s:$PATH"\n' "$just_dir" >> "$HOME/.profile"
        fi
        printf "\n"
    else
        printf "${RED}✗${NC} Failed to install just\n"
        exit 1
    fi
}

# Print post-setup recommendations
print_post_setup() {
    printf "\n${BOLD}${GREEN}Setup complete!${NC}\n\n"
    printf "${BOLD}Recommended post-setup steps:${NC}\n\n"

    printf "  ${BLUE}1.${NC} Configure CycloneDDS kernel buffers (improves DDS performance):\n"
    printf "     ${BOLD}./setup.sh cyclonedds-sysctl${NC}\n\n"

    printf "  ${BLUE}2.${NC} Install TurboVNC + VirtualGL (required for ZED camera in VNC):\n"
    printf "     ${BOLD}./setup.sh turbovnc-virtualgl${NC}\n\n"

    if ! command -v direnv &> /dev/null; then
        printf "  ${BLUE}3.${NC} Install direnv for environment management:\n"
        printf "     sudo apt install direnv\n"
        printf "     echo 'eval \"\$(direnv hook bash)\"' >> ~/.bashrc\n"
        printf "     source ~/.bashrc && direnv allow\n\n"
    else
        printf "  ${GREEN}✓${NC} direnv detected — run: ${BOLD}direnv allow${NC}\n\n"
    fi
}

# Main
main() {
    # Handle --help/-h anywhere in args
    for arg in "$@"; do
        if [[ "$arg" == "--help" ]] || [[ "$arg" == "-h" ]]; then
            show_usage
            exit 0
        fi
    done

    # Determine mode from first positional arg
    local mode=""
    local recipe=""
    local pass_args=()

    for arg in "$@"; do
        case "$arg" in
            --all|--minimal|--no-autoware|--no-isaac|--no-blickfeld)
                # flags handled separately
                ;;
            *)
                if [[ -z "$recipe" ]]; then
                    recipe="$arg"
                fi
                pass_args+=("$arg")
                ;;
        esac
    done

    # Detect mode
    for arg in "$@"; do
        case "$arg" in
            --all)     mode="all" ;;
            --minimal) mode="minimal" ;;
        esac
    done

    # Default: interactive setup
    if [[ -z "$recipe" ]] && [[ -z "$mode" ]]; then
        mode="interactive"
        recipe="setup"
    elif [[ -z "$recipe" ]]; then
        recipe="setup"
    fi

    ensure_just

    # Configure based on mode
    case "$mode" in
        all)
            printf "\n${BOLD}${BLUE}AutoSDV Setup${NC} (install all)\n\n"
            parse_flags "$@"
            export_choices
            print_summary
            ;;
        minimal)
            printf "\n${BOLD}${BLUE}AutoSDV Setup${NC} (minimal)\n\n"
            INSTALL_AUTOWARE="n"
            INSTALL_ISAAC_ROS="n"
            INSTALL_BLICKFELD="n"
            ACCEPT_BLICKFELD_EULA="0"
            export_choices
            print_summary
            ;;
        interactive)
            if [[ ! -t 0 ]]; then
                printf "${YELLOW}→${NC} Non-interactive mode (use --all or --minimal)\n"
                parse_flags "$@"
                export_choices
            else
                interactive_setup
            fi
            ;;
    esac

    # Start sudo loop if needed
    if needs_sudo "$recipe"; then
        start_sudo_loop
    fi

    # Run just
    cd "$SCRIPT_DIR"
    if [[ ${#pass_args[@]} -eq 0 ]]; then
        just setup
    else
        just "${pass_args[@]}"
    fi

    # Post-setup recommendations
    if [[ "$recipe" == "setup" ]]; then
        print_post_setup
    fi
}

main "$@"
