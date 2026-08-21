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
  ./setup.sh              Run interactive setup (component menu)
  ./setup.sh --all        Install everything without questions
  ./setup.sh --minimal    Install core only (ROS 2, dev tools, Rust, colcon-cargo-ros2,
                          Python deps)
  ./setup.sh --dry-run    Pick components and print the selection, install nothing
  ./setup.sh status       Show setup status
  ./setup.sh <recipe>     Run specific recipe (ros2, opencv, network-dds, ...)
  ./setup.sh --help       Show this help

Flags (combine with --all or --minimal):
  --no-autoware           Skip Autoware Debian packages
  --no-isaac              Skip Isaac ROS Visual Localization
  --no-blickfeld          Skip Blickfeld Scanner Library
  --no-opencv             Skip the OpenCV consistency fix
  --no-engines            Skip pre-compiling TensorRT engines (minutes)
  --no-zed                Skip the ZED SDK
  --no-network            Skip DDS network configuration

Examples:
  ./setup.sh                      # Interactive setup
  ./setup.sh --all                # Install everything, no questions
  ./setup.sh --all --no-isaac     # Everything except Isaac ROS
  ./setup.sh --minimal            # Core only
  ./setup.sh status               # Check what's installed
  ./setup.sh ros2                 # Install only ROS 2
  ./setup.sh clean-markers        # Reset all installation markers

For available recipes, run: cd setup && just --list
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
NO_SUDO_RECIPES="status clean-marker clean-markers opencv-check default"

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
    local default="${2-y}"
    local prompt response

    if [[ -z "$default" ]]; then
        prompt="[y/n/q]"
    elif [[ "$default" == "y" ]]; then
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

# ── Component menu ──────────────────────────────────────────────────────────
#
# One screen the user can review and toggle, rather than a run of yes/no
# prompts. The prompts had two problems: an answer could not be revised once
# given, and the Autoware step went on to ask ITS OWN questions partway
# through the install — a second interview arriving after you thought you were
# done. Those are folded in here (see AUTOWARE_PREREQ_* below) and passed to
# that script as flags.
#
# Each entry: key|default|indent|label|note
# `indent` marks a sub-option of the entry above it: shown indented, and
# ignored entirely unless its parent is selected.
MENU_ITEMS=(
  "AUTOWARE|y|0|Autoware Debian packages|~2-3 GB. Skip to build from source instead."
  "AUTOWARE_PREREQ_ROS|n|1|└ let Autoware install ROS 2 Humble|Normally NO: this setup installs ROS 2 itself, earlier."
  "AUTOWARE_PREREQ_SPCONV|n|1|└ SpConv/Cumm libraries|Only for BEVFusion-class models; this stack does not use them."
  "AUTOWARE_DATA|y|0|Writable Autoware data dir|Seconds. The packaged tree is root-owned, so without it TensorRT cannot cache an engine."
  "TENSORRT_ENGINES|n|1|└ compile TensorRT engines now|Minutes on an Orin. Otherwise the first launch pays it, with perception down until it finishes."
  "ISAAC_ROS|y|0|Isaac ROS Visual Localization|cuVSLAM + cuVGL, for pose_source:=visual/isaac. Needs an NVIDIA GPU."
  "BLICKFELD|y|0|Blickfeld Scanner Library|Driver for the Cube1 LiDAR. Selecting it accepts the library's licence terms."
  "COLCON_CARGO_ROS2|y|0|colcon-cargo-ros2 (Rust colcon support)|>= 0.5.1. Without it colcon skips cuda_ndt_matcher and the build aborts."
  "OPENCV|y|0|OpenCV consistency (4.5.4)|JetPack leaves 4.8.0 headers over a 4.5.4 runtime. Also what makes aruco/contrib available."
  "NETWORK_DDS|y|0|Network configuration (DDS)|REQUIRED to run ROS here. Both sub-steps below; .envrc warns when they are missing."
  "CYCLONEDDS_SYSCTL|y|1|└ kernel socket buffers|net.core.rmem_max=2GB + net.ipv4.ipfrag_*. Below 10MB no ros2 node can start."
  "MULTICAST_LO|y|1|└ multicast on lo (persistent)|cyclonedds.xml pins lo, and lo loses MULTICAST on reboot. Installs multicast-lo.service."
  "ZED_SDK|n|0|ZED SDK|Stereolabs SDK for the ZED X Mini. Large download; only needed for ZED cameras."
  "TURBOVNC_VIRTUALGL|n|0|TurboVNC + VirtualGL|GPU-accelerated rendering over VNC. Required to run the ZED tools in a VNC session."
)

declare -A MENU_STATE

menu_field() { printf '%s' "$1" | cut -d'|' -f"$2"; }

# Is this entry's parent selected? Sub-options are meaningless otherwise.
menu_parent_on() {
    local idx="$1" i
    for (( i = idx - 1; i >= 0; i-- )); do
        if [[ "$(menu_field "${MENU_ITEMS[$i]}" 3)" == "0" ]]; then
            [[ "${MENU_STATE[$(menu_field "${MENU_ITEMS[$i]}" 1)]}" == "y" ]]
            return $?
        fi
    done
    return 0
}

# Rendering is cursor-driven: the block is drawn once, then redrawn in place
# by moving the cursor back up over it. Clearing the whole screen instead would
# throw away whatever the user was looking at before running setup.
#
# That trick only works while the block fits on screen. Draw more lines than the
# terminal has and it scrolls, the cursor-up rewind lands in the wrong place, and
# every redraw smears a fresh copy down the terminal. So the item list is drawn
# through a viewport: MENU_TOP is the first item shown, the window is sized from
# the real terminal height on every render, and the block never scrolls.
MENU_LINES=0
MENU_TOP=0
MENU_CURSOR=0

# Rows one item occupies: its own line, plus its note line if it has one.
menu_item_height() {
    local note
    note=$(menu_field "${MENU_ITEMS[$1]}" 5)
    [[ -n "$note" ]] && printf 2 || printf 1
}

# Rows available for items, after the header, the footer and the two indicator
# slots. Read on every render so a resize is picked up without a redraw loop.
menu_body_budget() {
    local term_lines budget
    term_lines=$(tput lines 2>/dev/null) || term_lines="${LINES:-24}"
    [[ "$term_lines" =~ ^[0-9]+$ ]] || term_lines=24
    # 3 header + 2 footer + 2 indicator slots, and one spare line so the shell
    # prompt that follows does not push the block up by itself.
    budget=$(( term_lines - 8 ))
    # Below this there is no useful viewport left; show one item and let the
    # terminal be too small rather than dividing by nothing.
    (( budget < 2 )) && budget=2
    printf '%s' "$budget"
}

# Terminal width, for the truncation below.
menu_cols() {
    local cols
    cols=$(tput cols 2>/dev/null) || cols="${COLUMNS:-80}"
    [[ "$cols" =~ ^[0-9]+$ ]] || cols=80
    printf '%s' "$cols"
}

# Cut a label or note to the width it is drawn in.
#
# Not cosmetic. A line longer than the terminal wraps, and a wrapped line takes
# two physical rows while this code counts it as one. MENU_LINES then
# understates the block, the rewind lands inside it instead of above it, and
# every redraw leaves a copy of the tail behind. Several of these notes are over
# a hundred characters, so an 80-column terminal hits it immediately.
menu_fit() {
    local text="$1" width="$2"
    (( width < 10 )) && width=10
    if (( ${#text} > width )); then
        printf '%s…' "${text:0:width-1}"
    else
        printf '%s' "$text"
    fi
}

# Slide the viewport just far enough that the cursor's item is fully visible,
# note included. Only ever moves by whole items, so a note never appears
# orphaned from the label it belongs to.
menu_scroll_into_view() {
    local budget="$1" used i
    (( MENU_CURSOR < MENU_TOP )) && MENU_TOP=$MENU_CURSOR
    while (( MENU_TOP < MENU_CURSOR )); do
        used=0
        for (( i = MENU_TOP; i <= MENU_CURSOR; i++ )); do
            used=$(( used + $(menu_item_height "$i") ))
        done
        (( used <= budget )) && break
        MENU_TOP=$(( MENU_TOP + 1 ))
    done
}

menu_render() {
    local i key label note indent mark dim pointer lines=0
    local budget used=0 last_shown cols
    cols=$(menu_cols)
    budget=$(menu_body_budget)
    menu_scroll_into_view "$budget"

    printf "\n${BLUE}AutoSDV Setup${NC}  —  %s\n\n" \
        "$(menu_fit "core (ROS 2, dev tools, Rust, GeographicLib, Python deps) is always installed" $(( cols - 21 )))"
    lines=$(( lines + 3 ))

    # The indicator slots are always drawn, blank when there is nothing beyond
    # the edge. A slot that appears and disappears would change the block height
    # between renders, and the rewind is computed from that height.
    if (( MENU_TOP > 0 )); then
        printf "      ${BLUE}↑ %d more above${NC}\n" "$MENU_TOP"
    else
        printf "\n"
    fi
    lines=$(( lines + 1 ))

    last_shown=$(( MENU_TOP - 1 ))
    for (( i = MENU_TOP; i < ${#MENU_ITEMS[@]}; i++ )); do
        (( used + $(menu_item_height "$i") > budget )) && break
        used=$(( used + $(menu_item_height "$i") ))
        last_shown=$i
        key=$(menu_field "${MENU_ITEMS[$i]}" 1)
        indent=$(menu_field "${MENU_ITEMS[$i]}" 3)
        label=$(menu_field "${MENU_ITEMS[$i]}" 4)
        note=$(menu_field "${MENU_ITEMS[$i]}" 5)
        [[ "${MENU_STATE[$key]}" == "y" ]] && mark="${GREEN}x${NC}" || mark=" "

        # Grey out a sub-option whose parent is off — still listed, so its
        # existence is discoverable, but plainly not in play.
        dim=""
        if [[ "$indent" == "1" ]] && ! menu_parent_on "$i"; then
            dim="${YELLOW}"
            mark="-"
        fi

        # The cursor line is marked by a caret rather than by colour alone:
        # colour is what a dimmed sub-option already uses, so a second colour
        # would collide with it.
        if (( i == MENU_CURSOR )); then
            pointer="${BLUE}❯${NC}"
        else
            pointer=" "
        fi

        # Widths are the printed prefix: " x [x] " is 7 columns, an indented
        # label adds 4, and a note is indented by 8.
        if [[ "$indent" == "1" ]]; then
            printf " %b [%b] %b    %s${NC}\n" "$pointer" "$mark" "$dim" \
                "$(menu_fit "$label" $(( cols - 12 )))"
        else
            printf " %b [%b] %b%s${NC}\n" "$pointer" "$mark" "$dim" \
                "$(menu_fit "$label" $(( cols - 8 )))"
        fi
        lines=$(( lines + 1 ))
        if [[ -n "$note" ]]; then
            printf "        %b%s${NC}\n" "${dim:-$YELLOW}" \
                "$(menu_fit "$note" $(( cols - 9 )))"
            lines=$(( lines + 1 ))
        fi
    done

    local remaining=$(( ${#MENU_ITEMS[@]} - last_shown - 1 ))
    if (( remaining > 0 )); then
        printf "      ${BLUE}↓ %d more below${NC}\n" "$remaining"
    else
        printf "\n"
    fi
    lines=$(( lines + 1 ))

    # Two hint sets, because the full one is 96 columns and would wrap.
    if (( cols >= 100 )); then
        printf "\n  ${BLUE}↑↓${NC} move   ${BLUE}PgUp/PgDn${NC} page   ${BLUE}Home/End${NC} ends   ${BLUE}SPACE${NC} toggle   ${BLUE}a${NC}/${BLUE}n${NC} all/none   ${BLUE}ENTER${NC} go   ${BLUE}q${NC} quit\n"
    else
        printf "\n  ${BLUE}↑↓${NC} move  ${BLUE}SPACE${NC} toggle  ${BLUE}a${NC}/${BLUE}n${NC} all/none  ${BLUE}ENTER${NC} go  ${BLUE}q${NC} quit\n"
    fi
    lines=$(( lines + 2 ))
    MENU_LINES=$lines
}

# Move back over the block just drawn so the next render overwrites it.
menu_rewind() {
    (( MENU_LINES > 0 )) || return 0
    printf '\033[%dA\033[J' "$MENU_LINES"
}

# One keypress, with arrow keys decoded. Arrows arrive as ESC [ A/B, and the
# trailing reads are given a timeout so a bare ESC does not block.
menu_read_key() {
    local key rest
    IFS= read -rsn1 key || return 1
    if [[ "$key" == $'\033' ]]; then
        read -rsn2 -t 0.05 rest || rest=""
        # PgUp/PgDn/Home/End arrive as ESC [ <digit> ~, one byte longer than the
        # arrows. Without swallowing that trailing ~ it is read as the next
        # keystroke, and the menu reacts to a key nobody pressed.
        if [[ "$rest" =~ ^\[[0-9]$ ]]; then
            local tail
            read -rsn1 -t 0.05 tail || tail=""
            case "${rest}${tail}" in
                '[5~') printf 'pgup'  ; return 0 ;;
                '[6~') printf 'pgdn'  ; return 0 ;;
                '[1~'|'[7~') printf 'home' ; return 0 ;;
                '[4~'|'[8~') printf 'end'  ; return 0 ;;
                *)     printf 'esc'   ; return 0 ;;
            esac
        fi
        case "$rest" in
            '[A') printf 'up' ;;
            '[B') printf 'down' ;;
            '[C') printf 'right' ;;
            '[D') printf 'left' ;;
            '[H') printf 'home' ;;
            '[F') printf 'end' ;;
            'OH') printf 'home' ;;
            'OF') printf 'end' ;;
            *)    printf 'esc' ;;
        esac
        return 0
    fi
    case "$key" in
        '')      printf 'enter' ;;
        ' ')     printf 'space' ;;
        k|K)     printf 'up' ;;
        j|J)     printf 'down' ;;
        *)       printf '%s' "$key" ;;
    esac
}

menu_toggle_current() {
    local key indent
    key=$(menu_field "${MENU_ITEMS[$MENU_CURSOR]}" 1)
    indent=$(menu_field "${MENU_ITEMS[$MENU_CURSOR]}" 3)
    # A sub-option whose parent is off cannot be turned on from here: the run
    # would drop it anyway, so accepting the keystroke would be a lie.
    if [[ "$indent" == "1" ]] && ! menu_parent_on "$MENU_CURSOR"; then
        return 0
    fi
    [[ "${MENU_STATE[$key]}" == "y" ]] && MENU_STATE[$key]="n" || MENU_STATE[$key]="y"
}

# Seed MENU_STATE from the table's defaults, adjusted for the machine.
menu_defaults() {
    local i key def
    for i in "${!MENU_ITEMS[@]}"; do
        key=$(menu_field "${MENU_ITEMS[$i]}" 1)
        def=$(menu_field "${MENU_ITEMS[$i]}" 2)
        # Isaac ROS needs an NVIDIA GPU; do not offer it elsewhere.
        if [[ "$key" == "ISAAC_ROS" ]] && ! has_nvidia_gpu && ! is_jetson; then
            def="n"
        fi
        # The OpenCV step corrects a JetPack-specific conflict: NVIDIA's 4.8.0
        # libopencv-dev over Ubuntu's 4.5.4 runtime. Elsewhere there is nothing
        # to correct, so leave it off rather than run a no-op with sudo.
        if [[ "$key" == "OPENCV" ]] && ! is_jetson; then
            def="n"
        fi
        MENU_STATE["$key"]="$def"
    done
}

# Turn MENU_STATE into the environment the justfile reads.
export_choices() {
    local i key

    # A sub-option whose parent ended up off must not leak into the run.
    for i in "${!MENU_ITEMS[@]}"; do
        key=$(menu_field "${MENU_ITEMS[$i]}" 1)
        if [[ "$(menu_field "${MENU_ITEMS[$i]}" 3)" == "1" ]] && ! menu_parent_on "$i"; then
            MENU_STATE[$key]="n"
        fi
    done

    # SKIP_* keep their inverted sense because setup/justfile already reads
    # them that way.
    export SKIP_AUTOWARE_DEBIAN="$([[ "${MENU_STATE[AUTOWARE]}" == "n" ]] && echo 1 || echo 0)"
    export AUTOWARE_PREREQ_ROS="${MENU_STATE[AUTOWARE_PREREQ_ROS]}"
    export AUTOWARE_PREREQ_SPCONV="${MENU_STATE[AUTOWARE_PREREQ_SPCONV]}"
    export SETUP_AUTOWARE_DATA="${MENU_STATE[AUTOWARE_DATA]}"
    export BUILD_TENSORRT_ENGINES="${MENU_STATE[TENSORRT_ENGINES]}"
    export INSTALL_ISAAC_ROS="${MENU_STATE[ISAAC_ROS]}"
    export SKIP_BLICKFELD="$([[ "${MENU_STATE[BLICKFELD]}" == "n" ]] && echo 1 || echo 0)"
    # The Blickfeld installer reads its licence acceptance from the same
    # answer: selecting it in the menu is the acceptance, and the menu note
    # says so.
    export AUTOSDV_ACCEPT_BLICKFELD_EULA="$([[ "${MENU_STATE[BLICKFELD]}" == "y" ]] && echo 1 || echo 0)"
    export INSTALL_COLCON_CARGO_ROS2="${MENU_STATE[COLCON_CARGO_ROS2]}"
    export INSTALL_OPENCV="${MENU_STATE[OPENCV]}"
    export CONFIGURE_CYCLONEDDS_SYSCTL="${MENU_STATE[CYCLONEDDS_SYSCTL]}"
    export CONFIGURE_MULTICAST_LO="${MENU_STATE[MULTICAST_LO]}"
    export INSTALL_ZED_SDK="${MENU_STATE[ZED_SDK]}"
    export INSTALL_TURBOVNC_VIRTUALGL="${MENU_STATE[TURBOVNC_VIRTUALGL]}"
}

print_selection() {
    local i key
    printf "\n${BOLD}Installing:${NC} Core"
    for i in "${!MENU_ITEMS[@]}"; do
        key=$(menu_field "${MENU_ITEMS[$i]}" 1)
        [[ "${MENU_STATE[$key]}" == "y" ]] || continue
        [[ "$(menu_field "${MENU_ITEMS[$i]}" 3)" == "1" ]] && continue
        printf " + %s" "$(menu_field "${MENU_ITEMS[$i]}" 4)"
    done
    printf "\n\n"
}

interactive_setup() {
    local action key

    menu_defaults
    MENU_CURSOR=0

    # Without a terminal there are no keystrokes to read. Take the defaults and
    # say so, rather than blocking on a read that will never return.
    if [[ ! -t 0 ]]; then
        printf "${YELLOW}Not a terminal — using default component selection.${NC}\n"
    else
        # The cursor is hidden for the duration and restored however we leave,
        # including Ctrl-C: a terminal left without a cursor is a bad parting
        # gift. cleanup still runs, so the sudo loop is not orphaned either.
        printf '\033[?25l'
        trap 'printf "\033[?25h"; cleanup' EXIT

        while true; do
            menu_render
            action=$(menu_read_key) || { printf '\033[?25h'; printf "\n${YELLOW}Cancelled${NC}\n"; exit 130; }
            case "$action" in
                up)    (( MENU_CURSOR > 0 )) && MENU_CURSOR=$(( MENU_CURSOR - 1 )) ;;
                down)  (( MENU_CURSOR < ${#MENU_ITEMS[@]} - 1 )) && MENU_CURSOR=$(( MENU_CURSOR + 1 )) ;;
                pgup)  MENU_CURSOR=$(( MENU_CURSOR - 5 )); (( MENU_CURSOR < 0 )) && MENU_CURSOR=0 ;;
                pgdn)  MENU_CURSOR=$(( MENU_CURSOR + 5 ))
                       (( MENU_CURSOR > ${#MENU_ITEMS[@]} - 1 )) && MENU_CURSOR=$(( ${#MENU_ITEMS[@]} - 1 )) ;;
                home)  MENU_CURSOR=0 ;;
                end)   MENU_CURSOR=$(( ${#MENU_ITEMS[@]} - 1 )) ;;
                space) menu_toggle_current ;;
                a|A)   for key in "${!MENU_STATE[@]}"; do MENU_STATE[$key]="y"; done ;;
                n|N)   for key in "${!MENU_STATE[@]}"; do MENU_STATE[$key]="n"; done ;;
                enter) menu_rewind; menu_render; break ;;
                q|Q)   printf '\033[?25h'; printf "${YELLOW}Cancelled${NC}\n"; exit 0 ;;
            esac
            menu_rewind
        done

        printf '\033[?25h'
        trap cleanup EXIT
    fi

    export_choices
    print_selection

    # Only ask when there is someone to answer. Without a terminal the read
    # hits EOF immediately and the run would abort as if the user had declined.
    if [[ -t 0 ]] && ! ask_yes_no "Continue?" "y"; then
        printf "${YELLOW}Cancelled${NC}\n"
        exit 0
    fi
    printf "\n"
}

# Non-interactive selection: start from the table's defaults (or all-off for
# --minimal) and apply the --no-* flags.
noninteractive_setup() {
    local mode="$1"; shift
    local arg key i

    menu_defaults

    # --all means all: every row on, not just the rows that default on. ZED SDK,
    # TurboVNC and the engine build default off in the menu because they are
    # slow or hardware-specific, but a flag named --all that skipped them would
    # be lying. Use --no-* to drop the ones you do not want.
    if [[ "$mode" == "all" ]]; then
        for i in "${!MENU_ITEMS[@]}"; do
            key=$(menu_field "${MENU_ITEMS[$i]}" 1)
            MENU_STATE[$key]="y"
        done
        # The Autoware prerequisite sub-options stay off even under --all: this
        # setup installs ROS 2 itself, and SpConv is for models this stack does
        # not use. Turning them on is a deliberate act, not a side effect.
        MENU_STATE[AUTOWARE_PREREQ_ROS]="n"
        MENU_STATE[AUTOWARE_PREREQ_SPCONV]="n"
    fi

    if [[ "$mode" == "minimal" ]]; then
        for i in "${!MENU_ITEMS[@]}"; do
            key=$(menu_field "${MENU_ITEMS[$i]}" 1)
            MENU_STATE[$key]="n"
        done
        # colcon-cargo-ros2 stays on: cuda_ndt_matcher builds with ament_cargo,
        # so without it `just build` aborts. A "core only" install that cannot
        # build the workspace is not a useful minimum.
        MENU_STATE[COLCON_CARGO_ROS2]="y"
    fi

    for arg in "$@"; do
        case "$arg" in
            --no-autoware)  MENU_STATE[AUTOWARE]="n"; MENU_STATE[AUTOWARE_DATA]="n"
                            MENU_STATE[TENSORRT_ENGINES]="n" ;;
            --no-isaac)     MENU_STATE[ISAAC_ROS]="n" ;;
            --no-blickfeld) MENU_STATE[BLICKFELD]="n" ;;
            --no-opencv)    MENU_STATE[OPENCV]="n" ;;
            --no-engines)   MENU_STATE[TENSORRT_ENGINES]="n" ;;
            --no-zed)       MENU_STATE[ZED_SDK]="n" ;;
            --no-network)   MENU_STATE[NETWORK_DDS]="n" ;;
        esac
    done

    # --all means all, but Isaac ROS still needs a GPU to install against.
    if [[ "$mode" == "all" ]] && [[ "${MENU_STATE[ISAAC_ROS]}" == "y" ]] \
       && ! has_nvidia_gpu && ! is_jetson; then
        MENU_STATE[ISAAC_ROS]="n"
        printf "${YELLOW}→${NC} Isaac ROS skipped (no NVIDIA GPU detected)\n"
    fi

    export_choices
    print_selection
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
    printf "Check what landed: ${BOLD}./setup.sh status${NC}\n\n"

    if ! command -v direnv &> /dev/null; then
        printf "${BOLD}Install direnv for environment management:${NC}\n"
        printf "  sudo apt install direnv\n"
        printf "  echo 'eval \"\$(direnv hook bash)\"' >> ~/.bashrc\n"
        printf "  source ~/.bashrc && direnv allow\n\n"
        printf "See: ${BLUE}https://direnv.net/${NC}\n"
    else
        printf "${GREEN}✓${NC} direnv detected — run: ${BOLD}direnv allow${NC}\n\n"
    fi
}

# Main
main() {
    local arg k mode="" recipe="" dry_run=0 pass_args=()

    # Handle --help/-h anywhere in args
    for arg in "$@"; do
        if [[ "$arg" == "--help" ]] || [[ "$arg" == "-h" ]]; then
            show_usage
            exit 0
        fi
    done

    # Split our own flags from the recipe name and its arguments
    for arg in "$@"; do
        case "$arg" in
            --all|--minimal|--dry-run|--no-*)
                ;;
            *)
                [[ -z "$recipe" ]] && recipe="$arg"
                pass_args+=("$arg")
                ;;
        esac
    done

    # --dry-run is orthogonal to how the selection is made: it modifies --all
    # and --minimal as readily as the menu. Kept out of $mode for exactly that
    # reason — folding it in made `--all --dry-run` silently print the menu's
    # defaults instead of what --all had selected.
    for arg in "$@"; do
        case "$arg" in
            --all)     mode="all" ;;
            --minimal) mode="minimal" ;;
            --dry-run) dry_run=1 ;;
        esac
    done

    if [[ -z "$recipe" ]]; then
        recipe="setup"
        [[ -z "$mode" ]] && mode="interactive"
    fi

    ensure_just

    if [[ "$recipe" == "setup" ]]; then
        case "$mode" in
            all)
                printf "\n${BOLD}${BLUE}AutoSDV Setup${NC} (install all)\n"
                noninteractive_setup all "$@"
                ;;
            minimal)
                printf "\n${BOLD}${BLUE}AutoSDV Setup${NC} (minimal)\n"
                noninteractive_setup minimal "$@"
                ;;
            *)
                if [[ ! -t 0 ]]; then
                    printf "${YELLOW}→${NC} Non-interactive mode (use --all or --minimal)\n"
                    noninteractive_setup all "$@"
                else
                    interactive_setup
                fi
                ;;
        esac
    fi

    # Print the selection and stop, without touching the machine. Worth having
    # on its own terms, and it is the only way to exercise the menu without
    # running a multi-gigabyte install.
    if [[ $dry_run -eq 1 ]]; then
        printf "${BLUE}Dry run — nothing installed. Selection:${NC}\n"
        for k in SKIP_AUTOWARE_DEBIAN AUTOWARE_PREREQ_ROS AUTOWARE_PREREQ_SPCONV \
                 SETUP_AUTOWARE_DATA BUILD_TENSORRT_ENGINES INSTALL_ISAAC_ROS SKIP_BLICKFELD \
                 AUTOSDV_ACCEPT_BLICKFELD_EULA INSTALL_COLCON_CARGO_ROS2 \
                 INSTALL_OPENCV CONFIGURE_CYCLONEDDS_SYSCTL CONFIGURE_MULTICAST_LO \
                 INSTALL_ZED_SDK INSTALL_TURBOVNC_VIRTUALGL; do
            printf "  %-32s %s\n" "$k" "${!k}"
        done
        exit 0
    fi

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

    if [[ "$recipe" == "setup" ]]; then
        print_post_setup
    fi
}

main "$@"
