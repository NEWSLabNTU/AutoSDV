#!/usr/bin/env bash
# Workspace helper for the nano-ros arbiter firmware.
#
#   ws.sh setup     fetch Zephyr 4.4 + nano-ros, apply nano-ros's 4.4 patch set,
#                   create the Python venv, build the nros CLI (~10 min, ~1 GB)
#   ws.sh build     cross-compile app/ for the XIAO RP2350
#   ws.sh flash     copy the UF2 onto a board mounted in BOOTSEL mode
#   ws.sh sim       build the simulator variant and run it in Wokwi (needs
#                   wokwi-cli and a token: $WOKWI_CLI_TOKEN or ~/.config/wokwi/token)
#   ws.sh qemu [s]  build the QEMU mps2-an521 image and run it end to end for
#                   s seconds (default 30): serial link, rmw_zenohd, a ROS 2
#                   subscriber on /vehicle/arbiter/mode (needs ROS 2 Humble with
#                   rmw_zenoh_cpp, and socat)
#   ws.sh clean     remove the build directory (the workspace stays)
#   ws.sh env       print the environment the build uses, for `eval`
#
# The workspace lives in $NROS_ARBITER_WS (default: .ws beside this script,
# gitignored). It is a west workspace whose manifest is manifest/west.yml,
# reached through a symlink so the manifest stays in this repository.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${NROS_ARBITER_WS:-$HERE/.ws}"
BUILD="${NROS_ARBITER_BUILD:-$HERE/build}"
BOARD="xiao_rp2350/rp2350a/m33"
NROS="$WS/modules/lib/nano-ros"

die() { echo "ws.sh: $*" >&2; exit 1; }

# Arm GNU toolchain. Zephyr 4.4's own SDK is 1.0.1; the board also builds with
# the plain Arm GNU toolchain (gnuarmemb), which nano-ros's store already has.
toolchain() {
    if [ -n "${GNUARMEMB_TOOLCHAIN_PATH:-}" ]; then
        echo "$GNUARMEMB_TOOLCHAIN_PATH"; return
    fi
    local cand
    cand="$(ls -d "$HOME"/.nros/sdk/arm-none-eabi-gcc/*/ 2>/dev/null | sort -V | tail -1)"
    [ -n "$cand" ] || die "no Arm GNU toolchain: set GNUARMEMB_TOOLCHAIN_PATH, or run \`nros setup --tool arm-none-eabi-gcc\`"
    echo "${cand%/}"
}

print_env() {
    cat <<EOF
export PATH="$WS/.venv312/bin:\$PATH"
export ZEPHYR_BASE="$WS/zephyr"
export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
export GNUARMEMB_TOOLCHAIN_PATH="$(toolchain)"
export NROS_CLI="$WS/cli-target/release/nros"
export NROS_EXTRA_BOARD_PATH="$HERE/boards"
EOF
}

setup() {
    command -v west >/dev/null || die "west not found (pip install west)"
    mkdir -p "$WS/.west"
    [ -e "$WS/manifest" ] || ln -s "$HERE/manifest" "$WS/manifest"
    # Written by hand rather than with `west init -l`, which resolves the
    # symlink and would make this directory, not $WS, the workspace.
    if [ ! -f "$WS/.west/config" ]; then
        printf '[manifest]\npath = manifest\nfile = west.yml\n\n[zephyr]\nbase = zephyr\n' \
            > "$WS/.west/config"
    fi
    (cd "$WS" && west update --narrow -o=--depth=1)

    # nano-ros's 4.4 patch set: the Zephyr fixes nano-ros depends on, plus the
    # Python 3.12 venv Zephyr 4.4 requires (made with uv).
    NROS_ZEPHYR_CREATE_VENV=1 bash "$NROS/scripts/zephyr/patches/4.4.sh" "$WS"

    # Zephyr fixes this board needs that v4.4.0 lacks, backported from
    # upstream. Drop each once the pinned Zephyr contains it.
    local p
    for p in "$HERE"/patches/zephyr-*.patch; do
        if git -C "$WS/zephyr" apply --reverse --check "$p" 2>/dev/null; then
            continue    # already applied
        fi
        git -C "$WS/zephyr" apply "$p" || die "cannot apply $(basename "$p")"
    done

    # zenoh-pico fixes the pinned nano-ros lacks. nano-ros compiles zenoh-pico
    # from its shared store unless its own checkout's submodule carries edits
    # (the `local-edit` rung of `nros locate`), so the submodule is checked out
    # at nano-ros's pin and the patches go there, never into the store.
    local zp="$NROS/packages/rmw/zenoh/zpico-sys/zenoh-pico"
    if compgen -G "$HERE/patches/zenoh-pico-*.patch" >/dev/null; then
        git -C "$NROS" submodule update --init --depth 1 packages/rmw/zenoh/zpico-sys/zenoh-pico
        for p in "$HERE"/patches/zenoh-pico-*.patch; do
            if git -C "$zp" apply --reverse --check "$p" 2>/dev/null; then
                continue
            fi
            git -C "$zp" apply "$p" || die "cannot apply $(basename "$p")"
        done
    fi

    # The nros CLI at the pinned commit. The module calls it during configure;
    # an older installed CLI does not know the board-descriptor road.
    (cd "$NROS/packages/cli" &&
        CARGO_TARGET_DIR="$WS/cli-target" cargo build --release -p nros-cli --locked)

    # zenoh-pico source, from nano-ros's store.
    "$WS/cli-target/release/nros" setup --source zenoh-pico --index "$NROS/nros-sdk-index.toml"
    echo "ws.sh: workspace ready at $WS"
}

# build_image <west board> <system.toml image> <build dir> [cmake args...]
build_image() {
    [ -d "$WS/zephyr" ] || die "no workspace at $WS; run \`ws.sh setup\` first"
    local board="$1" image="$2" dir="$3"
    shift 3
    eval "$(print_env)"
    west build -p auto -b "$board" -d "$dir" "$HERE/app" -- -DNROS_IMAGE="$image" "$@"
}

build() { build_image "$BOARD" arbiter "$BUILD" "$@"; }

flash() {
    local uf2="$BUILD/zephyr/zephyr.uf2"
    [ -f "$uf2" ] || die "no $uf2; run \`ws.sh build\` first"
    local mnt="${RP2350_MOUNT:-}"
    if [ -z "$mnt" ]; then
        mnt="$(ls -d /media/"$USER"/RP2350 /run/media/"$USER"/RP2350 2>/dev/null | head -1 || true)"
    fi
    [ -n "$mnt" ] || die "no RP2350 drive mounted: hold BOOT while plugging the board in, or set RP2350_MOUNT"
    cp "$uf2" "$mnt/" && sync
    echo "ws.sh: flashed $uf2 to $mnt"
}

# Wokwi has no XIAO RP2350, but its Pi Pico 2 is the same chip. The only
# difference in the build is where the log goes (wokwi/sim.overlay).
sim() {
    build_image "$BOARD" arbiter "$HERE/build-sim" "-DEXTRA_DTC_OVERLAY_FILE=$HERE/wokwi/sim.overlay"
    command -v wokwi-cli >/dev/null || die "wokwi-cli not found: https://github.com/wokwi/wokwi-cli/releases"
    if [ -z "${WOKWI_CLI_TOKEN:-}" ] && [ -f "$HOME/.config/wokwi/token" ]; then
        WOKWI_CLI_TOKEN="$(cat "$HOME/.config/wokwi/token")"
        export WOKWI_CLI_TOKEN
    fi
    [ -n "${WOKWI_CLI_TOKEN:-}" ] || die "no Wokwi token: set WOKWI_CLI_TOKEN or write it to ~/.config/wokwi/token"
    wokwi-cli "$HERE/wokwi" "$@"
}

# QEMU's mps2-an521 is a Cortex-M33 like the RP2350's, with none of the car's
# hardware; the image simulates it (app/src/sim/board_sim.cpp).
qemu() {
    build_image mps2/an521/cpu0 arbiter_qemu "$HERE/build-qemu"
    "$HERE/qemu/e2e.sh" "$HERE/build-qemu/zephyr/zephyr.elf" "$HERE/build-qemu/e2e" "${1:-30}"
}

case "${1:-}" in
    setup) setup ;;
    build) shift; build "$@" ;;
    flash) flash ;;
    sim) shift; sim "$@" ;;
    qemu) shift; qemu "$@" ;;
    clean) rm -rf "$BUILD" ;;
    env) print_env ;;
    *) sed -n '2,19p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
