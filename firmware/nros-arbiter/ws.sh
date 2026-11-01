#!/usr/bin/env bash
# Workspace helper for the nano-ros arbiter firmware.
#
#   ws.sh setup     fetch Zephyr 4.4 + nano-ros, apply nano-ros's 4.4 patch set,
#                   create the Python venv, build the nros CLI (~10 min, ~1 GB)
#   ws.sh build     cross-compile app/ for the XIAO RP2350
#   ws.sh flash     copy the UF2 onto a board mounted in BOOTSEL mode
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

    # The nros CLI at the pinned commit. The module calls it during configure;
    # an older installed CLI does not know the board-descriptor road.
    (cd "$NROS/packages/cli" &&
        CARGO_TARGET_DIR="$WS/cli-target" cargo build --release -p nros-cli --locked)

    # zenoh-pico source, from nano-ros's store.
    "$WS/cli-target/release/nros" setup --source zenoh-pico --index "$NROS/nros-sdk-index.toml"
    echo "ws.sh: workspace ready at $WS"
}

build() {
    [ -d "$WS/zephyr" ] || die "no workspace at $WS; run \`ws.sh setup\` first"
    eval "$(print_env)"
    # EXTRA_CONF_FILE repeats the image's os_config fragments. nano-ros drops
    # them when the board has its own prj.conf (a `;` inside NROS_LEAF_CONF
    # splits it), which built an image with no zenoh link and no warning.
    # Remove once the pinned nano-ros carries the fix.
    west build -p auto -b "$BOARD" -d "$BUILD" "$HERE/app" -- \
        -DNROS_IMAGE=arbiter \
        "-DEXTRA_CONF_FILE=prj-link.conf" \
        "$@"
}

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

case "${1:-}" in
    setup) setup ;;
    build) shift; build "$@" ;;
    flash) flash ;;
    clean) rm -rf "$BUILD" ;;
    env) print_env ;;
    *) sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
