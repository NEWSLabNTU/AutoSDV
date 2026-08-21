#!/usr/bin/env bash
# Install/upgrade colcon-cargo-ros2 (Rust support for colcon)
#
# cuda_ndt_matcher builds with ament_cargo. Without this extension colcon does
# not process it at all: it reports it as "not processed", every dependent
# fails looking for its package.sh, and the rest of the workspace aborts. The
# error names the missing package.sh rather than the missing extension, so it
# is worth installing up front.

set -e

# 0.5.1 is the floor: earlier releases do not emit the [patch.crates-io]
# entries the two Rust packages rely on, so a stale install fails the build
# while still importing cleanly. An import check alone would never notice.
REQUIRED_VERSION="0.5.1"

installed_version() {
    python3 - <<'PY' 2>/dev/null
import importlib.metadata as m
try:
    print(m.version("colcon-cargo-ros2"))
except m.PackageNotFoundError:
    pass
PY
}

version_ok() {
    local have="$1"
    [[ -n "$have" ]] || return 1
    [[ "$(printf '%s\n%s\n' "$REQUIRED_VERSION" "$have" | sort -V | head -1)" == "$REQUIRED_VERSION" ]]
}

HAVE="$(installed_version)"

if version_ok "$HAVE"; then
    echo "colcon-cargo-ros2 ${HAVE} already installed (>= ${REQUIRED_VERSION})."
else
    if [[ -n "$HAVE" ]]; then
        echo "colcon-cargo-ros2 ${HAVE} is older than ${REQUIRED_VERSION}; upgrading..."
    else
        echo "Installing colcon-cargo-ros2 (>= ${REQUIRED_VERSION})..."
    fi
    # --no-deps is not an optimisation. The extension depends on colcon-core,
    # which depends on empy; without this pip installs its own copies into
    # ~/.local, shadowing the apt python3-colcon-* and python3-empy that ROS
    # Humble pins. It picks empy 4.x, and rosidl_adapter needs 3.x:
    #
    #   AttributeError: module 'em' has no attribute 'BUFFERED_OPT'
    #
    # which surfaces as a message-generation failure in whichever package
    # generates interfaces first, naming neither pip nor empy. Those
    # dependencies are already installed from apt on any ROS machine.
    pip3 install --user -U --no-deps "colcon-cargo-ros2>=${REQUIRED_VERSION}"

    HAVE="$(installed_version)"
    if ! version_ok "$HAVE"; then
        echo "ERROR: colcon-cargo-ros2 ${HAVE:-<none>} installed, need >= ${REQUIRED_VERSION}" >&2
        exit 1
    fi
    echo "colcon-cargo-ros2 ${HAVE} installed."
fi

if ! command -v cargo >/dev/null 2>&1; then
    echo
    echo "WARNING: cargo not found. The Rust packages cannot build without it."
    echo "  Install the toolchain, then re-run the build:"
    echo "    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
    echo
fi
