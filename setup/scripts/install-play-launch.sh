#!/usr/bin/env bash
# Install/upgrade play_launch (launch orchestrator)
#
# Every `just launch` runs through play_launch, and the floor is 0.10.0. Two
# releases put it there:
#
#   0.9.0   ships the startup governor (MemAvailable admission floor +
#           oom_score_adj on children), which is what stops a full Autoware
#           launch from freezing the box or feeding the desktop to the OOM
#           killer on an edge machine.
#   0.10.0  fixes array parameters being rendered as strings (f78745da). Under
#           0.8.2 that killed autoware_pose_initializer_node at startup and
#           took /localization/initialize with it, so the stack came up without
#           a way to be initialised.
#
# A NOTE ON THE RUST PARSER, which is the default. play_launch issue #0028 --
# global parameters never reaching a .launch.py across the loader boundary,
# surfacing as `KeyError: 'rear_overhang'` -- is fixed in 8adc52ad (2026-09-11,
# ABI 4), which is AFTER the 0.10.0 release. A plain `pip install play_launch`
# therefore satisfies this floor while still carrying that bug. Until it
# reaches a release there is no version to check for, so if a launch dies that
# way, pass `--parser python` or install from git past that commit.
#
# This is deliberately NOT a marker-gated step, and not part of python-deps:
# a machine set up before the floor carries a python-deps marker and would
# never upgrade past its old play_launch. This script checks the installed
# version on every run and is a no-op when it already satisfies the floor, so
# re-running setup keeps machines current.

set -e

REQUIRED_VERSION="0.10.0"

installed_version() {
    python3 - <<'PY' 2>/dev/null
import importlib.metadata as m
try:
    print(m.version("play_launch"))
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
    echo "play_launch ${HAVE} already installed (>= ${REQUIRED_VERSION})."
else
    if [[ -n "$HAVE" ]]; then
        echo "play_launch ${HAVE} is older than ${REQUIRED_VERSION}; upgrading..."
    else
        echo "Installing play_launch (>= ${REQUIRED_VERSION})..."
    fi
    pip3 install --user -U "play_launch>=${REQUIRED_VERSION}"

    HAVE="$(installed_version)"
    if ! version_ok "$HAVE"; then
        echo "ERROR: play_launch ${HAVE:-<none>} installed, need >= ${REQUIRED_VERSION}" >&2
        exit 1
    fi
    echo "play_launch ${HAVE} installed."
fi

if ! command -v play_launch >/dev/null 2>&1; then
    echo
    echo "WARNING: 'play_launch' is not on PATH. It installs to ~/.local/bin;"
    echo "  make sure that directory is in PATH (setup adds it via ~/.profile)."
fi
