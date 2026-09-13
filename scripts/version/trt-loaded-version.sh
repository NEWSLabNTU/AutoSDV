#!/usr/bin/env bash
# Print the TensorRT version an Autoware process on this machine actually
# resolves, e.g. "10.8.0". Empty (exit 1) if it cannot be determined.
#
# Asking `dpkg` instead is wrong, and quietly so: a workstation can carry
# several TensorRT installs at once -- an apt `libnvinfer10`, a tarball under
# /usr/local/cuda-*/targets, an older libnvinfer8 -- and the one that stamps an
# .engine is whichever the *loader* picks, not whichever dpkg lists. Measured on
# one box: dpkg said 10.15.1.29 while Autoware ran 10.9.0. amd64 has no
# `tensorrt` metapackage to query at all.
#
# Resolution honours LD_LIBRARY_PATH, so source scripts/trt-runtime-env.sh
# first to ask the question the way the build will answer it.
set -euo pipefail

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
PREFIX="$("${SCRIPT_DIR}/get-version.sh" autoware.install_prefix 2>/dev/null || true)"
: "${PREFIX:=/opt/autoware/1.5.0}"

so=""
# An Autoware library first: its own RPATH is part of how the loader answers.
probe="${PREFIX}/lib/libautoware_tensorrt_common.so"
if [[ -f "$probe" ]]; then
    so=$(ldd "$probe" 2>/dev/null | sed -n 's|.*libnvinfer\.so\.[0-9]* => \([^ ]*\).*|\1|p' | sed -n 1p)
fi
if [[ -z "$so" ]]; then
    so=$(ldconfig -p 2>/dev/null \
        | sed -n 's|.*libnvinfer\.so\.[0-9]* (libc6,x86-64) => \(.*\)|\1|p' | sed -n 1p)
fi

if [[ -z "$so" || ! -e "$so" ]]; then
    echo "error: no libnvinfer could be resolved on this machine" >&2
    exit 1
fi

version="$(basename "$(readlink -f "$so")" | sed -n 's/^libnvinfer\.so\.//p')"
if [[ -z "$version" ]]; then
    echo "error: could not read a version from $(readlink -f "$so")" >&2
    exit 1
fi
echo "$version"
