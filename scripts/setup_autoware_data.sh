#!/usr/bin/env bash
# Build a WRITABLE view of Autoware's model data at data/autoware_data.
#
# Why this exists
# ---------------
# Autoware's launch files default `data_path` to $HOME/autoware_data, which is
# what its online documentation tells you to create. This deployment installs
# Autoware from Debian packages instead, so the same data lives under
# /opt/autoware/<version>/data — owned by root and mode 0755.
#
# That breaks TensorRT. Every engine path is a parameter derived from the model
# directory, e.g.
#
#     encoder_engine_path: "$(var model_path)/pts_voxel_encoder_$(var model_name).engine"
#     model_path         = $(var data_path)/$(var node_name)
#
# so a node compiles its .onnx and then tries to write the .engine NEXT TO IT.
# In a root-owned directory that write fails:
#
#     [E] [TRT] Fail to open engine file
#     component_node failed: ... Failed to setup TensorRT engine
#
# and because the engine is written at the END of the build, the work is
# discarded and repeated on every launch. Measured on the AGX Orin: 8 of the
# perception stack's composable nodes failed this way, and zero engines were
# ever cached.
#
# What this does
# --------------
# Mirrors the directory tree and SYMLINKS every file, rather than copying.
# The models are 2.0 GB; the farm is under a megabyte. Reads resolve to the
# packaged files, and newly written .engine files land in this writable tree.
#
# Idempotent: safe to re-run after an Autoware upgrade, which is also when you
# must, because engines are tied to the TensorRT version and the GPU.
set -euo pipefail

AUTOSDV_REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
FARM="${AUTOSDV_DATA_PATH:-${AUTOSDV_REPO_ROOT}/data/autoware_data}"

# Where the packaged models live. Override for a non-default Autoware install.
SRC="${AUTOWARE_DATA_SRC:-}"
if [ -z "${SRC}" ]; then
    # Prefer whatever /opt/autoware exposes; there is normally one version.
    for candidate in /opt/autoware/*/data; do
        [ -d "${candidate}" ] && SRC="${candidate}"
    done
fi

if [ -z "${SRC}" ] || [ ! -d "${SRC}" ]; then
    echo "ERROR: no Autoware data directory found." >&2
    echo "  Looked for /opt/autoware/*/data; set AUTOWARE_DATA_SRC to override." >&2
    exit 1
fi

echo "source: ${SRC}"
echo "farm:   ${FARM}"

mkdir -p "${FARM}"

# Directories first, then a symlink per file. `-P` keeps paths relative to SRC
# so the tree is mirrored rather than nested.
(cd "${SRC}" && find . -mindepth 1 -type d -printf '%P\n') | while IFS= read -r d; do
    mkdir -p "${FARM}/${d}"
done

linked=0
skipped=0
while IFS= read -r f; do
    target="${FARM}/${f}"
    # Never clobber a real file — that is an engine this farm produced, and
    # rebuilding it costs minutes.
    if [ -e "${target}" ] && [ ! -L "${target}" ]; then
        skipped=$((skipped + 1))
        continue
    fi
    ln -sfn "${SRC}/${f}" "${target}"
    linked=$((linked + 1))
done < <(cd "${SRC}" && find . -type f -printf '%P\n')

engines=$(find "${FARM}" -name '*.engine' -type f 2>/dev/null | wc -l)

echo "linked ${linked} file(s), kept ${skipped} local file(s)"
echo "engines already built: ${engines}"
if [ "${engines}" -eq 0 ]; then
    echo
    echo "No TensorRT engines yet. Build them before the first drive:"
    echo "    just build-engines"
    echo "Otherwise the first launch compiles them inline — minutes per model,"
    echo "with perception unavailable until it finishes."
fi
