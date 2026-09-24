#!/usr/bin/env bash
# Build the desktop (course) image.
#
#   ./docker/desktop/build.sh                  amd64, tagged autosdv:desktop-dev
#   TARGET=base ./docker/desktop/build.sh      prereqs only, nothing read from src/
#   PLATFORM=linux/arm64 ./docker/desktop/build.sh
#   TAG=autosdv:desktop-0.2 ./docker/desktop/build.sh
#   FLATTEN=1 ./docker/desktop/build.sh         collapse to one layer (see below)
#
# Two targets live in the one Dockerfile, and NEITHER carries a built workspace.
# `base` is every setup.sh prerequisite, VNC, the `autosdv` account and the
# entrypoint. `desktop` is base plus the two setup steps that read the source
# tree (ros-deps, range-libc), after which the tree is deleted again -- so a
# checkout bind-mounted at /workspace builds with no further install step. The
# default stays `desktop` so every command already written down keeps meaning
# what it meant. The image that DID ship /opt/AutoSDV prebuilt (until
# 2026-09-24) is frozen as jerry73204/autosdv:sim -- see publish.sh -- and
# nothing here rebuilds it.
#
# The CUDA base image is read from versions.yaml rather than written here, so
# there is exactly one place that says which CUDA the project builds against.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

PLATFORM="${PLATFORM:-linux/amd64}"
TARGET="${TARGET:-desktop}"

case "$TARGET" in
    base|desktop) ;;
    *) echo "error: TARGET must be 'base' or 'desktop', not '$TARGET'" >&2
       exit 1 ;;
esac

# The tag follows the target unless told otherwise, so building base without
# setting TAG cannot quietly overwrite the desktop image.
TAG="${TAG:-autosdv:${TARGET}-dev}"

# The base differs by architecture, and not cosmetically: amd64 builds on an
# NVIDIA CUDA image, while arm64 is a JETSON-flavoured image (JetPack Autoware
# deb, Jetson CUDA/TensorRT) that runs on Apple Silicon. See versions.yaml.
case "$PLATFORM" in
    linux/amd64) BASE_KEY=container.desktop_base_amd64 ;;
    linux/arm64) BASE_KEY=container.desktop_base_arm64 ;;
    *) echo "error: unsupported PLATFORM '$PLATFORM' (linux/amd64 or linux/arm64)" >&2
       exit 1 ;;
esac

CUDA_IMAGE="$(./scripts/version/get-version.sh "$BASE_KEY")"
if [ -z "$CUDA_IMAGE" ]; then
    echo "error: $BASE_KEY is empty or missing in versions.yaml." >&2
    echo "       get-version.sh indexes the YAML directly, so a missing key" >&2
    echo "       yields nothing rather than failing loudly." >&2
    exit 1
fi

# Building arm64 under qemu on an x86 host works but takes hours: installing
# ROS 2 and Autoware, and building range_libc, all run through emulation. Build it on the Orin instead, which
# is arm64 natively. The Orin is only the BUILDER -- the image it produces
# targets Apple Silicon, and is never run on the Jetson itself.
if [ "$PLATFORM" = "linux/arm64" ] && [ "$(uname -m)" != "aarch64" ]; then
    echo "  note: building arm64 on $(uname -m) means qemu emulation, which"
    echo "        takes hours. Prefer building this on the Orin."
    echo
fi

echo "  base:     $CUDA_IMAGE"
echo "  platform: $PLATFORM"
echo "  target:   $TARGET"
echo "  tag:      $TAG"
echo

# The build context is the repository root, not this directory: the desktop
# stage copies the tree for rosdep and range_libc, then deletes it. Dockerfile.dockerignore trims ~30 GB to ~690 MB, and excludes
# setup/.markers -- without which setup.sh would import this host's state and
# silently skip installing ROS 2 and Autoware.
docker build \
    --progress=plain \
    --platform "$PLATFORM" \
    --target "$TARGET" \
    --build-arg "CUDA_IMAGE=$CUDA_IMAGE" \
    -f docker/desktop/Dockerfile \
    -t "$TAG" \
    "$@" \
    .

[ "${FLATTEN:-0}" = "1" ] || exit 0

# --- flatten -----------------------------------------------------------------
# The Dockerfile deletes ~4.7 GB of build-only weight that arrived in the BASE
# image: CUDA's static archives and Nsight Compute. A deletion cannot reclaim
# those, because the bytes belong to a layer this build does not own -- the `rm`
# only writes a whiteout on top. Exporting the container filesystem and
# re-importing it collapses every layer into one, and the deleted bytes are
# genuinely gone.
#
# THIS IS NOT FREE, which is why it is opt-in:
#
#   * one layer means no parallel download and no per-layer resume. A pull that
#     dies at 90% on classroom wifi restarts from zero. With 50 students on one
#     access point that is the difference between a slow morning and a lost one.
#   * every later rebuild re-pushes the whole image; nothing is shared with the
#     previous tag.
#
# So: flatten for a `docker save` tarball handed out on USB sticks, where size is
# everything and there is no resume to lose. Leave it off for a Docker Hub tag
# students pull over a network.
echo
echo "  flattening $TAG (one layer) ..."

cid="$(docker create --platform "$PLATFORM" "$TAG")"
trap 'docker rm -f "$cid" >/dev/null 2>&1 || true' EXIT

# Rebuild the image metadata from the image itself rather than restating it
# here, so this cannot drift from the Dockerfile.
opts=()
while IFS= read -r e; do
    if [ -n "$e" ]; then opts+=(-c "ENV $e"); fi
done < <(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$TAG")
while IFS= read -r port; do
    if [ -n "$port" ]; then opts+=(-c "EXPOSE ${port%%/*}"); fi
done < <(docker inspect -f '{{range $p, $v := .Config.ExposedPorts}}{{println $p}}{{end}}' "$TAG")
workdir="$(docker inspect -f '{{.Config.WorkingDir}}' "$TAG")"
if [ -n "$workdir" ]; then opts+=(-c "WORKDIR $workdir"); fi
opts+=(-c "ENTRYPOINT $(docker inspect -f '{{json .Config.Entrypoint}}' "$TAG")")
opts+=(-c "CMD $(docker inspect -f '{{json .Config.Cmd}}' "$TAG")")

docker export "$cid" | docker import "${opts[@]}" - "$TAG"

echo
docker images --format '  {{.Repository}}:{{.Tag}}  {{.Size}}' "${TAG%%:*}" | head -5
