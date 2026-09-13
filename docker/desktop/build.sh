#!/usr/bin/env bash
# Build the desktop (workshop) image.
#
#   ./docker/desktop/build.sh                  amd64, tagged autosdv:desktop-dev
#   PLATFORM=linux/arm64 ./docker/desktop/build.sh
#   TAG=autosdv:desktop-0.2 ./docker/desktop/build.sh
#
# The CUDA base image is read from versions.yaml rather than written here, so
# there is exactly one place that says which CUDA the project builds against.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

PLATFORM="${PLATFORM:-linux/amd64}"
TAG="${TAG:-autosdv:desktop-dev}"

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

# Building arm64 under qemu on an x86 host works but takes hours: the whole
# workspace is compiled through emulation. Build it on the Orin instead, which
# is arm64 natively. The Orin is only the BUILDER -- the image it produces
# targets Apple Silicon, and is never run on the Jetson itself.
if [ "$PLATFORM" = "linux/arm64" ] && [ "$(uname -m)" != "aarch64" ]; then
    echo "  note: building arm64 on $(uname -m) means qemu emulation, which"
    echo "        takes hours. Prefer building this on the Orin."
    echo
fi

echo "  base:     $CUDA_IMAGE"
echo "  platform: $PLATFORM"
echo "  tag:      $TAG"
echo

# The build context is the repository root, not this directory: the image copies
# the workspace. Dockerfile.dockerignore trims ~30 GB to ~690 MB, and excludes
# setup/.markers -- without which setup.sh would import this host's state and
# silently skip installing ROS 2 and Autoware.
exec docker build \
    --progress=plain \
    --platform "$PLATFORM" \
    --build-arg "CUDA_IMAGE=$CUDA_IMAGE" \
    -f docker/desktop/Dockerfile \
    -t "$TAG" \
    "$@" \
    .
