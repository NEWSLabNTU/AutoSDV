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

CUDA_IMAGE="$(./scripts/version/get-version.sh container.desktop_base)"
if [ -z "$CUDA_IMAGE" ]; then
    echo "error: container.desktop_base is empty or missing in versions.yaml." >&2
    echo "       get-version.sh indexes the YAML directly, so a missing key" >&2
    echo "       yields nothing rather than failing loudly." >&2
    exit 1
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
