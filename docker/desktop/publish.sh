#!/usr/bin/env bash
# Publish the desktop image to Docker Hub as ONE tag that works everywhere.
#
#   on the amd64 workstation:   ./docker/desktop/publish.sh push
#   on the arm64 server:        ./docker/desktop/publish.sh push
#   on either, once both exist: ./docker/desktop/publish.sh link
#
# A student then types the same thing on every platform:
#
#   docker pull jerry73204/autosdv:desktop
#
# --- why two machines and not one command ------------------------------------
#
# `docker buildx build --platform linux/amd64,linux/arm64` from a single host
# builds the foreign architecture under qemu, which compiles the entire ROS 2
# workspace through emulation and takes hours. Each architecture is therefore
# built natively on a machine of that architecture and pushed under its own tag;
# `link` then joins those tags into a manifest list registry-side, without
# either machine needing to reach the other or even be up at the same time.
#
# The arm64 image is JETSON-FLAVOURED but not Jetson-BUILT: everything
# architecture-specific about it (the jetpack62 Autoware localrepo, the Jetson
# apt repository for CUDA and TensorRT) is installed INSIDE a plain ubuntu:22.04
# container, so any arm64 Linux machine produces an identical image. An Orin
# works; so does an arm64 server, usually faster.
#
# --- what the single tag resolves to -----------------------------------------
#
#   Windows (Docker Desktop / WSL2)   linux/amd64   -> the amd64 image
#   Linux x86                         linux/amd64   -> the amd64 image
#   macOS on Intel                    linux/amd64   -> the amd64 image
#   macOS on Apple Silicon            linux/arm64   -> the arm64 image
#
# Apple Intel needs nothing special: it is an amd64 machine.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

HUB_REPO="${HUB_REPO:-$(./scripts/version/get-version.sh container.desktop_repo)}"
TAG_BASE="${TAG_BASE:-$(./scripts/version/get-version.sh container.desktop_tag)}"

if [ -z "$HUB_REPO" ] || [ -z "$TAG_BASE" ]; then
    echo "error: container.desktop_repo / container.desktop_tag missing from versions.yaml." >&2
    echo "       get-version.sh indexes the YAML directly, so a missing key yields" >&2
    echo "       nothing rather than failing loudly." >&2
    exit 1
fi

ARCHES=(amd64 arm64)

usage() {
    cat <<EOF
usage: $0 <command> [local-image]

  push [IMAGE]   tag the local image for its own architecture and push it.
                 IMAGE defaults to autosdv:desktop-dev.
                 Publishes ${HUB_REPO}:${TAG_BASE}-<arch>.

  link           join whichever per-architecture tags exist in the registry
                 into the single tag ${HUB_REPO}:${TAG_BASE}.
                 Safe to run with only one architecture published, and safe
                 to re-run later once the other appears.

  check          show what is published right now.

environment: HUB_REPO, TAG_BASE, DRY_RUN=1
EOF
}

run() {
    if [ "${DRY_RUN:-0}" = "1" ]; then
        echo "  DRY_RUN: $*"
    else
        "$@"
    fi
}

cmd_push() {
    local local_image="${1:-autosdv:desktop-dev}"

    if ! docker image inspect "$local_image" >/dev/null 2>&1; then
        echo "error: no local image '$local_image'." >&2
        echo "       Build it first:  ./docker/desktop/build.sh" >&2
        exit 1
    fi

    local arch os layers
    arch="$(docker image inspect -f '{{.Architecture}}' "$local_image")"
    os="$(docker image inspect -f '{{.Os}}' "$local_image")"
    layers="$(docker image inspect -f '{{len .RootFS.Layers}}' "$local_image")"

    if [ "$os" != "linux" ]; then
        echo "error: '$local_image' is $os/$arch; only linux images are published." >&2
        exit 1
    fi

    case " ${ARCHES[*]} " in
        *" $arch "*) ;;
        *) echo "error: '$local_image' is $arch, which is not one of: ${ARCHES[*]}" >&2
           exit 1 ;;
    esac

    # A flattened image is a single layer, and a single layer is the wrong shape
    # for a tag students pull: no parallel download, and a pull that dies at 90%
    # on shared wifi restarts from zero rather than resuming a layer. FLATTEN=1
    # exists for a `docker save` tarball handed out on USB sticks.
    if [ "$layers" -le 1 ]; then
        cat >&2 <<EOF

  error: '$local_image' has $layers layer(s) -- it looks flattened.

  Do not publish a flattened image to a registry. One layer means the pull
  cannot download in parallel and cannot resume: a student whose pull dies at
  90% starts again from nothing, and with 50 of them on one access point that
  is the difference between a slow morning and a lost one.

  Rebuild without it:

      ./docker/desktop/build.sh

  FLATTEN=1 is for a \`docker save\` tarball, not for Docker Hub.

EOF
        exit 1
    fi

    local remote="${HUB_REPO}:${TAG_BASE}-${arch}"
    echo "  local:  $local_image  ($os/$arch, $layers layers)"
    echo "  remote: $remote"
    echo

    run docker tag "$local_image" "$remote"
    run docker push "$remote"

    echo
    echo "  pushed. When the other architecture is published too, run:"
    echo "      ./docker/desktop/publish.sh link"
}

# Which per-architecture tags actually exist in the registry right now.
published_sources() {
    local arch tag
    for arch in "${ARCHES[@]}"; do
        tag="${HUB_REPO}:${TAG_BASE}-${arch}"
        if docker buildx imagetools inspect "$tag" >/dev/null 2>&1; then
            printf '%s\n' "$tag"
        fi
    done
}

cmd_link() {
    local arch
    local -a sources=()
    mapfile -t sources < <(published_sources)

    if [ ${#sources[@]} -eq 0 ]; then
        echo "error: none of these are published yet:" >&2
        # A loop, not printf with "${ARCHES[@]}": printf recycles its format
        # over the remaining arguments, so the second pass would get only the
        # architecture and print "arm64:-".
        for arch in "${ARCHES[@]}"; do
            echo "         ${HUB_REPO}:${TAG_BASE}-${arch}" >&2
        done
        echo "       Run '$0 push' on a build machine first." >&2
        exit 1
    fi

    echo "  joining into ${HUB_REPO}:${TAG_BASE}:"
    printf '    %s\n' "${sources[@]}"

    if [ ${#sources[@]} -lt ${#ARCHES[@]} ]; then
        cat <<EOF

  note: only ${#sources[@]} of ${#ARCHES[@]} architectures are published, so the
  tag will resolve for those hosts only. Everyone else gets

      no matching manifest for linux/<arch> in the manifest list entries

  This is a fine state to ship in -- publish amd64 first so most students can
  start, then re-run 'link' when the arm64 build lands. The tag updates in
  place and nobody has to change what they type.

EOF
    fi

    # imagetools rather than `docker manifest`: it is GA where `docker manifest`
    # is still experimental, it works purely registry-side by digest without
    # pulling either image, and it writes an OCI index correctly.
    run docker buildx imagetools create -t "${HUB_REPO}:${TAG_BASE}" "${sources[@]}"

    echo
    cmd_check
}

cmd_check() {
    echo "  ${HUB_REPO}:${TAG_BASE}"
    if docker buildx imagetools inspect "${HUB_REPO}:${TAG_BASE}" 2>/dev/null \
        | grep -E "^(Name|MediaType|Platform)" | sed 's/^/    /'; then
        :
    else
        echo "    (not published)"
    fi
    echo
    local arch tag
    for arch in "${ARCHES[@]}"; do
        tag="${HUB_REPO}:${TAG_BASE}-${arch}"
        if docker buildx imagetools inspect "$tag" >/dev/null 2>&1; then
            echo "  $tag  published"
        else
            echo "  $tag  MISSING"
        fi
    done
}

case "${1:-}" in
    push)  shift; cmd_push "$@" ;;
    link)  cmd_link ;;
    check) cmd_check ;;
    -h|--help|help|"") usage ;;
    *) echo "error: unknown command '$1'" >&2; echo; usage; exit 1 ;;
esac
