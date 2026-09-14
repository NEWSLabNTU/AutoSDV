#!/usr/bin/env bash
# Export the desktop images as files, for handing out on a local network or a
# USB stick instead of having fifty laptops pull from Docker Hub at once.
#
#   ./docker/desktop/export-images.sh              both architectures
#   ./docker/desktop/export-images.sh amd64        just one
#   OUT_DIR=/srv/autosdv ./docker/desktop/export-images.sh
#
# Produces, per architecture:
#
#   autosdv-desktop-<arch>.tar.gz        what the student loads
#   autosdv-desktop-<arch>.tar.gz.sha256 what proves the download finished
#
# gzip rather than zstd or xz, though both compress better: `docker load`
# decompresses gzip itself, so a student needs no decompression tool at all.
# That matters most on Windows, which ships none. pigz is used when present --
# same format, all cores instead of one.

set -euo pipefail

REPO_IMAGE="${AUTOSDV_IMAGE:-jerry73204/autosdv:desktop}"
OUT_DIR="${OUT_DIR:-$PWD}"
ARCHES=("${@:-amd64 arm64}")
read -ra ARCHES <<< "${ARCHES[*]}"

command -v docker >/dev/null || { echo "error: docker not found" >&2; exit 1; }
mkdir -p "$OUT_DIR"

if command -v pigz >/dev/null 2>&1; then
    GZIP=(pigz -9)
    echo "  compressing with pigz (all cores)"
else
    GZIP=(gzip -9)
    echo "  compressing with gzip -- single-threaded and slow on an image this size."
    echo "  'sudo apt-get install pigz' first if you would rather not wait."
fi
echo

for arch in "${ARCHES[@]}"; do
    tag="${REPO_IMAGE%:*}:$(basename "${REPO_IMAGE##*:}")-${arch}"
    out="${OUT_DIR}/autosdv-desktop-${arch}.tar.gz"

    echo "  === ${arch} ==="
    echo "  pulling ${tag}"
    # --platform as well as the arch-specific tag: the tag is single-arch, but
    # being explicit means a mistake fails here rather than producing a file
    # that silently contains the wrong architecture and fails on a student's
    # laptop an hour later.
    docker pull --platform "linux/${arch}" "$tag"

    # Saved under the MULTI-ARCH name, not the -amd64/-arm64 one. What the
    # student loads must carry the tag every script and slide already uses;
    # otherwise `docker run jerry73204/autosdv:desktop` reports "image not
    # found" on a machine that demonstrably has the image.
    docker tag "$tag" "$REPO_IMAGE"

    echo "  saving to ${out}"
    docker save "$REPO_IMAGE" | "${GZIP[@]}" > "$out"

    ( cd "$OUT_DIR" && sha256sum "$(basename "$out")" > "$(basename "$out").sha256" )

    echo "  $(du -h "$out" | cut -f1)  ${out}"
    echo
done

cat <<EOF
  Done. To serve them on the classroom network, from ${OUT_DIR}:

      python3 -m http.server 8000

  Students then fetch the file for their machine. Find this laptop's address
  with \`ip addr\` (Linux) or \`ipconfig getifaddr en0\` (macOS) and give them
  that, not "localhost".

EOF
