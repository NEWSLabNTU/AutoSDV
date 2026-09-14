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
#   autosdv-desktop-<arch>.tar.gz        the image
#   autosdv-desktop-<arch>.tar.gz.sha256 what proves the download finished
#
# plus a second, hard-linked name for each that says which laptop it is for --
# "Apple Silicon Mac" rather than "arm64" -- and a READ-ME-FIRST.txt.
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

    # A second name for the same bytes, saying what a student actually knows
    # about their laptop. "arm64" is not something to ask a room of fifty to
    # determine about themselves, and the cost of getting it wrong is silent:
    # the amd64 image runs on Apple Silicon, emulated, at a fraction of the
    # speed, with nothing to indicate why.
    #
    # A hard link rather than a copy or a symlink: no second 14 GB, and no
    # dependence on the web server following links.
    case "$arch" in
        amd64) friendly="AutoSDV-for-Windows-Linux-and-Intel-Mac.tar.gz" ;;
        arm64) friendly="AutoSDV-for-Apple-Silicon-Mac.tar.gz" ;;
        *)     friendly="" ;;
    esac
    if [ -n "$friendly" ]; then
        ln -f "$out" "${OUT_DIR}/${friendly}" 2>/dev/null \
            || cp "$out" "${OUT_DIR}/${friendly}"
        ( cd "$OUT_DIR" && sha256sum "$friendly" > "${friendly}.sha256" )
        echo "  also as: ${friendly}"
    fi

    echo "  $(du -h "$out" | cut -f1)  ${out}"
    echo
done

# A note beside the files, so a student who lands on the directory listing with
# no other context can still finish.
cat > "${OUT_DIR}/READ-ME-FIRST.txt" <<'NOTE'
AutoSDV desktop image
=====================

1. Download ONE file -- whichever describes your laptop:

     AutoSDV-for-Windows-Linux-and-Intel-Mac.tar.gz
     AutoSDV-for-Apple-Silicon-Mac.tar.gz

   Not sure which Mac you have?  Apple menu > About This Mac.
   "Apple M1/M2/M3/M4" is Apple Silicon.  "Intel" is the other file.

2. Load it into Docker.  Open a terminal (macOS/Linux) or PowerShell
   (Windows), change to wherever the file downloaded, and run:

     docker load -i AutoSDV-for-Windows-Linux-and-Intel-Mac.tar.gz

   ...using the name of the file you actually downloaded.  This takes a few
   minutes and prints nothing until it finishes.

3. Check it arrived:

     docker images jerry73204/autosdv

   You should see a line with the tag "desktop".

Then follow the lab instructions to start it.

If step 2 fails with a tar or gzip error, the download was incomplete.
Download it again rather than retrying the load.
NOTE
echo "  wrote ${OUT_DIR}/READ-ME-FIRST.txt"
echo

cat <<EOF
  Done. To serve them on the classroom network, from ${OUT_DIR}:

      python3 -m http.server 8000

  Students then fetch the file for their machine. Find this laptop's address
  with \`ip addr\` (Linux) or \`ipconfig getifaddr en0\` (macOS) and give them
  that, not "localhost".

EOF
