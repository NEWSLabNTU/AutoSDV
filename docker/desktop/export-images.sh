#!/usr/bin/env bash
# Export the desktop images as files, for handing out on a local network or a
# USB stick instead of having fifty laptops pull from Docker Hub at once.
#
#   ./docker/desktop/export-images.sh              both architectures
#   ./docker/desktop/export-images.sh amd64        just one
#   OUT_DIR=/srv/autosdv ./docker/desktop/export-images.sh
#   ROSBAG=0 ./docker/desktop/export-images.sh     images only
#   ROSBAG_ZIP=/path/to/it.zip ./docker/desktop/export-images.sh
#
# Produces, per architecture:
#
#   autosdv-desktop-<arch>.tar.gz        the image
#   autosdv-desktop-<arch>.tar.gz.sha256 what proves the download finished
#
# plus a second, hard-linked name for each that says which laptop it is for --
# "Apple Silicon Mac" rather than "arm64" -- and a READ-ME-FIRST.txt.
#
# AND the recording the workshop replays:
#
#   outdoor_20251226_153115.zip          1.7 GB, 2.8 GB unpacked
#   outdoor_20251226_153115.zip.sha256
#
# The recording is here because the image alone does not finish the lab. Half
# the workshop is `ros2 bag play`, and a student who has loaded the image and
# cannot replay anything is stuck in exactly the same way as one who never got
# the image -- except that it looks like their own mistake. Fifty laptops
# fetching 1.7 GB from a Synology share off campus is 85 GB through one uplink;
# off the TA laptop it is 85 GB through a switch.
#
# Kept as the .zip under its own name, NOT unpacked and not renamed: the slides
# name this exact file, and the instructions say to unzip it into
# AutoSDV/data/rosbags/ -- which is the same destination on both installation
# paths, because autosdv.sh mounts ${REPO}/data into the container.
#
# gzip rather than zstd or xz, though both compress better: `docker load`
# decompresses gzip itself, so a student needs no decompression tool at all.
# That matters most on Windows, which ships none. pigz is used when present --
# same format, all cores instead of one.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
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

# --- the recording -----------------------------------------------------------
# Four sources, tried in order; only the last one touches the network. A TA
# laptop that has ever run `just bag download` already holds the bytes, and a
# 1.7 GB transfer skipped is a 1.7 GB transfer that cannot fail halfway.
ROSBAG_ZIP_NAME="outdoor_20251226_153115.zip"
ROSBAG_DIR_NAME="outdoor_20251226_153115"
ROSBAG_OUT="${OUT_DIR}/${ROSBAG_ZIP_NAME}"

# Hard-link where the filesystem allows it, copy where it does not -- the NAS
# and the local disk are different mounts, so this is a real 1.7 GB copy there
# and free from data/rosbags/.
place_rosbag() {
    local src="$1"
    if [ "$src" -ef "$ROSBAG_OUT" ]; then
        return 0
    fi
    ln -f "$src" "$ROSBAG_OUT" 2>/dev/null || cp "$src" "$ROSBAG_OUT"
}

if [ "${ROSBAG:-1}" = "0" ]; then
    echo "  === recording: skipped (ROSBAG=0) ==="
    echo
else
    echo "  === recording ==="

    rosbag_src=""
    for cand in "${ROSBAG_ZIP:-}" "$ROSBAG_OUT" "${REPO}/data/rosbags/${ROSBAG_ZIP_NAME}"; do
        if [ -n "$cand" ] && [ -f "$cand" ]; then
            rosbag_src="$cand"
            break
        fi
    done

    # Unpacked on this machine but never kept as a zip: rebuild one. `just bag
    # download` deletes the archive after extracting it, so this is the normal
    # state of a workstation that has run the simulation.
    if [ -z "$rosbag_src" ] && [ -d "${REPO}/data/rosbags/${ROSBAG_DIR_NAME}" ]; then
        if command -v zip >/dev/null 2>&1; then
            echo "  no archive, but ${ROSBAG_DIR_NAME}/ is unpacked -- re-zipping"
            ( cd "${REPO}/data/rosbags" && zip -q -r "$ROSBAG_OUT" "$ROSBAG_DIR_NAME" )
            rosbag_src="$ROSBAG_OUT"
        else
            echo "  ${ROSBAG_DIR_NAME}/ is unpacked but 'zip' is not installed"
            echo "  ('sudo apt-get install zip' would avoid the download below)"
        fi
    fi

    # Last resort: fetch it. The share URL is read out of the download script
    # rather than repeated here, so there is one place it is written down and a
    # moved share breaks one file instead of two silently disagreeing.
    if [ -z "$rosbag_src" ]; then
        dl="${REPO}/scripts/rosbag/download-test-rosbag.sh"
        share_url="$(sed -n 's/^SHARE_URL="\(.*\)"$/\1/p' "$dl" 2>/dev/null || true)"
        if ! command -v synology-dl >/dev/null 2>&1; then
            echo "  error: no local copy of ${ROSBAG_ZIP_NAME}, and synology-dl is not" >&2
            echo "         installed to fetch one (cargo install synology-dl)." >&2
            echo "         Point ROSBAG_ZIP= at the archive, or ROSBAG=0 to skip it." >&2
            exit 1
        fi
        if [ -z "$share_url" ]; then
            echo "  error: could not read SHARE_URL from ${dl}" >&2
            exit 1
        fi
        echo "  downloading (1.7 GB) ..."
        synology-dl -f "$share_url" "$OUT_DIR"
        rosbag_src="$ROSBAG_OUT"
        [ -f "$rosbag_src" ] || { echo "  error: download produced no ${ROSBAG_ZIP_NAME}" >&2; exit 1; }
    fi

    place_rosbag "$rosbag_src"

    # A zip's index lives at the END of the file, so listing it is cheap and
    # proves the archive is whole -- which `unzip -t` would also prove, at the
    # cost of decompressing 2.8 GB. Truncation is the failure worth catching
    # here: handing out a short file makes fifty students think they downloaded
    # it wrong.
    if command -v unzip >/dev/null 2>&1; then
        if ! unzip -l "$ROSBAG_OUT" >/dev/null 2>&1; then
            echo "  error: ${ROSBAG_OUT} is not a readable zip (truncated?)" >&2
            exit 1
        fi
    fi

    # This checksum proves the student's download finished. It is not the one in
    # download-test-rosbag.sh, which covers the .db3 inside the archive.
    ( cd "$OUT_DIR" && sha256sum "$ROSBAG_ZIP_NAME" > "${ROSBAG_ZIP_NAME}.sha256" )

    echo "  $(du -h "$ROSBAG_OUT" | cut -f1)  ${ROSBAG_OUT}"
    echo
fi

# A note beside the files, so a student who lands on the directory listing with
# no other context can still finish.
cat > "${OUT_DIR}/READ-ME-FIRST.txt" <<'NOTE'
AutoSDV workshop files
======================

Two things are here.  Which of them you need depends on your laptop.

  A. The container image -- Windows, macOS, or any Linux you would rather
     not install onto.  Skip this section if you are installing AutoSDV
     natively on Ubuntu 22.04.

  B. The recording -- EVERYONE needs this one, on both paths.


A. The container image
----------------------

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

If step 2 fails with a tar or gzip error, the download was incomplete.
Download it again rather than retrying the load.


B. The recording
----------------

The second half of the workshop replays a drive that was recorded on the
vehicle.  Without it, `ros2 bag play` has nothing to play.

1. Download:

     outdoor_20251226_153115.zip        (1.7 GB, 2.8 GB unpacked)

2. Unzip it into the repository you cloned, under data/rosbags/, so that
   you end up with:

     AutoSDV/
     └── data/
         └── rosbags/
             └── outdoor_20251226_153115/
                 ├── metadata.yaml
                 └── outdoor_20251226_153115_0.db3

   Container users: this is the same place.  The start script mounts your
   AutoSDV/data directory into the container, so a recording unzipped on
   your laptop is visible inside it -- unzip it on the laptop, not in the
   container.

3. Check the path, because a wrong one fails only later, at replay time:

     ls AutoSDV/data/rosbags/outdoor_20251226_153115/metadata.yaml
NOTE
echo "  wrote ${OUT_DIR}/READ-ME-FIRST.txt"
echo

cat <<EOF
  Done. To serve them on the classroom network, from ${OUT_DIR}:

      python3 -m http.server 8000

  Students then fetch the image for their machine, and the recording
  regardless of machine. Find this laptop's address with \`ip addr\` (Linux)
  or \`ipconfig getifaddr en0\` (macOS) and give them that, not "localhost".

  serve-images.sh does both of those -- it serves ${OUT_DIR} and prints the
  address for every interface a student could reach it on.

EOF
