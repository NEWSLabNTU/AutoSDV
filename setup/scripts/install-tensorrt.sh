#!/usr/bin/env bash
# TensorRT runtime libraries, for the Autoware Debian packages.
#
# Why this exists at all: setup.sh does not install CUDA or TensorRT, because
# on the vehicle they come from JetPack and on a workstation from the host
# image. That holds on every machine this project was developed on, and on none
# that a student brings. On a clean Ubuntu install `autoware-debian` fails with
#
#   ros-humble-autoware-tensorrt-common-1-5-0 : Depends: libnvinfer10
#                                                        but it is not installable
#
# which names neither the cause nor the fix.
#
# This installs THREE runtime libraries and nothing else. Specifically it does
# NOT install:
#
#   * a CUDA toolkit -- `cuda-toolkit-X-Y-config-common` runs
#     update-alternatives on /usr/local/cuda, which would repoint the default
#     toolkit for EVERY user of a shared workstation. Which toolkit is selected
#     is the user's choice; see .envrc and scripts/check-cuda-arch.sh.
#   * a driver -- `cuda` and `cuda-X-Y` pull cuda-drivers, and replacing a
#     running driver is the one genuinely destructive act available here.
#   * an apt preferences pin -- NVIDIA's repository ships its own at priority
#     600, and a competing pin of ours would just fight it.
#
# The three packages have no dependencies of their own, so installing them
# cannot pull a toolkit in by the back door.

set -euo pipefail

SONAMES=(libnvinfer.so.10 libnvinfer_plugin.so.10 libnvonnxparser.so.10)
PACKAGES=(libnvinfer10 libnvinfer-plugin10 libnvonnxparsers10)

say() { printf '  %s\n' "$*"; }

# ---------------------------------------------------------------- arm64 -----
# JetPack carries TensorRT 10.3 already, and the Jetson apt repo serves these as
# ordinary arm64 packages if it does not.
if [ "$(uname -m)" = "aarch64" ]; then
    say "arm64: TensorRT comes from JetPack. Nothing to do."
    exit 0
fi

# --------------------------------------------------- tier 1: already here ---
# Deliberately a soname check and not a package check: it is true however the
# libraries got here -- NVIDIA's network repo, a downloaded local-repo deb, the
# tar, or a runfile. A machine that already has TensorRT is not touched, and no
# apt configuration is read or written.
# ldconfig's output is captured ONCE rather than piped into `grep -q` per
# soname. Under `set -o pipefail` that pipeline reports failure even on a
# match: grep -q exits at the first hit, ldconfig takes SIGPIPE, and pipefail
# surfaces its 141. The step would then decide TensorRT was missing on every
# machine, including ones that already had it, and try to install over the top.
ldcache="$(ldconfig -p 2>/dev/null || true)"

missing=()
for so in "${SONAMES[@]}"; do
    grep -qF "$so" <<<"$ldcache" || missing+=("$so")
done

if [ ${#missing[@]} -eq 0 ]; then
    say "TensorRT 10 already present:"
    for so in "${SONAMES[@]}"; do
        say "    $(grep -m1 -F "$so" <<<"$ldcache" | sed 's/^[[:space:]]*//')"
    done
    exit 0
fi

say "missing: ${missing[*]}"

# ------------------------------------------- a tar or runfile install? ------
# ldconfig only knows what is in its cache. TensorRT unpacked from the tar and
# put on LD_LIBRARY_PATH is invisible above, and installing apt packages
# alongside it would leave two copies whose precedence depends on link order.
# Stop and let a person decide rather than quietly doubling up.
if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    IFS=':' read -ra _dirs <<< "$LD_LIBRARY_PATH"
    for d in "${_dirs[@]}"; do
        [ -n "$d" ] || continue
        if ls "$d"/libnvinfer.so.10* >/dev/null 2>&1; then
            cat >&2 <<EOF

  TensorRT appears to be installed outside apt:

      $d/libnvinfer.so.10

  It is on LD_LIBRARY_PATH but not in the ldconfig cache, which is what a tar
  or runfile install looks like. Installing the apt packages now would leave
  two copies, and which one loads would depend on link order.

  Either add it to the ldconfig cache, so this step can see it:

      echo '$d' | sudo tee /etc/ld.so.conf.d/tensorrt.conf
      sudo ldconfig

  or skip this step:

      ./setup.sh --run --profile dev --skip tensorrt

EOF
            exit 1
        fi
    done
fi

# ------------------------------------ tier 2: a configured source has it ----
# The user may already have NVIDIA's network repo, or a local-repo deb they
# downloaded. Install from whatever they configured; do not add a competing
# source.
# Captured, not piped into `grep -q` -- the same pipefail/SIGPIPE trap as the
# ldconfig check above, which would send every machine to tier 3 and add a
# source over the top of one the user already configured.
policy="$(apt-cache policy "${PACKAGES[0]}" 2>/dev/null || true)"
candidate="$(awk '/Candidate:/ {print $2}' <<<"$policy")"

if [ -n "$candidate" ] && [ "$candidate" != "(none)" ]; then
    say "a configured apt source already offers TensorRT ($candidate); using it"
    say "adding no apt source of our own"
    sudo apt-get install -y --no-install-recommends "${PACKAGES[@]}"
    say "done"
    exit 0
fi

# ------------------------------------------- tier 3: add NVIDIA's source ----
# Only reached when the machine has no TensorRT and nothing that offers it, so
# there is nothing to conflict with. This is NVIDIA's own documented network
# repository method.
# shellcheck source=/dev/null
. /etc/os-release

# The repository path is ubuntuXXYY, so this only makes sense on Ubuntu.
# setup.sh's own OS check normally stops before here, but --ignore-os-check
# exists and a wrong path would 404 with nothing explaining why.
if [ "${ID:-}" != "ubuntu" ]; then
    echo "error: NVIDIA publishes this repository for Ubuntu; found ID=${ID:-unknown}." >&2
    echo "       Install TensorRT 10 by hand, then re-run --status to confirm." >&2
    exit 1
fi
distro="ubuntu$(echo "${VERSION_ID:?VERSION_ID missing from /etc/os-release}" | tr -d '.')"

cat <<EOF

  No TensorRT, and no configured apt source provides it.

  About to add NVIDIA's CUDA network repository and install three runtime
  libraries:

      ${PACKAGES[*]}

  Files this adds, all removable:

      /etc/apt/sources.list.d/cuda-${distro}-x86_64.list
      /usr/share/keyrings/cuda-archive-keyring.gpg

  No CUDA toolkit, no driver, and /usr/local/cuda is not touched.

EOF

# Older recipes leave a cuda.list that conflicts with the keyring-managed one.
sudo rm -f /etc/apt/sources.list.d/cuda.list

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
keyring="cuda-keyring_1.1-1_all.deb"
url="https://developer.download.nvidia.com/compute/cuda/repos/${distro}/x86_64/${keyring}"

say "fetching ${url}"
if command -v wget >/dev/null 2>&1; then
    wget -q -O "$tmp/$keyring" "$url"
elif command -v curl >/dev/null 2>&1; then
    curl -fsSL -o "$tmp/$keyring" "$url"
else
    echo "error: neither wget nor curl is available to fetch the keyring." >&2
    echo "       sudo apt-get install -y wget   then re-run this step." >&2
    exit 1
fi

sudo dpkg -i "$tmp/$keyring"
sudo apt-get update
sudo apt-get install -y --no-install-recommends "${PACKAGES[@]}"

# Prove it, rather than assume the install did what it said.
sudo ldconfig
ldcache="$(ldconfig -p 2>/dev/null || true)"
for so in "${SONAMES[@]}"; do
    grep -qF "$so" <<<"$ldcache" || {
        echo "error: ${so} still not present after install" >&2
        exit 1
    }
done
say "TensorRT 10 installed"
