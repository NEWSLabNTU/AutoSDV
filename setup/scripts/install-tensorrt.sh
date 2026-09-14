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

REPO_DIR="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"

# Which PATCH of TensorRT 10 gets installed is not a free choice, even though
# the Autoware packages' own `Depends: libnvinfer10` says it is.
# `autoware_tensorrt_common` discards any cached .engine whose recorded
# TensorRT version differs from the one its libraries were compiled against, so
# a machine that lands on 10.16 satisfies apt and then rebuilds all five
# perception engines on every single launch -- minutes, with perception down,
# forever. Ask for that version by name when a configured source has it, and
# fall back to the newest rather than failing: any 10.x makes Autoware
# installable, which is this step's actual job, and setup step
# `tensorrt-runtime` fixes the engine side afterwards on a host that cannot be
# moved.
say() { printf '  %s\n' "$*"; }

# Reading that version needs PyYAML, and THAT is a trap worth spelling out,
# because it already cost 4.4 GB once. get-version.sh parses versions.yaml with
# PyYAML (deliberately -- see the comment in it). A clean machine does not have
# PyYAML: in the desktop image, python3-yaml did not arrive until 21:43, while
# this step ran at 18:02. get-version.sh therefore failed, `|| true` swallowed
# it, ENGINE_ABI was empty, the pin silently degraded to bare package names, and
# apt installed the newest TensorRT (10.16.1.11) instead of the 10.8.0 the
# Autoware debs were built against. The `tensorrt-runtime` step then installed a
# SECOND, 4.4 GB TensorRT to repair the mismatch this step had just created.
#
# So: install PyYAML rather than degrade past it, and if it still cannot be read,
# say loudly what the fallback costs instead of printing nothing.
#
# It is read lazily, inside pinned_packages, so a machine that already has
# TensorRT exits at tier 1 without paying for an apt-get update.
ensure_pyyaml() {
    python3 -c 'import yaml' 2>/dev/null && return 0
    # stderr, not stdout: engine_abi() is captured with $(...), so anything this
    # prints on stdout lands INSIDE the version string.
    say "installing python3-yaml (needed to read the pinned TensorRT version)" >&2
    sudo apt-get update -qq >/dev/null 2>&1 || true
    sudo apt-get install -y --no-install-recommends python3-yaml >/dev/null 2>&1 || true
    python3 -c 'import yaml' 2>/dev/null
}

engine_abi() {
    ensure_pyyaml || return 0
    "${REPO_DIR}/scripts/version/get-version.sh" nvidia_amd64.tensorrt_engine_abi 2>/dev/null || true
}

warn_unpinned() {
    cat >&2 <<EOF

  WARNING: installing the NEWEST TensorRT 10.x rather than the pinned patch.

      reason: $1

  Autoware's autoware_tensorrt_common discards any cached .engine whose recorded
  TensorRT version differs from the one its own libraries were built against. On
  a mismatch every perception engine is rebuilt on EVERY launch -- minutes each
  time, with perception down -- and setup step \`tensorrt-runtime\` will later
  install a second, multi-gigabyte TensorRT to repair it.

  Any 10.x still makes the Autoware packages installable, which is this step's
  actual job, so this is a degradation and not a failure.

EOF
}

# Print `pkg=version` for each package when the exact ABI version is offered by
# a configured source, and bare package names otherwise.
pinned_packages() {
    local pkg listing want spec abi
    local -a specs=()
    abi="$(engine_abi)"
    if [ -z "$abi" ]; then
        warn_unpinned "could not read nvidia_amd64.tensorrt_engine_abi from versions.yaml"
        printf '%s\n' "${PACKAGES[@]}"
        return
    fi
    for pkg in "${PACKAGES[@]}"; do
        listing="$(apt-cache madison "$pkg" 2>/dev/null || true)"
        # No `{exit}` in the awk: closing the pipe early costs apt-cache a
        # SIGPIPE and `set -o pipefail` turns that into a fatal 141.
        want="$(awk -v v="${abi}." '$3 ~ ("^" v) && !seen { print $3; seen = 1 }' <<<"$listing")"
        if [ -z "$want" ]; then
            warn_unpinned "no configured apt source offers ${pkg} ${abi}.*"
            printf '%s\n' "${PACKAGES[@]}"
            return
        fi
        specs+=("${pkg}=${want}")
    done
    printf '%s\n' "${specs[@]}"
}

ARCH="$(uname -m)"

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
        # /opt/tensorrt/<version> is OUR prefix, put there by the
        # `tensorrt-runtime` step and on LD_LIBRARY_PATH by scripts/env.sh. It
        # is deliberate, it is deliberately outside the ldconfig cache, and it
        # exists precisely because the system TensorRT is a different patch --
        # so stopping here would refuse the step on every machine that has
        # already had the engine problem fixed.
        case "$d" in /opt/tensorrt/*) continue ;; esac
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
    mapfile -t specs < <(pinned_packages)
    say "installing: ${specs[*]}"
    sudo apt-get install -y --no-install-recommends --allow-downgrades "${specs[@]}"
    say "done"
    exit 0
fi

# ------------------------------------------- tier 3: add NVIDIA's source ----
# Only reached when the machine has no TensorRT and nothing that offers it, so
# there is nothing to conflict with. This is NVIDIA's own documented network
# repository method.
#
# A real Jetson never arrives here: JetPack puts TensorRT in the ldconfig cache,
# so tier 1 has already exited.
#
# What arrives here on arm64 is the DESKTOP CONTAINER for Apple Silicon. That
# image is Jetson-flavoured on purpose -- Autoware comes from the jetpack62
# localrepo, because that is the only arm64 Autoware build there is -- so its
# CUDA and TensorRT must come from the same place, the Jetson repository, and
# not from some other arm64 packaging. Verified: cuda-cudart and libnvinfer10
# install cleanly into a plain ubuntu:22.04 arm64 container from that
# repository, pulling only config packages and nothing Tegra-specific.
#
# arm64 hosts other than the Orin are not a supported platform; this path
# exists so an Apple Silicon laptop can run the Jetson container natively
# rather than through emulation.
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

if [ "$ARCH" = "aarch64" ]; then
    say "arm64 without JetPack (the Apple Silicon container): installing from"
    say "the Jetson repository, to match the jetpack62 Autoware packages"
    sudo install -d /usr/share/keyrings
    if command -v wget >/dev/null 2>&1; then
        wget -qO- https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
            | sudo gpg --dearmor --yes -o /usr/share/keyrings/jetson.gpg
    else
        curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
            | sudo gpg --dearmor --yes -o /usr/share/keyrings/jetson.gpg
    fi
    # The pocket follows the L4T minor this project pins (JetPack 6.2.2+ -> 36.5);
    # override with JETSON_REPO for a board deliberately left behind it.
    _l4t="$("${REPO_DIR}/scripts/version/get-version.sh" nvidia_arm64.l4t 2>/dev/null || true)"
    : "${_l4t:=36.5}"
    echo "deb [signed-by=/usr/share/keyrings/jetson.gpg] https://repo.download.nvidia.com/jetson/common ${JETSON_REPO:-r${_l4t}} main" \
        | sudo tee /etc/apt/sources.list.d/nvidia-jetson.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y --no-install-recommends "${PACKAGES[@]}"
    sudo ldconfig
    ldcache="$(ldconfig -p 2>/dev/null || true)"
    for so in "${SONAMES[@]}"; do
        grep -qF "$so" <<<"$ldcache" || {
            echo "error: ${so} still not present after install" >&2
            exit 1
        }
    done
    say "TensorRT 10 installed from the Jetson repository"
    exit 0
fi

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
mapfile -t specs < <(pinned_packages)
say "installing: ${specs[*]}"
sudo apt-get install -y --no-install-recommends "${specs[@]}"

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
