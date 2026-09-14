#!/usr/bin/env bash
# Check for the ZED SDK, and say exactly what to download when it is missing.
#
# This step does not install anything, and that is not an omission. Stereolabs
# publishes no apt repository: the only official artifact is a self-extracting
# `.run` installer that asks questions (licence, CUDA, samples, Python API,
# static libraries) and reboots nothing but expects answers. Driving that from
# an unattended `--yes` run would mean answering on the user's behalf for a
# proprietary licence, and the previous version of this script did something
# worse -- it installed SDK 4.2 from a third-party Debian repackage
# (`jerry73204/zed-sdk-debian-package`, built for JetPack 6.0), which no longer
# matches either the wrapper this workspace builds or the JetPack the vehicle
# runs.
#
# So: report what is here, name the download that fits this machine, and let the
# caller's summary repeat it at the end of the run. A missing SDK is not a
# failure -- everything except the ZED camera works without it, and
# `zed_components` skips itself rather than failing the workspace build.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
GET_VERSION="${REPO_DIR}/scripts/version/get-version.sh"

WANT="$("${GET_VERSION}" zed.sdk_version)"
SERIES="$("${GET_VERSION}" zed.sdk_series)"
URL_BASE="$("${GET_VERSION}" zed.url_base)"
# The L4T minor this project pins (JetPack 6.2.2 and up ship 36.5), used only
# when a Jetson's own release file cannot be read.
L4T_DEFAULT="$("${GET_VERSION}" nvidia_arm64.l4t)"
: "${L4T_DEFAULT:=36.5}"

# Only when stdout is a terminal that wants colour: this output is also read
# through `tee`, in CI logs, and by the curses menu's plain fallback.
if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
    BOLD=$'\033[1m'; CYAN=$'\033[1;36m'; YELLOW=$'\033[1;33m'; OFF=$'\033[0m'
else
    BOLD=""; CYAN=""; YELLOW=""; OFF=""
fi

# The version is in the SDK's own cmake package file, which is what
# `find_package(ZED)` reads -- so this is the same answer the build will get,
# not a guess from a directory name.
installed_version() {
    local f=/usr/local/zed/zed-config-version.cmake
    [[ -f "$f" ]] || return 1
    sed -n 's/^set(PACKAGE_VERSION "\([0-9.]*\)").*/\1/p' "$f" | head -1
}

# Which download fits this machine. The path components are Stereolabs' own
# stable redirects; the CDN filename behind them carries the patch version and
# changes with every release, so never hard-code that.
platform_path() {
    if [[ -f /etc/nv_tegra_release ]]; then
        # e.g. "# R36 (release), REVISION: 4.4" -> l4t36.4
        local major minor
        major=$(sed -n 's/^# R\([0-9]\+\).*/\1/p' /etc/nv_tegra_release)
        minor=$(sed -n 's/.*REVISION: \([0-9]\+\).*/\1/p' /etc/nv_tegra_release)
        if [[ -n "$major" && -n "$minor" ]]; then
            echo "l4t${major}.${minor}/jetsons"
            return
        fi
        # A Tegra whose release file we could not parse: assume the pin.
        echo "l4t${L4T_DEFAULT}/jetsons"
        return
    fi
    # amd64: CUDA 12, because that is what Autoware pins (nvidia_amd64.cuda).
    echo "cu12/ubuntu22"
}

URL="${URL_BASE}/${SERIES}/$(platform_path)"

# `--check` is what the setup step's `verify` runs: same comparison, no output,
# exit status only.
if [[ "${1:-}" == "--check" ]]; then
    version="$(installed_version)" || exit 1
    [[ "$version" == "$WANT" ]] || exit 1
    exit 0
fi

if version="$(installed_version)"; then
    if [[ "$version" == "$WANT" ]]; then
        echo "ZED SDK ${version} is installed at /usr/local/zed."
        exit 0
    fi
    echo "${YELLOW}ZED SDK ${version} is installed, but this workspace builds the"
    echo "ROS 2 wrapper against ${WANT}.${OFF}"
    echo
    echo "zed_components compiles against the SDK's headers, so a mismatch is a"
    echo "build or a runtime failure rather than a degraded mode. Install ${WANT}:"
else
    echo "${YELLOW}The ZED SDK is not installed.${OFF} Nothing else is affected:"
    echo "zed_components skips itself, and the rest of the workspace builds."
    echo
    echo "For a ZED camera, install SDK ${WANT} by hand -- Stereolabs ships no"
    echo "apt package, only an interactive installer:"
fi

echo
echo "    ${CYAN}${URL}${OFF}"
echo
echo "  ${BOLD}curl -fsSL -o zed_sdk.run '${URL}'${OFF}"
echo "  ${BOLD}chmod +x zed_sdk.run && ./zed_sdk.run${OFF}"
echo
echo "That URL is a redirect Stereolabs keeps stable; it resolves to the file"
echo "for this machine:"
echo "    amd64, Ubuntu 22.04, CUDA 12          ${SERIES}/cu12/ubuntu22"
echo "    Jetson, L4T 36.5 (JetPack 6.2.2+)     ${SERIES}/l4t36.5/jetsons"
echo "    Jetson, L4T 36.4 (JetPack 6.2/6.2.1)  ${SERIES}/l4t36.4/jetsons"
echo
echo "Then re-run this step to confirm:  ./setup.sh --rerun zed-sdk"

# Deliberately 0. This step reports; it does not fail a provisioning run over a
# camera the machine may not have.
exit 0
