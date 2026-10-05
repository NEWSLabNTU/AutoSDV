#!/usr/bin/env bash
# Install and enable the AutoSDV Robin-W PTP grandmaster.
#
# This deliberately installs /etc/systemd/system overrides with stable unit
# names instead of editing linuxptp's package-owned files under /lib/systemd.

set -euo pipefail

INTERFACE=eno1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONFIG_SOURCE_DIR="${REPO_ROOT}/setup/files/linuxptp"
UNIT_SOURCE_DIR="${REPO_ROOT}/setup/files/systemd"
UTILITY_SOURCE="${REPO_ROOT}/setup/files/bin/innovusion_lidar_util"

PTP4L_CONFIG="${CONFIG_SOURCE_DIR}/autosdv-robin-w-ptp4l.conf"
PHC2SYS_CONFIG="${CONFIG_SOURCE_DIR}/autosdv-robin-w-phc2sys.conf"
PTP4L_UNIT="${UNIT_SOURCE_DIR}/ptp4l.service"
PHC2SYS_UNIT="${UNIT_SOURCE_DIR}/phc2sys.service"

die() {
    echo "ERROR: $*" >&2
    exit 1
}

if [[ "${EUID}" -eq 0 ]]; then
    SUDO=()
else
    command -v sudo >/dev/null 2>&1 || die "sudo is required when not running as root"
    SUDO=(sudo)
fi

for required_file in "${PTP4L_CONFIG}" "${PHC2SYS_CONFIG}" "${PTP4L_UNIT}" "${PHC2SYS_UNIT}" "${UTILITY_SOURCE}"; do
    [[ -f "${required_file}" ]] || die "repository file is missing: ${required_file}"
done
[[ -x "${UTILITY_SOURCE}" ]] || die "repository utility is not executable: ${UTILITY_SOURCE}"

APT_PACKAGES=(linuxptp ethtool netcat-openbsd chrony)
packages_missing=0
for package in "${APT_PACKAGES[@]}"; do
    if ! dpkg-query -W -f='${Status}' "${package}" 2>/dev/null \
        | grep -q '^install ok installed$'; then
        packages_missing=1
        break
    fi
done

if (( packages_missing )); then
    echo "Installing Robin-W PTP dependencies: ${APT_PACKAGES[*]}..."
    "${SUDO[@]}" apt-get update
    "${SUDO[@]}" apt-get install -y --no-install-recommends "${APT_PACKAGES[@]}"
fi

[[ -x /usr/sbin/ptp4l ]] || die "linuxptp installation did not provide /usr/sbin/ptp4l"
[[ -x /usr/sbin/phc2sys ]] || die "linuxptp installation did not provide /usr/sbin/phc2sys"
command -v ethtool >/dev/null 2>&1 || die "ethtool installation did not provide ethtool"
command -v nc >/dev/null 2>&1 || die "netcat-openbsd installation did not provide nc"
command -v ip >/dev/null 2>&1 || die "iproute2 is required to inspect ${INTERFACE}"

[[ -d "/sys/class/net/${INTERFACE}" ]] || die "network interface ${INTERFACE} does not exist"
if ! ip link show dev "${INTERFACE}" | grep -q 'LOWER_UP'; then
    die "${INTERFACE} has no carrier; connect the Robin-W network before enabling PTP"
fi

TIMESTAMP_CAPABILITIES="$(ethtool -T "${INTERFACE}" 2>/dev/null || true)"
grep -q 'hardware-transmit' <<<"${TIMESTAMP_CAPABILITIES}" \
    || die "${INTERFACE} has no hardware transmit timestamp capability"
grep -q 'hardware-receive' <<<"${TIMESTAMP_CAPABILITIES}" \
    || die "${INTERFACE} has no hardware receive timestamp capability"
grep -q 'hardware-raw-clock' <<<"${TIMESTAMP_CAPABILITIES}" \
    || die "${INTERFACE} has no hardware PHC capability"

shopt -s nullglob
PHC_DEVICES=(/sys/class/net/${INTERFACE}/device/ptp/ptp*)
shopt -u nullglob
(( ${#PHC_DEVICES[@]} > 0 )) \
    || die "${INTERFACE} has no PTP hardware clock under /sys/class/net"

echo "Installing AutoSDV Robin-W PTP configuration for ${INTERFACE}..."
"${SUDO[@]}" install -d -m 0755 /etc/linuxptp /etc/systemd/system
"${SUDO[@]}" install -m 0644 "${PTP4L_CONFIG}" \
    /etc/linuxptp/autosdv-robin-w-ptp4l.conf
"${SUDO[@]}" install -m 0644 "${PHC2SYS_CONFIG}" \
    /etc/linuxptp/autosdv-robin-w-phc2sys.conf
"${SUDO[@]}" install -m 0644 "${PTP4L_UNIT}" /etc/systemd/system/ptp4l.service
"${SUDO[@]}" install -m 0644 "${PHC2SYS_UNIT}" /etc/systemd/system/phc2sys.service

# Do not allow the package template instances to compete with the dedicated
# units above. These commands are idempotent and tolerate units that were never
# enabled on this host.
for unit in "ptp4l@${INTERFACE}.service" "phc2sys@${INTERFACE}.service"; do
    "${SUDO[@]}" systemctl disable --now "${unit}" >/dev/null 2>&1 || true
done

"${SUDO[@]}" systemctl daemon-reload
"${SUDO[@]}" systemctl enable ptp4l.service phc2sys.service
"${SUDO[@]}" systemctl restart ptp4l.service
"${SUDO[@]}" systemctl restart phc2sys.service

for unit in ptp4l.service phc2sys.service; do
    if ! "${SUDO[@]}" systemctl is-active --quiet "${unit}"; then
        echo "ERROR: ${unit} did not become active" >&2
        "${SUDO[@]}" systemctl --no-pager --full status "${unit}" || true
        exit 1
    fi
done

echo "Robin-W PTP services are enabled and active."
echo "  ptp4l:   sudo systemctl status ptp4l.service"
echo "  phc2sys: sudo systemctl status phc2sys.service"
echo "  offset:  sudo pmc -u -b 1 \"GET TIME_STATUS_NP\""
echo ""
echo "After PTP is active, configure the Robin-W with:"
echo "  ${UTILITY_SOURCE} <LIDAR_IP> get_config time ptp_en"
echo "  ${UTILITY_SOURCE} <LIDAR_IP> set_config time ptp_en 1"
echo "  ${UTILITY_SOURCE} <LIDAR_IP> set_config time ptp_automotive 0"
echo "  ${UTILITY_SOURCE} <LIDAR_IP> get_config time ptp_en"
echo "  ${UTILITY_SOURCE} <LIDAR_IP> get_config time ptp_automotive"
echo ""
echo "Some Robin-W units ship a PTP config their own ptp4l cannot parse. Check with:"
echo "  ${REPO_ROOT}/setup/files/bin/robin_w_ptp_config <LIDAR_IP> check"
echo "  (see docs/guides/robin_w_ptp.md, Troubleshooting)"
