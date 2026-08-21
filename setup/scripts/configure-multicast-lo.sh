#!/usr/bin/env bash
# Enable multicast on the loopback interface, and keep it enabled across reboots.
#
#     ./setup.sh multicast-lo
#
# CycloneDDS discovers peers by multicast. cyclonedds.xml at the repo root pins
# the `lo` interface, and every profile declares
# <SocketReceiveBufferSize min="10MB"/>, both of which CycloneDDS treats as hard
# requirements -- so on a host where `lo` has no MULTICAST flag every ros2
# process dies at startup with:
#
#     selected interface "lo" is not multicast-capable: disabling multicast
#     rmw_create_node: failed to create domain, error Error
#
# WHY A SYSTEMD UNIT AND NOT JUST `ip link set`
#
# `ip link set lo multicast on` does not survive a reboot. configure-cyclonedds-
# sysctl.sh runs exactly that, and separately persists its sysctl values to
# /etc/sysctl.d/ -- so before this script existed, half the configuration came
# back after a reboot and half did not. The host then failed in the confusing
# direction: everything looked configured, `sysctl` reported the right numbers,
# and ROS still could not create a domain.
set -e

UNIT=/etc/systemd/system/multicast-lo.service

echo "Enabling multicast on the loopback interface..."
echo ""

if [ -f "${UNIT}" ]; then
    echo "  ${UNIT} already exists — refreshing it"
fi

sudo tee "${UNIT}" > /dev/null << 'EOF'
[Unit]
Description=Enable Multicast on Loopback
# ip needs the interface to exist; network-pre is early enough for lo.
After=network-pre.target
Wants=network-pre.target

[Service]
Type=oneshot
# RemainAfterExit so `systemctl is-active` reports "active" rather than
# "inactive" after the one-shot completes. Without it the unit looks like it
# failed, and the natural next step is to "fix" a thing that is already working.
RemainAfterExit=yes
ExecStart=/usr/sbin/ip link set lo multicast on

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now multicast-lo.service

echo ""
if ip link show lo | grep -q MULTICAST; then
    echo "✓ lo now has the MULTICAST flag, and will after a reboot."
else
    echo "✗ lo still has no MULTICAST flag. ROS will not start." >&2
    echo "  Check: systemctl status multicast-lo" >&2
    exit 1
fi
echo ""
echo "Verify with:"
echo "  ip link show lo | head -1"
echo "  systemctl is-enabled multicast-lo"
