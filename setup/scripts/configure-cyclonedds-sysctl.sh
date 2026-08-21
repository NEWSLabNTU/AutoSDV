#!/usr/bin/env bash
# Configure CycloneDDS kernel network buffers (system-wide)
# Based on: https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/

set -e

echo "Configuring CycloneDDS kernel network buffers (system-wide)..."
echo ""
echo "Settings to be applied:"
echo "  ip link set lo multicast on"
echo "  net.core.rmem_max=2147483647"
echo "  net.ipv4.ipfrag_time=3"
echo "  net.ipv4.ipfrag_high_thresh=134217728"
echo ""

echo "Enable lo multicast"
sudo ip link set lo multicast on

# Apply immediately
echo "Applying sysctl settings..."
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728

# Make persistent across reboots.
#
# Numbered 99- so it applies last. The previous name, 10-cyclone-max.conf, lost
# to the ZED SDK's /etc/sysctl.d/60-zed-buffers.conf, which sets
# net.core.rmem_max=1048576 - a *lower* value. The result on the orin was
# CycloneDDS failing outright:
#   failed to increase socket receive buffer size to at least 10485760 bytes,
#   current is 2097152 bytes
#   rmw_create_node: failed to create domain
# because SocketReceiveBufferSize min="10MB" in our profiles is a hard minimum.
echo "Creating persistent configuration..."
sudo tee /etc/sysctl.d/99-cyclonedds-max.conf > /dev/null << 'EOF'
# CycloneDDS kernel network buffer optimization
# Configured by AutoSDV setup
# Numbered 99- to win against the ZED SDK's 60-zed-buffers.conf, which sets a
# lower net.core.rmem_max.
# See: https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/

net.core.rmem_max=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF

# Drop the old lower-priority file so the two cannot disagree.
if [ -f /etc/sysctl.d/10-cyclone-max.conf ]; then
    echo "Removing superseded /etc/sysctl.d/10-cyclone-max.conf..."
    sudo rm -f /etc/sysctl.d/10-cyclone-max.conf
fi

echo ""
echo "✓ Kernel buffers configured successfully!"
echo "  Settings will persist across reboots."
echo ""
echo "Verify configuration:"
echo "  sysctl net.core.rmem_max net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh"
echo ""

# Remove warning marker if it exists (force .envrc to re-check)
AUTOSDV_ROOT="$(cd "$(dirname "$0")/../.." && pwd)" || { echo "Error: cannot resolve repo root"; exit 1; }
if [ -f "$AUTOSDV_ROOT/.envrc.sysctl-warned" ]; then
    rm -f "$AUTOSDV_ROOT/.envrc.sysctl-warned"
    echo "Note: .envrc will re-check configuration on next activation"
fi
