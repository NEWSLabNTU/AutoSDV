#!/usr/bin/env bash
# Configure CycloneDDS kernel network buffers (system-wide)
# Based on: https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/

set -e

echo "Configuring CycloneDDS kernel network buffers (system-wide)..."
echo ""
echo "Settings to be applied:"
echo "  net.core.rmem_max=2147483647"
echo "  net.ipv4.ipfrag_time=3"
echo "  net.ipv4.ipfrag_high_thresh=134217728"
echo ""

# Apply immediately
echo "Applying sysctl settings..."
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728

# Make persistent across reboots
echo "Creating persistent configuration..."
sudo tee /etc/sysctl.d/10-cyclone-max.conf > /dev/null << 'EOF'
# CycloneDDS kernel network buffer optimization
# Configured by AutoSDV setup
# See: https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/

net.core.rmem_max=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF

echo ""
echo "✓ Kernel buffers configured successfully!"
echo "  Settings will persist across reboots."
echo ""
echo "Verify configuration:"
echo "  sysctl net.core.rmem_max net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh"
echo ""

# Remove warning marker if it exists (force .envrc to re-check)
AUTOSDV_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
if [ -f "$AUTOSDV_ROOT/.envrc.sysctl-warned" ]; then
    rm -f "$AUTOSDV_ROOT/.envrc.sysctl-warned"
    echo "Note: .envrc will re-check configuration on next activation"
fi
