#!/usr/bin/env bash
# Installation script for U-Blox GPS udev rules
# This script installs udev rules to set proper permissions for U-Blox GPS devices

set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

UDEV_RULE_FILE="99-ublox-gps.rules"
UDEV_RULES_DIR="/etc/udev/rules.d"

echo "Installing U-Blox GPS udev rules..."

# Copy udev rule to system directory
sudo cp -v "$UDEV_RULE_FILE" "$UDEV_RULES_DIR/"

# Add current user to dialout group (if not already a member)
if ! groups | grep -q dialout; then
    echo "Adding user $USER to dialout group..."
    sudo usermod -aG dialout "$USER"
    echo "NOTE: You need to log out and log back in for group changes to take effect."
else
    echo "User $USER is already in dialout group."
fi

# Reload udev rules
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "Installation complete!"
echo "The U-Blox GPS device will appear as /dev/ublox-gps (symlink to /dev/ttyACM0)"
echo ""
echo "If the device was already connected, you may need to:"
echo "  1. Unplug and replug the GPS device, OR"
echo "  2. Run: sudo udevadm trigger"
echo ""
if ! groups | grep -q dialout; then
    echo "IMPORTANT: Log out and log back in for group membership to take effect!"
fi
