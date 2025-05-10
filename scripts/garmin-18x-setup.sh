#!/bin/bash

set -e

RULE_FILE="/etc/udev/rules.d/99-ftdi-serial.rules"
VENDOR_ID="0403"
PRODUCT_ID="6001"

echo "Creating udev rule for FTDI FT232 device..."

# Create the rule
sudo bash -c "cat > $RULE_FILE" <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="$VENDOR_ID", ATTRS{idProduct}=="$PRODUCT_ID", MODE="0666", GROUP="plugdev"
EOF

echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Done. Please replug your FTDI device and run:"
echo "  ls -l /dev/ttyUSB*"
echo "to confirm the permissions are crw-rw-rw- (0666)."
