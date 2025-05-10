#!/usr/bin/env bash

# Script to create NetworkManager profile for static IP connection (e.g., LiDAR)
# Created for AutoSDV project

set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
CONNECTIONS_DIR="/etc/NetworkManager/system-connections"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi

echo "Setting up static IP NetworkManager profile..."

# Define template directories
TEMPLATE_DIR="${SCRIPT_DIR}/templates"
mkdir -p "${TEMPLATE_DIR}"

# Check for Velodyne LiDAR connection template
VELODYNE_TEMPLATE="${TEMPLATE_DIR}/velodyne.nmconnection.in"
if [ ! -f "${VELODYNE_TEMPLATE}" ]; then
  echo "Velodyne template not found at ${VELODYNE_TEMPLATE}. Please create it first."
  exit 1
fi

# Generate connection file from template
echo "Generating NetworkManager profile from template..."
cat "${VELODYNE_TEMPLATE}" > /tmp/velodyne.nmconnection

# Install connection file
echo "Installing NetworkManager profile..."
install -m 600 /tmp/velodyne.nmconnection "${CONNECTIONS_DIR}/velodyne.nmconnection"

# Clean up temporary file
rm /tmp/velodyne.nmconnection

# Reload NetworkManager connections
echo "Reloading NetworkManager connections..."
nmcli connection reload

echo "Done. Static IP connection 'velodyne' is now available."
echo "You can activate it with:"
echo "  sudo nmcli connection up velodyne"
