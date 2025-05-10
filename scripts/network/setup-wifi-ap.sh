#!/usr/bin/env bash

# Script to create NetworkManager profile for WiFi AP
# Created for AutoSDV project

set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
CONNECTIONS_DIR="/etc/NetworkManager/system-connections"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi

# Get MAC address of wlan0 and extract last 6 characters
if [ -e /sys/class/net/wlan0/address ]; then
  MAC_ADDR=$(cat /sys/class/net/wlan0/address | tr -d ':' | tail -c 7)
else
  echo "Warning: Could not get MAC address of wlan0, using default identifier"
  MAC_ADDR="UNKN"
fi

WIFI_AP_NAME="AutoSDV-AP-${MAC_ADDR}"
echo "Setting up WiFi AP NetworkManager profile..."

# Define template directories
TEMPLATE_DIR="${SCRIPT_DIR}/templates"
mkdir -p "${TEMPLATE_DIR}"

# Create WiFi AP connection template
WIFI_AP_TEMPLATE="${TEMPLATE_DIR}/autosdv-ap.nmconnection.in"
if [ ! -f "${WIFI_AP_TEMPLATE}" ]; then
  echo "WiFi AP template not found at ${WIFI_AP_TEMPLATE}. Please create it first."
  exit 1
fi

# Generate connection file from template
echo "Generating NetworkManager profile from template..."

# Replace placeholders in WiFi AP template
sed "s/@WIFI_AP_NAME@/${WIFI_AP_NAME}/g" "${WIFI_AP_TEMPLATE}" > /tmp/autosdv-ap.nmconnection

# Install connection file
echo "Installing NetworkManager profile..."
install -m 600 /tmp/autosdv-ap.nmconnection "${CONNECTIONS_DIR}/autosdv-ap.nmconnection"

# Clean up temporary file
rm /tmp/autosdv-ap.nmconnection

# Reload NetworkManager connections
echo "Reloading NetworkManager connections..."
nmcli connection reload

echo "Done. WiFi AP connection '${WIFI_AP_NAME}' is now available."
echo "You can activate it with:"
echo "  sudo nmcli connection up \"${WIFI_AP_NAME}\""
