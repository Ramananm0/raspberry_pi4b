#!/bin/bash
# setup_wifi.sh — configures two WiFi networks (primary hotspot + fallback home WiFi)
# Run once on the Pi: bash setup_wifi.sh <hotspot_ssid> <hotspot_pass>
set -e

HOTSPOT_SSID="${1:-}"
HOTSPOT_PASS="${2:-}"
FALLBACK_SSID="MOUROUGANE-2.4G"
FALLBACK_PASS="RamKeeraman"

NETPLAN_FILE="/etc/netplan/99-wifi.yaml"

# Disable cloud-init overwriting our netplan on reboot
sudo mkdir -p /etc/cloud/cloud.cfg.d
echo "network: {config: disabled}" | sudo tee /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg > /dev/null

# Build access-points section
HOTSPOT_ENTRY=""
if [ -n "$HOTSPOT_SSID" ]; then
    HOTSPOT_ENTRY="        \"${HOTSPOT_SSID}\":
          password: \"${HOTSPOT_PASS}\""
fi

sudo tee "$NETPLAN_FILE" > /dev/null <<EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
$([ -n "$HOTSPOT_ENTRY" ] && echo "$HOTSPOT_ENTRY")
        "${FALLBACK_SSID}":
          password: "${FALLBACK_PASS}"
EOF

sudo chmod 600 "$NETPLAN_FILE"
sudo netplan generate && sudo netplan apply
echo "=== WiFi config applied ==="
echo "  Networks configured:"
[ -n "$HOTSPOT_SSID" ] && echo "    1) $HOTSPOT_SSID (primary)"
echo "    2) $FALLBACK_SSID (fallback)"
