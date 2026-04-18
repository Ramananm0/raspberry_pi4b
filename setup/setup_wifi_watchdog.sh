#!/bin/bash
# setup_wifi_watchdog.sh — installs the WiFi watchdog service + sets NM autoconnect
set -e

SCRIPT_DIR="$(dirname "$0")"

# 1. Copy watchdog script
sudo cp "$SCRIPT_DIR/wifi_watchdog.sh" /usr/local/bin/wifi_watchdog.sh
sudo chmod +x /usr/local/bin/wifi_watchdog.sh

# 2. Install systemd service
sudo cp "$SCRIPT_DIR/wifi-watchdog.service" /etc/systemd/system/wifi-watchdog.service
sudo systemctl daemon-reload
sudo systemctl enable wifi-watchdog.service
sudo systemctl start  wifi-watchdog.service

# 3. Tell NetworkManager to always auto-reconnect all WiFi connections
nmcli -f NAME,TYPE connection show | awk '/wifi/{print $1}' | while read -r CON; do
    echo "  Setting autoconnect on: $CON"
    nmcli connection modify "$CON" \
        connection.autoconnect yes \
        connection.autoconnect-retries -1
done

echo "=== WiFi watchdog installed and running ==="
echo "  Status:  sudo systemctl status wifi-watchdog"
echo "  Logs:    journalctl -u wifi-watchdog -f"
