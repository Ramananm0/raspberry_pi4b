#!/bin/bash
# wifi_watchdog.sh — runs as a systemd service, reconnects WiFi if it drops

PING_HOST="8.8.8.8"
IFACE="wlan0"
CHECK_INTERVAL=20   # seconds between checks
FAIL_COUNT=0
FAIL_THRESHOLD=3    # consecutive failures before reconnect

while true; do
    if ping -c 1 -W 3 "$PING_HOST" &>/dev/null; then
        FAIL_COUNT=0
    else
        FAIL_COUNT=$((FAIL_COUNT + 1))
        echo "[$(date)] ping failed ($FAIL_COUNT/$FAIL_THRESHOLD)"

        if [ "$FAIL_COUNT" -ge "$FAIL_THRESHOLD" ]; then
            echo "[$(date)] WiFi down — reconnecting $IFACE..."
            nmcli device disconnect "$IFACE" 2>/dev/null || true
            sleep 3
            nmcli device connect "$IFACE" 2>/dev/null || \
                nmcli networking off && nmcli networking on
            FAIL_COUNT=0
            echo "[$(date)] Reconnect attempted"
        fi
    fi
    sleep "$CHECK_INTERVAL"
done
