#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${1:-$ROOT_DIR/config/monitoring.env}"
[[ ! -f "$CONFIG_FILE" ]] || source "$CONFIG_FILE"
PID_DIR="$ROOT_DIR/.monitoring/pids"

yarp disconnect "${RGB_LEFT_PROCESSED_SOURCE:-/unused}" \
    /primi-ball-balance/monitor/rgb/left/processed:i >/dev/null 2>&1 || true
yarp disconnect "${RGB_RIGHT_PROCESSED_SOURCE:-/unused}" \
    /primi-ball-balance/monitor/rgb/right/processed:i >/dev/null 2>&1 || true
yarp disconnect /primi-ball-balance/skin/right_hand/compensated:o \
    /skinGui/right_hand:i >/dev/null 2>&1 || true

if [[ -d "$PID_DIR" ]]; then
    for file in "$PID_DIR"/*.pid; do
        [[ -f "$file" ]] || continue
        pid="$(cat "$file")"
        kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 1
    for file in "$PID_DIR"/*.pid; do
        [[ -f "$file" ]] || continue
        pid="$(cat "$file")"
        kill -9 "$pid" 2>/dev/null || true
        rm -f "$file"
    done
fi
echo "Monitoring stopped."
