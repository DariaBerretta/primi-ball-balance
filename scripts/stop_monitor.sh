#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${1:-$ROOT_DIR/config/monitoring.env}"
PID_DIR="$ROOT_DIR/.monitoring/pids"

if [[ -f "$CONFIG_FILE" ]]; then
    # shellcheck disable=SC1090
    source "$CONFIG_FILE"
fi

disconnect()
{
    yarp disconnect "$1" "$2" >/dev/null 2>&1 || true
}

disconnect "${RGB_LEFT_RAW_SOURCE:-/unused}" \
    /primi-ball-balance/monitor/rgb/left/raw:i

disconnect "${RGB_RIGHT_RAW_SOURCE:-/unused}" \
    /primi-ball-balance/monitor/rgb/right/raw:i

disconnect "${RGB_LEFT_PROCESSED_SOURCE:-/unused}" \
    /primi-ball-balance/monitor/rgb/left/processed:i

disconnect "${RGB_RIGHT_PROCESSED_SOURCE:-/unused}" \
    /primi-ball-balance/monitor/rgb/right/processed:i

if [[ ! -d "$PID_DIR" ]]; then
    echo "No monitoring processes found."
    exit 0
fi

for pid_file in "$PID_DIR"/*.pid; do
    [[ -e "$pid_file" ]] || continue

    process_name="$(basename "$pid_file" .pid)"
    process_pid="$(cat "$pid_file")"

    if kill -0 "$process_pid" 2>/dev/null; then
        echo "Stopping $process_name..."
        kill -TERM "$process_pid" 2>/dev/null || true
    fi
done

sleep 1

for pid_file in "$PID_DIR"/*.pid; do
    [[ -e "$pid_file" ]] || continue

    process_pid="$(cat "$pid_file")"

    if kill -0 "$process_pid" 2>/dev/null; then
        kill -9 "$process_pid" 2>/dev/null || true
    fi

    rm -f "$pid_file"
done

echo "Monitoring application stopped."