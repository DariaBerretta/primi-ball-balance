#!/usr/bin/env bash
set -Eeuo pipefail

# Keep this launcher alongside the three acquisition scripts.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

scripts=(run_eventCam.sh run_rgbCam.sh run_touch.sh)
pids=()

command -v setsid >/dev/null || { echo "Missing command: setsid" >&2; exit 1; }
for script in "${scripts[@]}"; do
    [[ -f "$script" ]] || { echo "Missing script: $PROJECT_DIR/$script" >&2; exit 1; }
done

cleanup() {
    trap '' INT TERM
    echo "Stopping sensor acquisition..."
    # Signal each launcher's process group, including its local child processes.
    for pid in "${pids[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    for pid in "${pids[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for script in "${scripts[@]}"; do
    echo "Starting $script"
    setsid bash "$PROJECT_DIR/$script" &
    pids+=("$!")
done

echo "Sensor acquisition started. Press Ctrl+C to stop all three."
# Stop the remaining scripts if any launcher exits or fails.
wait -n "${pids[@]}"
