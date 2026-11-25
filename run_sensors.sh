#!/usr/bin/env bash
set -Eeuo pipefail
exec 3>&1

# Quiet by default, verbose output only when requested.
verbose_args=()
if [[ "${1:-}" == "--verbose" ]]; then
    verbose_args=(--verbose)
    shift
else
    exec >/dev/null
fi
if (( $# != 0 )); then
    printf 'Usage: %s [--verbose]\n' "${0##*/}" >&2
    exit 2
fi
printf '[INFO] Starting sensor acquisition...\n' >&3

# Keep this launcher alongside the three acquisition scripts.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

# Only the three primary sensors start here. BMA400 and pose are opt-in.
scripts=(run_event_cameras.sh run_rgb_cameras.sh run_touch.sh)
pids=()

command -v setsid >/dev/null || { echo "Missing command: setsid" >&2; exit 1; }
for script in "${scripts[@]}"; do
    [[ -f "$script" ]] || { echo "Missing script: $PROJECT_DIR/$script" >&2; exit 1; }
done

cleanup() {
    trap '' INT TERM
    if [[ -t 3 ]]; then printf '\n' >&3; fi
    printf '[INFO] Stopping sensor acquisition...\n' >&3
    # Signal each launcher's process group, including its local child processes.
    for pid in "${pids[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    for pid in "${pids[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    printf '[INFO] Sensor acquisition stopped.\n' >&3
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for script in "${scripts[@]}"; do
    echo "Starting $script"
    PRIMI_SENSOR_GROUP=1 setsid bash "$PROJECT_DIR/$script" "${verbose_args[@]}" >&3 &
    pids+=("$!")
done

printf '[INFO] Sensor launchers started. Press Ctrl+C to stop.\n' >&3

# If a launcher exits, stop the remaining acquisition pipelines.
if wait -n "${pids[@]}"; then
    printf '[WARN] A sensor launcher exited; stopping acquisition.\n' >&2
else
    printf '[ERROR] A sensor launcher failed; stopping acquisition.\n' >&2
fi
exit 1
