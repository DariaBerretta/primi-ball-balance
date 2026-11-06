#!/usr/bin/env bash
set -Eeuo pipefail

# Quiet by default, verbose output only when requested.
verbose_args=()
if [[ "${1:-}" == "--verbose" ]]; then
    verbose_args=(--verbose)
    shift
else
    exec >/dev/null 2>&1
fi
if (( $# != 0 )); then
    printf 'Usage: %s [--verbose]\n' "${0##*/}" >&2
    exit 2
fi

# Keep this launcher alongside the three acquisition scripts.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

scripts=(run_eventCam.sh run_rgbCam.sh run_touch.sh)
pids=()

command -v setsid >/dev/null || { echo "Missing command: setsid" >&2; exit 1; }
for script in "${scripts[@]}"; do
    [[ -f "$script" ]] || { echo "Missing script: $PROJECT_DIR/$script" >&2; exit 1; }
done
[[ -f scripts/run_moveenet_ofk.sh ]] || {
    echo "Missing script: $PROJECT_DIR/scripts/run_moveenet_ofk.sh" >&2; exit 1;
}

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
    setsid bash "$PROJECT_DIR/$script" "${verbose_args[@]}" &
    pids+=("$!")
done

# MoveEnetOFK requires the locally republished event stream.
ready=0
for ((i = 0; i < 700; ++i)); do
    if yarp exists /primi-ball-balance/eventCam/left/raw:o >/dev/null 2>&1 &&
       yarp exists /primi-ball-balance/eventCam/right/raw:o >/dev/null 2>&1; then
        ready=1
        break
    fi
    kill -0 "${pids[0]}" 2>/dev/null || exit 1
    sleep 0.2
done
if (( ! ready )); then
    echo "Timed out waiting for event-camera outputs" >&2
    exit 1
fi
echo "Starting scripts/run_moveenet_ofk.sh"
setsid bash "$PROJECT_DIR/scripts/run_moveenet_ofk.sh" "${verbose_args[@]}" &
pids+=("$!")

echo "Sensor acquisition and pose estimation started. Press Ctrl+C to stop."
# Stop the remaining scripts if any launcher exits or fails.
wait -n "${pids[@]}"
