#!/usr/bin/env bash
set -Eeuo pipefail

# This launcher owns only a skinManager it starts. It does not republish taxels.
exec 3>&1
if [[ "${1:-}" == "--verbose" ]]; then
    shift
else
    exec >/dev/null
fi
if (( $# != 0 )); then
    printf 'Usage: %s [--verbose]\n' "${0##*/}" >&2
    exit 2
fi

RIGHT_RAW_PORT="${RIGHT_RAW_PORT:-/icub/skin/right_hand}"
RIGHT_COMPENSATED_SOURCE="${RIGHT_COMPENSATED_SOURCE:-/icub/skin/right_hand_comp}"
REMOTE_RUN_SERVER="${REMOTE_RUN_SERVER:-/icub-head}"
REMOTE_SKINMANAGER_TAG="${REMOTE_SKINMANAGER_TAG:-primi-skinmanager}"
REMOTE_SKINMANAGER_COMMAND="${REMOTE_SKINMANAGER_COMMAND:-skinManager --from skinManAll.ini --context skinGui --binarization false}"
SKIN_WAIT_SECONDS="${SKIN_WAIT_SECONDS:-60}"
STOP_REMOTE_SKINMANAGER_ON_EXIT="${STOP_REMOTE_SKINMANAGER_ON_EXIT:-0}"

started_manager=0
port_exists() { yarp exists "$1" >/dev/null 2>&1; }
status() { printf '[INFO] %s\n' "$*" >&3; }
error() { printf '[ERROR] %s\n' "$*" >&2; }

cleanup() {
    if (( started_manager )) && [[ "$STOP_REMOTE_SKINMANAGER_ON_EXIT" == 1 ]]; then
        yarp run --on "$REMOTE_RUN_SERVER" --sigterm "$REMOTE_SKINMANAGER_TAG" \
            >/dev/null 2>&1 || true
    fi
    status "Tactile acquisition stopped."
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Reuse compensated data already published on the robot. If absent, launch the
# existing skin manager and wait for its compensated output.
yarp where >/dev/null 2>&1 || { error "YARP name server unavailable"; exit 1; }
status "Starting tactile acquisition..."
if ! port_exists "$RIGHT_COMPENSATED_SOURCE"; then
    port_exists "$RIGHT_RAW_PORT" || { error "Missing raw skin port: $RIGHT_RAW_PORT"; exit 1; }
    port_exists "$REMOTE_RUN_SERVER" || { error "Missing run server: $REMOTE_RUN_SERVER"; exit 1; }
    if ! yarp run --on "$REMOTE_RUN_SERVER" --as "$REMOTE_SKINMANAGER_TAG" \
        --cmd "$REMOTE_SKINMANAGER_COMMAND" >/dev/null 2>&1; then
        error "Could not start skinManager. Check the remote YARP logs."
        exit 1
    fi
    started_manager=1
    for ((i = 0; i < SKIN_WAIT_SECONDS * 5; ++i)); do
        port_exists "$RIGHT_COMPENSATED_SOURCE" && break
        sleep 0.2
    done
    port_exists "$RIGHT_COMPENSATED_SOURCE" || {
        error "Timed out waiting for $RIGHT_COMPENSATED_SOURCE"
        exit 1
    }
fi

status "Tactile acquisition is running. Press Ctrl+C to stop."
# run_sensors.sh supervises this process; report a source that disappears.
while port_exists "$RIGHT_COMPENSATED_SOURCE"; do
    sleep 1
done
error "Compensated right-hand skin source disappeared."
exit 1
