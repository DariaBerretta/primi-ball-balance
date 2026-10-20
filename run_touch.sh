#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
READER_EXECUTABLE="${READER_EXECUTABLE:-$PROJECT_DIR/build/touch_reader}"

LEFT_RAW_PORT="${LEFT_RAW_PORT:-/icub/skin/left_hand}"
RIGHT_RAW_PORT="${RIGHT_RAW_PORT:-/icub/skin/right_hand}"
LEFT_COMPENSATED_SOURCE="${LEFT_COMPENSATED_SOURCE:-/icub/skin/left_hand_comp}"
RIGHT_COMPENSATED_SOURCE="${RIGHT_COMPENSATED_SOURCE:-/icub/skin/right_hand_comp}"
LEFT_LOCAL_INPUT="${LEFT_LOCAL_INPUT:-/primi-ball-balance/skin/left_hand:i}"
RIGHT_LOCAL_INPUT="${RIGHT_LOCAL_INPUT:-/primi-ball-balance/skin/right_hand:i}"
LEFT_LOCAL_OUTPUT="${LEFT_LOCAL_OUTPUT:-/primi-ball-balance/skin/left_hand/compensated:o}"
RIGHT_LOCAL_OUTPUT="${RIGHT_LOCAL_OUTPUT:-/primi-ball-balance/skin/right_hand/compensated:o}"

REMOTE_RUN_SERVER="${REMOTE_RUN_SERVER:-/icub-head}"
REMOTE_SKINMANAGER_TAG="${REMOTE_SKINMANAGER_TAG:-primi-skinmanager}"
REMOTE_SKINMANAGER_COMMAND="${REMOTE_SKINMANAGER_COMMAND:-skinManager --from skinManAll.ini --context skinGui --binarization false}"
REMOTE_CARRIER="${REMOTE_CARRIER:-tcp}"
SKIN_WAIT_SECONDS="${SKIN_WAIT_SECONDS:-60}"
READER_WAIT_SECONDS="${READER_WAIT_SECONDS:-15}"
STOP_REMOTE_SKINMANAGER_ON_EXIT="${STOP_REMOTE_SKINMANAGER_ON_EXIT:-0}"

READER_PID=""
REMOTE_SKINMANAGER_STARTED=0

port_exists() { yarp exists "$1" >/dev/null 2>&1; }
wait_for_port() {
    local attempts=$(( $2 * 5 ))
    echo "Waiting for $1"
    for ((i=0; i<attempts; i++)); do
        port_exists "$1" && return 0
        sleep 0.2
    done
    echo "Timed out waiting for $1" >&2
    return 1
}

cleanup() {
    yarp disconnect "$LEFT_COMPENSATED_SOURCE" "$LEFT_LOCAL_INPUT" >/dev/null 2>&1 || true
    yarp disconnect "$RIGHT_COMPENSATED_SOURCE" "$RIGHT_LOCAL_INPUT" >/dev/null 2>&1 || true
    if [[ -n "$READER_PID" ]]; then
        kill -TERM "$READER_PID" 2>/dev/null || true
        wait "$READER_PID" 2>/dev/null || true
    fi
    if (( REMOTE_SKINMANAGER_STARTED )) && [[ "$STOP_REMOTE_SKINMANAGER_ON_EXIT" == 1 ]]; then
        yarp run --on "$REMOTE_RUN_SERVER" --sigterm "$REMOTE_SKINMANAGER_TAG" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

yarp where >/dev/null || { echo "YARP name server unavailable" >&2; exit 1; }
[[ -x "$READER_EXECUTABLE" ]] || { echo "Missing reader: $READER_EXECUTABLE" >&2; exit 1; }
port_exists "$REMOTE_RUN_SERVER" || { echo "Missing run server: $REMOTE_RUN_SERVER" >&2; exit 1; }
for port in "$LEFT_RAW_PORT" "$RIGHT_RAW_PORT"; do
    port_exists "$port" || { echo "Missing raw skin port: $port" >&2; exit 1; }
done

pkill -x touch_reader 2>/dev/null || true
for port in "$LEFT_LOCAL_INPUT" "$RIGHT_LOCAL_INPUT" "$LEFT_LOCAL_OUTPUT" "$RIGHT_LOCAL_OUTPUT"; do
    yarp name unregister "$port" >/dev/null 2>&1 || true
done

if port_exists "$LEFT_COMPENSATED_SOURCE" && port_exists "$RIGHT_COMPENSATED_SOURCE"; then
    echo "Using existing compensated skin streams"
elif port_exists "$LEFT_COMPENSATED_SOURCE" || port_exists "$RIGHT_COMPENSATED_SOURCE"; then
    echo "Only one compensated skin stream is available" >&2
    exit 1
else
    yarp run --on "$REMOTE_RUN_SERVER" --sigterm "$REMOTE_SKINMANAGER_TAG" >/dev/null 2>&1 || true
    yarp run --on "$REMOTE_RUN_SERVER" --as "$REMOTE_SKINMANAGER_TAG" --cmd "$REMOTE_SKINMANAGER_COMMAND"
    REMOTE_SKINMANAGER_STARTED=1
    wait_for_port "$LEFT_COMPENSATED_SOURCE" "$SKIN_WAIT_SECONDS"
    wait_for_port "$RIGHT_COMPENSATED_SOURCE" "$SKIN_WAIT_SECONDS"
fi

"$READER_EXECUTABLE" &
READER_PID=$!
for port in "$LEFT_LOCAL_INPUT" "$RIGHT_LOCAL_INPUT" "$LEFT_LOCAL_OUTPUT" "$RIGHT_LOCAL_OUTPUT"; do
    wait_for_port "$port" "$READER_WAIT_SECONDS"
done
yarp connect "$LEFT_COMPENSATED_SOURCE" "$LEFT_LOCAL_INPUT" "$REMOTE_CARRIER"
yarp connect "$RIGHT_COMPENSATED_SOURCE" "$RIGHT_LOCAL_INPUT" "$REMOTE_CARRIER"
echo "Tactile acquisition running; skin visualization is owned by run_monitor.sh"
wait "$READER_PID"
