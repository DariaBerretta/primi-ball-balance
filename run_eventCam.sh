#!/usr/bin/env bash

set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
READER_EXECUTABLE="${READER_EXECUTABLE:-$PROJECT_DIR/build/eventCam_reader}"

REMOTE_RUN_SERVER="${REMOTE_RUN_SERVER:-/icub-ultrascale}"
REMOTE_SSH_HOST="${REMOTE_SSH_HOST:-icub@icub-ultrascale}"
REMOTE_COMMAND_TAG="${REMOTE_COMMAND_TAG:-zynqGrabber}"
REMOTE_COMMAND="${REMOTE_COMMAND:-zynqGrabber}"

LEFT_REMOTE_SOURCE="${LEFT_REMOTE_SOURCE:-/zynqGrabber/left/AE:o}"
RIGHT_REMOTE_SOURCE="${RIGHT_REMOTE_SOURCE:-/zynqGrabber/right/AE:o}"

LEFT_LOCAL_INPUT="${LEFT_LOCAL_INPUT:-/primi-ball-balance/eventCam/left:i}"
RIGHT_LOCAL_INPUT="${RIGHT_LOCAL_INPUT:-/primi-ball-balance/eventCam/right:i}"
LEFT_LOCAL_OUTPUT="${LEFT_LOCAL_OUTPUT:-/primi-ball-balance/eventCam/left/raw:o}"
RIGHT_LOCAL_OUTPUT="${RIGHT_LOCAL_OUTPUT:-/primi-ball-balance/eventCam/right/raw:o}"

CARRIER="${CARRIER:-fast_tcp}"
REMOTE_WAIT_SECONDS="${REMOTE_WAIT_SECONDS:-60}"
READER_WAIT_SECONDS="${READER_WAIT_SECONDS:-15}"

READER_PID=""
CLEANUP_DONE=0

log()
{
    printf '[INFO] %s\n' "$*"
}

error()
{
    printf '[ERROR] %s\n' "$*" >&2
}

port_exists()
{
    yarp exists "$1" >/dev/null 2>&1
}

wait_for_port()
{
    local port_name="$1"
    local timeout_seconds="$2"
    local description="$3"
    local checks=$((timeout_seconds * 5))
    local i

    log "Waiting for ${description}: ${port_name}"

    for ((i = 0; i < checks; ++i)); do
        if port_exists "$port_name"; then
            return 0
        fi

        sleep 0.2
    done

    error "Timed out waiting for ${description}: ${port_name}"
    return 1
}

stop_local_reader()
{
    if [[ -n "$READER_PID" ]] && kill -0 "$READER_PID" 2>/dev/null; then
        kill -TERM "$READER_PID" 2>/dev/null || true
        wait "$READER_PID" 2>/dev/null || true
    fi

    pkill -x eventCam_reader 2>/dev/null || true
    READER_PID=""
}

stop_zynq_grabber()
{
    yarp run --on "$REMOTE_RUN_SERVER" \
        --sigterm "$REMOTE_COMMAND_TAG" >/dev/null 2>&1 || true

    ssh "$REMOTE_SSH_HOST" '
        pids=$(pgrep -f "[z]ynqGrabber" || true)

        if [ -n "$pids" ]; then
            kill $pids
            sleep 1
        fi

        pids=$(pgrep -f "[z]ynqGrabber" || true)

        if [ -n "$pids" ]; then
            kill -9 $pids
        fi
    ' >/dev/null 2>&1 || true
}

unregister_port()
{
    yarp name unregister "$1" >/dev/null 2>&1 || true
}

cleanup()
{
    if (( CLEANUP_DONE )); then
        return
    fi

    CLEANUP_DONE=1
    printf '\n'

    yarp disconnect "$LEFT_REMOTE_SOURCE" "$LEFT_LOCAL_INPUT" \
        >/dev/null 2>&1 || true
    yarp disconnect "$RIGHT_REMOTE_SOURCE" "$RIGHT_LOCAL_INPUT" \
        >/dev/null 2>&1 || true

    log "Stopping local event-camera reader"
    stop_local_reader

    log "Stopping zynqGrabber"
    stop_zynq_grabber
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if ! yarp where >/dev/null 2>&1; then
    error "The YARP name server is not reachable."
    exit 1
fi

if [[ ! -x "$READER_EXECUTABLE" ]]; then
    error "Event-camera reader not found: $READER_EXECUTABLE"
    error "Build it with: cmake --build \"$PROJECT_DIR/build\" -j\"\$(nproc)\""
    exit 1
fi

if ! port_exists "$REMOTE_RUN_SERVER"; then
    error "YARP run server is not available: $REMOTE_RUN_SERVER"
    exit 1
fi

log "Stopping previous event-camera processes"
stop_local_reader
stop_zynq_grabber

log "Removing stale event-camera port registrations"
unregister_port "$LEFT_REMOTE_SOURCE"
unregister_port "$RIGHT_REMOTE_SOURCE"
unregister_port "$LEFT_LOCAL_INPUT"
unregister_port "$RIGHT_LOCAL_INPUT"
unregister_port "$LEFT_LOCAL_OUTPUT"
unregister_port "$RIGHT_LOCAL_OUTPUT"

sleep 1

log "Starting zynqGrabber on ${REMOTE_RUN_SERVER}"

yarp run --on "$REMOTE_RUN_SERVER" \
    --as "$REMOTE_COMMAND_TAG" \
    --cmd "$REMOTE_COMMAND"

wait_for_port "$LEFT_REMOTE_SOURCE" "$REMOTE_WAIT_SECONDS" \
    "left event-camera source"
wait_for_port "$RIGHT_REMOTE_SOURCE" "$REMOTE_WAIT_SECONDS" \
    "right event-camera source"

log "Starting local event-camera reader"
"$READER_EXECUTABLE" &
READER_PID=$!

wait_for_port "$LEFT_LOCAL_INPUT" "$READER_WAIT_SECONDS" \
    "left event-camera reader input"
wait_for_port "$RIGHT_LOCAL_INPUT" "$READER_WAIT_SECONDS" \
    "right event-camera reader input"
wait_for_port "$LEFT_LOCAL_OUTPUT" "$READER_WAIT_SECONDS" \
    "left local event output"
wait_for_port "$RIGHT_LOCAL_OUTPUT" "$READER_WAIT_SECONDS" \
    "right local event output"

log "Connecting remote event streams"
yarp connect "$LEFT_REMOTE_SOURCE" "$LEFT_LOCAL_INPUT" "$CARRIER"
yarp connect "$RIGHT_REMOTE_SOURCE" "$RIGHT_LOCAL_INPUT" "$CARRIER"

log "Event-camera pipeline is running"
log "Local outputs:"
log "  $LEFT_LOCAL_OUTPUT"
log "  $RIGHT_LOCAL_OUTPUT"
log "Press Ctrl+C to stop"

wait "$READER_PID"
