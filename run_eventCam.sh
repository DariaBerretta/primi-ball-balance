#!/usr/bin/env bash

set -Eeuo pipefail

# Preserve stdout for important status messages.
exec 3>&1

# Quiet by default, verbose output only when requested.
if [[ "${1:-}" == "--verbose" ]]; then
    shift
    exec 4>&1
else
    exec >/dev/null
    exec 4>/dev/null
fi

if (( $# != 0 )); then
    printf 'Usage: %s [--verbose]\n' "${0##*/}" >&2
    exit 2
fi

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
CLEANUP_ARMED=0

log()
{
    printf '[INFO] %s\n' "$*"
}

status()
{
    printf '[INFO] %s\n' "$*" >&3
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
    if [[ -n "$READER_PID" ]]; then
        if kill -0 "$READER_PID" 2>/dev/null; then
            kill -TERM "$READER_PID" 2>/dev/null || true
        fi

        wait "$READER_PID" 2>/dev/null || true
        READER_PID=""
    fi
}

# Normal shutdown: stop the registered remote command through YARP.
stop_zynq_grabber()
{
    yarp run --on "$REMOTE_RUN_SERVER" \
        --sigterm "$REMOTE_COMMAND_TAG" >/dev/null 2>&1 || true
}

# Startup recovery: retain the existing SSH fallback for orphaned processes.
recover_zynq_grabber()
{
    stop_zynq_grabber

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
    if (( ! CLEANUP_ARMED || CLEANUP_DONE )); then
        return
    fi

    CLEANUP_DONE=1

    if [[ "${PRIMI_SENSOR_GROUP:-0}" != 1 ]]; then printf '\n' >&3; fi
    status "Stopping event-camera pipeline..."

    yarp disconnect "$LEFT_REMOTE_SOURCE" "$LEFT_LOCAL_INPUT" \
        >/dev/null 2>&1 || true

    yarp disconnect "$RIGHT_REMOTE_SOURCE" "$RIGHT_LOCAL_INPUT" \
        >/dev/null 2>&1 || true

    log "Stopping local event-camera reader"
    stop_local_reader

    log "Stopping zynqGrabber"
    stop_zynq_grabber

    status "Event-camera pipeline stopped."
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Preliminary checks must not affect existing acquisition processes.

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

# Do not terminate a reader belonging to another invocation.
if pgrep -x eventCam_reader >/dev/null 2>&1; then
    error "An eventCam_reader process is already running."
    error "Stop the existing acquisition before starting another instance."
    exit 1
fi

CLEANUP_ARMED=1

status "Starting event-camera pipeline..."

# Recover remote acquisition processes left by previous executions.
log "Recovering previous remote event-camera processes"
recover_zynq_grabber

log "Removing stale event-camera port registrations"
unregister_port "$LEFT_REMOTE_SOURCE"
unregister_port "$RIGHT_REMOTE_SOURCE"
unregister_port "$LEFT_LOCAL_INPUT"
unregister_port "$RIGHT_LOCAL_INPUT"
unregister_port "$LEFT_LOCAL_OUTPUT"
unregister_port "$RIGHT_LOCAL_OUTPUT"

sleep 1

log "Starting zynqGrabber on ${REMOTE_RUN_SERVER}"

if ! yarp run --on "$REMOTE_RUN_SERVER" \
    --as "$REMOTE_COMMAND_TAG" \
    --cmd "$REMOTE_COMMAND" >&4 2>&4; then
    error "Could not start zynqGrabber on $REMOTE_RUN_SERVER."
    exit 1
fi

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

status "Event-camera pipeline is running. Press Ctrl+C to stop."

log "Local outputs:"
log "  $LEFT_LOCAL_OUTPUT"
log "  $RIGHT_LOCAL_OUTPUT"

wait "$READER_PID"