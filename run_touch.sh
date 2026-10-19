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

SKIN_GUI_EXECUTABLE="${SKIN_GUI_EXECUTABLE:-iCubSkinGui}"
SKIN_GUI_CONFIG="${SKIN_GUI_CONFIG:-right_hand_V2_1.ini}"
GUI_INPUT_PORT="${GUI_INPUT_PORT:-/skinGui/right_hand:i}"

REMOTE_RUN_SERVER="${REMOTE_RUN_SERVER:-/icub-head}"
REMOTE_SKINMANAGER_TAG="${REMOTE_SKINMANAGER_TAG:-primi-skinmanager}"
REMOTE_SKINMANAGER_COMMAND="${REMOTE_SKINMANAGER_COMMAND:-skinManager --from skinManAll.ini --context skinGui --binarization false}"

REMOTE_CARRIER="${REMOTE_CARRIER:-tcp}"
LOCAL_CARRIER="${LOCAL_CARRIER:-shmem}"
SKIN_WAIT_SECONDS="${SKIN_WAIT_SECONDS:-60}"
READER_WAIT_SECONDS="${READER_WAIT_SECONDS:-15}"
GUI_WAIT_SECONDS="${GUI_WAIT_SECONDS:-15}"
STOP_REMOTE_SKINMANAGER_ON_EXIT="${STOP_REMOTE_SKINMANAGER_ON_EXIT:-0}"

READER_PID=""
GUI_PID=""
REMOTE_SKINMANAGER_STARTED=0
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

stop_local_processes()
{
    if [[ -n "$GUI_PID" ]] && kill -0 "$GUI_PID" 2>/dev/null; then
        kill -TERM "$GUI_PID" 2>/dev/null || true
        wait "$GUI_PID" 2>/dev/null || true
    fi

    if [[ -n "$READER_PID" ]] && kill -0 "$READER_PID" 2>/dev/null; then
        kill -TERM "$READER_PID" 2>/dev/null || true
        wait "$READER_PID" 2>/dev/null || true
    fi

    pkill -x touch_reader 2>/dev/null || true
    pkill -x "$(basename "$SKIN_GUI_EXECUTABLE")" 2>/dev/null || true

    GUI_PID=""
    READER_PID=""
}

stop_remote_skinmanager()
{
    yarp run --on "$REMOTE_RUN_SERVER" \
        --sigterm "$REMOTE_SKINMANAGER_TAG" >/dev/null 2>&1 || true
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

    yarp disconnect "$LEFT_COMPENSATED_SOURCE" "$LEFT_LOCAL_INPUT" \
        >/dev/null 2>&1 || true
    yarp disconnect "$RIGHT_COMPENSATED_SOURCE" "$RIGHT_LOCAL_INPUT" \
        >/dev/null 2>&1 || true
    yarp disconnect "$RIGHT_LOCAL_OUTPUT" "$GUI_INPUT_PORT" \
        >/dev/null 2>&1 || true

    log "Stopping local skin processes"
    stop_local_processes

    if (( REMOTE_SKINMANAGER_STARTED )) &&
       [[ "$STOP_REMOTE_SKINMANAGER_ON_EXIT" == "1" ]]; then
        log "Stopping remote skinManager"
        stop_remote_skinmanager
    fi
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if ! yarp where >/dev/null 2>&1; then
    error "The YARP name server is not reachable."
    exit 1
fi

if [[ ! -x "$READER_EXECUTABLE" ]]; then
    error "Touch reader not found: $READER_EXECUTABLE"
    error "Build it with: cmake --build \"$PROJECT_DIR/build\" -j\"\$(nproc)\""
    exit 1
fi

if ! command -v "$SKIN_GUI_EXECUTABLE" >/dev/null 2>&1; then
    error "Skin GUI executable not found: $SKIN_GUI_EXECUTABLE"
    exit 1
fi

if ! port_exists "$REMOTE_RUN_SERVER"; then
    error "YARP run server is not available: $REMOTE_RUN_SERVER"
    exit 1
fi

if ! port_exists "$LEFT_RAW_PORT" || ! port_exists "$RIGHT_RAW_PORT"; then
    error "The raw left and right hand skin streams must be available first."
    error "Missing ports:"
    port_exists "$LEFT_RAW_PORT" || error "  $LEFT_RAW_PORT"
    port_exists "$RIGHT_RAW_PORT" || error "  $RIGHT_RAW_PORT"
    exit 1
fi

log "Stopping previous local skin processes"
stop_local_processes

log "Removing stale local skin port registrations"
unregister_port "$LEFT_LOCAL_INPUT"
unregister_port "$RIGHT_LOCAL_INPUT"
unregister_port "$LEFT_LOCAL_OUTPUT"
unregister_port "$RIGHT_LOCAL_OUTPUT"
unregister_port "$GUI_INPUT_PORT"

sleep 1

if port_exists "$LEFT_COMPENSATED_SOURCE" &&
   port_exists "$RIGHT_COMPENSATED_SOURCE"; then
    log "Using the existing compensated skin streams"
elif port_exists "$LEFT_COMPENSATED_SOURCE" ||
     port_exists "$RIGHT_COMPENSATED_SOURCE"; then
    error "Only one compensated hand port is available."
    error "Stop the stale skinManager instance and run this script again."
    exit 1
else
    log "Starting skinManager on ${REMOTE_RUN_SERVER}"
    stop_remote_skinmanager

    yarp run --on "$REMOTE_RUN_SERVER" \
        --as "$REMOTE_SKINMANAGER_TAG" \
        --cmd "$REMOTE_SKINMANAGER_COMMAND"

    REMOTE_SKINMANAGER_STARTED=1

    wait_for_port "$LEFT_COMPENSATED_SOURCE" "$SKIN_WAIT_SECONDS" \
        "left compensated skin source"
    wait_for_port "$RIGHT_COMPENSATED_SOURCE" "$SKIN_WAIT_SECONDS" \
        "right compensated skin source"
fi

log "Starting local touch reader"
"$READER_EXECUTABLE" &
READER_PID=$!

wait_for_port "$LEFT_LOCAL_INPUT" "$READER_WAIT_SECONDS" \
    "left touch-reader input"
wait_for_port "$RIGHT_LOCAL_INPUT" "$READER_WAIT_SECONDS" \
    "right touch-reader input"
wait_for_port "$LEFT_LOCAL_OUTPUT" "$READER_WAIT_SECONDS" \
    "left local compensated output"
wait_for_port "$RIGHT_LOCAL_OUTPUT" "$READER_WAIT_SECONDS" \
    "right local compensated output"

log "Connecting compensated skin streams to touch_reader"
yarp connect "$LEFT_COMPENSATED_SOURCE" "$LEFT_LOCAL_INPUT" "$REMOTE_CARRIER"
yarp connect "$RIGHT_COMPENSATED_SOURCE" "$RIGHT_LOCAL_INPUT" "$REMOTE_CARRIER"

log "Starting right-hand skin GUI"
"$SKIN_GUI_EXECUTABLE" --from "$SKIN_GUI_CONFIG" --useCalibration &
GUI_PID=$!

wait_for_port "$GUI_INPUT_PORT" "$GUI_WAIT_SECONDS" \
    "right-hand skin GUI input"

log "Connecting the local right-hand output to the GUI"
yarp connect "$RIGHT_LOCAL_OUTPUT" "$GUI_INPUT_PORT" "$LOCAL_CARRIER"

log "Skin pipeline is running"
log "Local outputs:"
log "  $LEFT_LOCAL_OUTPUT"
log "  $RIGHT_LOCAL_OUTPUT"
log "Press Ctrl+C to stop"

wait "$READER_PID"
