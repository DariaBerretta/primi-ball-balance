#!/usr/bin/env bash

set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

#============ Local RGB reader ============

READER_EXECUTABLE="${READER_EXECUTABLE:-$PROJECT_DIR/build/rgbCam_reader}"
LOCAL_INPUT_PORT="${LOCAL_INPUT_PORT:-/primi-ball-balance/rgbCam/stereo:i}"
CAMERA_OUTPUT_PORT="${CAMERA_OUTPUT_PORT:-/grabber}"
CARRIER="${CARRIER:-tcp}"

#============ Remote RGB camera ============

REMOTE_RUN_SERVER="${REMOTE_RUN_SERVER:-/icub-ultrascale}"
REMOTE_COMMAND_TAG="${REMOTE_COMMAND_TAG:-rgbcommand}"
REMOTE_CAMERA_COMMAND="${REMOTE_CAMERA_COMMAND:-/home/icub/yarp-install/bin/yarpdev --verbose --from /home/icub/yarp-device-ultrapython/ini/hiultra.ini}"

CAMERA_WAIT_SECONDS="${CAMERA_WAIT_SECONDS:-60}"
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

    pkill -x rgbCam_reader 2>/dev/null || true
    READER_PID=""
}

stop_remote_camera()
{
    yarp run --on "$REMOTE_RUN_SERVER" \
        --sigterm "$REMOTE_COMMAND_TAG" >/dev/null 2>&1 || true
}

unregister_port()
{
    local port_name="$1"

    if port_exists "$port_name"; then
        yarp name unregister "$port_name" >/dev/null 2>&1 || true
    fi
}

cleanup()
{
    if (( CLEANUP_DONE )); then
        return
    fi

    CLEANUP_DONE=1
    printf '\n'

    if port_exists "$CAMERA_OUTPUT_PORT" &&
       port_exists "$LOCAL_INPUT_PORT"; then
        log "Disconnecting ${CAMERA_OUTPUT_PORT} from ${LOCAL_INPUT_PORT}"
        yarp disconnect "$CAMERA_OUTPUT_PORT" "$LOCAL_INPUT_PORT" \
            >/dev/null 2>&1 || true
    fi

    log "Stopping local RGB reader"
    stop_local_reader

    log "Stopping remote RGB camera"
    stop_remote_camera
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

#============ Preflight checks ============

if ! yarp where >/dev/null 2>&1; then
    error "The YARP name server is not reachable."
    exit 1
fi

log "YARP network is available"

if [[ ! -x "$READER_EXECUTABLE" ]]; then
    error "RGB reader executable not found: $READER_EXECUTABLE"
    error "Build it with: cmake --build \"$PROJECT_DIR/build\" -j\"\$(nproc)\""
    exit 1
fi

if ! port_exists "$REMOTE_RUN_SERVER"; then
    error "YARP run server is not available: $REMOTE_RUN_SERVER"
    error "Start yarp run --server $REMOTE_RUN_SERVER on the Ultrascale."
    exit 1
fi

#============ Stop previous pipeline ============

log "Stopping previous RGB processes"

stop_local_reader
stop_remote_camera

sleep 1

#============ Remove stale port registrations ============

log "Removing previous RGB port registrations"

unregister_port "$CAMERA_OUTPUT_PORT"
unregister_port "$LOCAL_INPUT_PORT"

sleep 1

#============ Start remote camera ============

log "Starting RGB camera on ${REMOTE_RUN_SERVER}"

if ! yarp run --on "$REMOTE_RUN_SERVER" \
    --as "$REMOTE_COMMAND_TAG" \
    --cmd "$REMOTE_CAMERA_COMMAND"; then
    error "Could not start the remote RGB camera command."
    exit 1
fi

if ! wait_for_port "$CAMERA_OUTPUT_PORT" "$CAMERA_WAIT_SECONDS" \
    "RGB camera output"; then
    exit 1
fi

#============ Start local RGB reader ============

log "Starting local RGB reader: ${READER_EXECUTABLE}"

"$READER_EXECUTABLE" &
READER_PID=$!

if ! wait_for_port "$LOCAL_INPUT_PORT" "$READER_WAIT_SECONDS" \
    "local RGB reader input"; then
    if ! kill -0 "$READER_PID" 2>/dev/null; then
        error "rgbCam_reader exited before opening ${LOCAL_INPUT_PORT}."
    fi

    exit 1
fi

#============ Connect pipeline ============

log "Connecting ${CAMERA_OUTPUT_PORT} -> ${LOCAL_INPUT_PORT} using ${CARRIER}"

if ! yarp connect "$CAMERA_OUTPUT_PORT" "$LOCAL_INPUT_PORT" "$CARRIER"; then
    error "Could not connect the RGB camera to rgbCam_reader."
    error "Try manually:"
    error "  yarp connect ${CAMERA_OUTPUT_PORT} ${LOCAL_INPUT_PORT} ${CARRIER}"
    exit 1
fi

log "RGB pipeline is running"
log "${CAMERA_OUTPUT_PORT} -> ${LOCAL_INPUT_PORT}"
log "Press Ctrl+C to stop"

wait "$READER_PID"