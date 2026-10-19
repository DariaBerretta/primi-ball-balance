#!/usr/bin/env bash

set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

READER_EXECUTABLE="${READER_EXECUTABLE:-$PROJECT_DIR/build/rgbCam_reader}"
LOCAL_INPUT_PORT="${LOCAL_INPUT_PORT:-/primi-ball-balance/rgbCam/stereo:i}"
CAMERA_OUTPUT_PORT="${CAMERA_OUTPUT_PORT:-/grabber}"
CAMERA_RPC_PORT="${CAMERA_RPC_PORT:-${CAMERA_OUTPUT_PORT}/rpc}"
CARRIER="${CARRIER:-fast_tcp}"

LEFT_RAW_OUTPUT="${LEFT_RAW_OUTPUT:-/primi-ball-balance/rgbCam/left/raw:o}"
RIGHT_RAW_OUTPUT="${RIGHT_RAW_OUTPUT:-/primi-ball-balance/rgbCam/right/raw:o}"
LEFT_PROCESSED_OUTPUT="${LEFT_PROCESSED_OUTPUT:-/primi-ball-balance/rgbCam/left/processed:o}"
RIGHT_PROCESSED_OUTPUT="${RIGHT_PROCESSED_OUTPUT:-/primi-ball-balance/rgbCam/right/processed:o}"
LEFT_DETECTION_OUTPUT="${LEFT_DETECTION_OUTPUT:-/primi-ball-balance/rgbCam/left/detection:o}"
RIGHT_DETECTION_OUTPUT="${RIGHT_DETECTION_OUTPUT:-/primi-ball-balance/rgbCam/right/detection:o}"

REMOTE_RUN_SERVER="${REMOTE_RUN_SERVER:-/icub-ultrascale}"
REMOTE_COMMAND_TAG="${REMOTE_COMMAND_TAG:-rgbcommand}"
REMOTE_CAMERA_COMMAND="${REMOTE_CAMERA_COMMAND:-/home/icub/yarp-install/bin/yarpdev --verbose --from /home/icub/yarp-device-ultrapython/ini/hiultra.ini}"

REMOTE_CAMERA_CLI="${REMOTE_CAMERA_CLI:-/home/icub/yarp-device-ultrapython/build/bin/ultrapythoncli}"
REMOTE_SETTING_TAG_PREFIX="${REMOTE_SETTING_TAG_PREFIX:-rgbsetting}"
CAMERA_SETTING_DELAY="${CAMERA_SETTING_DELAY:-1.5}"

APPLY_CAMERA_SETTINGS="${APPLY_CAMERA_SETTINGS:-1}"
CAMERA_EXPOSURE="${CAMERA_EXPOSURE:-25}"
CAMERA_GAIN="${CAMERA_GAIN:-2}"
CAMERA_BRIGHTNESS="${CAMERA_BRIGHTNESS:-200}"
CAMERA_CONTRAST="${CAMERA_CONTRAST:-50}"
CAMERA_RED_GAIN="${CAMERA_RED_GAIN:-50}"
CAMERA_GREEN_GAIN="${CAMERA_GREEN_GAIN:-50}"
CAMERA_BLUE_GAIN="${CAMERA_BLUE_GAIN:-50}"

CAMERA_WAIT_SECONDS="${CAMERA_WAIT_SECONDS:-60}"
READER_WAIT_SECONDS="${READER_WAIT_SECONDS:-15}"
STOP_REMOTE_CAMERA_ON_EXIT="${STOP_REMOTE_CAMERA_ON_EXIT:-1}"

READER_PID=""
REMOTE_CAMERA_STARTED=0
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

        if [[ -n "$READER_PID" ]] && ! kill -0 "$READER_PID" 2>/dev/null; then
            error "The local RGB reader exited while waiting for ${port_name}."
            return 1
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
    yarp name unregister "$1" >/dev/null 2>&1 || true
}

apply_camera_setting()
{
    local label="$1"
    local code="$2"
    local value="$3"
    local tag="${REMOTE_SETTING_TAG_PREFIX}-${code}"

    log "Setting ${label}=${value}"

    unregister_port /xxx

    yarp run --on "$REMOTE_RUN_SERVER" \
        --sigterm "$tag" >/dev/null 2>&1 || true

    if ! yarp run --on "$REMOTE_RUN_SERVER" \
        --as "$tag" \
        --cmd "$REMOTE_CAMERA_CLI --remote $CAMERA_OUTPUT_PORT --set ${code}=${value}"; then
        error "Could not launch the command for ${label}."
        return 1
    fi

    sleep "$CAMERA_SETTING_DELAY"
    unregister_port /xxx
}

apply_camera_settings()
{
    log "Applying RGB camera settings on ${REMOTE_RUN_SERVER}"

    apply_camera_setting "exposure" 61 "$CAMERA_EXPOSURE"
    apply_camera_setting "gain" 60 "$CAMERA_GAIN"
    apply_camera_setting "brightness" 62 "$CAMERA_BRIGHTNESS"
    apply_camera_setting "contrast" 75 "$CAMERA_CONTRAST"
    apply_camera_setting "red gain" 63 "$CAMERA_RED_GAIN"
    apply_camera_setting "green gain" 65 "$CAMERA_GREEN_GAIN"
    apply_camera_setting "blue gain" 64 "$CAMERA_BLUE_GAIN"

    log "RGB camera settings applied"
}

cleanup()
{
    if (( CLEANUP_DONE )); then
        return
    fi

    CLEANUP_DONE=1
    printf '\n'

    yarp disconnect "$CAMERA_OUTPUT_PORT" "$LOCAL_INPUT_PORT" \
        >/dev/null 2>&1 || true

    log "Stopping local RGB reader"
    stop_local_reader

    if (( REMOTE_CAMERA_STARTED )) &&
       [[ "$STOP_REMOTE_CAMERA_ON_EXIT" == "1" ]]; then
        log "Stopping remote RGB camera"
        stop_remote_camera
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
    error "RGB reader executable not found: $READER_EXECUTABLE"
    error "Build it with: cmake --build \"$PROJECT_DIR/build\" -j\"\$(nproc)\""
    exit 1
fi

if ! port_exists "$REMOTE_RUN_SERVER"; then
    error "YARP run server is not available: $REMOTE_RUN_SERVER"
    exit 1
fi

log "Stopping previous RGB processes"
stop_local_reader
stop_remote_camera

log "Removing stale RGB port registrations"
unregister_port "$CAMERA_OUTPUT_PORT"
unregister_port "$CAMERA_RPC_PORT"
unregister_port "$LOCAL_INPUT_PORT"
unregister_port "$LEFT_RAW_OUTPUT"
unregister_port "$RIGHT_RAW_OUTPUT"
unregister_port "$LEFT_PROCESSED_OUTPUT"
unregister_port "$RIGHT_PROCESSED_OUTPUT"
unregister_port "$LEFT_DETECTION_OUTPUT"
unregister_port "$RIGHT_DETECTION_OUTPUT"
unregister_port /xxx

sleep 1

log "Starting RGB camera on ${REMOTE_RUN_SERVER}"

if ! yarp run --on "$REMOTE_RUN_SERVER" \
    --as "$REMOTE_COMMAND_TAG" \
    --cmd "$REMOTE_CAMERA_COMMAND"; then
    error "Could not start the remote RGB camera."
    exit 1
fi

REMOTE_CAMERA_STARTED=1

wait_for_port "$CAMERA_OUTPUT_PORT" "$CAMERA_WAIT_SECONDS" \
    "RGB camera output"

wait_for_port "$CAMERA_RPC_PORT" "$CAMERA_WAIT_SECONDS" \
    "RGB camera RPC"

if [[ "$APPLY_CAMERA_SETTINGS" == "1" ]]; then
    apply_camera_settings
fi

log "Starting local RGB reader"
"$READER_EXECUTABLE" &
READER_PID=$!

wait_for_port "$LOCAL_INPUT_PORT" "$READER_WAIT_SECONDS" \
    "local RGB reader input"
wait_for_port "$LEFT_RAW_OUTPUT" "$READER_WAIT_SECONDS" \
    "left raw RGB output"
wait_for_port "$RIGHT_RAW_OUTPUT" "$READER_WAIT_SECONDS" \
    "right raw RGB output"
wait_for_port "$LEFT_PROCESSED_OUTPUT" "$READER_WAIT_SECONDS" \
    "left processed RGB output"
wait_for_port "$RIGHT_PROCESSED_OUTPUT" "$READER_WAIT_SECONDS" \
    "right processed RGB output"
wait_for_port "$LEFT_DETECTION_OUTPUT" "$READER_WAIT_SECONDS" \
    "left RGB detection output"
wait_for_port "$RIGHT_DETECTION_OUTPUT" "$READER_WAIT_SECONDS" \
    "right RGB detection output"

log "Connecting ${CAMERA_OUTPUT_PORT} -> ${LOCAL_INPUT_PORT} using ${CARRIER}"

if ! yarp connect "$CAMERA_OUTPUT_PORT" "$LOCAL_INPUT_PORT" "$CARRIER"; then
    error "Could not connect the RGB camera to the local reader."
    exit 1
fi

log "RGB pipeline is running"
log "Camera settings:"
log "  exposure:   ${CAMERA_EXPOSURE} ms"
log "  gain:       ${CAMERA_GAIN}"
log "  brightness: ${CAMERA_BRIGHTNESS}"
log "  contrast:   ${CAMERA_CONTRAST}"
log "  RGB gains:  ${CAMERA_RED_GAIN}/${CAMERA_GREEN_GAIN}/${CAMERA_BLUE_GAIN}"
log "Local outputs:"
log "  $LEFT_RAW_OUTPUT"
log "  $RIGHT_RAW_OUTPUT"
log "  $LEFT_PROCESSED_OUTPUT"
log "  $RIGHT_PROCESSED_OUTPUT"
log "  $LEFT_DETECTION_OUTPUT"
log "  $RIGHT_DETECTION_OUTPUT"
log "Press Ctrl+C to stop"

wait "$READER_PID"