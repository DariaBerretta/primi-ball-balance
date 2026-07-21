#!/usr/bin/env bash

set -Eeuo pipefail

# Local program and ports.
READER_EXECUTABLE="${READER_EXECUTABLE:-./build/rgbCam_reader}"
LOCAL_INPUT_PORT="${LOCAL_INPUT_PORT:-/primi-ball-balance/rgbCam:i}"
CAMERA_OUTPUT_PORT="${CAMERA_OUTPUT_PORT:-/grabber}"
CARRIER="${CARRIER:-tcp}"

# Ultrascale SSH configuration.
REMOTE_SSH_TARGET="${REMOTE_SSH_TARGET:-icub@icub-ultrascale}"
REMOTE_PID_FILE="${REMOTE_PID_FILE:-/tmp/primi-rgb-cameras.pid}"
REMOTE_YARPDEV="${REMOTE_YARPDEV:-/home/icub/yarp-install/bin/yarpdev}"
REMOTE_CONFIG_DIR="${REMOTE_CONFIG_DIR:-/home/icub/yarp-device-ultrapython/ini}"
REMOTE_CONFIG_NAME="${REMOTE_CONFIG_NAME:-hiultra.ini}"

CAMERA_WAIT_SECONDS="${CAMERA_WAIT_SECONDS:-60}"
READER_WAIT_SECONDS="${READER_WAIT_SECONDS:-15}"
STOP_REMOTE_CAMERA_ON_EXIT="${STOP_REMOTE_CAMERA_ON_EXIT:-1}"

READER_PID=""
REMOTE_SSH_PID=""
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

        if (( REMOTE_CAMERA_STARTED )) && [[ -n "$REMOTE_SSH_PID" ]] && ! kill -0 "$REMOTE_SSH_PID" 2>/dev/null; then
            error "The RGB camera process on the Ultrascale exited early."
            return 1
        fi

        sleep 0.2
    done

    return 1
}

stop_remote_camera()
{
    ssh -n \
        -o BatchMode=yes \
        -o ConnectTimeout=5 \
        "$REMOTE_SSH_TARGET" \
        "/bin/bash -c 'if [ -r \"$REMOTE_PID_FILE\" ]; then pid=\$(cat \"$REMOTE_PID_FILE\"); kill -TERM \"\$pid\" 2>/dev/null || true; rm -f \"$REMOTE_PID_FILE\"; fi'" \
        >/dev/null 2>&1 || true
}

cleanup()
{
    if (( CLEANUP_DONE )); then
        return
    fi
    CLEANUP_DONE=1

    printf '\n'

    if port_exists "$CAMERA_OUTPUT_PORT" && port_exists "$LOCAL_INPUT_PORT"; then
        log "Disconnecting ${CAMERA_OUTPUT_PORT} from ${LOCAL_INPUT_PORT}"
        yarp disconnect "$CAMERA_OUTPUT_PORT" "$LOCAL_INPUT_PORT" >/dev/null 2>&1 || true
    fi

    if [[ -n "$READER_PID" ]] && kill -0 "$READER_PID" 2>/dev/null; then
        log "Stopping local RGB reader (PID ${READER_PID})"
        kill -TERM "$READER_PID" 2>/dev/null || true
        wait "$READER_PID" 2>/dev/null || true
    fi

    if (( REMOTE_CAMERA_STARTED )) && [[ "$STOP_REMOTE_CAMERA_ON_EXIT" == "1" ]]; then
        log "Stopping RGB camera driver on ${REMOTE_SSH_TARGET}"
        stop_remote_camera
    fi

    if [[ -n "$REMOTE_SSH_PID" ]] && kill -0 "$REMOTE_SSH_PID" 2>/dev/null; then
        kill -TERM "$REMOTE_SSH_PID" 2>/dev/null || true
        wait "$REMOTE_SSH_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if ! yarp where >/dev/null 2>&1; then
    error "The YARP name server is not reachable."
    exit 1
fi
log "YARP network is available"

if [[ ! -x "$READER_EXECUTABLE" ]]; then
    error "RGB reader executable not found or not executable: ${READER_EXECUTABLE}"
    error "Build it with: cmake --build build -j\"\$(nproc)\""
    exit 1
fi

if port_exists "$LOCAL_INPUT_PORT"; then
    error "The local input port is already open: ${LOCAL_INPUT_PORT}"
    error "Stop the old rgbCam_reader process before running this script."
    exit 1
fi

if port_exists "$CAMERA_OUTPUT_PORT"; then
    log "RGB camera output is already available: ${CAMERA_OUTPUT_PORT}"
else
    log "Checking passwordless SSH access to ${REMOTE_SSH_TARGET}"
    if ! ssh -n -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE_SSH_TARGET" true; then
        error "Passwordless SSH access is not configured for ${REMOTE_SSH_TARGET}."
        error "Run once from the laptop: ssh-copy-id ${REMOTE_SSH_TARGET}"
        exit 1
    fi

    # Stop a stale process started by an earlier run of this script.
    stop_remote_camera

    log "Starting the RGB camera driver on ${REMOTE_SSH_TARGET}"

    remote_command=$(cat <<EOF_REMOTE
/usr/bin/env \\
    HOME=/home/icub \\
    USER=icub \\
    SHELL=/bin/bash \\
    /bin/bash -ic '
        export PATH=/home/icub/yarp-install/bin:\$PATH
        export LD_LIBRARY_PATH=/home/icub/yarp-install/lib:\${LD_LIBRARY_PATH:-}
        export YARP_DATA_DIRS=/home/icub/yarp-install/share/yarp:/home/icub/yarp-install/share/event-driven
        cd "$REMOTE_CONFIG_DIR"
        echo \$\$ > "$REMOTE_PID_FILE"
        exec "$REMOTE_YARPDEV" --from "$REMOTE_CONFIG_NAME"
    '
EOF_REMOTE
)

    # -tt reproduces the interactive terminal in which the manual command worked.
    # -n prevents SSH from consuming this script's standard input.
    ssh -n -tt \
        -o BatchMode=yes \
        -o ServerAliveInterval=5 \
        -o ServerAliveCountMax=3 \
        "$REMOTE_SSH_TARGET" \
        "$remote_command" &

    REMOTE_SSH_PID=$!
    REMOTE_CAMERA_STARTED=1

    if ! wait_for_port "$CAMERA_OUTPUT_PORT" "$CAMERA_WAIT_SECONDS" "RGB camera output"; then
        error "The Ultrascale RGB driver did not create ${CAMERA_OUTPUT_PORT}."
        exit 1
    fi
fi

log "RGB camera output is available: ${CAMERA_OUTPUT_PORT}"

log "Starting local reader: ${READER_EXECUTABLE}"
"$READER_EXECUTABLE" &
READER_PID=$!

if ! wait_for_port "$LOCAL_INPUT_PORT" "$READER_WAIT_SECONDS" "local RGB reader input"; then
    if ! kill -0 "$READER_PID" 2>/dev/null; then
        error "rgbCam_reader exited before opening ${LOCAL_INPUT_PORT}."
    else
        error "Timed out waiting for ${LOCAL_INPUT_PORT}."
    fi
    exit 1
fi

log "Connecting ${CAMERA_OUTPUT_PORT} -> ${LOCAL_INPUT_PORT} using ${CARRIER}"
if ! yarp connect "$CAMERA_OUTPUT_PORT" "$LOCAL_INPUT_PORT" "$CARRIER"; then
    error "Could not connect ${CAMERA_OUTPUT_PORT} to ${LOCAL_INPUT_PORT}."
    exit 1
fi

log "RGB pipeline is running"
log "Press Ctrl+C to stop"

wait "$READER_PID"