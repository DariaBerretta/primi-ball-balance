#!/usr/bin/env bash

set -Eeuo pipefail

# -----------------------------------------------------------------------------
# Local configuration
# -----------------------------------------------------------------------------
SKIN_GUI_EXECUTABLE="${SKIN_GUI_EXECUTABLE:-iCubSkinGui}"
SKIN_GUI_CONFIG="${SKIN_GUI_CONFIG:-right_hand_V2_1.ini}"
LOCAL_INPUT_PORT="${LOCAL_INPUT_PORT:-/skinGui/right_hand:i}"
CARRIER="${CARRIER:-tcp}"

# -----------------------------------------------------------------------------
# Skin ports
# -----------------------------------------------------------------------------
# The raw port is produced by the robot interface.
RAW_SKIN_PORT="${RAW_SKIN_PORT:-/icub/skin/right_hand}"

# The compensated port is produced by skinManager.
SKIN_OUTPUT_PORT="${SKIN_OUTPUT_PORT:-/icub/skin/right_hand_comp}"

# -----------------------------------------------------------------------------
# Remote skinManager configuration
# -----------------------------------------------------------------------------
REMOTE_SERVER="${REMOTE_SERVER:-/icub-head}"
REMOTE_SKINMANAGER_TAG="${REMOTE_SKINMANAGER_TAG:-primi-skinmanager}"

# skinManAll.ini provides both hand streams. Binarization is disabled so the
# compensated output retains continuous activation magnitudes.
REMOTE_SKINMANAGER_CMD="${REMOTE_SKINMANAGER_CMD:-skinManager --from skinManAll.ini --context skinGui --binarization false}"

SKIN_WAIT_SECONDS="${SKIN_WAIT_SECONDS:-60}"
GUI_WAIT_SECONDS="${GUI_WAIT_SECONDS:-15}"

# skinManager is left running by default because other applications may depend
# on it. Set this to 1 only when this script owns the skinManager process.
STOP_REMOTE_SKINMANAGER_ON_EXIT="${STOP_REMOTE_SKINMANAGER_ON_EXIT:-0}"

GUI_PID=""
REMOTE_SKINMANAGER_STARTED=0
CLEANUP_DONE=0

log()
{
    printf '[INFO] %s\n' "$*"
}

warning()
{
    printf '[WARNING] %s\n' "$*" >&2
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

    return 1
}

cleanup()
{
    if (( CLEANUP_DONE )); then
        return
    fi

    CLEANUP_DONE=1
    printf '\n'

    if port_exists "$SKIN_OUTPUT_PORT" && port_exists "$LOCAL_INPUT_PORT"; then
        log "Disconnecting ${SKIN_OUTPUT_PORT} from ${LOCAL_INPUT_PORT}"
        yarp disconnect "$SKIN_OUTPUT_PORT" "$LOCAL_INPUT_PORT" \
            >/dev/null 2>&1 || true
    fi

    if [[ -n "$GUI_PID" ]] && kill -0 "$GUI_PID" 2>/dev/null; then
        log "Stopping local skin GUI (PID ${GUI_PID})"
        kill -TERM "$GUI_PID" 2>/dev/null || true
        wait "$GUI_PID" 2>/dev/null || true
    fi

    if (( REMOTE_SKINMANAGER_STARTED )) &&
       [[ "$STOP_REMOTE_SKINMANAGER_ON_EXIT" == "1" ]]; then
        log "Stopping skinManager on ${REMOTE_SERVER}"
        yarp run --on "$REMOTE_SERVER" \
            --sigterm "$REMOTE_SKINMANAGER_TAG" >/dev/null 2>&1 || true
    fi
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# -----------------------------------------------------------------------------
# Preliminary checks
# -----------------------------------------------------------------------------
if ! yarp where >/dev/null 2>&1; then
    error "The YARP name server is not reachable."
    exit 1
fi

log "YARP network is available"

if ! port_exists "$REMOTE_SERVER"; then
    error "The yarprun server is not available: ${REMOTE_SERVER}"
    error "Run the robot startup script before this script."
    exit 1
fi

log "Remote yarprun server is available: ${REMOTE_SERVER}"

if ! command -v "$SKIN_GUI_EXECUTABLE" >/dev/null 2>&1; then
    error "Local executable not found on PATH: ${SKIN_GUI_EXECUTABLE}"
    exit 1
fi

if port_exists "$LOCAL_INPUT_PORT"; then
    error "The local GUI input port is already open: ${LOCAL_INPUT_PORT}"
    error "Stop the existing iCubSkinGui process before running this script."
    exit 1
fi

if ! port_exists "$RAW_SKIN_PORT"; then
    error "The raw skin stream is not available: ${RAW_SKIN_PORT}"
    error "The robot interface and skin boards must be started first."
    exit 1
fi

log "Raw skin stream is available: ${RAW_SKIN_PORT}"

# -----------------------------------------------------------------------------
# Start skinManager when its compensated output is not already available.
# -----------------------------------------------------------------------------
if port_exists "$SKIN_OUTPUT_PORT"; then
    log "Compensated skin output is already available: ${SKIN_OUTPUT_PORT}"
    warning "The existing skinManager configuration is being reused."
    warning "This script cannot guarantee that its binarization is disabled."
else
    log "Starting skinManager on ${REMOTE_SERVER}"

    # Remove a stale yarprun process using the same tag, if one exists.
    yarp run --on "$REMOTE_SERVER" \
        --sigterm "$REMOTE_SKINMANAGER_TAG" >/dev/null 2>&1 || true

    if ! yarp run --on "$REMOTE_SERVER" \
        --as "$REMOTE_SKINMANAGER_TAG" \
        --cmd "$REMOTE_SKINMANAGER_CMD"; then
        error "Could not start skinManager on ${REMOTE_SERVER}."
        exit 1
    fi

    REMOTE_SKINMANAGER_STARTED=1

    if ! wait_for_port "$SKIN_OUTPUT_PORT" "$SKIN_WAIT_SECONDS" \
        "compensated skin output"; then
        error "Timed out after ${SKIN_WAIT_SECONDS}s waiting for ${SKIN_OUTPUT_PORT}."
        error "Inspect remote processes with:"
        error "  yarp run --on ${REMOTE_SERVER} --ps"
        exit 1
    fi
fi

log "Compensated skin output is available: ${SKIN_OUTPUT_PORT}"

# -----------------------------------------------------------------------------
# Start the local GUI in compensated-data mode.
# -----------------------------------------------------------------------------
log "Starting local GUI: ${SKIN_GUI_EXECUTABLE} --from ${SKIN_GUI_CONFIG} --useCalibration"

"$SKIN_GUI_EXECUTABLE" \
    --from "$SKIN_GUI_CONFIG" \
    --useCalibration &

GUI_PID=$!

if ! wait_for_port "$LOCAL_INPUT_PORT" "$GUI_WAIT_SECONDS" \
    "local skin GUI input"; then
    if ! kill -0 "$GUI_PID" 2>/dev/null; then
        error "${SKIN_GUI_EXECUTABLE} exited before opening ${LOCAL_INPUT_PORT}."
    else
        error "Timed out after ${GUI_WAIT_SECONDS}s waiting for ${LOCAL_INPUT_PORT}."
    fi

    exit 1
fi

# -----------------------------------------------------------------------------
# Connect the compensated skin stream to the GUI.
# -----------------------------------------------------------------------------
log "Connecting ${SKIN_OUTPUT_PORT} -> ${LOCAL_INPUT_PORT} using ${CARRIER}"

if ! yarp connect "$SKIN_OUTPUT_PORT" "$LOCAL_INPUT_PORT" "$CARRIER"; then
    error "Could not connect ${SKIN_OUTPUT_PORT} to ${LOCAL_INPUT_PORT}."
    error "Try manually:"
    error "  yarp connect ${SKIN_OUTPUT_PORT} ${LOCAL_INPUT_PORT} ${CARRIER}"
    exit 1
fi

log "Skin pipeline is running"
log "Close the GUI or press Ctrl+C to stop this launcher"

# Exit when the GUI is closed. The cleanup trap removes the connection.
wait "$GUI_PID" || true
GUI_PID=""

log "Local skin GUI closed"

git add -A -- CMakeLists.txt TODOs.md run_perception.sh run_rgbCam.sh run_touch.sh setup_cluster.sh src/Touch_reader.cpp src/rgbCam_reader.cpp .monitoring/logs/detections_terminal.log .monitoring/logs/processed_left_view.log .monitoring/logs/processed_right_view.log .monitoring/logs/raw_left_view.log .monitoring/logs/raw_right_view.log .monitoring/logs/terminal_four.log .monitoring/logs/terminal_three.log .monitoring/logs/terminal_two.log .monitoring/logs/vframer.log .monitoring/logs/yarpscope.log config/monitoring.env run_eventCam.sh scripts/print_rgb_detections.sh scripts/run_monitor.sh scripts/stop_monitor.sh