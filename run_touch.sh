#!/usr/bin/env bash

set -Eeuo pipefail

# -----------------------------------------------------------------------------
# IMPORTANT - manual steps NOT covered by this script
# -----------------------------------------------------------------------------
# 1. The usual "wake up the robot" procedure.
# 2. Physically switching on the Motor button.
# 3. Waiting for the boards to wake up.
# These are physical/safety steps on the real robot and are intentionally left
# out of this script - run them yourself before launching it.

# -----------------------------------------------------------------------------
# Local configuration
# -----------------------------------------------------------------------------
SKIN_GUI_EXECUTABLE="${SKIN_GUI_EXECUTABLE:-iCubSkinGui}"
SKIN_GUI_CONFIG="${SKIN_GUI_CONFIG:-right_hand_V2_1.ini}"
LOCAL_INPUT_PORT="${LOCAL_INPUT_PORT:-/skinGui/right_hand:i}"
SKIN_OUTPUT_PORT="${SKIN_OUTPUT_PORT:-/icub/skin/right_hand}"
CARRIER="${CARRIER:-tcp}"

# -----------------------------------------------------------------------------
# icub-head configuration
# -----------------------------------------------------------------------------
REMOTE_SERVER="${REMOTE_SERVER:-/icub-head}"

REMOTE_ROBOTINTERFACE_TAG="${REMOTE_ROBOTINTERFACE_TAG:-primi-robotinterface}"
REMOTE_ROBOTINTERFACE_CMD="${REMOTE_ROBOTINTERFACE_CMD:-yarprobotinterface}"

REMOTE_SKINMANAGER_TAG="${REMOTE_SKINMANAGER_TAG:-primi-skinmanager}"
REMOTE_SKINMANAGER_CMD="${REMOTE_SKINMANAGER_CMD:-skinManager}"

# yarprobotinterface has no single deterministic "ready" port to poll, so we
# just wait a fixed amount of time and let the user confirm with
# FirmwareUpdater (see below). Adjust if your boards need longer.
ROBOTINTERFACE_SETTLE_SECONDS="${ROBOTINTERFACE_SETTLE_SECONDS:-15}"

# FirmwareUpdater -a is an interactive visual check ("did all the boards wake
# up?"), not something with a port to poll - we only offer to launch it, we
# never gate the rest of the script on it.
RUN_FIRMWARE_CHECK="${RUN_FIRMWARE_CHECK:-1}"
REMOTE_FIRMWAREUPDATER_TAG="${REMOTE_FIRMWAREUPDATER_TAG:-primi-firmwarecheck}"
REMOTE_FIRMWAREUPDATER_CMD="${REMOTE_FIRMWAREUPDATER_CMD:-FirmwareUpdater -a}"

# Stopping yarprobotinterface or skinManager on exit is DISABLED by default:
# other modules (motion control, etc.) likely depend on them staying up.
# Only enable these if you know this script owns the full session.
STOP_REMOTE_SKINMANAGER_ON_EXIT="${STOP_REMOTE_SKINMANAGER_ON_EXIT:-0}"
STOP_REMOTE_ROBOTINTERFACE_ON_EXIT="${STOP_REMOTE_ROBOTINTERFACE_ON_EXIT:-0}"

SKIN_WAIT_SECONDS="${SKIN_WAIT_SECONDS:-60}"
GUI_WAIT_SECONDS="${GUI_WAIT_SECONDS:-15}"

GUI_PID=""
REMOTE_ROBOTINTERFACE_STARTED=0
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
        yarp disconnect "$SKIN_OUTPUT_PORT" "$LOCAL_INPUT_PORT" >/dev/null 2>&1 || true
    fi

    if [[ -n "$GUI_PID" ]] && kill -0 "$GUI_PID" 2>/dev/null; then
        log "Stopping local skin GUI (PID ${GUI_PID})"
        kill -TERM "$GUI_PID" 2>/dev/null || true
        wait "$GUI_PID" 2>/dev/null || true
    fi

    if (( REMOTE_SKINMANAGER_STARTED )) && [[ "$STOP_REMOTE_SKINMANAGER_ON_EXIT" == "1" ]]; then
        log "Stopping skinManager on ${REMOTE_SERVER}"
        yarp run --on "$REMOTE_SERVER" --sigterm "$REMOTE_SKINMANAGER_TAG" >/dev/null 2>&1 || true
    fi

    if (( REMOTE_ROBOTINTERFACE_STARTED )) && [[ "$STOP_REMOTE_ROBOTINTERFACE_ON_EXIT" == "1" ]]; then
        log "Stopping yarprobotinterface on ${REMOTE_SERVER}"
        yarp run --on "$REMOTE_SERVER" --sigterm "$REMOTE_ROBOTINTERFACE_TAG" >/dev/null 2>&1 || true
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
    error "The icub-head yarprun server is not available: ${REMOTE_SERVER}"
    exit 1
fi
log "icub-head server is available: ${REMOTE_SERVER}"

if ! command -v "$SKIN_GUI_EXECUTABLE" >/dev/null 2>&1; then
    error "Local executable not found on PATH: ${SKIN_GUI_EXECUTABLE}"
    exit 1
fi

if port_exists "$LOCAL_INPUT_PORT"; then
    error "The local input port is already open: ${LOCAL_INPUT_PORT}"
    error "Stop the old iCubSkinGui process before running this script."
    exit 1
fi

# -----------------------------------------------------------------------------
# Start yarprobotinterface remotely, unless it is already running.
# -----------------------------------------------------------------------------
if port_exists "/icub/skin/right_hand/rpc:i" 2>/dev/null || port_exists "${SKIN_OUTPUT_PORT}/rpc:i" 2>/dev/null; then
    log "Boards already appear to be up; skipping yarprobotinterface start."
else
    log "Starting yarprobotinterface on ${REMOTE_SERVER}"

    yarp run --on "$REMOTE_SERVER" --sigterm "$REMOTE_ROBOTINTERFACE_TAG" >/dev/null 2>&1 || true

    yarp run \
        --on "$REMOTE_SERVER" \
        --as "$REMOTE_ROBOTINTERFACE_TAG" \
        --cmd "$REMOTE_ROBOTINTERFACE_CMD"

    REMOTE_ROBOTINTERFACE_STARTED=1

    log "Waiting ${ROBOTINTERFACE_SETTLE_SECONDS}s for the boards to wake up"
    sleep "$ROBOTINTERFACE_SETTLE_SECONDS"
fi

# -----------------------------------------------------------------------------
# Optional: launch FirmwareUpdater -a as a manual visual check.
# -----------------------------------------------------------------------------
if [[ "$RUN_FIRMWARE_CHECK" == "1" ]]; then
    log "Launching FirmwareUpdater -a on ${REMOTE_SERVER} for a manual board check"
    log "(this does not block the rest of the script - close it once you've verified the boards)"

    yarp run --on "$REMOTE_SERVER" --sigterm "$REMOTE_FIRMWAREUPDATER_TAG" >/dev/null 2>&1 || true

    yarp run \
        --on "$REMOTE_SERVER" \
        --as "$REMOTE_FIRMWAREUPDATER_TAG" \
        --cmd "$REMOTE_FIRMWAREUPDATER_CMD" || true
fi

# -----------------------------------------------------------------------------
# Start skinManager remotely, unless its output port already exists.
# -----------------------------------------------------------------------------
if port_exists "$SKIN_OUTPUT_PORT"; then
    log "Skin output is already available: ${SKIN_OUTPUT_PORT}"
else
    log "Starting skinManager on ${REMOTE_SERVER}"

    yarp run --on "$REMOTE_SERVER" --sigterm "$REMOTE_SKINMANAGER_TAG" >/dev/null 2>&1 || true

    yarp run \
        --on "$REMOTE_SERVER" \
        --as "$REMOTE_SKINMANAGER_TAG" \
        --cmd "$REMOTE_SKINMANAGER_CMD"

    REMOTE_SKINMANAGER_STARTED=1

    if ! wait_for_port "$SKIN_OUTPUT_PORT" "$SKIN_WAIT_SECONDS" "skin output"; then
        error "Timed out after ${SKIN_WAIT_SECONDS}s waiting for ${SKIN_OUTPUT_PORT}."
        error "Check the remote process with: yarp run --on ${REMOTE_SERVER} --ps"
        exit 1
    fi
fi

log "Skin output is available: ${SKIN_OUTPUT_PORT}"

# -----------------------------------------------------------------------------
# Start the local skin GUI.
# -----------------------------------------------------------------------------
log "Starting local GUI: ${SKIN_GUI_EXECUTABLE} --from ${SKIN_GUI_CONFIG}"
"$SKIN_GUI_EXECUTABLE" --from "$SKIN_GUI_CONFIG" &
GUI_PID=$!

if ! wait_for_port "$LOCAL_INPUT_PORT" "$GUI_WAIT_SECONDS" "local skin GUI input"; then
    if ! kill -0 "$GUI_PID" 2>/dev/null; then
        error "${SKIN_GUI_EXECUTABLE} exited before opening ${LOCAL_INPUT_PORT}."
    else
        error "Timed out after ${GUI_WAIT_SECONDS}s waiting for ${LOCAL_INPUT_PORT}."
    fi
    exit 1
fi

# -----------------------------------------------------------------------------
# Connect the skin stream to the GUI.
# -----------------------------------------------------------------------------
log "Connecting ${SKIN_OUTPUT_PORT} -> ${LOCAL_INPUT_PORT} using ${CARRIER}"

if ! yarp connect "$SKIN_OUTPUT_PORT" "$LOCAL_INPUT_PORT" "$CARRIER"; then
    error "Could not connect ${SKIN_OUTPUT_PORT} to ${LOCAL_INPUT_PORT}."
    error "Try manually: yarp connect ${SKIN_OUTPUT_PORT} ${SKIN_OUTPUT_PORT}/rpc"
    exit 1
fi

log "Skin pipeline is running"
log "Press Ctrl+C to stop"

# Deliberately do NOT "wait $GUI_PID" here: if the GUI window is closed, that
# would make the script exit too. Instead, loop forever (only Ctrl+C / SIGINT
# ends it, via the trap above), and just note if the GUI process has died.
GUI_RUNNING=1
while true; do
    if (( GUI_RUNNING )) && ! kill -0 "$GUI_PID" 2>/dev/null; then
        GUI_RUNNING=0
        log "Local skin GUI closed - script keeps running (Ctrl+C to stop)."
    fi
    sleep 1
done