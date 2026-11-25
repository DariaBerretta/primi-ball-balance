#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GUI="$PROJECT_DIR/build/event_control_gui"

REMOTE_SERVER="/icub-ultrascale"
REMOTE_EXECUTABLE="/home/icub/primi-camera-control/build/camera_control"
REMOTE_TAG="primi-camera-control"
RPC_PORT="/primi-ball-balance/cameraControl/rpc"

STARTED_CONTROLLER=0

cleanup()
{
    if (( STARTED_CONTROLLER )); then
        echo "[INFO] Stopping camera controller..."
        yarp run --on "$REMOTE_SERVER" \
            --sigterm "$REMOTE_TAG" >/dev/null 2>&1 || true
    fi
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Check prerequisites.
if [[ ! -x "$GUI" ]]; then
    echo "[ERROR] GUI executable not found: $GUI" >&2
    exit 1
fi

if ! yarp where >/dev/null 2>&1; then
    echo "[ERROR] YARP name server unavailable." >&2
    exit 1
fi

# Reuse an existing controller if one is already running.
if yarp exists "$RPC_PORT" >/dev/null 2>&1; then
    echo "[INFO] Camera controller already running."
else
    if ! yarp exists "$REMOTE_SERVER" >/dev/null 2>&1; then
        echo "[ERROR] Ultrascale YARP run server unavailable." >&2
        exit 1
    fi

    echo "[INFO] Starting camera controller on Ultrascale..."

    # Arm cleanup before attempting the remote launch.
    STARTED_CONTROLLER=1

    yarp run --on "$REMOTE_SERVER" \
        --as "$REMOTE_TAG" \
        --cmd "$REMOTE_EXECUTABLE --serve --allow-writes"

    # Wait for the RPC service to become available.
    READY=0

    for ((i = 0; i < 50; ++i)); do
        if yarp exists "$RPC_PORT" >/dev/null 2>&1; then
            READY=1
            break
        fi
        sleep 0.2
    done

    if (( ! READY )); then
        echo "[ERROR] Camera controller failed to start." >&2
        exit 1
    fi
fi

echo "[INFO] Camera controller ready."
echo "[INFO] Starting event-control GUI..."

"$GUI"
