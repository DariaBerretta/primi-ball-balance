#!/usr/bin/env bash
set -Eeuo pipefail

# Keep important status messages visible in quiet mode.
exec 3>&1
if [[ "${1:-}" == "--verbose" ]]; then
    shift
else
    exec >/dev/null
fi
if (( $# != 0 )); then
    printf 'Usage: %s [--verbose]\n' "${0##*/}" >&2
    exit 2
fi

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
READER_EXECUTABLE="${READER_EXECUTABLE:-$PROJECT_DIR/build/touch_reader}"

RIGHT_RAW_PORT="${RIGHT_RAW_PORT:-/icub/skin/right_hand}"
RIGHT_COMPENSATED_SOURCE="${RIGHT_COMPENSATED_SOURCE:-/icub/skin/right_hand_comp}"
RIGHT_LOCAL_INPUT="${RIGHT_LOCAL_INPUT:-/primi-ball-balance/skin/right_hand:i}"
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
CLEANUP_ARMED=0
CLEANUP_DONE=0

log() { printf '[INFO] %s\n' "$*"; }
status() { printf '[INFO] %s\n' "$*" >&3; }
error() { printf '[ERROR] %s\n' "$*" >&2; }
port_exists() { yarp exists "$1" >/dev/null 2>&1; }

wait_for_port()
{
    local port="$1"
    local attempts=$(( $2 * 5 ))
    local i
    log "Waiting for $port"
    for ((i = 0; i < attempts; ++i)); do
        if port_exists "$port"; then
            return 0
        fi
        if [[ -n "$READER_PID" ]] && ! kill -0 "$READER_PID" 2>/dev/null; then
            error "touch_reader exited while waiting for $port"
            return 1
        fi
        sleep 0.2
    done
    error "Timed out waiting for $port"
    return 1
}

cleanup()
{
    if (( ! CLEANUP_ARMED || CLEANUP_DONE )); then
        return
    fi
    CLEANUP_DONE=1

    if [[ "${PRIMI_SENSOR_GROUP:-0}" != 1 ]]; then printf '\n' >&3; fi
    status "Stopping tactile acquisition..."

    if [[ -n "$READER_PID" ]]; then
        yarp disconnect "$RIGHT_COMPENSATED_SOURCE" "$RIGHT_LOCAL_INPUT" >/dev/null 2>&1 || true
        kill -TERM "$READER_PID" 2>/dev/null || true
        wait "$READER_PID" 2>/dev/null || true
    fi

    if (( REMOTE_SKINMANAGER_STARTED )) && [[ "$STOP_REMOTE_SKINMANAGER_ON_EXIT" == "1" ]]; then
        yarp run --on "$REMOTE_RUN_SERVER" --sigterm "$REMOTE_SKINMANAGER_TAG" >/dev/null 2>&1 || true
    fi

    status "Tactile acquisition stopped."
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Validate without disturbing an existing acquisition.
yarp where >/dev/null 2>&1 || { error "YARP name server unavailable"; exit 1; }
[[ -x "$READER_EXECUTABLE" ]] || { error "Missing reader: $READER_EXECUTABLE"; exit 1; }
port_exists "$REMOTE_RUN_SERVER" || { error "Missing run server: $REMOTE_RUN_SERVER"; exit 1; }
port_exists "$RIGHT_RAW_PORT" || { error "Missing raw skin port: $RIGHT_RAW_PORT"; exit 1; }

if pgrep -x touch_reader >/dev/null 2>&1; then
    error "A touch_reader process is already running. Stop it before starting another."
    exit 1
fi
for port in "$RIGHT_LOCAL_INPUT" "$RIGHT_LOCAL_OUTPUT"; do
    if port_exists "$port"; then
        error "Port already registered: $port. Check for a stale registration before restarting."
        exit 1
    fi
done

CLEANUP_ARMED=1
status "Starting tactile acquisition..."

if port_exists "$RIGHT_COMPENSATED_SOURCE"; then
    log "Using existing compensated right-hand skin stream"
else
    log "Starting skinManager on $REMOTE_RUN_SERVER"
    if ! yarp run --on "$REMOTE_RUN_SERVER" --as "$REMOTE_SKINMANAGER_TAG" --cmd "$REMOTE_SKINMANAGER_COMMAND" >/dev/null 2>&1; then
        error "Could not start skinManager. Check the remote YARP command tag and logs."
        exit 1
    fi
    REMOTE_SKINMANAGER_STARTED=1
    wait_for_port "$RIGHT_COMPENSATED_SOURCE" "$SKIN_WAIT_SECONDS"
fi

"$READER_EXECUTABLE" &
READER_PID=$!
wait_for_port "$RIGHT_LOCAL_INPUT" "$READER_WAIT_SECONDS"
wait_for_port "$RIGHT_LOCAL_OUTPUT" "$READER_WAIT_SECONDS"

if ! yarp connect "$RIGHT_COMPENSATED_SOURCE" "$RIGHT_LOCAL_INPUT" "$REMOTE_CARRIER"; then
    error "Could not connect the compensated right-hand stream to touch_reader"
    exit 1
fi

status "Tactile acquisition is running. Press Ctrl+C to stop."
log "Skin visualization is managed by run_monitor.sh"
wait "$READER_PID"
