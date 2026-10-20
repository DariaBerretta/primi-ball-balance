#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${1:-$ROOT_DIR/config/monitoring.env}"
source "$CONFIG_FILE"

PID_DIR="$ROOT_DIR/.monitoring/pids"
LOG_DIR="$ROOT_DIR/.monitoring/logs"
mkdir -p "$PID_DIR" "$LOG_DIR"

for cmd in yarp yarpview yarpscope vFramer terminator wmctrl xrandr stdbuf; do
    command -v "$cmd" >/dev/null || { echo "Missing command: $cmd"; exit 1; }
done

yarp where >/dev/null || { echo "YARP name server unavailable"; exit 1; }

screen_size="$(xrandr --current | awk '/\*/ {print $1; exit}')"
MONITOR_WIDTH="${MONITOR_WIDTH:-0}"
MONITOR_HEIGHT="${MONITOR_HEIGHT:-0}"
(( MONITOR_WIDTH > 0 )) || MONITOR_WIDTH="${screen_size%x*}"
(( MONITOR_HEIGHT > 0 )) || MONITOR_HEIGHT="${screen_size#*x}"
MONITOR_X="${MONITOR_X:-0}"
MONITOR_Y="${MONITOR_Y:-0}"
WINDOW_GAP="${WINDOW_GAP:-4}"

start_process() {
    local name="$1"
    shift

    if [[ -f "$PID_DIR/$name.pid" ]] &&
       kill -0 "$(cat "$PID_DIR/$name.pid")" 2>/dev/null; then
        echo "$name is already running" >&2
        return 1
    fi

    "$@" >"$LOG_DIR/$name.log" 2>&1 &
    echo "$!" >"$PID_DIR/$name.pid"
}

wait_for_port() {
    echo "Waiting for $1"

    for ((i=0; i<100; i++)); do
        yarp exists "$1" >/dev/null 2>&1 && return 0
        sleep 0.1
    done

    echo "Timed out waiting for port: $1" >&2
    return 1
}

wait_for_window() {
    echo "Waiting for window: $1"

    for ((i=0; i<100; i++)); do
        wmctrl -l | grep -Fq "$1" && return 0
        sleep 0.1
    done

    echo "Timed out waiting for window: $1" >&2
    return 1
}

place_window() {
    wmctrl -r "$1" -b remove,maximized_vert,maximized_horz >/dev/null 2>&1 || true
    wmctrl -r "$1" -e "0,$2,$3,$4,$5"
}

trap '"$ROOT_DIR/scripts/stop_monitor.sh" "$CONFIG_FILE" >/dev/null 2>&1 || true' ERR

# Screen geometry
qw=$(((MONITOR_WIDTH - WINDOW_GAP) / 2))
qh=$(((MONITOR_HEIGHT - WINDOW_GAP) / 2))
cw=$(((qw - WINDOW_GAP) / 2))
ch=$(((qh - WINDOW_GAP) / 2))

lx="$MONITOR_X"
rx=$((lx + qw + WINDOW_GAP))
ty="$MONITOR_Y"
by=$((ty + qh + WINDOW_GAP))

# Skin GUI (temporarily disabled)
# skin_source="/primi-ball-balance/skin/right_hand/compensated:o"
# skin_input="/skinGui/right_hand:i"

left_view="/primi-ball-balance/monitor/rgb/left/processed:i"
right_view="/primi-ball-balance/monitor/rgb/right/processed:i"

scope_title="PRIMI - YARP Scope"
rgb_left_title="PRIMI - RGB Processed Left"
rgb_right_title="PRIMI - RGB Processed Right"
detections_title="PRIMI - RGB Detections"
terminal_title="PRIMI - Terminal 2"

# Wait for acquisition streams
for port in "$RGB_LEFT_PROCESSED_SOURCE" "$RGB_RIGHT_PROCESSED_SOURCE" \
            "$RGB_LEFT_DETECTION_SOURCE" "$RGB_RIGHT_DETECTION_SOURCE" \
            "$EVENT_LEFT_SOURCE" "$EVENT_RIGHT_SOURCE" "$SCOPE_REMOTE"; do
    wait_for_port "$port"
done

# Start visualization processes
start_process yarpscope yarpscope \
    --title "$scope_title" \
    --remote "$SCOPE_REMOTE" \
    --index "$SCOPE_INDEX" \
    --min "$SCOPE_MIN" \
    --max "$SCOPE_MAX"

start_process vframer vFramer \
    --name /primi-ball-balance/monitor/vFramer \
    --src "$EVENT_LEFT_SOURCE" \
    --src1 "$EVENT_RIGHT_SOURCE" \
    --width "$EVENT_WIDTH" \
    --height "$EVENT_HEIGHT" \
    --fps "$EVENT_FPS" \
    --window_size 1.0 
    # --grey

start_process processed_left_view yarpview \
    --minimal \--
    --title "$rgb_left_title" \
    --name "$left_view"

start_process processed_right_view yarpview \
    --minimal \
    --title "$rgb_right_title" \
    --name "$right_view"

# Skin GUI (temporarily disabled)
# start_process skin_gui iCubSkinGui --from right_hand_V2_1.ini --useCalibration

start_process detections_terminal terminator \
    --no-dbus \
    --title "$detections_title" \
    --working-directory "$ROOT_DIR" \
    -x "$ROOT_DIR/scripts/print_rgb_detections.sh" \
    "$RGB_LEFT_DETECTION_SOURCE" "$RGB_RIGHT_DETECTION_SOURCE"

start_process terminal_two terminator \
    --no-dbus \
    --title "$terminal_title" \
    --working-directory "$ROOT_DIR"

# Connect visualization streams
wait_for_port "$left_view"
wait_for_port "$right_view"
# wait_for_port "$skin_input"

yarp connect "$RGB_LEFT_PROCESSED_SOURCE" "$left_view" shmem
yarp connect "$RGB_RIGHT_PROCESSED_SOURCE" "$right_view" shmem
# yarp connect "$skin_source" "$skin_input" shmem

# Wait for visualization windows
for title in "$scope_title" "$EVENT_LEFT_SOURCE" "$EVENT_RIGHT_SOURCE" \
             "$rgb_left_title" "$rgb_right_title" \
             "$detections_title" "$terminal_title"; do
    wait_for_window "$title"
done

# Skin GUI (temporarily disabled)
# wait_for_window "iCubSkinGui"

# Top-left: YARP Scope (25%)
place_window "$scope_title" "$lx" "$ty" "$qw" "$qh"

# Top-right: Four cameras (25% total)
place_window "$EVENT_LEFT_SOURCE" \
    "$rx" "$ty" "$cw" "$ch"

place_window "$EVENT_RIGHT_SOURCE" \
    "$((rx + cw + WINDOW_GAP))" "$ty" "$cw" "$ch"

place_window "$rgb_left_title" \
    "$rx" "$((ty + ch + WINDOW_GAP))" "$cw" "$ch"

place_window "$rgb_right_title" \
    "$((rx + cw + WINDOW_GAP))" "$((ty + ch + WINDOW_GAP))" "$cw" "$ch"

# Bottom-left: Skin GUI (25%, temporarily disabled)
# place_window "iCubSkinGui" "$lx" "$by" "$qw" "$qh"

# Bottom-right: Two vertically stacked terminals (25%)
place_window "$detections_title" \
    "$rx" "$by" "$qw" "$ch"

place_window "$terminal_title" \
    "$rx" "$((by + ch + WINDOW_GAP))" "$qw" "$ch"

trap - ERR

echo "Monitoring started (skin GUI disabled)."