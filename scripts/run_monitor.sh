#!/usr/bin/env bash
set -Eeuo pipefail
export LC_ALL=C

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${1:-$ROOT_DIR/config/monitoring.env}"
source "$CONFIG_FILE"

PID_DIR="$ROOT_DIR/.monitoring/pids"
LOG_DIR="$ROOT_DIR/.monitoring/logs"
mkdir -p "$PID_DIR" "$LOG_DIR"

for cmd in yarp yarpview yarpscope vFramer terminator wmctrl xrandr xprop xwininfo stdbuf iCubSkinGui; do
    command -v "$cmd" >/dev/null || { echo "Missing command: $cmd"; exit 1; }
done

yarp where >/dev/null || { echo "YARP name server unavailable"; exit 1; }

# Desktop work area excludes the dock and top panel.
read -r work_x work_y work_w work_h < <(
    wmctrl -d | awk '$2 == "*" {
        split($8, origin, ",")
        split($9, size, "x")
        print origin[1], origin[2], size[1], size[2]
    }'
)

# Use the actual desktop work area rather than stale monitor dimensions
# from monitoring.env. Coordinates refer to the outside of each window.
WINDOW_GAP=8
MONITOR_X=$((work_x + WINDOW_GAP))
MONITOR_Y=$((work_y + WINDOW_GAP))
MONITOR_WIDTH=$((work_w - 2 * WINDOW_GAP))
MONITOR_HEIGHT=$((work_h - 2 * WINDOW_GAP))

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

    local attempts=$(( ${2:-10} * 10 ))
    for ((i=0; i<attempts; i++)); do
        yarp exists "$1" >/dev/null 2>&1 && return 0
        sleep 0.1
    done

    echo "Timed out waiting for port: $1" >&2
    return 1
}

wait_for_window() {
    echo "Waiting for window: $1"

    for ((i=0; i<100; i++)); do
        wmctrl -l | grep -F "$1" >/dev/null && return 0
        sleep 0.1
    done

    echo "Timed out waiting for window: $1" >&2
    return 1
}

place_window() {
    local title="$1" x="$2" y="$3" width="$4" height="$5"
    local id extents left=0 right=0 top=0 bottom=0
    local client_w client_h actual_x actual_y actual_w actual_h
    local outer_x outer_y outer_w outer_h attempt
    local move_x="$x" move_y="$y"

    id="$(wmctrl -l | awk -v title="$title" '{
        id=$1
        $1=$2=$3=""
        sub(/^ +/, "")
        if ($0 == title && !found) {print id; found=1}
    }')"
    [[ -n "$id" ]] || { echo "Window not found: $title" >&2; return 1; }

    wmctrl -i -r "$id" -b remove,maximized_vert,maximized_horz
    sleep 0.2

    extents="$(xprop -id "$id" _NET_FRAME_EXTENTS | awk -F ' = ' '
        NF == 2 {gsub(/,/, "", $2); print $2}')"
    if [[ -n "$extents" ]]; then
        read -r left right top bottom <<< "$extents"
    fi
    client_w=$((width - left - right))
    client_h=$((height - top - bottom))

    # Resize first, then move. Use explicit NorthWest gravity (1).
    wmctrl -i -r "$id" -e "1,-1,-1,$client_w,$client_h"
    sleep 0.2

    for ((attempt=0; attempt<3; attempt++)); do
        wmctrl -i -r "$id" -e "1,$move_x,$move_y,-1,-1"
        sleep 0.2

        # Read true root coordinates; do not use wmctrl -lG for feedback.
        read -r actual_x actual_y actual_w actual_h < <(
            xwininfo -id "$id" | awk '
                /Absolute upper-left X:/ {x=$NF}
                /Absolute upper-left Y:/ {y=$NF}
                /^[[:space:]]*Width:/ {w=$NF}
                /^[[:space:]]*Height:/ {h=$NF}
                END {print x, y, w, h}'
        )
        outer_x=$((actual_x - left))
        outer_y=$((actual_y - top))
        outer_w=$((actual_w + left + right))
        outer_h=$((actual_h + top + bottom))

        if (( outer_x >= x - 2 && outer_x <= x + 2 &&
              outer_y >= y - 2 && outer_y <= y + 2 )); then
            break
        fi
        move_x=$((move_x + x - outer_x))
        move_y=$((move_y + y - outer_y))
    done

    printf '%s: outer position %s,%s; size %sx%s; tile %s,%s %sx%s\n' \
        "$title" "$outer_x" "$outer_y" "$outer_w" "$outer_h" \
        "$x" "$y" "$width" "$height"

    if (( outer_x < x - 2 || outer_x > x + 2 ||
          outer_y < y - 2 || outer_y > y + 2 ||
          outer_w > width || outer_h > height )); then
        echo "Warning: window manager did not fit '$title' inside its tile." >&2
    fi
    # A placement warning must not tear down the working acquisition displays.
    return 0
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

# Right-hand compensated touch visualisation
skin_source="/primi-ball-balance/skin/right_hand/compensated:o"
skin_input="/skinGui/right_hand:i"
skin_title="SkinGui (part: right_hand)"

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
            "$EVENT_LEFT_SOURCE" "$EVENT_RIGHT_SOURCE" "$SCOPE_REMOTE" "$skin_source"; do
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

start_process processed_left_view yarpview \
    --minimal \
    --title "$rgb_left_title" \
    --name "$left_view"

start_process processed_right_view yarpview \
    --minimal \
    --title "$rgb_right_title" \
    --name "$right_view"

# Right-hand skin GUI in compensated-data mode
start_process skin_gui iCubSkinGui --from right_hand_V2_1.ini --useCalibration

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
wait_for_port "$skin_input" 15

yarp connect "$RGB_LEFT_PROCESSED_SOURCE" "$left_view" shmem
yarp connect "$RGB_RIGHT_PROCESSED_SOURCE" "$right_view" shmem
yarp connect "$skin_source" "$skin_input" shmem

# Wait for visualization windows
for title in "$scope_title" "$EVENT_LEFT_SOURCE" "$EVENT_RIGHT_SOURCE" \
             "$rgb_left_title" "$rgb_right_title" \
             "$detections_title" "$terminal_title"; do
    wait_for_window "$title"
done

wait_for_window "$skin_title"

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

# Bottom-left: Right-hand skin GUI (25%)
place_window "$skin_title" "$lx" "$by" "$qw" "$qh"

# Bottom-right: Two vertically stacked terminals (25%)
place_window "$detections_title" \
    "$rx" "$by" "$qw" "$ch"

place_window "$terminal_title" \
    "$rx" "$((by + ch + WINDOW_GAP))" "$qw" "$ch"

trap - ERR

echo "Monitoring started (right-hand skin GUI enabled)."
