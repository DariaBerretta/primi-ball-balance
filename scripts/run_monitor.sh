#!/usr/bin/env bash

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${1:-$ROOT_DIR/config/monitoring.env}"

STATE_DIR="$ROOT_DIR/.monitoring"
PID_DIR="$STATE_DIR/pids"
LOG_DIR="$STATE_DIR/logs"

if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "Monitoring configuration not found: $CONFIG_FILE"
    exit 1
fi

# shellcheck disable=SC1090
source "$CONFIG_FILE"

required_commands=(
    yarp
    yarpview
    yarpscope
    vFramer
    terminator
    wmctrl
    xrandr
    stdbuf
)

for command_name in "${required_commands[@]}"; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "Required command not found: $command_name"
        exit 1
    fi
done

if ! yarp where >/dev/null 2>&1; then
    echo "The YARP name server is not reachable."
    exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
    echo "DISPLAY is not set."
    exit 1
fi

screen_size="$(xrandr --current | awk '/\*/ {print $1; exit}')"

if [[ -z "$screen_size" ]]; then
    echo "Could not determine the display resolution."
    exit 1
fi

detected_width="${screen_size%x*}"
detected_height="${screen_size#*x}"

if [[ "${MONITOR_WIDTH:-0}" -le 0 ]]; then
    MONITOR_WIDTH="$detected_width"
fi

if [[ "${MONITOR_HEIGHT:-0}" -le 0 ]]; then
    MONITOR_HEIGHT="$detected_height"
fi

MONITOR_X="${MONITOR_X:-0}"
MONITOR_Y="${MONITOR_Y:-0}"
WINDOW_GAP="${WINDOW_GAP:-4}"

mkdir -p "$PID_DIR" "$LOG_DIR"

start_process()
{
    local name="$1"
    shift

    local pid_file="$PID_DIR/$name.pid"

    if [[ -f "$pid_file" ]]; then
        local existing_pid
        existing_pid="$(cat "$pid_file")"

        if kill -0 "$existing_pid" 2>/dev/null; then
            echo "$name is already running with PID $existing_pid"
            exit 1
        fi

        rm -f "$pid_file"
    fi

    echo "Starting $name..."

    "$@" >"$LOG_DIR/$name.log" 2>&1 &
    echo "$!" >"$pid_file"
}

wait_for_port()
{
    local port="$1"
    local attempts="${2:-100}"

    echo "Waiting for $port"

    for _ in $(seq 1 "$attempts"); do
        if yarp exists "$port" >/dev/null 2>&1; then
            return 0
        fi

        sleep 0.1
    done

    echo "Timed out waiting for port: $port"
    return 1
}

wait_for_window()
{
    local title="$1"
    local attempts="${2:-100}"

    for _ in $(seq 1 "$attempts"); do
        if wmctrl -l | grep -Fq "$title"; then
            return 0
        fi

        sleep 0.1
    done

    echo "Timed out waiting for window: $title"
    return 1
}

place_window()
{
    local title="$1"
    local x="$2"
    local y="$3"
    local width="$4"
    local height="$5"

    wmctrl -r "$title" -b remove,maximized_vert,maximized_horz \
        >/dev/null 2>&1 || true

    wmctrl -r "$title" -e "0,$x,$y,$width,$height"
}

cleanup_after_error()
{
    echo
    echo "Monitoring startup failed."
    "$ROOT_DIR/scripts/stop_monitor.sh" "$CONFIG_FILE" >/dev/null 2>&1 || true
}

trap cleanup_after_error ERR

quadrant_width=$(((MONITOR_WIDTH - WINDOW_GAP) / 2))
quadrant_height=$(((MONITOR_HEIGHT - WINDOW_GAP) / 2))

left_x="$MONITOR_X"
right_x=$((MONITOR_X + quadrant_width + WINDOW_GAP))

top_y="$MONITOR_Y"
bottom_y=$((MONITOR_Y + quadrant_height + WINDOW_GAP))

inner_width=$(((quadrant_width - WINDOW_GAP) / 2))
inner_height=$(((quadrant_height - WINDOW_GAP) / 2))

inner_right_offset=$((inner_width + WINDOW_GAP))
inner_bottom_offset=$((inner_height + WINDOW_GAP))

scope_title="PRIMI - YARP Scope"

event_left_title="$EVENT_LEFT_SOURCE"
event_right_title="$EVENT_RIGHT_SOURCE"

raw_left_title="PRIMI - RGB Raw Left"
raw_right_title="PRIMI - RGB Raw Right"
processed_left_title="PRIMI - RGB Processed Left"
processed_right_title="PRIMI - RGB Processed Right"

detections_title="PRIMI - RGB Detections"
terminal_two_title="PRIMI - Terminal 2"
terminal_three_title="PRIMI - Terminal 3"
terminal_four_title="PRIMI - Terminal 4"

left_raw_view="/primi-ball-balance/monitor/rgb/left/raw:i"
right_raw_view="/primi-ball-balance/monitor/rgb/right/raw:i"

left_processed_view="/primi-ball-balance/monitor/rgb/left/processed:i"
right_processed_view="/primi-ball-balance/monitor/rgb/right/processed:i"

wait_for_port "$RGB_LEFT_RAW_SOURCE"
wait_for_port "$RGB_RIGHT_RAW_SOURCE"
wait_for_port "$RGB_LEFT_PROCESSED_SOURCE"
wait_for_port "$RGB_RIGHT_PROCESSED_SOURCE"
wait_for_port "$RGB_LEFT_DETECTION_SOURCE"
wait_for_port "$RGB_RIGHT_DETECTION_SOURCE"

wait_for_port "$EVENT_LEFT_SOURCE"
wait_for_port "$EVENT_RIGHT_SOURCE"
wait_for_port "$SCOPE_REMOTE"

start_process yarpscope \
    yarpscope \
    --title "$scope_title" \
    --remote "$SCOPE_REMOTE" \
    --index "$SCOPE_INDEX" \
    --min "$SCOPE_MIN" \
    --max "$SCOPE_MAX"

start_process vframer \
    vFramer \
    --name /primi-ball-balance/monitor/vFramer \
    --src "$EVENT_LEFT_SOURCE" \
    --src1 "$EVENT_RIGHT_SOURCE" \
    --width "$EVENT_WIDTH" \
    --height "$EVENT_HEIGHT" \
    --fps "$EVENT_FPS" \
    --grey

start_process raw_left_view \
    yarpview \
    --minimal \
    --title "$raw_left_title" \
    --name "$left_raw_view"

start_process raw_right_view \
    yarpview \
    --minimal \
    --title "$raw_right_title" \
    --name "$right_raw_view"

start_process processed_left_view \
    yarpview \
    --minimal \
    --title "$processed_left_title" \
    --name "$left_processed_view"

start_process processed_right_view \
    yarpview \
    --minimal \
    --title "$processed_right_title" \
    --name "$right_processed_view"

wait_for_port "$left_raw_view"
wait_for_port "$right_raw_view"
wait_for_port "$left_processed_view"
wait_for_port "$right_processed_view"

yarp connect "$RGB_LEFT_RAW_SOURCE" "$left_raw_view" tcp
yarp connect "$RGB_RIGHT_RAW_SOURCE" "$right_raw_view" tcp
yarp connect "$RGB_LEFT_PROCESSED_SOURCE" "$left_processed_view" tcp
yarp connect "$RGB_RIGHT_PROCESSED_SOURCE" "$right_processed_view" tcp

start_process detections_terminal \
    terminator \
    --no-dbus \
    --title "$detections_title" \
    --working-directory "$ROOT_DIR" \
    -x "$ROOT_DIR/scripts/print_rgb_detections.sh" \
    "$RGB_LEFT_DETECTION_SOURCE" "$RGB_RIGHT_DETECTION_SOURCE"

start_process terminal_two \
    terminator \
    --no-dbus \
    --title "$terminal_two_title" \
    --working-directory "$ROOT_DIR"

start_process terminal_three \
    terminator \
    --no-dbus \
    --title "$terminal_three_title" \
    --working-directory "$ROOT_DIR"

start_process terminal_four \
    terminator \
    --no-dbus \
    --title "$terminal_four_title" \
    --working-directory "$ROOT_DIR"

wait_for_window "$scope_title"

wait_for_window "$event_left_title"
wait_for_window "$event_right_title"

wait_for_window "$raw_left_title"
wait_for_window "$raw_right_title"
wait_for_window "$processed_left_title"
wait_for_window "$processed_right_title"

wait_for_window "$detections_title"
wait_for_window "$terminal_two_title"
wait_for_window "$terminal_three_title"
wait_for_window "$terminal_four_title"

#============ Top-left: YARP scope ============

place_window "$scope_title" \
    "$left_x" "$top_y" \
    "$quadrant_width" "$quadrant_height"

#============ Top-right: event cameras ============

place_window "$event_left_title" \
    "$right_x" "$top_y" \
    "$inner_width" "$quadrant_height"

place_window "$event_right_title" \
    "$((right_x + inner_right_offset))" "$top_y" \
    "$inner_width" "$quadrant_height"

#============ Bottom-left: RGB ============

place_window "$raw_left_title" \
    "$left_x" "$bottom_y" \
    "$inner_width" "$inner_height"

place_window "$raw_right_title" \
    "$((left_x + inner_right_offset))" "$bottom_y" \
    "$inner_width" "$inner_height"

place_window "$processed_left_title" \
    "$left_x" "$((bottom_y + inner_bottom_offset))" \
    "$inner_width" "$inner_height"

place_window "$processed_right_title" \
    "$((left_x + inner_right_offset))" \
    "$((bottom_y + inner_bottom_offset))" \
    "$inner_width" "$inner_height"

#============ Bottom-right: terminals ============

place_window "$detections_title" \
    "$right_x" "$bottom_y" \
    "$inner_width" "$inner_height"

place_window "$terminal_two_title" \
    "$((right_x + inner_right_offset))" "$bottom_y" \
    "$inner_width" "$inner_height"

place_window "$terminal_three_title" \
    "$right_x" "$((bottom_y + inner_bottom_offset))" \
    "$inner_width" "$inner_height"

place_window "$terminal_four_title" \
    "$((right_x + inner_right_offset))" \
    "$((bottom_y + inner_bottom_offset))" \
    "$inner_width" "$inner_height"

trap - ERR

echo
echo "Monitoring application started."
echo "No camera or reader processes were started."