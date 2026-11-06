#!/usr/bin/env bash
# Starts only the local pose-estimation pipeline; NEVER restarts zynqGrabber.
set -Eeuo pipefail

# --verbose precedes the optional config-file argument.
if [[ "${1:-}" == "--verbose" ]]; then
    shift
else
    exec >/dev/null 2>&1
fi
if (( $# > 1 )); then
    printf 'Usage: %s [--verbose] [config.env]\n' "${0##*/}" >&2
    exit 2
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${1:-$ROOT/config/moveEnetOFK.env}"
if [[ ! -f "$CONFIG" ]]; then
  echo "[ERROR] Missing config: $CONFIG" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "$CONFIG"

PYTHON="$ROOT/.venv/bin/python"
CPP="$ROOT/build/moveEnetOFK_online"
SIDE_CAR="$ROOT/python/moveenet_upper_online.py"
SOURCE="/primi-ball-balance/eventCam/$CAMERA/raw:o"
BASE="/primi-ball-balance/moveEnetOFK/$CAMERA"

for file in "$PYTHON" "$CPP" "$SIDE_CAR" "$CHECKPOINT"; do
  [[ -f "$file" ]] || { echo "[ERROR] Missing file: $file" >&2; exit 1; }
done
[[ -x "$PYTHON" && -x "$CPP" ]] || {
  echo '[ERROR] Python or C++ executable not executable. Build first.' >&2
  exit 1
}
[[ "$CAMERA" == left || "$CAMERA" == right ]] || {
  echo '[ERROR] CAMERA must be left or right' >&2
  exit 1
}
yarp where >/dev/null || { echo '[ERROR] YARP name server unavailable' >&2; exit 1; }
yarp exists "$SOURCE" >/dev/null || {
  echo "[ERROR] Event source $SOURCE unavailable." >&2
  echo '[ERROR] Start the existing run_eventCam.sh in another terminal first.' >&2
  exit 1
}

wait_for_port() {
  local name="$1"
  for ((i=0; i<400; i++)); do
    yarp exists "$name" >/dev/null 2>&1 && return 0
    kill -0 "$PY_PID" 2>/dev/null || return 1
    if [[ -n "$CPP_PID" ]]; then kill -0 "$CPP_PID" 2>/dev/null || return 1; fi
    sleep 0.1
  done
  echo "[ERROR] Timeout waiting for $name" >&2
  return 1
}

PY_PID=""
CPP_PID=""
VIEW_PID=""
VIEW_PORT="/primi-ball-balance/monitor/moveEnetOFK/overlay:i"
# Set AUTO_VIEW_OVERLAY=false in config/moveEnetOFK.env to disable the viewer.
AUTO_VIEW_OVERLAY="${AUTO_VIEW_OVERLAY:-true}"
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$VIEW_PID" ]]; then kill -TERM "$VIEW_PID" 2>/dev/null || true; fi
  if [[ -n "$CPP_PID" ]]; then kill -TERM "$CPP_PID" 2>/dev/null || true; fi
  if [[ -n "$PY_PID" ]]; then kill -TERM "$PY_PID" 2>/dev/null || true; fi
  sleep 1
  for pid in "$VIEW_PID" "$CPP_PID" "$PY_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  if [[ -n "$VIEW_PID" ]]; then wait "$VIEW_PID" 2>/dev/null || true; fi
  if [[ -n "$CPP_PID" ]]; then wait "$CPP_PID" 2>/dev/null || true; fi
  if [[ -n "$PY_PID" ]]; then wait "$PY_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

export PYTHONPATH="$HPE_CORE_DIR${PYTHONPATH:+:$PYTHONPATH}"
"$PYTHON" "$SIDE_CAR" --camera "$CAMERA" --checkpoint "$CHECKPOINT" &
PY_PID=$!
wait_for_port "$BASE/inference/img:i"
wait_for_port "$BASE/inference/sklt:o"

"$CPP" --camera "$CAMERA" --width "$WIDTH" --height "$HEIGHT" \
  --f_net "$F_NET" --f_flow "$F_FLOW" --f_vis "$F_VIS" \
  --confidence_th "$CONFIDENCE_TH" --track_timeout "$TRACK_TIMEOUT" \
  --pu "$PU" --muD "$MUD" --muV "$MUV" \
  --roi "$ROI" --use_lc "$USE_LC" \
  --noise_filter "$NOISE_FILTER" \
  --noise_refractory_ms "$NOISE_REFRACTORY_MS" \
  --noise_spatial_ms "$NOISE_SPATIAL_MS" \
  --noise_radius "$NOISE_RADIUS" \
  --show_movenet "${SHOW_MOVENET:-true}" \
  --show_ofk "${SHOW_OFK:-true}" &
CPP_PID=$!
wait_for_port "$BASE/events:i"
wait_for_port "$BASE/model/img:o"
wait_for_port "$BASE/model/sklt:i"

echo '[INFO] Connecting locally republished events to online pose module'
yarp connect "$SOURCE" "$BASE/events:i" "$LOCAL_CARRIER"
yarp connect "$BASE/model/img:o" "$BASE/inference/img:i" "$LOCAL_CARRIER"
yarp connect "$BASE/inference/sklt:o" "$BASE/model/sklt:i" "$LOCAL_CARRIER"

# Launch the overlay window automatically, when a graphical session is available.
# If the viewer already exists (e.g. opened manually), reuse it and do not own it.
if [[ "$AUTO_VIEW_OVERLAY" == "true" ]]; then
  if ! command -v yarpview >/dev/null 2>&1; then
    echo '[WARN] yarpview is not installed; overlay window skipped.' >&2
  elif [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    echo '[WARN] No graphical session detected; overlay window skipped.' >&2
  else
    if ! yarp exists "$VIEW_PORT" >/dev/null 2>&1; then
      yarpview --name "$VIEW_PORT" &
      VIEW_PID=$!
      for ((i=0; i<100; i++)); do
        yarp exists "$VIEW_PORT" >/dev/null 2>&1 && break
        kill -0 "$VIEW_PID" 2>/dev/null || break
        sleep 0.1
      done
    fi
    if yarp exists "$VIEW_PORT" >/dev/null 2>&1; then
      if yarp connect "$BASE/overlay:o" "$VIEW_PORT" tcp; then
        echo "[INFO] Overlay viewer connected: $VIEW_PORT"
      else
        echo '[WARN] Overlay viewer connection failed; pipeline continues.' >&2
      fi
    else
      echo '[WARN] Overlay viewer did not open; pipeline continues.' >&2
    fi
  fi
fi

echo '[INFO] Online MoveEnetOFK is running.'
echo "[INFO] Overlay: $BASE/overlay:o"
echo "[INFO] Skeleton: $BASE/skeleton:o"
echo '[INFO] The camera reader is NOT owned by this script.'
echo '[INFO] Ctrl+C stops only MoveEnetOFK and its Python sidecar.'

# Stop the other child if either one exits (bash >= 4.3).
wait -n "$CPP_PID" "$PY_PID" || true
