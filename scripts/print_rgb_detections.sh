#!/usr/bin/env bash

set -euo pipefail

LEFT_REMOTE="${1:?Missing left detection port}"
RIGHT_REMOTE="${2:?Missing right detection port}"

LEFT_LOCAL="/primi-ball-balance/monitor/detections/left:i"
RIGHT_LOCAL="/primi-ball-balance/monitor/detections/right:i"

cleanup()
{
    local running_jobs
    running_jobs="$(jobs -pr)"

    if [[ -n "$running_jobs" ]]; then
        kill $running_jobs 2>/dev/null || true
    fi
}

trap cleanup EXIT HUP INT TERM

printf '\033c'

echo "RGB detection streams"
echo
echo "Format:"
echo "  [valid, center_x, center_y, radius, confidence, timestamp]"
echo

stdbuf -oL yarp read "$LEFT_LOCAL" "$LEFT_REMOTE" |
    sed -u 's/^/[LEFT ] /' &

stdbuf -oL yarp read "$RIGHT_LOCAL" "$RIGHT_REMOTE" |
    sed -u 's/^/[RIGHT] /' &

wait