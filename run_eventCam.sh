#!/usr/bin/env bash

set -u

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
READER_EXECUTABLE="$PROJECT_DIR/build/eventCam_reader"

READER_PID=""

stop_zynq_grabber() {
    ssh icub@icub-ultrascale '
        pids=$(pgrep -f "[z]ynqGrabb" || true)

        if [ -n "$pids" ]; then
            kill $pids
            sleep 1
        fi

        pids=$(pgrep -f "[z]ynqGrabb" || true)

        if [ -n "$pids" ]; then
            kill -9 $pids
        fi
    '
}

cleanup() {
    kill "$READER_PID" 2>/dev/null || true
    stop_zynq_grabber
}

trap cleanup EXIT INT TERM

echo "Stopping previous processes..."

pkill -x eventCam_reader 2>/dev/null || true
stop_zynq_grabber

echo "Removing previous port registrations..."

yarp name unregister /zynqGrabber/left/AE:o 2>/dev/null || true
yarp name unregister /zynqGrabber/right/AE:o 2>/dev/null || true
yarp name unregister /primi-ball-balance/eventCam/left:i 2>/dev/null || true
yarp name unregister /primi-ball-balance/eventCam/right:i 2>/dev/null || true

sleep 1

echo "Starting zynqGrabber on icub-ultrascale..."

yarp run --on /icub-ultrascale \
    --as zynqGrabber \
    --cmd "zynqGrabber"

yarp wait /zynqGrabber/left/AE:o
yarp wait /zynqGrabber/right/AE:o

echo "Starting eventCam_reader..."

"$READER_EXECUTABLE" &
READER_PID=$!

yarp wait /primi-ball-balance/eventCam/left:i
yarp wait /primi-ball-balance/eventCam/right:i

echo "Connecting camera streams..."

yarp connect /zynqGrabber/left/AE:o /primi-ball-balance/eventCam/left:i fast_tcp
yarp connect /zynqGrabber/right/AE:o /primi-ball-balance/eventCam/right:i fast_tcp

echo "Perception pipeline is running."
echo "Press Ctrl+C to stop."

wait "$READER_PID"