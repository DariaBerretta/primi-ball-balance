#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# 1. Configuration and session logs
# ============================================================

YARP_NAMESPACE="/icub02"
SSH_USER="icub"
LOCAL_HOST="$(hostname -s)"

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_ROOT="$PROJECT_DIR/logs"
ACTIVE_SESSION="$LOG_ROOT/active_session"

RUN_ID="$(date +%Y-%m-%d_%H-%M-%S)"
LOG_DIR="$LOG_ROOT/$RUN_ID"

command -v terminator >/dev/null || {
    echo "ERROR: Terminator is not installed." >&2
    exit 1
}

mkdir -p "$LOG_ROOT"

if [[ -e "$ACTIVE_SESSION" ]]; then
    echo "ERROR: An existing cluster session is recorded." >&2
    echo "Run ./close_cluster.sh first." >&2
    exit 1
fi

[[ ! -e "$LOG_DIR" ]] || LOG_DIR="${LOG_DIR}_$$"

mkdir -p "$LOG_DIR"

printf '%s\n' "$LOG_DIR" > "$ACTIVE_SESSION"

echo "Logs: $LOG_DIR"


# ============================================================
# 2. YARP run server management
# ============================================================

start_node() {
    local node="$1"
    local display="${2:-}"

    if yarp exists "/$node"; then
        echo "/$node is already running."
        return
    fi

    echo "Starting yarp run on $node..."

    if [[ "$node" == "$LOCAL_HOST" ]]; then

        if [[ -n "$display" ]]; then
            DISPLAY="$display" nohup yarp run --server "/$node" \
                >"/tmp/yarp-run-$node.log" 2>&1 &
        else
            nohup yarp run --server "/$node" \
                >"/tmp/yarp-run-$node.log" 2>&1 &
        fi

    else

        if [[ -n "$display" ]]; then
            ssh "$SSH_USER@$node" \
                "DISPLAY=$display nohup yarp run --server /$node >/tmp/yarp-run-$node.log 2>&1 </dev/null &"
        else
            ssh "$SSH_USER@$node" \
                "nohup yarp run --server /$node >/tmp/yarp-run-$node.log 2>&1 </dev/null &"
        fi

    fi
}


# ============================================================
# 3. YARP namespace and name server
# ============================================================

echo "Using YARP namespace $YARP_NAMESPACE"
yarp namespace "$YARP_NAMESPACE"

if pgrep -x yarpserver >/dev/null; then
    echo "yarpserver is already running."
else
    echo "Starting yarpserver on $LOCAL_HOST..."

    nohup yarpserver --portdb :memory: --subdb :memory: \
        >/tmp/yarpserver.log 2>&1 &

    sleep 2
fi


# ============================================================
# 4. Start cluster nodes
# ============================================================

start_node icub-head
start_node icub-ultrascale
start_node iiticublap267 ":1.0"

# Optional additional computer.
# start_node iiticublap268 ":1.0"

sleep 2


# ============================================================
# 5. Verify cluster status
# ============================================================

echo
echo "Cluster status:"

for node in icub-head icub-ultrascale iiticublap267; do
    if yarp exists "/$node"; then
        echo "  /$node: ready"
    else
        echo "  /$node: unavailable"
    fi
done


# ============================================================
# 6. Startup readiness checks
# ============================================================

# A registered YARP port does not necessarily mean its
# application has completed initialization.
#
# We therefore wait for the actual startup messages
# reported by the robot interface and Cartesian solver.

wait_for_log() {
    local node="$1"
    local logfile="$2"
    local message="$3"
    local timeout="${4:-240}"

    echo "Waiting for: $message"

    for ((i = 0; i < timeout; i++)); do

        if [[ "$node" == "$LOCAL_HOST" ]]; then
            if grep -Fq "$message" "$logfile" 2>/dev/null; then
                echo "Ready: $message"
                return 0
            fi
        else
            if ssh -o BatchMode=yes "$SSH_USER@$node" \
                "grep -Fq '$message' '$logfile'" 2>/dev/null; then
                echo "Ready: $message"
                return 0
            fi
        fi

        sleep 1
    done

    echo "ERROR: Timed out waiting for: $message" >&2
    return 1
}


# ============================================================
# 7. Monitoring terminals
# ============================================================

# Each application writes to a temporary file on its host.
#
# Its Terminator window:
#   - reads the output using tail;
#   - displays and saves it locally with tee;
#   - checks for the session's stop file;
#   - exits when close_cluster.sh requests shutdown.
#
# The session logs remain on this computer.

open_monitor() {
    local tag="$1"
    local node="$2"

    local remote_log="/tmp/primi-$RUN_ID-$tag.log"
    local output

    if [[ "$node" == "$LOCAL_HOST" ]]; then
        output="tail -n +1 -F '$remote_log'"
    else
        output="ssh '$SSH_USER@$node' tail -n +1 -F '$remote_log'"
    fi

    terminator -T "$tag" -x bash -c "
        setsid bash -c \"$output 2>&1 | tee -a '$LOG_DIR/$tag.log'\" &
        monitor_pid=\$!

        while [[ ! -f '$LOG_DIR/stop' ]] &&
              kill -0 \$monitor_pid 2>/dev/null; do
            sleep 0.5
        done

        kill -TERM -- -\$monitor_pid 2>/dev/null || true
        wait \$monitor_pid 2>/dev/null || true
    " &
}


# ============================================================
# 8. Start robot interface
# ============================================================

echo
echo "Starting yarprobotinterface..."

ROBOT_LOG="/tmp/primi-$RUN_ID-yarprobotinterface.log"

yarp run --on /icub-head --as yarprobotinterface \
    --cmd "bash -c \"exec yarprobotinterface --config /usr/local/src/robot/robotology-superbuild/src/robots-configuration/iCubGenova02/icub_right_arm.xml > $ROBOT_LOG 2>&1\""

open_monitor yarprobotinterface icub-head


# ============================================================
# 9. Wait for robot calibration stage
# ============================================================

# The robot interface exposes some control-board ports
# before completing calibration.
#
# Wait until startup action level 10 finishes before
# attempting to initialize the Cartesian solver.
#
# Do not wait for the entire robot interface here:
# its Cartesian controller still needs the solver.

wait_for_log \
    icub-head \
    "$ROBOT_LOG" \
    "All actions for action level 10 of startup phase finished."


# ============================================================
# 10. Start clock
# ============================================================

echo
echo "Starting clock..."

yarp run --on /icub-ultrascale --as start_clock \
    --cmd "bash -c \"exec /home/icub/start_clock.sh > /tmp/primi-$RUN_ID-start_clock.log 2>&1\""

open_monitor start_clock icub-ultrascale


# ============================================================
# 11. Start Cartesian solver
# ============================================================

echo
echo "Starting Cartesian solver..."

SOLVER_LOG="/tmp/primi-$RUN_ID-iKinCartesianSolver.log"

yarp run --on /iiticublap267 --as iKinCartesianSolver \
    --cmd "bash -c \"exec iKinCartesianSolver --part right_arm > $SOLVER_LOG 2>&1\""

open_monitor iKinCartesianSolver "$LOCAL_HOST"


# ============================================================
# 12. Wait for Cartesian solver
# ============================================================

# The solver opens its RPC port before completing
# initialization. Wait for its startup confirmation.

wait_for_log \
    "$LOCAL_HOST" \
    "$SOLVER_LOG" \
    "cartesianSolver/right_arm started successfully"


# ============================================================
# 13. Wait for robot Cartesian controller
# ============================================================

# The robot interface initializes its Cartesian controller
# after connecting to the solver.
#
# Wait for this connection and for the controller to start.

wait_for_log \
    icub-head \
    "$ROBOT_LOG" \
    "Connections established with cartesianSolver/right_arm"

wait_for_log \
    icub-head \
    "$ROBOT_LOG" \
    "icub/cartesianController/right_arm started successfully"

wait_for_log \
    icub-head \
    "$ROBOT_LOG" \
    "startup phase finished."


# ============================================================
# 14. Start Cartesian interface
# ============================================================

echo
echo "Starting Cartesian interface..."

yarp run --on /iiticublap267 --as cartesianInterfaceControl \
    --cmd "bash -c \"exec cartesianInterfaceControl --part right_arm --only xyz > /tmp/primi-$RUN_ID-cartesianInterfaceControl.log 2>&1\""

open_monitor cartesianInterfaceControl "$LOCAL_HOST"


# ============================================================
# 15. Startup complete
# ============================================================

echo
echo "Cluster startup commands completed."
echo "Logs: $LOG_DIR"
echo
echo "Stop using: ./close_cluster.sh"