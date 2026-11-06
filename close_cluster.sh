#!/usr/bin/env bash
set -euo pipefail


# ============================================================
# 1. Configuration
# ============================================================

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ACTIVE_SESSION="$PROJECT_DIR/logs/active_session"

YARP_NAMESPACE="/icub02"


# ============================================================
# 2. YARP namespace
# ============================================================

yarp namespace "$YARP_NAMESPACE"


# ============================================================
# 3. Stop robot applications
# ============================================================

# Stop applications in reverse startup order.
# Use the same YARP tags defined in setup_cluster.sh.


# ------------------------------------------------------------
# 3.1 Cartesian interface
# ------------------------------------------------------------

echo "Stopping Cartesian interface..."

yarp run --on /iiticublap267 \
    --sigterm cartesianInterfaceControl || true


# ------------------------------------------------------------
# 3.2 Cartesian solver
# ------------------------------------------------------------

echo "Stopping Cartesian solver..."

yarp run --on /iiticublap267 \
    --sigterm iKinCartesianSolver || true


# ------------------------------------------------------------
# 3.3 Clock
# ------------------------------------------------------------

echo "Stopping clock..."

yarp run --on /icub-ultrascale \
    --sigterm start_clock || true


# ------------------------------------------------------------
# 3.4 Robot interface
# ------------------------------------------------------------

echo "Stopping yarprobotinterface..."

yarp run --on /icub-head \
    --sigterm yarprobotinterface || true


# ============================================================
# 4. Close monitoring terminals
# ============================================================

# The monitoring shell in each Terminator window checks for
# a file named "stop" inside the active session directory.
#
# Creating this file causes the monitoring shells to:
#   1. Stop their tail/SSH processes.
#   2. Stop writing to the local log files.
#   3. Exit, closing their Terminator windows.
#
# The log files themselves are preserved.

if [[ -f "$ACTIVE_SESSION" ]]; then

    LOG_DIR="$(cat "$ACTIVE_SESSION")"

    if [[ -d "$LOG_DIR" &&
          "$LOG_DIR" == "$PROJECT_DIR"/logs/* ]]; then

        touch "$LOG_DIR/stop"

        echo "Closing monitoring terminals..."
    fi

    # The session is no longer active.
    rm -f "$ACTIVE_SESSION"

fi


# ============================================================
# 5. Shutdown complete
# ============================================================

echo
echo "Cluster stopped."
echo "Logs preserved."