#!/usr/bin/env bash

set -euo pipefail

YARP_NAMESPACE="/icub02"
SSH_USER="icub"
LOCAL_HOST="$(hostname -s)"

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

start_node icub-head
start_node icub-ultrascale
start_node iiticublap267 ":1.0"
# start_node iiticublap268 ":1.0"

sleep 2

echo
echo "Cluster status:"

for node in icub-head icub-ultrascale iiticublap267; do     # -optional- iiticublap268; do
    if yarp exists "/$node"; then
        echo "  /$node: ready"
    else
        echo "  /$node: unavailable"
    fi
done

echo "Started yarprobotinterface on icub-head and start_clock on icub-ultrascale."
yarp run --on /icub-head --as yarprobotinterface --cmd "yarprobotinterface"
yarp run --on /icub-ultrascale --as start_clock --cmd "/home/icub/start_clock.sh"