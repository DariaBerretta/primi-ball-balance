#!/usr/bin/env bash

set -euo pipefail

yarp namespace /icub02

echo "Stopping Cartesian interface..."
sleep 1

yarp run --on /iiticublap267 --sigterm cartesianInterfaceControl || true

echo "Stopping Cartesian solver..."
sleep 1
yarp run --on /iiticublap267 --sigterm iKinCartesianSolver || true

echo "Stopping clock..."
sleep 1
yarp run --on /icub-ultrascale --sigterm start_clock || true

echo "Stopping yarprobotinterface..."
sleep 1
yarp run --on /icub-head --sigterm yarprobotinterface || true

echo "Cluster applications stopped."
sleep 1