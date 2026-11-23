#!/usr/bin/env bash
set -Eeuo pipefail

# Launch the standalone C++ IMU reader without activating a Python venv.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$PROJECT_DIR/config/bma400.env"
EXECUTABLE="${BMA400_EXECUTABLE:-$PROJECT_DIR/build/bma400_reader}"

if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "Missing configuration: $CONFIG_FILE" >&2
    exit 1
fi

# Load the existing device, YARP and optional CSV settings.
set -a
# shellcheck disable=SC1090
source "$CONFIG_FILE"
set +a

if [[ ! -x "$EXECUTABLE" ]]; then
    echo "Missing compiled reader: $EXECUTABLE" >&2
    echo 'Build it with: cmake --build build --target bma400_reader -j4' >&2
    exit 1
fi
if ! yarp where >/dev/null 2>&1; then
    echo 'YARP name server is not reachable' >&2
    exit 1
fi
if [[ ! -r "$BMA400_SERIAL_PORT" ]]; then
    echo "Cannot read Feather device: $BMA400_SERIAL_PORT" >&2
    exit 1
fi

# Prepare the optional recording location before the C++ process starts.
if [[ -n "${BMA400_CSV:-}" ]]; then
    if [[ "$BMA400_CSV" = /* ]]; then
        mkdir -p "$(dirname "$BMA400_CSV")"
    else
        mkdir -p "$(dirname "$PROJECT_DIR/$BMA400_CSV")"
    fi
fi

cd "$PROJECT_DIR"
# exec preserves signal propagation (Ctrl+C, SIGTERM) to the reader.
exec "$EXECUTABLE" "$@"
