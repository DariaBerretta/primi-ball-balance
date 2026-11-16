#!/usr/bin/env bash
set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE="/primi-ball-balance/skin/right_hand/compensated:o"
DUMPER="/record/touch/$$"
touch_pid=""
dumper_pid=""

cleanup() {
    for pid in "$dumper_pid" "$touch_pid"; do
        if [[ -n "$pid" ]]; then
            kill -TERM "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
    dumper_pid=""
    touch_pid=""
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_for_port() {
    until yarp exists "$1" >/dev/null 2>&1; do
        kill -0 "$2" 2>/dev/null || { echo "Failed to open $1" >&2; exit 1; }
        sleep 0.2
    done
}

# Reuse the existing touch stream; otherwise start your launcher.
if ! yarp exists "$SOURCE" >/dev/null 2>&1; then
    bash "${1:-$PROJECT_DIR/run_touch.sh}" &
    touch_pid=$!
    wait_for_port "$SOURCE" "$touch_pid"
fi

while true; do
    read -r -p "Type yes to start recording: " command || exit 0
    [[ "$command" == "yes" ]] && break
done

OUTPUT="touch_$(date +%Y%m%d_%H%M%S)_$$"
yarpdatadumper --name "$DUMPER" --type bottle --dir "$OUTPUT" &
dumper_pid=$!
wait_for_port "$DUMPER" "$dumper_pid"
yarp connect "$SOURCE" "$DUMPER" tcp

echo "Recording in $PWD/$OUTPUT"
while true; do
    read -r -p "Type stop to finish: " command || break
    [[ "$command" == "stop" ]] && break
done

# Wait for the dumper to finish saving before reading data.log.
cleanup
python3 - "$OUTPUT/data.log" <<'PY'
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

log = Path(sys.argv[1])
if log.stat().st_size == 0:
    sys.exit("No samples recorded; no plot created.")

with log.open() as f:
    data = np.loadtxt((line.replace("(", " ").replace(")", " ") for line in f), ndmin=2)
time = data[:, 1] - data[0, 1]  # packet ID, timestamp, then taxel values
taxels = data[:, 2:]

# Standard 192-channel iCub hand layout; IDs are zero-based.
# Edit these groups if your robot uses a different mapping.
AREAS = {
    "Thumb": range(48, 60),
    "Index": range(0, 12),
    "Middle": range(12, 24),
    "Ring": range(24, 36),
    "Little": range(36, 48),
    "Palm": range(96, 144),
}
if taxels.shape[1] != 192:
    sys.exit(f"Expected 192 hand channels, found {taxels.shape[1]}; check the layout.")

fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True, sharey=True,
                         constrained_layout=True)
for ax, (area, ids) in zip(axes.flat, AREAS.items()):
    norm = plt.Normalize(ids[0], ids[-1])
    for i in ids:
        ax.plot(time, taxels[:, i], color=plt.cm.viridis(norm(i)), linewidth=0.8,
                marker="." if len(time) == 1 else None)
    ax.set_title(area)
    ax.grid(alpha=0.25)
    fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap="viridis"), ax=ax,
                 label="Taxel ID", ticks=[ids[0], ids[-1]])
fig.suptitle("Right-hand touch — one trace per taxel")
fig.supxlabel("Time since first sample (s)")
fig.supylabel("Compensated taxel value (a.u.)")
output = log.with_name("taxels_by_area.png")
fig.savefig(output, dpi=180)
plt.close(fig)
print(f"Saved plot: {output.resolve()}")
PY
