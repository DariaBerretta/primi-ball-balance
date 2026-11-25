#!/usr/bin/env python3
"""Plot iCub touch and optional BMA400 acceleration from a recording session."""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import numpy as np


HAND_AREAS = {
    "Thumb": slice(48, 60),
    "Index": slice(0, 12),
    "Middle": slice(12, 24),
    "Ring": slice(24, 36),
    "Little": slice(36, 48),
    "Palm": slice(96, 144),
}


# Read the numeric YARP dumper log without changing sample order.
def load_log(path):
    # Accept YARP Bottle parentheses and trailing backslashes in pasted logs.
    delimiters = str.maketrans("()", "  ")
    with path.open() as log:
        lines = (line.rstrip().rstrip("\\").translate(delimiters) for line in log)
        data = np.loadtxt(lines, ndmin=2)
    if not data.size:
        raise ValueError(f"No samples in {path}")
    return data


# Display taxel groups and optional acceleration on a shared receiver-time
# axis. No resampling or artificial time synchronization is introduced.
def plot_session(session):
    touch_path = session / "touch" / "data.log"
    acceleration = None
    if touch_path.is_file():
        touch = load_log(touch_path)
        time_column = 2  # Packet ID, sender time, receiver time, then taxels.
        acceleration_path = session / "bma400" / "data.log"
        if acceleration_path.parent.is_dir():
            acceleration = load_log(acceleration_path)
            if acceleration.shape[1] != 8:
                raise ValueError(f"Expected eight columns in {acceleration_path}")
        time_label = "Time since first receiver timestamp (s)"
    else:
        touch = load_log(session / "data.log")
        time_column = 1  # Original touch-only format has a single timestamp.
        time_label = "Time since first logged timestamp (s)"

    taxels = touch[:, time_column + 1:]
    if not taxels.shape[1]:
        raise ValueError("Touch log has no taxel columns.")

    first_time = touch[0, time_column]
    if acceleration is not None:
        first_time = min(first_time, acceleration[0, 2])
    touch_time = touch[:, time_column] - first_time
    marker = "." if len(touch_time) == 1 else None

    standard_hand = taxels.shape[1] == 192
    touch_rows = 3 if standard_hand else 1
    rows = touch_rows + int(acceleration is not None)
    fig = plt.figure(figsize=(13, 3.1 * rows + 0.7), constrained_layout=True)
    grid = fig.add_gridspec(rows, 2)
    first_axis = None

    if standard_hand:
        for index, (area, channels) in enumerate(HAND_AREAS.items()):
            ax = fig.add_subplot(grid[index // 2, index % 2],
                                 sharex=first_axis, sharey=first_axis)
            if first_axis is None:
                first_axis = ax
            ids = np.arange(channels.start, channels.stop)
            norm = Normalize(ids[0], ids[-1])
            ax.set_prop_cycle(color=plt.cm.viridis(norm(ids)))
            ax.plot(touch_time, taxels[:, channels], linewidth=0.8, marker=marker)
            ax.set_title(area)
            ax.grid(alpha=0.25)
            fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap="viridis"), ax=ax,
                         label="Taxel ID", ticks=[ids[0], ids[-1]])
            if index % 2 == 0:
                ax.set_ylabel("Compensated taxel value (a.u.)")
    else:
        first_axis = fig.add_subplot(grid[0, :])
        first_axis.plot(touch_time, taxels, linewidth=0.8, marker=marker)
        first_axis.set_title(f"{taxels.shape[1]} taxels")
        first_axis.set_ylabel("Compensated taxel value (a.u.)")
        first_axis.grid(alpha=0.25)

    if acceleration is not None:
        ax = fig.add_subplot(grid[-1, :], sharex=first_axis)
        imu_time = acceleration[:, 2] - first_time
        # The fifth, sixth, and seventh fields are acceleration X, Y, and Z.
        for column, axis in enumerate(("X", "Y", "Z"), start=4):
            ax.plot(imu_time, acceleration[:, column], label=axis, linewidth=1.0,
                    marker="." if len(imu_time) == 1 else None)
        ax.set_title("BMA400 accelerometer")
        ax.set_ylabel("Acceleration (m/s²)")
        ax.legend(ncol=3)
        ax.grid(alpha=0.25)

    fig.suptitle("Right-hand touch and acceleration" if acceleration is not None
                 else "Right-hand touch")
    fig.supxlabel(time_label)
    output = session / "taxels_by_area.png"
    fig.savefig(output, dpi=180)
    plt.close(fig)
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path, help="Recording session directory")
    args = parser.parse_args()
    try:
        output = plot_session(args.session)
    except (OSError, ValueError) as error:
        parser.exit(1, f"{error}\n")
    print(f"Saved plot to {output}")
