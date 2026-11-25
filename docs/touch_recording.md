# Touch and BMA400 recording

From the repository root, start the robot's YARP stack, then run one of these:

```bash
./scripts/record_touch.sh           # right-hand touch
./scripts/record_touch.sh --bma400  # right-hand touch and acceleration
```

The script reuses running producers. It starts `run_touch.sh` if the
compensated skin source is missing. In BMA400 mode it also starts
`run_bma400.sh` if needed. Configure the Feather USB device in
`config/bma400.env`. Run the recorder on the PC connected to the Feather if it
needs to start the publisher. The producer executables must already be built.

Type `yes` to start and `stop` to finish. The script saves a unique directory
under `data/` containing `touch/data.log` and, in BMA400 mode,
`bma400/data.log`. YARP also writes an `info.log` alongside each log. Treat a
session reported as incomplete as a partial recording.

## Timestamps

Both dumpers run on the recording PC and save the packet sequence, sender
timestamp, receiver timestamp, and original payload. Both use the PC's system
clock for receiver timestamps. A sender stamp can be missing or invalid,
especially on compensated touch. The BMA400 payload includes the Feather
timestamp in microseconds, acceleration in m/s², and the serial reader's PC
receipt time in Unix seconds.

The two streams keep their original rates. Their receiver timestamps provide
an arrival-time axis. Use the overlap of both streams when comparing them.
For acquisition-time measurements, verify the source clock domains and
quantify timing variability with repeated contacts observed in both signals.

## Plot a saved session

With NumPy and Matplotlib installed:

```bash
python3 scripts/plot_touch.py data/touch_YYYYMMDD_HHMMSS_SUFFIX
```

This creates `taxels_by_area.png`. For the 192-channel hand layout, the plot
keeps the original six area groupings. If an acceleration log is present, its
three axes appear on the same receiver-time axis. The same command can plot
older touch-only session directories with a top-level `data.log`.
