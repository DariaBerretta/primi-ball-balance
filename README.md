# PRIMI ball balance

Experimental iCub software for ball detection, right-hand tactile acquisition,
robot motion, and event-camera processing. All inter-process sensor streams use
YARP. The programs run on the iCub cluster; this is not a standalone simulation.

## What runs

| Component | Program | Output or role |
| --- | --- | --- |
| RGB stereo | `run_rgb_cameras.sh` / `rgb_processor` | Annotated left/right images and ball detections |
| Event stereo | `run_event_cameras.sh` / `event_relay` | One remote event feed per eye, then local fan-out |
| Right-hand skin | `run_touch.sh` | Starts/reuses robot-side compensated touch; no local relay |
| BMA400 | `run_bma400.sh` / `bma400_reader` | Optional accelerometer stream over USB and YARP |
| Online pose | `scripts/run_moveenet_ofk.sh` | Optional MoveEnet + optical-flow/Kalman pose estimates |
| Event GUI | `scripts/run_event_control_gui.sh` | Optional event views and camera bias control |
| Monitoring | `scripts/run_monitor.sh` | RGB/event/skin visualizations and detection scope |
| Motion | `Robot_control` | **Separate** right-arm trajectory playback; not started by the sensor launchers |

The event relay is intentionally retained: multiple local viewers and pose
modules should not independently transfer the same high-rate events from the
robot. Touch recording connects directly to the robot-side compensated port.

The sensor launchers do **not** start the motion controller. `run_sensors.sh`
starts only RGB, event, and touch. The accelerometer, online pose estimator,
and visualization tools are opt-in.

## Build

The laptop build requires CMake, a C++17 compiler, YARP, event-driven,
OpenCV, hpe-core and Qt Widgets. Build from the repository root:

```bash
cmake -S . -B build
cmake --build build -j4
```

The camera-bias controller in `tools/camera_control/` is built separately on
`icub-ultrascale`; see `docs/event camera control gui.md`.

## Run

Start the already configured iCub cluster first, then use separate terminals
for acquisition and optional visualization:

```bash
./setup_cluster.sh
./run_sensors.sh
./scripts/run_monitor.sh
```

Use `--verbose` on `run_sensors.sh` to show launcher details. On shutdown,
stop acquisition/monitoring first and then run `./close_cluster.sh`.

Optional processes (started separately when needed):

```bash
./run_bma400.sh
./scripts/run_moveenet_ofk.sh
./scripts/run_event_control_gui.sh
./scripts/record_touch.sh --bma400
```

`record_touch.sh` starts a missing producer when possible and reuses existing
ones. It records YARP timestamps and payloads; **touch and BMA400 acquisition
clocks are not hardware-synchronized**. Receiver timestamps provide a shared
PC arrival-time axis, not guaranteed simultaneous sampling. See
`docs/touch_recording.md` for recording and plotting details.

## Motion and experimental assets

`Robot_control` sends commands to the physical right arm. It requires a
user-reviewed joint lookup table supplied with `--lookup FILE`; its existing
default path is machine-specific. Verify joint indices, end poses, and safety
clearance before enabling motion. The repository does **not** start motion
automatically. `data/recorded_arm_samples.txt` and the pose images are retained
as research references, not used by the sensor launchers.

## Configuration and technical notes

- `config/bma400.env`: USB device, serial settings, and YARP output.
- `config/moveEnetOFK.env`: camera side, estimator rates, and event filtering.
- `config/monitoring.env`: YARP input streams used by the monitor.
- `docs/Cluster Startup and Race Issues.md`: cluster startup dependencies.
- `docs/IO Architecture Guidelines.md`: port ownership and data flow.
- `docs/MoveEnetOFK-Online.md`: estimator behavior and limitations.

Keep YARP port names and camera source assignments consistent with the
cluster. Test any motion or camera-bias changes on the actual hardware only
under the appropriate safety procedure.
