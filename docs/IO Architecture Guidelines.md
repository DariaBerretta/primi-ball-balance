# PRIMI data flow and port ownership

This document describes the code in `feature/live-event-eros-control` on
25 November 2026. It distinguishes running modules from future work.

## Acquisition and local outputs

Each high-bandwidth remote camera feed is received once by a project reader
on the laptop. The event relay is essential when vFramer, the GUI, and pose
estimation subscribe together: without it each independently transfers the
full event stream across the robot-to-laptop network.

| Source | Local reader input | Local outputs |
| --- | --- | --- |
| `/zynqGrabber/left/AE:o` | `/primi-ball-balance/eventCameraStream/left/sink:i` | `/primi-ball-balance/eventCameraStream/left/source:o` |
| `/zynqGrabber/right/AE:o` | `/primi-ball-balance/eventCameraStream/right/sink:i` | `/primi-ball-balance/eventCameraStream/right/source:o` |
| `/grabber` (side-by-side stereo RGB) | `/primi-ball-balance/rgbCam/stereo:i` | `/primi-ball-balance/rgbCam/{left,right}/{processed,detection}:o` |
| `/icub/skin/right_hand_comp` | None | Consumers subscribe directly to this robot-side output |

The table uses `{left,right}` to mean two separate port names. The RGB reader
splits the combined image into left/right halves, blurs for HSV-based ball
detection, and publishes an annotated view and a six-element numerical result:

```text
[valid, center_x_px, center_y_px, radius_px, confidence, source_time_s]
```

The event reader copies `ev::AE` packets and their original envelopes; it
**does not** apply the local GUI or pose-estimator noise filters. Only the
right-hand compensated skin stream is implemented. Left-hand skin and a skin
summary are not active outputs.

The three main launchers are `run_event_cameras.sh`, `run_rgb_cameras.sh`, and
`run_touch.sh`. `run_sensors.sh` starts exactly these three. Both camera
links use `fast_tcp` by default. The touch launcher starts or reuses
`skinManager` but creates no local ports; the monitor and recorder connect to
`/icub/skin/right_hand_comp` over `tcp`. Local event consumers normally use
`shmem`; `vFramer` is an exception because it requests `fast_tcp`.

## Optional producers

`Robot_control` is a standalone right-arm trajectory player. It opens
the robot's right-arm controlboard directly; it does not currently
consume RGB detections, event poses, touch, or BMA400 data. It is
never launched automatically by the sensor startup scripts.

`run_bma400.sh` starts the optional USB accelerometer reader. It publishes
`/primi-ball-balance/imu/bma400:o` as a YARP Bottle containing:

```text
[feather_time_us, acceleration_x_m_s2, acceleration_y_m_s2,
 acceleration_z_m_s2, pc_receipt_unix_s]
```

The Feather acquisition time and the PC clock are **different domains**.
A PC receipt timestamp does not imply hardware synchronization.

## Event processing and camera control

`scripts/run_moveenet_ofk.sh` optionally starts a C++ event processor and
Python MoveEnet inference worker for a single configured eye. It reads the
local `/primi-ball-balance/eventCameraStream/<eye>/source:o` feed and maintains filtered
EROS and SAE representations. Pose inference, flow updates, and visualization
run at their separate configured rates. Outputs include:

```text
/primi-ball-balance/moveEnetOFK/<eye>/eros:o
/primi-ball-balance/moveEnetOFK/<eye>/overlay:o
/primi-ball-balance/moveEnetOFK/<eye>/skeleton:o
/primi-ball-balance/moveEnetOFK/<eye>/velocity:o
/primi-ball-balance/moveEnetOFK/<eye>/movenet/skeleton:o
/primi-ball-balance/moveEnetOFK/<eye>/ofk/skeleton:o
```

The online estimator uses `ev::window<ev::AE>` for acquisition and
the native `ev::vNoiseFilter` before EROS and SAE updates. See
`docs/MoveEnetOFK-Online.md` for the mathematical/timestamp constraints.

`event_control_gui` separately reads both *local* raw event outputs and
renders EROS, SAE, or TOS. Its filtering and representation parameters affect
the GUI only, not the upstream grabber or the pose estimator. Camera bias
changes are explicitly applied through the independent
`/primi-ball-balance/cameraControl/rpc` service on `icub-ultrascale`.
Loading a saved GUI profile does **not** modify physical camera biases.

## Visualization and recording

`scripts/run_monitor.sh` subscribes to locally published RGB images,
RGB detections, event streams, and right-hand compensated taxels. Its screen
layout comes from the active window manager, not fixed monitor coordinates
in `config/monitoring.env`.

`scripts/record_touch.sh` starts one `yarpdatadumper` per selected stream.
The touch recorder and skin viewer both connect directly to
`/icub/skin/right_hand_comp`. Both data consumers run on the laptop.
Neither requires an intermediate local skin-forwarding process. With `--bma400`, the second dumper
subscribes to the local BMA400 output. Both write sender and receiver
metadata in separate logs, at their native sample rates. They are aligned by
PC arrival time for plotting, **not** sampled synchronously. See
`docs/touch_recording.md` for exact files and interpretation.

## Ownership and restart rules

- `setup_cluster.sh` and `close_cluster.sh` manage the robot interface and
  Cartesian support processes; they do not start the sensor launchers.
- Camera launchers manage their own readers and robot-side sources. The
  touch launcher starts/reuses skinManager. Avoid restarting a remote source
  while other processes are using it.
- Monitor, recorder, event GUI, and pose estimation run independently; their
  shutdown must not terminate unrelated YARP producers.
- Physical motion is **never** started by `run_sensors.sh` or the GUI. Test
  `Robot_control` separately, with the recorded joint trajectory and robot
  clearance verified before issuing commands.

The current implementation is an acquisition/perception and experimental
motion toolkit. Closed-loop multimodal ball balancing remains future work.
