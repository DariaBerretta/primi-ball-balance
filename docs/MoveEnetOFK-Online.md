# PRIMI — MoveEnetOFK Online (upper body)

## Purpose

Runs **one** iCub event camera through EROS -> asynchronous GPU MoveEnet (7
upper-body joints) and SAE -> optical flow -> hpe-core latency-compensated
fusion. Publishes both an EROS+skeleton RGB overlay and a numeric joint stream
on YARP. **No robot motors are controlled.** This module is a sensor publisher;
`Robot_control` remains independent.

This integration **does not alter** `eventCam_reader.cpp`, `run_eventCam.sh`,
`run_monitor.sh`, or the existing robot controller. It consumes only the
locally republished stream, respecting `docs/IO Architecture Guidelines.md`.

## Requirements on iiticublap267

- YARP, event-driven, hpe-core C++ from the existing Robotology installation.
- `./.venv/bin/python`, with YARP Python, PyTorch 1.12.1+cu113, NumPy,
  OpenCV, and the existing hpe-core MoveEnet Python imports working.
- Checkpoint: `~/robot-code/hpe-core/example/movenet/models/upper.pth`.
- Running YARP namespace/server and `./run_eventCam.sh` **in a separate terminal**.
- One local source stream, e.g. `/primi-ball-balance/eventCam/left/raw:o`.

## Build

```bash
cd ~/robot-code/primi-ball-balance
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH="$HOME/robot-code/robotology-superbuild/build/install"
cmake --build build --target moveEnetOFK_online -j2
```

Do not install packages or start a second `yarpserver` from the pose launcher.
The existing YARP environment is reused.

## Start

Terminal A: launch the existing camera reader as you normally do.

Terminal B:

```bash
cd ~/robot-code/primi-ball-balance
./scripts/run_moveenet_ofk.sh
```

The launcher checks for the existing local event stream; it **will not**
launch/stop `zynqGrabber` or alter robot motors. `Ctrl+C` terminates only its
Python and C++ child processes. Set options in `config/moveEnetOFK.env`:
`CAMERA=left|right`, `F_NET`, `F_FLOW`, `F_VIS`, `ROI`, `CONFIDENCE_TH`, etc.

Preview (terminal C):

```bash
yarpview --name /primi-ball-balance/monitor/moveEnetOFK/overlay:i
```

Connect:

```bash
yarp connect /primi-ball-balance/moveEnetOFK/left/overlay:o \
  /primi-ball-balance/monitor/moveEnetOFK/overlay:i shmem
```

Numeric skeleton stream can be inspected with:

```bash
yarp read /primi-ball-balance/test/moveEnetOFK/sklt:i \
  /primi-ball-balance/moveEnetOFK/left/skeleton:o
```

The `yarp read` CLI is used in the same source/destination convention as the
existing PRIMI diagnostic scripts.

## YARP ports (selected side)

All port names begin with `/primi-ball-balance/moveEnetOFK/<camera>/`.

| Port suffix | Direction | Type | Purpose |
| --- | --- | --- | --- |
| `events:i` | input | `ev::AE` packet | Raw event packets from **local** `eventCam_reader` |
| `model/img:o` | output | `ImageMono` | EROS frames at requested `F_NET` |
| `model/sklt:i` | input | `Bottle` | Timestamped MoveEnet results |
| `eros:o` | output | `ImageMono` | EROS surface |
| `overlay:o` | output | `ImageRgb` | EROS and upper-body skeleton |
| `skeleton:o` | output | `Bottle` | 13-slot filtered pose plus confidence |
| `velocity:o` | output | `Bottle` | 13-slot optical-flow velocity plus confidence |

The sidecar listens on `.../inference/img:i` and produces
`.../inference/sklt:o`, both connected by the launcher.

### Skeleton protocol

The skeleton uses the **hpe-core compatible** Bottle shape:

```
("SKLT" (x0 y0 x1 y1 ... x12 y12 c0 c1 ... c12))
```

The YARP **envelope Stamp** (`count`, `time`) identifies the publication and
provides the **original** event source timestamp. Internal SAE/LC calculations
use seconds relative to the first input packet, rather than large absolute
timestamps; this prevents precision loss in triplet optical flow. Coordinates are **pixel coordinates** in
`WIDTH x HEIGHT` (not normalized). `confidence` is zero for unavailable and
expired joints; those coordinates are zeroed. The 13 hpe-core indices are:

```
0 head, 1 shoulderR, 2 shoulderL, 3 elbowR, 4 elbowL,
5 hipL, 6 hipR, 7 handR, 8 handL,
9 kneeR, 10 kneeL, 11 footR, 12 footL
```

**Only indices 0,1,2,3,4,7,8 are estimated.** The remaining six entries are
zero; never interpret them as measured positions. The raw network output is
not published as the fused skeleton; it is used internally as measurements.

Velocity uses the same structure but starts with `"JVEL"`; its 26 numbers
are `(vx,vy)` pairs in pixels/second, zeroed for invalid joints.

### Asynchrony and latency compensation

- C++ event acquisition and SAE updates never call MoveEnet.
- A distinct request worker sends **at most one inference request in flight**;
  no unbounded frame queue. When the network is slower than `F_NET`, skipped
  samples reduce *effective* inference frequency. The two frequencies are
  **requested ceilings**; throughput depends on hardware and event rate.
- Python executes each request serially and **echoes the original image
  envelope Stamp**, including unique request ID and source time. There is no
  `count+time` mixing or substitution of the inference latency as timestamp.
- The response worker matches request IDs and corrects the filter at the
  **image acquisition time** in an internally relative clock (to preserve
  microsecond precision), replaying accumulated optical flow through
  `hpecore::multiJointLatComp` (as in offline MoveEnetOFK).
- The flow worker independently runs at `F_FLOW`, using a locked SAE snapshot
  and tracking only valid upper-body joints.
- The visualization worker publishes at `F_VIS`.
- The hpe-core `skeleton13` flow overload multiplies velocity by 1.4; the
  module cancels this factor to match the existing offline experiment.

### Known limitations / prerequisites for reliable OF

1. **Sensor timestamps:** the upstream `event-driven` library may be built
   with `VLIB_ENABLE_TS=OFF`. In that case individual `ev::AE` timestamps are
   unavailable and the code uses packet envelope timestamps to update SAE.
   EROS and network inference still work; **optical flow accuracy and latency
   compensation quality are not yet demonstrated**. The C++ process logs a
   warning if event timestamps are disabled. Do not manufacture per-event
   timestamps. Confirm the local event-driven build, input stamp monotonicity,
   and actual optical-flow output on iCub before relying on results.
2. **hpe-core LC:** upstream `multiJointLatComp` has uninitialized per-joint
   velocity-history members if default-initialized. This module
   **value-initializes** the tracker and bounds active-joint age to less than
   4 seconds to avoid upstream history buffer overflow. This should still be
   tested carefully at the real flow rates, and ideally fixed upstream.
3. `measUV` is accepted for configuration parity with the offline code, but
   upstream `multiJointLatComp::initialise()` does not use its third argument.
4. Missing/low-confidence joint outputs are **not** passed into the filter.
   Stale joints stop receiving flow updates and are hidden after
   `TRACK_TIMEOUT`; accurate reacquisition and confidence calibration need
   evaluation on live person-in-view data.
5. If event packet timestamps jump **backward**, the process stops instead of
   silently corrupting the Kalman history. Check the source timestamps.
6. CPU cost: copying SAE at 200 Hz can stress the laptop. Monitor effective
   rates and adjust `F_FLOW` (e.g. 100 Hz) if required.
7. Port installation/compilation has **not been tested on iiticublap267**;
   run the build and report diagnostics before considering the module ready.

## First validation

1. Before enabling the controller, launch only the reader and online pipeline.
2. Confirm C++ `[stats] events` and `net_requests` are increasing.
3. Confirm `net_results` and `flow_steps` increase independently.
4. Check `.../eros:o` and `.../overlay:o` in `yarpview`.
5. With a human visible, verify head, shoulders, elbows, and hands are
   correctly aligned and confident; then verify optical-flow tracking between
   inference updates. A synthetic black image is not suitable for this test.
6. Check requested vs achieved frequencies and latency by observing logs.
7. **Do not connect the filtered skeleton to motor-control commands** before
   timestamp, pose validity, confidence, and coordinate-frame tests pass.

## Configurable event denoising (November 2026)

The C++ module optionally filters the camera event stream **before** updating
both EROS (MoveEnet input) and SAE (optical flow). The original
`/primi-ball-balance/eventCam/<camera>/raw:o` stream remains unchanged.

Parameters in `config/moveEnetOFK.env`:

| Setting | Initial value | Meaning |
| --- | --- | --- |
| `NOISE_FILTER` | `true` | Enable/disable filtering, including for A/B comparisons. |
| `NOISE_REFRACTORY_MS` | `1.0` | Reject same-polarity events at the same pixel within this interval. Set `0` to disable this stage. |
| `NOISE_SPATIAL_MS` | `5.0` | Require an event at a **different neighboring pixel** within the preceding interval. Set `0` to disable this stage. |
| `NOISE_RADIUS` | `1` | Neighbor radius in pixels; allowed range `1..3`. |

The spatial stage excludes the center pixel to prevent an isolated hot pixel
from supporting its own repeated events. Both stages update their temporal
history even when events are rejected, allowing a neighboring event to
subsequently provide spatial support. This is a **causal event filter**, not an
image blur. It affects the inference input and optical-flow SAE, so joint
tracking must be re-validated with and without filtering.

At startup the C++ module logs the chosen settings. Every five seconds
`[stats]` reports `raw_events`, `events_used`, `noise_refractory`,
`noise_spatial`, and `invalid`; they are cumulative counts. The intended
accounting is `raw_events = events_used + noise_refractory + noise_spatial`.
An event outside the configured sensor bounds increments `invalid` and is
excluded from `raw_events`.

**Timing warning:** when the installed event-driven library was built without
individual AE timestamps, the filter and SAE use packet-level timestamps.
The spatial/refractory windows then have only packet-level precision; test
tracking quality before using filtered joint outputs for robot control.

To compare behavior, run once with `NOISE_FILTER=false`, then with
`NOISE_FILTER=true`, keeping other settings unchanged. Stop the existing
pipeline cleanly (Ctrl+C) before each restart. Do not restart the remote
`zynqGrabber` solely to change these software filters.

## Simultaneous MoveEnet vs OF+KF visualization (November 2026)

`/primi-ball-balance/moveEnetOFK/<camera>/overlay:o` now shows both
upper-body skeletons on the **same filtered EROS** frame:

- **RED circles/lines:** the most recent **raw MoveEnet** pose (not fused or
  propagated), updated only when an inference response arrives (at most
  `F_NET`). It is held until the next response; the overlay legend displays
  the age since the original input frame timestamp. Invalid/old poses expire
  after `TRACK_TIMEOUT`. If an inference returns zero valid joints, the old
  red pose is cleared immediately.
- **GREEN crosses/lines:** the current optical-flow-propagated and
  latency-compensated Kalman estimate, updated by flow steps (nominally
  `F_FLOW`) and corrected when a new MoveEnet result arrives.

The colored skeletons have different joint markers so they remain visible
when their estimated coordinates coincide. Colors are converted from OpenCV
BGR to YARP PixelRgb explicitly (red remains red in yarpview).

The image legend shows achieved/target update rates (measured over roughly
1 second) and the age in milliseconds of the last raw MoveEnet frame.
These are independent of the GUI image rate.

The combined **image** runs at `F_VIS` (default 30 Hz). It does **not** transmit
200 RGB frames/sec: with 640x480 images that would impose unnecessary
bandwidth and CPU load. The underlying estimators remain asynchronous.

The two additional **numerical pose streams** reflect their true update rates:

| Port | When a new SKLT message is published |
| --- | --- |
| `<base>/movenet/skeleton:o` | Upon each matched and decoded MoveEnet reply (<= `F_NET`); envelope time = original EROS input-frame time. |
| `<base>/ofk/skeleton:o` | Upon each actual OF/KF update with fresh events (<= `F_FLOW`); envelope time = processed event-packet time. |
| `<base>/skeleton:o` | Legacy fused output, still emitted at `F_VIS` for existing subscribers. |

The numerical SKLT payload is unchanged (`SKLT` + list of 26 x/y coordinates
and 13 confidence values). Only the seven upper-body joints can be valid.
`net_replies` is now reported separately from `net_results` (which counts
replies with at least one valid detected joint).

The new optional display flags in `config/moveEnetOFK.env` are
`SHOW_MOVENET=true` and `SHOW_OFK=true`. Turning either `false` hides that
skeleton **only from the combined overlay**; it does not disable inference,
flow, Kalman updates, or their numerical ports. The previously configured
automatic yarpview launcher continues to show the same `overlay:o` port.

**Interpretation:** a fixed red pose between network replies and a moving
green pose is expected, not a synchronization failure. If the red pose jumps
but the green pose tracks continuously, OF+KF propagation is working. For
rate/latency evaluation, count or log numerical pose messages: `F_NET` and
`F_FLOW` are target rates; achieved rates depend on inference time, event
arrival, CPU load, and port subscribers.
