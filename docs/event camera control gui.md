# Event camera control GUI

## Overview

`event_control_gui` runs on the laptop (`iiticublap267`) and displays the two raw event streams and the selected processed representations. It connects to the locally republished YARP outputs:

```
/primi-ball-balance/eventCam/left/raw:o
/primi-ball-balance/eventCam/right/raw:o
```

The existing acquisition pipeline and `zynqGrabber` are independent of the GUI.

A separate `camera_control` executable runs on `icub-ultrascale`. It controls the GEN3 camera biases through `/dev/i2c-0` (left `0x10`, right `0x11`) and exposes `/primi-ball-balance/cameraControl/rpc`. This does not require modifying `zynqGrabber` or the installed `event-driven` library.

## Build and run the laptop GUI

```bash
cmake -S . -B build
cmake --build build --target event_control_gui -j"$(nproc)"
./run_eventCam.sh                 # in another terminal, when not already running
./scripts/run_event_control_gui.sh
```

The GUI requires Qt Widgets, YARP, OpenCV, and `event-driven` as configured by the root `CMakeLists.txt`.

## Ultrascale camera controller

The controller's source is maintained in `tools/camera_control/` on the laptop. The matching source and `CMakeLists.txt` are copied to `~/primi-camera-control/` on the Ultrascale and compiled there as a separate CMake project:

```bash
cmake -S ~/primi-camera-control -B ~/primi-camera-control/build
cmake --build ~/primi-camera-control/build -j2
~/primi-camera-control/build/camera_control --inspect
```

`--inspect` identifies both cameras and reads their bias registers. The service can be started with `--serve` (read-only) or `--serve --allow-writes` (bias changes enabled). The GUI launcher may start it through the existing YARP run server; that requires the locally updated `scripts/run_event_control_gui.sh` to have been installed.

**Hardware changes are not applied when loading GUI settings.** The **Apply camera settings** button sends the four displayed sensitivity/refractory values to the controller. The register readback confirms programmed values but not the physical effect or absence of transients. Keep robot motion disabled when testing bias adjustments.

## GUI controls and saved profiles

- The top row renders raw events. The raw display time window is a **software arrival-time window**, not a camera or HPU setting.
- The lower row uses the installed `ev::EROS`, `ev::SAE`, and `ev::TOS` implementations.
- The optional salt-and-pepper filter uses the installed `ev::vNoiseFilter` **before the processed representation**, not the raw display.
- EROS kernel and decay changes update GUI processing only. They do not update MoveEnet's own EROS representation.
- On GUI versions with profile management, the `config/event_camera_profiles.json` file stores named profiles and the last selected profile. **New Profile** creates a new entry; **Save** on an existing profile requires overwrite confirmation; **Defaults** changes current GUI values without overwriting a profile. This machine-specific file is ignored by Git.

The profile-enabled GUI revision and the automated controller-launch script may exist only in the laptop checkout until committed and pushed. Check `git status` before replacing local files with versions from GitHub.

## Limitations

`zynqGrabber --filter` controls the acquisition-side temporal filter and remains a startup-time option. GUI controls do not modify that setting. The installed `event-driven` EROS/TOS implementation may cache certain parameters in static variables; independently configured left/right settings should be verified experimentally before treating them as equivalent to isolated representations.
