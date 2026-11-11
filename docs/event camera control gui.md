# Event camera control GUI

## Purpose

A standalone Qt viewer for both locally republished event streams and their EROS representations. It subscribes only to `/primi-ball-balance/eventCam/{left,right}/raw:o` and does not create extra robot-to-laptop event connections or restart acquisition.

## Build and run

Add `CMakeLists.event_control.txt` to the existing root `CMakeLists.txt` and rebuild:

```bash
cmake -S . -B build
cmake --build build -j"$(nproc)" --target event_control_gui
./scripts/run_event_control_gui.sh
```

Requires the existing YARP, event-driven, OpenCV, and Qt5/Qt6 Widgets development packages. Start `./run_eventCam.sh` in a separate terminal. The viewer reconnects to the local streams after acquisition restarts.

## Controls

* Four panes: left/right raw events and left/right EROS.
* Raw accumulation is a host **arrival-time display window** in milliseconds. Event polarity is color coded.
* EROS is calculated from raw (unfiltered) events using the event-driven algorithm: multiply the event's padded spatial neighbourhood by `decay^(1/kernel)` and set the centre to 1. Changing kernel or decay resets the respective surface.
* Left/right link checkboxes synchronize sliders.
* Camera settings are sent **only upon clicking Apply camera settings**. These are hardware bias-control values, **not calibrated physical units**.

## Robot-side live control

`patches/zynqGrabber-live-bias-control.patch` is a proposed change to `robotology/event-driven`, introducing `/zynqGrabber/control/rpc` in `zynqModule.cpp`. It uses the existing camera-specific I2C operations and accepts Bottle RPC commands such as `set sensitivity left 70` and `set refractory right 1`.

Apply it in the **actual Ultrascale checkout** from which `zynqGrabber` is built:

```bash
cd /path/to/robotology-event-driven
git apply --check /path/to/primi-ball-balance/patches/zynqGrabber-live-bias-control.patch
git apply /path/to/primi-ball-balance/patches/zynqGrabber-live-bias-control.patch
# Rebuild and install zynqGrabber using the established Ultrascale process.
```

**Do not deploy without first reviewing and compiling on the installed version.** The upstream bias setters do not return register-write success, so a positive RPC reply does not prove the hardware accepted the change. Test conservatively with robot motion disabled.

## Limitations and validation

* EROS here is **independent** of `moveEnetOFK_online`: changing GUI parameters does not alter MoveEnet's own filtered EROS processing.
* The `--filter` HPU acquisition setting remains startup-only. There is no verified safe live setter yet.
* The upstream `ev::EROS` caches the decay with a function-local static, so the GUI implements its spatial update directly without that stale-parameter behavior.
* Image size is currently 640x480. Use one GUI instance at a time due to fixed YARP port names.
* The new code **has not been compiled or exercised against the robot's YARP/event-driven/Qt versions or camera hardware**.

Manual acceptance checks: build on `iiticublap267`, display live left/right frames and event rates, change each EROS slider individually, test kernel changes without crashes, restart acquisition and check reconnection, then validate the robot-side RPC and verify sensor response before relying on live hardware control.
