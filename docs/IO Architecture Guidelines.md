# PRIMI Ball-Balance I/O Architecture

## 1. Purpose

The PRIMI ball-balancing system uses several sensors distributed across the iCub robot and the local processing computer:

* stereo RGB cameras;
* left and right event cameras;
* tactile skin;
* joint encoders;
* robot control interfaces.

Many of these sensor streams originate on remote computers embedded in or connected to the robot. High-bandwidth sources, particularly event cameras and RGB cameras, should not be transmitted independently to every visualization, recording, and processing module.

The system therefore follows a centralized acquisition architecture:

> Each hardware or remote sensor stream is received once by one local acquisition module. All other project modules consume data from that acquisition module’s local output ports.

This separates hardware communication from project-level processing and provides a consistent structure across all sensor modalities.

---

## 2. Core architecture

The general data path is:

```text
Remote hardware source
        |
        | One robot-to-local connection
        v
Local acquisition module
        |
        +--> Raw local output
        +--> Processed local output
        +--> Detection or summary output
        |
        +--> Controller
        +--> Visualizer
        +--> Recorder
        +--> Debugging tools
```

Only the acquisition module communicates directly with the original hardware source.

All visualizers, controllers, recorders, and downstream processing modules connect to ports under the project namespace:

```text
/primi-ball-balance/...
```

They should not normally connect directly to ports such as:

```text
/grabber
/zynqGrabber/...
/icub/skin/...
```

These hardware-facing names remain internal to the relevant acquisition launcher and reader module.

---

## 3. Motivation

### 3.1 Reduced network load

A YARP output may have several consumers, but each connection can require the source computer to transmit another copy of the stream.

For example:

```text
/zynqGrabber/left/AE:o
        +--> eventCam_reader
        +--> visualizer
        +--> recorder
```

This may cause the Ultrascale computer to transmit the same event stream several times.

The preferred structure is:

```text
/zynqGrabber/left/AE:o
        |
        | One remote connection
        v
eventCam_reader
        |
        +--> visualizer
        +--> recorder
        +--> event processing
```

The high-bandwidth remote stream crosses the network once. Fan-out happens locally.

### 3.2 Consistent preprocessing

When every consumer connects directly to a hardware source, each consumer may:

* interpret the data differently;
* apply different preprocessing;
* use different timestamps;
* define different validity rules;
* produce inconsistent results.

A centralized acquisition module establishes one shared representation for the rest of the project.

### 3.3 Easier restart and process management

Each launcher owns one section of the pipeline:

* hardware process;
* local acquisition process;
* relevant connections;
* stale-port cleanup;
* shutdown behaviour.

A visualization script should not need to start or restart the hardware.

### 3.4 Hardware abstraction

Downstream modules should not need to know whether the source is:

* the real iCub;
* `icubSim`;
* a recorded dataset;
* a test generator;
* another computer.

They depend only on stable project ports.

For example, a controller should read:

```text
/primi-ball-balance/encoders/left_arm:o
```

rather than opening its own direct connection to:

```text
/icub/left_arm
```

---

# 4. Port naming convention

Project ports should follow:

```text
/primi-ball-balance/<sensor>/<side-or-part>/<stream>:<direction>
```

Where:

```text
:i    input owned by a project module
:o    output published by a project module
```

Examples:

```text
/primi-ball-balance/eventCam/left/raw:o
/primi-ball-balance/eventCam/right/raw:o

/primi-ball-balance/rgbCam/left/processed:o
/primi-ball-balance/rgbCam/right/detection:o

/primi-ball-balance/skin/left_hand/compensated:o
/primi-ball-balance/skin/right_hand/summary:o

/primi-ball-balance/encoders/left_arm:o
```

An input port should not be treated as a downstream source.

For example:

```text
/primi-ball-balance/eventCam/left:i
```

is the input owned by `eventCam_reader`. Visualizers and processing modules should connect to an output such as:

```text
/primi-ball-balance/eventCam/left/raw:o
```

---

# 5. Event-camera architecture

## 5.1 Hardware sources

The event streams originate from:

```text
/zynqGrabber/left/AE:o
/zynqGrabber/right/AE:o
```

These ports are produced by `zynqGrabber` on the Ultrascale computer.

## 5.2 Acquisition inputs

The remote streams connect once to:

```text
/primi-ball-balance/eventCam/left:i
/primi-ball-balance/eventCam/right:i
```

The connections are:

```bash
yarp connect /zynqGrabber/left/AE:o \
    /primi-ball-balance/eventCam/left:i fast_tcp

yarp connect /zynqGrabber/right/AE:o \
    /primi-ball-balance/eventCam/right:i fast_tcp
```

## 5.3 Required local outputs

`eventCam_reader` should republish the received packets on:

```text
/primi-ball-balance/eventCam/left/raw:o
/primi-ball-balance/eventCam/right/raw:o
```

The resulting architecture is:

```text
/zynqGrabber/left/AE:o
        |
        | fast_tcp
        v
/primi-ball-balance/eventCam/left:i
        |
        | eventCam_reader
        v
/primi-ball-balance/eventCam/left/raw:o
        |
        +--> event visualizer
        +--> event recorder
        +--> event preprocessing
        +--> later event-based detection modules
```

The same applies to the right camera.

## 5.4 Display configuration

Visualizers should use:

```bash
EVENT_LEFT_SOURCE="/primi-ball-balance/eventCam/left/raw:o"
EVENT_RIGHT_SOURCE="/primi-ball-balance/eventCam/right/raw:o"
```

They should not connect directly to the `zynqGrabber` ports.

## 5.5 Future processed outputs

Additional event-camera modules may later publish:

```text
/primi-ball-balance/eventCam/left/frame:o
/primi-ball-balance/eventCam/right/frame:o

/primi-ball-balance/eventCam/left/detection:o
/primi-ball-balance/eventCam/right/detection:o

/primi-ball-balance/eventCam/left/rate:o
/primi-ball-balance/eventCam/right/rate:o
```

These outputs should be derived from the locally republished raw stream.

---

# 6. RGB-camera architecture

## 6.1 Hardware source

The remote RGB camera currently publishes:

```text
/grabber
```

This appears to contain the left and right images concatenated into one stereo image.

## 6.2 Acquisition input

The hardware source connects once to:

```text
/primi-ball-balance/rgbCam/stereo:i
```

The connection is:

```bash
yarp connect /grabber \
    /primi-ball-balance/rgbCam/stereo:i tcp
```

## 6.3 Local outputs

`rgbCam_reader` already follows the desired structure.

It publishes separated raw images:

```text
/primi-ball-balance/rgbCam/left/raw:o
/primi-ball-balance/rgbCam/right/raw:o
```

It publishes annotated processed images:

```text
/primi-ball-balance/rgbCam/left/processed:o
/primi-ball-balance/rgbCam/right/processed:o
```

It publishes numerical ball detections:

```text
/primi-ball-balance/rgbCam/left/detection:o
/primi-ball-balance/rgbCam/right/detection:o
```

The complete architecture is:

```text
/grabber
    |
    | One remote connection
    v
/primi-ball-balance/rgbCam/stereo:i
    |
    | rgbCam_reader
    |
    +--> left/raw:o
    +--> right/raw:o
    +--> left/processed:o
    +--> right/processed:o
    +--> left/detection:o
    +--> right/detection:o
```

## 6.4 Downstream use

Visualizers should use:

```text
/primi-ball-balance/rgbCam/left/raw:o
/primi-ball-balance/rgbCam/right/raw:o
```

or:

```text
/primi-ball-balance/rgbCam/left/processed:o
/primi-ball-balance/rgbCam/right/processed:o
```

Controllers should use:

```text
/primi-ball-balance/rgbCam/left/detection:o
/primi-ball-balance/rgbCam/right/detection:o
```

No downstream module should connect directly to `/grabber`.

---

# 7. Tactile-skin architecture

## 7.1 Hardware and skinManager ports

The raw skin values originate from ports such as:

```text
/icub/skin/left_hand
/icub/skin/right_hand
```

`skinManager` receives the raw values and publishes compensated values:

```text
/icub/skin/left_hand_comp
/icub/skin/right_hand_comp
```

The compensated ports normally have:

```text
approximately 0 at rest
larger values under contact
```

## 7.2 Acquisition inputs

The compensated ports should connect once to `touch_reader`:

```text
/icub/skin/left_hand_comp
        |
        v
/primi-ball-balance/skin/left_hand:i
```

```text
/icub/skin/right_hand_comp
        |
        v
/primi-ball-balance/skin/right_hand:i
```

## 7.3 Required local outputs

`touch_reader` should republish the complete tactile vectors as:

```text
/primi-ball-balance/skin/left_hand/compensated:o
/primi-ball-balance/skin/right_hand/compensated:o
```

It may also publish reduced summaries:

```text
/primi-ball-balance/skin/left_hand/summary:o
/primi-ball-balance/skin/right_hand/summary:o
```

A possible summary format is:

```text
[
    contact_valid,
    strongest_taxel_index,
    strongest_taxel_value,
    total_activation,
    timestamp
]
```

The exact summary format should be formally documented before downstream modules depend on it.

## 7.4 Skin GUI

The skin GUI should consume the local republished data:

```text
/primi-ball-balance/skin/right_hand/compensated:o
        |
        v
/skinGui/right_hand:i
```

rather than creating another direct robot-to-laptop connection from:

```text
/icub/skin/right_hand_comp
```

This keeps one remote compensated stream per hand.

---

# 8. Proprioception architecture

## 8.1 Hardware interfaces

`proprio_reader` currently connects through YARP `remote_controlboard` devices:

```text
/icub/left_arm
/icub/right_arm
/icub/head
```

It reads encoder values and republishes them locally.

## 8.2 Local outputs

The current project outputs are:

```text
/primi-ball-balance/encoders/left_arm:o
/primi-ball-balance/encoders/right_arm:o
/primi-ball-balance/encoders/head:o
```

The architecture is:

```text
/icub/left_arm
        |
        | remote_controlboard
        v
proprio_reader
        |
        v
/primi-ball-balance/encoders/left_arm:o
        |
        +--> controller
        +--> logger
        +--> visualizer
        +--> state estimator
```

The same structure applies to the right arm and head.

## 8.3 Separation of reading and control

`proprio_reader` should be the project’s centralized source of joint-state information.

Other project modules should not independently create duplicate encoder-reading threads.

In particular, `Robot_control` should consume:

```text
/primi-ball-balance/encoders/left_arm:o
/primi-ball-balance/encoders/right_arm:o
/primi-ball-balance/encoders/head:o
```

for state observation.

The controller may still require direct `remote_controlboard` interfaces to send motor commands. The separation is:

```text
proprio_reader
    owns state acquisition and encoder publication

Robot_control
    owns motor-command interfaces
    consumes local encoder outputs
```

---

# 9. Controller-facing interface

The controller should depend only on project-local outputs.

## RGB detections

```text
/primi-ball-balance/rgbCam/left/detection:o
/primi-ball-balance/rgbCam/right/detection:o
```

## Event-camera information

Initially:

```text
/primi-ball-balance/eventCam/left/raw:o
/primi-ball-balance/eventCam/right/raw:o
```

Later, preferably:

```text
/primi-ball-balance/eventCam/left/detection:o
/primi-ball-balance/eventCam/right/detection:o
```

## Tactile information

```text
/primi-ball-balance/skin/left_hand/compensated:o
/primi-ball-balance/skin/right_hand/compensated:o
```

or the reduced summaries:

```text
/primi-ball-balance/skin/left_hand/summary:o
/primi-ball-balance/skin/right_hand/summary:o
```

## Proprioception

```text
/primi-ball-balance/encoders/left_arm:o
/primi-ball-balance/encoders/right_arm:o
/primi-ball-balance/encoders/head:o
```

The controller should not depend directly on:

```text
/grabber
/zynqGrabber/...
/icub/skin/...
```

This keeps the control layer independent of the physical sensor topology.

---

# 10. Carrier policy

## 10.1 Remote high-bandwidth streams

For high-bandwidth event streams:

```text
fast_tcp
```

is the current preferred carrier.

Example:

```bash
yarp connect /zynqGrabber/left/AE:o \
    /primi-ball-balance/eventCam/left:i fast_tcp
```

## 10.2 Lower-bandwidth remote streams

For data where reliable delivery is more important than minimum latency:

```text
tcp
```

is the default choice.

Examples include:

* tactile vectors;
* detections;
* encoder vectors;
* status information.

## 10.3 Local same-machine communication

When both processes are on the same computer, local connections should use:

```text
shmem
```

where the port types and YARP installation support it.

Example:

```bash
yarp connect /primi-ball-balance/eventCam/left/raw:o \
    /event-viewer/left:i shmem
```

This avoids using a TCP network transport for communication between local processes.

## 10.4 Carrier choice remains configurable

Launchers should expose carriers through environment variables:

```bash
CARRIER="${CARRIER:-tcp}"
```

or more specifically:

```bash
REMOTE_CARRIER="${REMOTE_CARRIER:-fast_tcp}"
LOCAL_CARRIER="${LOCAL_CARRIER:-shmem}"
```

This permits testing without modifying source code.

---

# 11. Launcher ownership

Each launcher should own a well-defined pipeline.

## Event-camera launcher

Owns:

```text
zynqGrabber
eventCam_reader
remote-to-local event connections
stale event-port cleanup
```

It should not own event visualizers.

## RGB-camera launcher

Owns:

```text
remote RGB yarpdev process
rgbCam_reader
/grabber -> rgbCam_reader connection
stale RGB-port cleanup
```

It should not own RGB viewers unless explicitly designed as a combined demonstration launcher.

## Skin launcher

Owns:

```text
skinManager
touch_reader
skinManager -> touch_reader connections
optional skin GUI connection
```

The robot startup script remains responsible for:

```text
yarprobotinterface
skin boards
general robot hardware startup
```

## Proprioception launcher

Owns:

```text
proprio_reader
its project encoder-output ports
```

## Controller launcher

Owns:

```text
Robot_control
controller input connections
motor-command interfaces
```

It should not restart sensor hardware.

---

# 12. Restart behaviour

Each acquisition launcher should support a complete restart of the processes it owns.

The expected sequence is:

```text
1. Stop the previous local acquisition process.
2. Stop the corresponding remote hardware process, when owned by the launcher.
3. Remove stale YARP registrations belonging to that pipeline.
4. Start the remote source.
5. Wait for the remote output port.
6. Start the local acquisition module.
7. Wait for the local input and output ports.
8. Connect the remote source to the local acquisition input.
9. Leave downstream local consumers independent.
```

A launcher should not unregister or terminate ports belonging to another pipeline.

For example, the RGB launcher may manage:

```text
/grabber
/primi-ball-balance/rgbCam/stereo:i
```

but should not modify:

```text
/primi-ball-balance/eventCam/...
```

---

# 13. Module status

## Already close to the target architecture

### `rgbCam_reader`

It already:

* receives one remote stereo stream;
* separates left and right images;
* publishes local raw images;
* publishes local processed images;
* publishes local detection vectors.

### `proprio_reader`

It already:

* centralizes encoder acquisition;
* publishes project-local encoder vectors.

It still requires downstream consumers to stop opening duplicate state-reading interfaces.

## Requires republishing outputs

### `eventCam_reader`

Currently it receives event packets and reports packet counts.

It should additionally publish:

```text
/primi-ball-balance/eventCam/left/raw:o
/primi-ball-balance/eventCam/right/raw:o
```

### `touch_reader`

Currently it receives tactile vectors and prints a summary.

It should additionally publish:

```text
/primi-ball-balance/skin/left_hand/compensated:o
/primi-ball-balance/skin/right_hand/compensated:o
```

Optional summary outputs should also be added.

## Requires architectural cleanup

### `Robot_control`

It currently duplicates encoder acquisition.

It should instead:

* subscribe to `proprio_reader`;
* retain only the interfaces required for sending motor commands;
* consume local sensor and detection ports;
* avoid knowledge of remote sensor source names.

---

# 14. Final system structure

The intended complete system is:

```text
REMOTE SOURCES
==============

/zynqGrabber/left/AE:o
/zynqGrabber/right/AE:o
/grabber
/icub/skin/left_hand_comp
/icub/skin/right_hand_comp
/icub/left_arm
/icub/right_arm
/icub/head

        |
        | One acquisition path per source
        v

LOCAL ACQUISITION MODULES
=========================

eventCam_reader
rgbCam_reader
touch_reader
proprio_reader

        |
        | Project-local outputs
        v

PROJECT DATA BUS
================

/primi-ball-balance/eventCam/...
/primi-ball-balance/rgbCam/...
/primi-ball-balance/skin/...
/primi-ball-balance/encoders/...

        |
        +--> controller
        +--> visualizers
        +--> recorders
        +--> debugging tools
        +--> future perception modules
```

The governing rule is:

> Hardware ports terminate at acquisition modules. All project components communicate through stable `/primi-ball-balance/...` output ports.

This architecture should be applied consistently to every new sensor, processor, visualizer, recorder, and controller added to the project.
