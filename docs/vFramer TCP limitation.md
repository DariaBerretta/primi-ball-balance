# vFramer carrier and event traffic

`vFramer` opens its own YARP input ports and connects to the requested source
using `fast_tcp`. The installed version does not expose a carrier selection.

The `event_relay` therefore receives each remote `/zynqGrabber/<eye>/AE:o`
stream **once**, then exposes these local outputs for all viewers and pose
modules:

```text
/primi-ball-balance/eventCameraStream/left/source:o
/primi-ball-balance/eventCameraStream/right/source:o
```

The GUI requests local `shmem`, but `vFramer` requests local `fast_tcp`. This
still avoids multiple high-rate robot-to-laptop event subscriptions. Removing
the relay without changing the transport design would restore that network
bandwidth cost. Do not connect `vFramer`, the GUI, and MoveEnet independently
to the robot-side event source during normal acquisition.
