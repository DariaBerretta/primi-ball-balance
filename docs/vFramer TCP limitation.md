# vFramer Local Carrier Limitation

`vFramer` creates its own input ports and connects them to the supplied event sources internally. In the current `event-driven` implementation, this connection is hardcoded to use the `fast_tcp` carrier:

```cpp
yarp::os::Network::connect(sourceName, portName, "fast_tcp");
```

As a result, the monitoring script cannot request `shmem` for the connection between:

```text
/primi-ball-balance/eventCam/left/raw:o
/primi-ball-balance/eventCam/right/raw:o
```

and `vFramer`.

This does not reintroduce the original robot-side bandwidth problem. `vFramer` now consumes the locally republished event streams rather than connecting directly to `zynqGrabber`, so the robot-to-laptop event data is still transmitted only once per camera.

The remaining issue is only that the local `eventCam_reader` → `vFramer` connections use `fast_tcp` instead of shared memory.

Using `shmem` would require modifying the `vFramer` source, replacing `"fast_tcp"` with `"shmem"`, and rebuilding and reinstalling `vFramer`. A cleaner long-term solution would be to make the carrier configurable through a command-line option rather than hardcoding it.
