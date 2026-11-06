# iCub02 cluster startup and shutdown

## Purpose

The scripts `setup_cluster.sh` and `close_cluster.sh` manage the right-arm iCub02 YARP stack from `iiticublap267`. They start and stop the robot interface, FPGA clock initialization, Cartesian solver, and Cartesian control interface. Application output is displayed in Terminator and archived locally.

**Verified session:** 2026-11-06 12:29:28, YARP namespace `/icub02`.

## Commands

Run from `~/robot-code/primi-ball-balance` on `iiticublap267`:

```bash
./setup_cluster.sh
# Work with the robot.
./close_cluster.sh
```

`setup_cluster.sh` returns to the shell after starting the applications. Keep `close_cluster.sh` available for shutdown; closing the original shell is not the intended shutdown method. The scripts do not stop the YARP name server or `yarp run --server` nodes.

## Applications and hosts

| YARP run alias | Execution host | Command |
| --- | --- | --- |
| `yarprobotinterface` | `icub-head` | `yarprobotinterface --config /usr/local/src/robot/robotology-superbuild/src/robots-configuration/iCubGenova02/icub_right_arm.xml` |
| `start_clock` | `icub-ultrascale` | `/home/icub/start_clock.sh` |
| `iKinCartesianSolver` | `iiticublap267` | `iKinCartesianSolver --part right_arm` |
| `cartesianInterfaceControl` | `iiticublap267` | `cartesianInterfaceControl --part right_arm --only xyz` |

The name server and `yarp run` servers are reused when already running. The setup script uses `yarp namespace /icub02`.

## Startup dependencies

A YARP port can register before the module behind it is fully initialized. An earlier version waited only for `/icub/torso/rpc:i`, `/icub/right_arm/rpc:i`, `/cartesianSolver/right_arm/in`, and `/icub/cartesianController/right_arm/rpc:i`. On 2026-11-06 at 12:24:42, these checks passed but `cartesianInterfaceControl` still failed with `unable to connect to solver!`.

The working startup sequence waits for actual initialization messages:

1. Start `yarprobotinterface` on `icub-head`.
2. Wait for its log message `All actions for action level 10 of startup phase finished.` (robot startup/calibration stage).
3. Run `start_clock` on `icub-ultrascale`.
4. Start `iKinCartesianSolver` on `iiticublap267`.
5. Wait for the solver log message `cartesianSolver/right_arm started successfully`.
6. Wait for the robot-interface log messages `Connections established with cartesianSolver/right_arm`, `icub/cartesianController/right_arm started successfully`, and `startup phase finished.`
7. Start `cartesianInterfaceControl` on `iiticublap267`.

**Why this order matters:** The robot's Cartesian controller depends on the separately started solver, so waiting for the *entire* robot interface to finish before starting the solver would be incorrect. Conversely, starting the Cartesian client before the controller and solver have connected caused the observed failure. The setup script polls the relevant logs with a timeout rather than using fixed sleep durations as a readiness test.

## Monitoring and logs

The setup script opens a Terminator monitor when each application starts. Each monitor shows application output and copies it to a timestamped directory **on `iiticublap267`**:

```text
logs/YYYY-MM-DD_HH-MM-SS/
    yarprobotinterface.log
    start_clock.log
    iKinCartesianSolver.log
    cartesianInterfaceControl.log
```

Applications initially redirect output to session-specific `/tmp/primi-...` files on their execution hosts. Remote monitor terminals read those files over SSH; `tee` writes the retained copies on the working PC. Remote `/tmp` files are only intermediate sources, not the intended log archive.

`logs/active_session` identifies the current log directory. On shutdown, `close_cluster.sh` creates an empty `stop` file in that directory. The monitoring shells observe it, stop their `tail`/SSH/`tee` processes, and exit; the four `.log` files remain. **There is no need to upload or manually create the `stop` file.** Log collection is tied to the monitoring terminals: closing one early can interrupt that application's local log capture.

Add `logs/` to `.gitignore` to exclude generated logs from commits.

## Shutdown sequence

`close_cluster.sh` sends YARP SIGTERM in reverse dependency order:

1. `cartesianInterfaceControl` on `/iiticublap267`.
2. `iKinCartesianSolver` on `/iiticublap267`.
3. `start_clock` on `/icub-ultrascale`.
4. `yarprobotinterface` on `/icub-head`.

It then creates the session `stop` file, allowing the Terminator monitors to exit, and clears `logs/active_session` while preserving the log directory.

`start_clock.sh` is a short-lived initialization command. Its log reports `ad9522 module already loaded` and `Command executed with exit value: 0`. Consequently, `sigterm FAILED` for `start_clock` during shutdown is expected when that process has already exited, and does not by itself indicate an error.

## Verification: 2026-11-06 12:29:28

| Component | Observed result |
| --- | --- |
| `yarprobotinterface` | Completed startup; right-arm Cartesian controller started successfully. `sigterm OK` at shutdown. |
| `start_clock` | Completed with exit code 0; subsequent SIGTERM reported `FAILED` because it was no longer running. |
| `iKinCartesianSolver` | Connected to torso and right-arm control boards; reported `cartesianSolver/right_arm started successfully`. `sigterm OK`. |
| `cartesianInterfaceControl` | Connected to Cartesian controller, created the client wrapper, detected arm kinematics version 2.0, and reported `Thread started successfully`. Produced live Cartesian state/error output. `sigterm OK`. |

This confirms that the revised startup ordering resolved the previously observed Cartesian client initialization failure in this session. It does **not** establish that every hardware/calibration warning was resolved, or that Terminator window closure was independently verified: the terminal output only reports `Closing monitoring terminals...`.

## Separate hardware warnings

The robot-interface log from this successful cluster session also contains warnings/errors that should be assessed separately from script ordering:

- `Head_Calibrator`: joint 2 timed out while going to zero; the affected calibration set was disabled.
- `Right_Arm_Calibrator`: hardware-fault checks for joints 9 and 10 reported out-of-range indices.
- Motor control configuration for `left_arm-eb2-j4_15` and `right_arm-eb4-j4_15` reported `Jointsset 0 hasn't joints`.

These messages did not prevent the observed Cartesian modules from starting, but they should not be treated as proof of fully healthy robot calibration. Resolve hardware issues with the robot's normal safety and calibration procedures before relying on affected joints.

## Troubleshooting

- **Startup waits indefinitely or times out:** Inspect the corresponding monitor and timestamped log. A registered YARP port is not sufficient evidence of application readiness.
- **Cartesian client reports `unable to connect to solver!`:** Confirm the solver-start and controller-connection messages occurred before launching the client; do not merely increase a fixed sleep.
- **`sigterm FAILED` for `start_clock`:** Check its log for exit code 0; it normally exits after initialization.
- **`sigterm FAILED` for a long-running module:** Review its log and check whether it already exited; a `STARTED` response confirms launch, not sustained operation.
- **Monitoring windows do not close:** Check that `logs/active_session` points to the current session and that `close_cluster.sh` created `logs/<session>/stop`. The monitoring-window behavior has not been independently confirmed in the supplied terminal transcript.

This document describes the tested November 6 startup sequence and the current two-script design. It does not require changing the working application commands.
