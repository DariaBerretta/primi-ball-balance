# iCub Cartesian controller: sequential tests

Copy the `include/` and `src/` files to the matching folders in `primi-ball-balance`. Replace the repository root `CMakeLists.txt` with the supplied full version. The existing joint-space controller `Robot_control.cpp` is not changed.

Edit `include/CartesianConfig.h` to define START and END coordinates in meters and the movement times. All three executables use the same configuration.

**Important:** the palm-facing-normal sign (`PALM_FACING_NORMAL_IS_LOCAL_POSITIVE_X`) is provisional until checked against your specific iCub's physical hand frame. The YARP end-effector coordinate frame is not itself evidence of which side is the palm-facing surface. Do not run physical motions before confirming it in simulation/kinematic visualization. The code checks only endpoint palm tilt and position; it does not prove obstacle clearance or continuous horizontal alignment.

Configure and build:

```bash
cmake -S . -B build
cmake --build build -j4
```

Preflight (IK only, no commanded movement):

```bash
./build/Cartesian_to_start --robot icub
./build/Cartesian_to_end --robot icub
./build/Robot_control_cartesian_bear --robot icub
```

Physical movement only after review of predicted position/orientation and workspace:

```bash
./build/Cartesian_to_start --robot icub --execute
./build/Cartesian_to_end --robot icub --execute
./build/Robot_control_cartesian_bear --robot icub --execute
```

In the final program, the arm moves to START then executes `CYCLES` complete START-to-END-to-START movements. Finger joints are not yet controlled. This repository was not compiled against the real robot's installed YARP libraries; test in a simulation before running on hardware.
