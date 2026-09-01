#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IControlMode.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>


namespace
{

constexpr int LOOKUP_JOINTS = 16;

constexpr double PI =
    3.14159265358979323846;


// ============================================================
// Ctrl+C handling
// ============================================================

volatile std::sig_atomic_t stopRequested = 0;

void signalHandler(int)
{
    stopRequested = 1;
}


// ============================================================
// Data structures
// ============================================================

struct Waypoint
{
    double timestamp;
    std::vector<double> q;
};


struct Options
{
    // Robot name.
    // If YARP_ROBOT_NAME exists it will be used automatically.
    std::string YARP_ROBOT_NAME = "icub";
    std::string robot;

    // Lookup table generated from the recorded joint angles.
    std::string lookupFile = "/home/icub/robot-code/primi-ball-balance/lookupTableV2.txt";

    /*
     * Duration of one COMPLETE oscillation:
     *
     * INIT -> UP -> INIT
     *
     * Example:
     *
     * cycleDuration = 6 s
     *
     * INIT ---- 3 s ----> UP ---- 3 s ----> INIT
     */
    double cycleDuration = 6.0;

    // Number of complete INIT -> UP -> INIT oscillations.
    int cycles = 3;

    /*
     * POSITION_DIRECT update period.
     *
     * 0.010 s = 100 Hz
     */
    double controlPeriod = 0.010;

    /*
     * The lookup table contains 16 right_arm joints.
     *
     * We control only the first 7 by default:
     * shoulder/elbow/wrist.
     *
     * The remaining joints belong to the hand.
     */
    int controlledJoints = 16;

    /*
     * Speed used ONLY to reach INIT before starting
     * POSITION_DIRECT.
     */
    double initSpeed = 20.0; // deg/s

    // Maximum time allowed to reach INIT.
    double initTimeout = 30.0; // seconds
};


// ============================================================
// Command line
// ============================================================

void printUsage(const char* program)
{
    std::cout
        << "\nUsage:\n"
        << "  " << program << " [options]\n\n"

        << "Options:\n"

        << "  --robot <name>\n"
        << "      Robot name.\n\n"

        << "  --lookup <file>\n"
        << "      Path to lookupTable.txt.\n\n"

        << "  --cycle-duration <seconds>\n"
        << "      Duration of one complete:\n"
        << "          INIT -> UP -> INIT\n"
        << "      Default: 6.0 s\n\n"

        << "  --cycles <N>\n"
        << "      Number of complete oscillations.\n"
        << "      Default: 3\n\n"

        << "  --period <seconds>\n"
        << "      POSITION_DIRECT control period.\n"
        << "      Default: 0.010 s = 100 Hz\n\n"

        << "  --joints <7|16>\n"
        << "      Number of right-arm joints to replay.\n"
        << "      Default: 7\n\n"

        << "  --init-speed <deg/s>\n"
        << "      Speed used to reach INIT.\n"
        << "      Default: 20 deg/s\n\n"

        << "  --init-timeout <seconds>\n"
        << "      Timeout while reaching INIT.\n"
        << "      Default: 30 s\n\n";
}


bool parseArguments(
    int argc,
    char* argv[],
    Options& opt)
{
    const char* envRobot =
        std::getenv("YARP_ROBOT_NAME");

    opt.robot =
        envRobot ? envRobot : "icub";

    try
    {
        for (int i = 1; i < argc; ++i)
        {
            const std::string arg =
                argv[i];

            if (arg == "--help" ||
                arg == "-h")
            {
                printUsage(argv[0]);
                return false;
            }

            else if (
                arg == "--robot" &&
                i + 1 < argc)
            {
                opt.robot =
                    argv[++i];
            }

            else if (
                arg == "--lookup" &&
                i + 1 < argc)
            {
                opt.lookupFile =
                    argv[++i];
            }

            else if (
                arg == "--cycle-duration" &&
                i + 1 < argc)
            {
                opt.cycleDuration =
                    std::stod(argv[++i]);
            }

            else if (
                arg == "--cycles" &&
                i + 1 < argc)
            {
                opt.cycles =
                    std::stoi(argv[++i]);
            }

            else if (
                arg == "--period" &&
                i + 1 < argc)
            {
                opt.controlPeriod =
                    std::stod(argv[++i]);
            }

            else if (
                arg == "--joints" &&
                i + 1 < argc)
            {
                opt.controlledJoints =
                    std::stoi(argv[++i]);
            }

            else if (
                arg == "--init-speed" &&
                i + 1 < argc)
            {
                opt.initSpeed =
                    std::stod(argv[++i]);
            }

            else if (
                arg == "--init-timeout" &&
                i + 1 < argc)
            {
                opt.initTimeout =
                    std::stod(argv[++i]);
            }

            else
            {
                std::cerr
                    << "Unknown or incomplete argument: "
                    << arg
                    << '\n';

                return false;
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "Invalid command-line argument: "
            << e.what()
            << '\n';

        return false;
    }


    if (opt.controlledJoints != 7 &&
        opt.controlledJoints != 16)
    {
        std::cerr
            << "--joints must be 7 or 16\n";

        return false;
    }


    if (opt.cycleDuration <= 0.0)
    {
        std::cerr
            << "--cycle-duration must be > 0\n";

        return false;
    }


    if (opt.cycles <= 0)
    {
        std::cerr
            << "--cycles must be > 0\n";

        return false;
    }


    if (opt.controlPeriod <= 0.0)
    {
        std::cerr
            << "--period must be > 0\n";

        return false;
    }


    if (opt.initSpeed <= 0.0)
    {
        std::cerr
            << "--init-speed must be > 0\n";

        return false;
    }


    return true;
}


// ============================================================
// Load lookupTable.txt
// ============================================================

bool loadLookupTable(
    const std::string& fileName,
    int controlledJoints,
    std::vector<Waypoint>& trajectory)
{
    std::ifstream file(fileName);

    if (!file.is_open())
    {
        std::cerr
            << "Could not open lookup table: "
            << fileName
            << '\n';

        return false;
    }


    std::string line;

    std::size_t lineNumber = 0;


    while (std::getline(file, line))
    {
        ++lineNumber;


        if (line.empty())
        {
            continue;
        }


        std::istringstream iss(line);


        /*
         * yarpdatadumper format:
         *
         * sample_id timestamp q0 q1 ... q15
         */

        long long sampleId;

        double timestamp;


        if (!(iss >> sampleId >> timestamp))
        {
            std::cerr
                << "Invalid line header at line "
                << lineNumber
                << '\n';

            return false;
        }


        std::vector<double>
            allJoints(LOOKUP_JOINTS);


        for (int j = 0;
             j < LOOKUP_JOINTS;
             ++j)
        {
            if (!(iss >> allJoints[j]))
            {
                std::cerr
                    << "Expected "
                    << LOOKUP_JOINTS
                    << " joint values at line "
                    << lineNumber
                    << '\n';

                return false;
            }
        }


        Waypoint point;

        point.timestamp =
            timestamp;


        /*
         * Default:
         *
         * keep q0 ... q6 only.
         *
         * With --joints 16 all values are kept.
         */

        point.q.assign(
            allJoints.begin(),
            allJoints.begin()
                + controlledJoints);


        trajectory.push_back(
            std::move(point));
    }


    if (trajectory.size() < 2)
    {
        std::cerr
            << "Lookup table contains fewer than "
            << "2 trajectory points\n";

        return false;
    }


    /*
     * Check that timestamps are valid.
     */

    for (std::size_t i = 1;
         i < trajectory.size();
         ++i)
    {
        if (trajectory[i].timestamp <=
            trajectory[i - 1].timestamp)
        {
            std::cerr
                << "Timestamps are not strictly "
                << "increasing at point "
                << i
                << '\n';

            return false;
        }
    }


    return true;
}


// ============================================================
// Change control mode
// ============================================================

bool setJointModes(
    yarp::dev::IControlMode* iMode,
    const std::vector<int>& joints,
    int desiredMode)
{
    const int n =
        static_cast<int>(
            joints.size());


    std::vector<int>
        requestedModes(
            n,
            desiredMode);


    std::vector<int>
        currentModes(n);


    if (!iMode->setControlModes(
            n,
            joints.data(),
            requestedModes.data()))
    {
        return false;
    }


    /*
     * Verify that the control board actually changed
     * all requested joints to the desired mode.
     */

    for (int attempt = 0;
         attempt < 50;
         ++attempt)
    {
        if (iMode->getControlModes(
                n,
                joints.data(),
                currentModes.data()))
        {
            bool allCorrect = true;


            for (const int mode :
                 currentModes)
            {
                if (mode != desiredMode)
                {
                    allCorrect = false;
                    break;
                }
            }


            if (allCorrect)
            {
                return true;
            }
        }


        yarp::os::Time::delay(
            0.02);
    }


    return false;
}


// ============================================================
// Move from arbitrary initial configuration to INIT
// ============================================================

bool moveToInit(
    yarp::dev::IPositionControl* iPos,
    yarp::dev::IControlMode* iMode,
    const std::vector<int>& joints,
    const std::vector<double>& initPose,
    double speed,
    double timeout)
{
    const int n =
        static_cast<int>(
            joints.size());


    std::cout
        << "\n========================================\n"
        << "MOVING TO INIT\n"
        << "========================================\n";


    std::cout
        << "[INIT] Switching to POSITION mode...\n";


    if (!setJointModes(
            iMode,
            joints,
            VOCAB_CM_POSITION))
    {
        std::cerr
            << "[ERROR] Could not switch joints "
            << "to POSITION mode\n";

        return false;
    }


    /*
     * Set reference speed used by the internal
     * trajectory generator ONLY for the initial
     * motion toward INIT.
     */

    std::vector<double>
        speeds(n, speed);


    if (!iPos->setRefSpeeds(
            n,
            joints.data(),
            speeds.data()))
    {
        std::cerr
            << "[ERROR] Could not set INIT "
            << "reference speeds\n";

        return false;
    }


    std::cout
        << "[INIT] Moving to first "
        << "lookup-table pose...\n";

    std::cout
        << "[INIT] Speed: "
        << speed
        << " deg/s\n";


    if (!iPos->positionMove(
            n,
            joints.data(),
            initPose.data()))
    {
        std::cerr
            << "[ERROR] positionMove(INIT) failed\n";

        return false;
    }


    const auto start =
        std::chrono::steady_clock::now();


    while (!stopRequested)
    {
        bool done = false;


        /*
         * For the subset overload, flag is the
         * logical AND of all requested joints.
         */
        if (iPos->checkMotionDone(
                n,
                joints.data(),
                &done))
        {
            if (done)
            {
                std::cout
                    << "[INIT] INIT reached.\n";

                return true;
            }
        }


        const double elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()
                - start)
                .count();


        if (elapsed > timeout)
        {
            std::cerr
                << "[ERROR] Timeout while moving "
                << "to INIT\n";

            return false;
        }


        yarp::os::Time::delay(
            0.05);
    }


    return false;
}


// ============================================================
// Interpolate the 33 lookup-table points
// ============================================================

void interpolateTrajectory(
    const std::vector<Waypoint>& trajectory,
    double progress,
    std::vector<double>& command)
{
    /*
     * progress:
     *
     * 0.0 -> first lookup-table position = INIT
     *
     * 1.0 -> last lookup-table position = UP
     */


    progress =
        std::clamp(
            progress,
            0.0,
            1.0);


    if (progress <= 0.0)
    {
        command =
            trajectory.front().q;

        return;
    }


    if (progress >= 1.0)
    {
        command =
            trajectory.back().q;

        return;
    }


    /*
     * Convert normalized progress into the
     * ORIGINAL lookup-table time coordinate.
     *
     * This preserves the relative timing between
     * the 33 recorded samples.
     */

    const double tStart =
        trajectory.front().timestamp;

    const double tEnd =
        trajectory.back().timestamp;


    const double targetTimestamp =
        tStart
        + progress
        * (tEnd - tStart);


    /*
     * Find:
     *
     * trajectory[i0].timestamp <= targetTimestamp
     * trajectory[i1].timestamp >  targetTimestamp
     */

    auto upper =
        std::upper_bound(
            trajectory.begin(),
            trajectory.end(),
            targetTimestamp,
            [](double value,
               const Waypoint& point)
            {
                return value <
                       point.timestamp;
            });


    if (upper == trajectory.begin())
    {
        command =
            trajectory.front().q;

        return;
    }


    if (upper == trajectory.end())
    {
        command =
            trajectory.back().q;

        return;
    }


    const std::size_t i1 =
        static_cast<std::size_t>(
            upper
            - trajectory.begin());


    const std::size_t i0 =
        i1 - 1;


    const double time0 =
        trajectory[i0].timestamp;

    const double time1 =
        trajectory[i1].timestamp;


    const double alpha =
        (targetTimestamp - time0)
        /
        (time1 - time0);


    const std::size_t n =
        trajectory[i0].q.size();


    command.resize(n);


    /*
     * Linear interpolation in joint space.
     */

    for (std::size_t j = 0;
         j < n;
         ++j)
    {
        const double q0 =
            trajectory[i0].q[j];

        const double q1 =
            trajectory[i1].q[j];


        command[j] =
            q0
            + alpha
            * (q1 - q0);
    }
}


// ============================================================
// Continuous oscillation using POSITION_DIRECT
// ============================================================

bool replayOscillations(
    yarp::dev::IPositionDirect* iDirect,
    const std::vector<int>& joints,
    const std::vector<Waypoint>& trajectory,
    double cycleDuration,
    int numberOfCycles,
    double controlPeriod,
    std::vector<double>& lastCommand)
{
    using Clock =
        std::chrono::steady_clock;


    const int n =
        static_cast<int>(
            joints.size());


    const double totalDuration =
        cycleDuration
        * static_cast<double>(
            numberOfCycles);


    std::cout
        << "\n========================================\n"
        << "POSITION_DIRECT PLAYBACK\n"
        << "========================================\n";

    std::cout
        << "[PLAYBACK] Lookup points: "
        << trajectory.size()
        << '\n';

    std::cout
        << "[PLAYBACK] Cycles: "
        << numberOfCycles
        << '\n';

    std::cout
        << "[PLAYBACK] Cycle duration: "
        << cycleDuration
        << " s\n";

    std::cout
        << "[PLAYBACK] Up duration: "
        << cycleDuration / 2.0
        << " s\n";

    std::cout
        << "[PLAYBACK] Down duration: "
        << cycleDuration / 2.0
        << " s\n";

    std::cout
        << "[PLAYBACK] Total duration: "
        << totalDuration
        << " s\n";

    std::cout
        << "[PLAYBACK] Control period: "
        << controlPeriod
        << " s\n";

    std::cout
        << "[PLAYBACK] Control rate: "
        << 1.0 / controlPeriod
        << " Hz\n";


    /*
     * Start exactly from INIT.
     */

    lastCommand =
        trajectory.front().q;


    if (!iDirect->setPositions(
            n,
            joints.data(),
            lastCommand.data()))
    {
        std::cerr
            << "[ERROR] Could not send initial "
            << "POSITION_DIRECT command\n";

        return false;
    }


    const auto period =
        std::chrono::duration_cast<
            Clock::duration>(
                std::chrono::duration<double>(
                    controlPeriod));


    const auto start =
        Clock::now();


    auto nextTick =
        start;


    int lastPrintedCycle =
        -1;


    while (!stopRequested)
    {
        const auto now =
            Clock::now();


        const double elapsed =
            std::chrono::duration<double>(
                now - start)
                .count();


        if (elapsed >= totalDuration)
        {
            break;
        }


        // ----------------------------------------------------
        // Determine current cycle
        // ----------------------------------------------------

        const int currentCycle =
            static_cast<int>(
                elapsed / cycleDuration);


        if (currentCycle !=
            lastPrintedCycle)
        {
            std::cout
                << "[PLAYBACK] Cycle "
                << currentCycle + 1
                << " / "
                << numberOfCycles
                << '\n';

            lastPrintedCycle =
                currentCycle;
        }


        // ----------------------------------------------------
        // CONTINUOUS PHASE
        // ----------------------------------------------------

        /*
         * phase evolves continuously:
         *
         * 0 -> 2*pi for every complete cycle.
         *
         * We intentionally DO NOT reset a trajectory
         * generator at each lookup point.
         */

        const double phase =
            2.0
            * PI
            * elapsed
            / cycleDuration;


        /*
         * Cosine trajectory:
         *
         * progress =
         *      0 at INIT
         *      1 at UP
         *      0 at INIT
         *
         *
         *        UP
         *         1
         *         /\
         *        /  \
         *       /    \
         *      /      \
         * 0 --          -- 0
         * INIT          INIT
         *
         *
         * derivative is zero at:
         *
         * INIT
         * UP
         * INIT
         *
         * Therefore direction changes smoothly.
         */

        const double progress =
            0.5
            * (1.0 - std::cos(phase));


        // ----------------------------------------------------
        // Convert progress into joint positions
        // ----------------------------------------------------

        interpolateTrajectory(
            trajectory,
            progress,
            lastCommand);


        // ----------------------------------------------------
        // Send direct-position reference
        // ----------------------------------------------------

        if (!iDirect->setPositions(
                n,
                joints.data(),
                lastCommand.data()))
        {
            std::cerr
                << "[ERROR] POSITION_DIRECT "
                << "command failed\n";

            return false;
        }


        // ----------------------------------------------------
        // Maintain requested control period
        // ----------------------------------------------------

        nextTick += period;


        std::this_thread::sleep_until(
            nextTick);
    }


    /*
     * At the end of all complete cycles the desired
     * position is exactly INIT.
     */

    if (!stopRequested)
    {
        lastCommand =
            trajectory.front().q;


        if (!iDirect->setPositions(
                n,
                joints.data(),
                lastCommand.data()))
        {
            std::cerr
                << "[ERROR] Could not send final "
                << "INIT reference\n";

            return false;
        }


        std::cout
            << "[PLAYBACK] "
            << numberOfCycles
            << " cycles completed.\n";
    }


    return true;
}


// ============================================================
// Hold final position
// ============================================================

bool holdPosition(
    yarp::dev::IPositionControl* iPos,
    yarp::dev::IControlMode* iMode,
    const std::vector<int>& joints,
    const std::vector<double>& pose,
    double speed)
{
    const int n =
        static_cast<int>(
            joints.size());


    std::cout
        << "\n[HOLD] Switching back "
        << "to POSITION mode...\n";


    if (!setJointModes(
            iMode,
            joints,
            VOCAB_CM_POSITION))
    {
        std::cerr
            << "[ERROR] Could not restore "
            << "POSITION mode\n";

        return false;
    }


    std::vector<double>
        speeds(n, speed);


    if (!iPos->setRefSpeeds(
            n,
            joints.data(),
            speeds.data()))
    {
        std::cerr
            << "[ERROR] Could not set "
            << "hold reference speeds\n";

        return false;
    }


    /*
     * Explicitly set the final target after changing
     * from POSITION_DIRECT back to POSITION.
     */

    if (!iPos->positionMove(
            n,
            joints.data(),
            pose.data()))
    {
        std::cerr
            << "[ERROR] Could not hold "
            << "final pose\n";

        return false;
    }


    std::cout
        << "[HOLD] Position held.\n";


    return true;
}

} // namespace


// ============================================================
// MAIN
// ============================================================

int main(
    int argc,
    char* argv[])
{
    std::signal(
        SIGINT,
        signalHandler);

    std::signal(
        SIGTERM,
        signalHandler);


    // --------------------------------------------------------
    // YARP network
    // --------------------------------------------------------

    yarp::os::Network yarp;


    if (!yarp::os::Network::checkNetwork(
            2.0))
    {
        std::cerr
            << "Could not connect to "
            << "YARP network.\n";

        return 1;
    }


    // --------------------------------------------------------
    // Parse options
    // --------------------------------------------------------

    Options opt;


    if (!parseArguments(
            argc,
            argv,
            opt))
    {
        return 1;
    }


    // --------------------------------------------------------
    // Load trajectory
    // --------------------------------------------------------

    std::vector<Waypoint>
        trajectory;


    if (!loadLookupTable(
            opt.lookupFile,
            opt.controlledJoints,
            trajectory))
    {
        return 1;
    }


    const double recordedDuration =
        trajectory.back().timestamp
        - trajectory.front().timestamp;


    std::cout
        << "\n========================================\n"
        << "ROBOT CONTROL\n"
        << "========================================\n";

    std::cout
        << "Robot: "
        << opt.robot
        << '\n';

    std::cout
        << "Lookup table: "
        << opt.lookupFile
        << '\n';

    std::cout
        << "Loaded points: "
        << trajectory.size()
        << '\n';

    std::cout
        << "Original one-way recording duration: "
        << recordedDuration
        << " s\n";

    std::cout
        << "Controlled joints: "
        << opt.controlledJoints
        << '\n';


    // --------------------------------------------------------
    // Open right_arm remote_controlboard
    // --------------------------------------------------------

    yarp::os::Property options;


    options.put(
        "device",
        "remote_controlboard");


    options.put(
        "local",
        "/proprio_control/right_arm/client");


    options.put(
        "remote",
        "/"
        + opt.robot
        + "/right_arm");


    yarp::dev::PolyDriver
        armDriver;


    if (!armDriver.open(options))
    {
        std::cerr
            << "Could not open remote_controlboard for "
            << "/"
            << opt.robot
            << "/right_arm\n";

        return 1;
    }


    // --------------------------------------------------------
    // Get YARP interfaces
    // --------------------------------------------------------

    yarp::dev::IPositionControl*
        iPos = nullptr;


    yarp::dev::IPositionDirect*
        iDirect = nullptr;


    yarp::dev::IControlMode*
        iMode = nullptr;


    const bool interfacesOK =
        armDriver.view(iPos)
        &&
        armDriver.view(iDirect)
        &&
        armDriver.view(iMode);


    if (!interfacesOK ||
        iPos == nullptr ||
        iDirect == nullptr ||
        iMode == nullptr)
    {
        std::cerr
            << "Could not acquire required "
            << "YARP control interfaces.\n";

        armDriver.close();

        return 1;
    }


    // --------------------------------------------------------
    // Check number of axes
    // --------------------------------------------------------

    int axes = 0;


    if (!iPos->getAxes(&axes))
    {
        std::cerr
            << "Could not read number "
            << "of right-arm axes.\n";

        armDriver.close();

        return 1;
    }


    std::cout
        << "Robot right_arm axes: "
        << axes
        << '\n';


    if (axes <
        opt.controlledJoints)
    {
        std::cerr
            << "Robot exposes only "
            << axes
            << " axes, but "
            << opt.controlledJoints
            << " were requested.\n";

        armDriver.close();

        return 1;
    }


    // --------------------------------------------------------
    // Joint IDs
    // --------------------------------------------------------

    /*
     * Default:
     *
     * joints = {0,1,2,3,4,5,6}
     */

    std::vector<int>
        joints(
            opt.controlledJoints);


    for (int j = 0;
         j < opt.controlledJoints;
         ++j)
    {
        joints[j] = j;
    }


    // --------------------------------------------------------
    // 1. Go from arbitrary robot pose to INIT
    // --------------------------------------------------------

    const std::vector<double>& initPose =
        trajectory.front().q;


    if (!moveToInit(
            iPos,
            iMode,
            joints,
            initPose,
            opt.initSpeed,
            opt.initTimeout))
    {
        armDriver.close();

        return 1;
    }


    if (stopRequested)
    {
        armDriver.close();

        return 0;
    }


    // --------------------------------------------------------
    // 2. POSITION -> POSITION_DIRECT
    // --------------------------------------------------------

    std::cout
        << "\n[PLAYBACK] Switching joints "
        << "to POSITION_DIRECT...\n";


    if (!setJointModes(
            iMode,
            joints,
            VOCAB_CM_POSITION_DIRECT))
    {
        std::cerr
            << "[ERROR] Could not switch "
            << "to POSITION_DIRECT\n";

        armDriver.close();

        return 1;
    }


    std::cout
        << "[PLAYBACK] POSITION_DIRECT active.\n";


    // --------------------------------------------------------
    // 3. Execute N smooth oscillations
    // --------------------------------------------------------

    std::vector<double>
        lastCommand =
            initPose;


    const bool playbackOK =
        replayOscillations(
            iDirect,
            joints,
            trajectory,
            opt.cycleDuration,
            opt.cycles,
            opt.controlPeriod,
            lastCommand);


    // --------------------------------------------------------
    // 4. Return to POSITION and hold current pose
    // --------------------------------------------------------

    /*
     * Normally lastCommand == INIT because all cycles
     * are complete.
     *
     * If Ctrl+C occurs in the middle of a movement,
     * lastCommand is the last POSITION_DIRECT reference.
     */

    const bool holdOK =
        holdPosition(
            iPos,
            iMode,
            joints,
            lastCommand,
            opt.initSpeed);


    // --------------------------------------------------------
    // Close
    // --------------------------------------------------------

    armDriver.close();


    std::cout
        << "\nRobot_control terminated.\n";


    return
        (playbackOK && holdOK)
            ? 0
            : 1;
}