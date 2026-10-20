#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
namespace
{
    constexpr double PI = 3.14159265358979323846;
    // Cartesian endpoints [m].
    constexpr double START_X = -0.240;
    constexpr double START_Y = 0.157;
    constexpr double START_Z = -0.074;
    constexpr double END_X = -0.240;
    constexpr double END_Y = 0.157;
    constexpr double END_Z = 0.152;
    volatile std::sig_atomic_t stopRequested = 0;
    void signalHandler(int)
    {
        stopRequested = 1;
    }
    struct Options
    {
        std::string robot = "icub";
        // Duration of one complete START -> END -> START oscillation.
        double cycleDuration = 6.0;
        // Number of complete oscillations.
        int cycles = 3;
        // Number of points strictly between START and END.
        // Therefore 40 means 42 positions including the two endpoints.
        int intermediatePoints = 40;
        // Time allowed for the initial move to START.
        double initTimeout = 10.0;
        // Cartesian in-target tolerance [m].
        double inTargetTol = 0.005;
    };
    void printUsage(const char *program)
    {
        std::cout
            << "\nUsage:\n"
            << "  " << program << " [options]\n\n"
            << "Options:\n"
            << "  --robot <name>\n"
            << "      Robot name. Default: $YARP_ROBOT_NAME or icub\n\n"
            << "  --cycle-duration <seconds>\n"
            << "      Duration of START -> END -> START.\n"
            << "      Default: 6.0 s\n\n"
            << "  --cycles <N>\n"
            << "      Number of complete oscillations.\n"
            << "      Default: 3\n\n"
            << "  --intermediate-points <N>\n"
            << "      Number of Cartesian points strictly between START and END.\n"
            << "      Default: 40 (42 positions including endpoints)\n\n"
            << "  --init-timeout <seconds>\n"
            << "      Timeout while moving to START.\n"
            << "      Default: 10 s\n\n"
            << "  --target-tol <meters>\n"
            << "      Cartesian in-target tolerance.\n"
            << "      Default: 0.005 m\n\n";
    }
    bool parseArguments(int argc, char *argv[], Options &opt)
    {
        const char *envRobot = std::getenv("YARP_ROBOT_NAME");
        opt.robot = envRobot ? envRobot : "icub";
        try
        {
            for (int i = 1; i < argc; ++i)
            {
                const std::string arg = argv[i];
                if (arg == "--help" || arg == "-h")
                {
                    printUsage(argv[0]);
                    return false;
                }
                else if (arg == "--robot" && i + 1 < argc)
                {
                    opt.robot = argv[++i];
                }
                else if (arg == "--cycle-duration" && i + 1 < argc)
                {
                    opt.cycleDuration = std::stod(argv[++i]);
                }
                else if (arg == "--cycles" && i + 1 < argc)
                {
                    opt.cycles = std::stoi(argv[++i]);
                }
                else if (arg == "--intermediate-points" && i + 1 < argc)
                {
                    opt.intermediatePoints = std::stoi(argv[++i]);
                }
                else if (arg == "--init-timeout" && i + 1 < argc)
                {
                    opt.initTimeout = std::stod(argv[++i]);
                }
                else if (arg == "--target-tol" && i + 1 < argc)
                {
                    opt.inTargetTol = std::stod(argv[++i]);
                }
                else
                {
                    std::cerr << "Unknown or incomplete argument: " << arg << '\n';
                    return false;
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Invalid command-line argument: " << e.what() << '\n';
            return false;
        }
        if (opt.cycleDuration <= 0.0)
        {
            std::cerr << "--cycle-duration must be > 0\n";
            return false;
        }
        if (opt.cycles <= 0)
        {
            std::cerr << "--cycles must be > 0\n";
            return false;
        }
        if (opt.intermediatePoints < 0)
        {
            std::cerr << "--intermediate-points must be >= 0\n";
            return false;
        }
        if (opt.initTimeout <= 0.0)
        {
            std::cerr << "--init-timeout must be > 0\n";
            return false;
        }
        if (opt.inTargetTol <= 0.0)
        {
            std::cerr << "--target-tol must be > 0\n";
            return false;
        }
        return true;
    }


    yarp::sig::Vector makePosition(double x, double y, double z)
    {
        yarp::sig::Vector p(3);
        p[0] = x;
        p[1] = y;
        p[2] = z;
        return p;
    }


    std::vector<yarp::sig::Vector> generateWaypoints(int intermediatePoints)
    {
        const int totalPoints = intermediatePoints + 2;
        const yarp::sig::Vector start =
              makePosition(START_X, START_Y, START_Z);
        const yarp::sig::Vector end =
            makePosition(END_X, END_Y, END_Z);
        std::vector<yarp::sig::Vector> waypoints;
        waypoints.reserve(totalPoints);
        for (int i = 0; i < totalPoints; ++i)
        {
            const double u =
                static_cast<double>(i) /
                static_cast<double>(totalPoints - 1);
            const double s =
                0.5 * (1.0 - std::cos(PI * u));
            yarp::sig::Vector p(3);
            for (int k = 0; k < 3; ++k)
            {
                p[k] = start[k] + s * (end[k] - start[k]);
            }
            waypoints.push_back(p);
        }
        return waypoints;
    }
    double positionErrorNorm(
        const yarp::sig::Vector &actual,
        const yarp::sig::Vector &target)
    {
        double squaredError = 0.0;
        for (std::size_t i = 0; i < 3; ++i)
        {
            const double error = actual[i] - target[i];
            squaredError += error * error;
        }
        return std::sqrt(squaredError);
    }
    bool moveToStart(
        yarp::dev::ICartesianControl *iCart,
        const yarp::sig::Vector &start,
        const yarp::sig::Vector &fixedOrientation,
        double timeout,
        double positionTol)
    {
        std::cout
            << "\n========================================\n"
            << "MOVING TO CARTESIAN START\n"
            << "========================================\n"
            << "START [m] = "
            << start.toString()
            << '\n';
            
        // Give the initial repositioning a comfortable trajectory time.
        if (!iCart->goToPoseSync(start, fixedOrientation, 2.0))
        {
            std::cerr << "[ERROR] goToPoseSync(START) failed\n";
            return false;
        }

        const bool controllerDone = iCart->waitMotionDone(0.05, timeout);
        // ======================================
        std::cout << "[INIT] waitMotionDone returned: "
                << std::boolalpha << controllerDone << '\n';

        yarp::sig::Vector solvedPos, solvedOrient, solvedJoints;
        if (iCart->getDesired(solvedPos, solvedOrient, solvedJoints)
            && solvedPos.size() == 3)
        {
            std::cout
                << "[INIT] Requested position: " << start.toString() << '\n'
                << "[INIT] Solver position:    " << solvedPos.toString() << '\n'
                << "[INIT] Solver/request position error [m]: "
                << positionErrorNorm(solvedPos, start) << '\n';
        }
        // ======================================

        yarp::sig::Vector actualPos;
        yarp::sig::Vector actualOrient;
        if (!iCart->getPose(actualPos, actualOrient))
        {
            std::cerr << "[ERROR] Could not read Cartesian pose after startup motion.\n";
            return false;
        }
        const double positionError = positionErrorNorm(actualPos, start);
        std::cout
            << "[INIT] Actual end-effector position [m] = "
            << actualPos.toString(6, 6)
            << '\n'
            << "[INIT] Position error norm [m] = "
            << positionError
            << '\n';
        if (positionError > positionTol)
        {
            std::cerr
                << "[ERROR] START is outside the accepted Cartesian tolerance: "
                << positionError
                << " m > "
                << positionTol
                << " m\n";
            return false;
        }
        if (!controllerDone)
        {
            std::cerr
                << "[WARNING] Cartesian controller did not report motion-done, "
                << "but the measured pose is within tolerance.\n"
                << "This means we're probably close enough; continuing.\n";
        }
        return true;
    }
    bool sendWaypoint(
        yarp::dev::ICartesianControl *iCart,
        const yarp::sig::Vector &target,
        const yarp::sig::Vector &fixedOrientation,
        double trajectoryTime)
    {
        return iCart->goToPose(
            target,
            fixedOrientation,
            trajectoryTime);
    }
    bool runOscillations(
        yarp::dev::ICartesianControl *iCart,
        const std::vector<yarp::sig::Vector> &waypoints,
        const yarp::sig::Vector &fixedOrientation,
        double cycleDuration,
        int cycles)
    {
        using Clock = std::chrono::steady_clock;
        if (waypoints.size() < 2)
        {
            std::cerr << "[ERROR] Need at least two Cartesian waypoints\n";
            return false;
        }
        const int intervalsOneWay =
            static_cast<int>(waypoints.size()) - 1;
        const double halfCycleDuration =
            cycleDuration / 2.0;
        const double waypointPeriod =
            halfCycleDuration /
            static_cast<double>(intervalsOneWay);
        const auto period =
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(waypointPeriod));
        std::cout
            << "\n========================================\n"
            << "CARTESIAN OSCILLATION\n"
            << "========================================\n"
            << "Points START -> END: "
            << waypoints.size()
            << " ("
            << waypoints.size() - 2
            << " intermediate)\n"
            << "Cycles: "
            << cycles
            << '\n'
            << "Cycle duration: "
            << cycleDuration
            << " s\n"
            << "START -> END duration: "
            << halfCycleDuration
            << " s\n"
            << "Waypoint period: "
            << waypointPeriod
            << " s\n"
            << "Waypoint update rate: "
            << 1.0 / waypointPeriod
            << " Hz\n";
        auto nextTick = Clock::now();
        for (int cycle = 0;
             cycle < cycles && !stopRequested;
             ++cycle)
        {
            std::cout
                << "[CARTESIAN] Cycle "
                << cycle + 1
                << " / "
                << cycles
                << '\n';
            // START -> END.
            // Skip index 0 because moveToStart() already placed us there.
            for (std::size_t i = 1;
                 i < waypoints.size() && !stopRequested;
                 ++i)
            {
                if (!sendWaypoint(
                        iCart,
                        waypoints[i],
                        fixedOrientation,
                        waypointPeriod))
                {
                    std::cerr
                        << "[ERROR] Failed to send upward Cartesian waypoint "
                        << i
                        << '\n';
                    return false;
                }
                nextTick += period;
                std::this_thread::sleep_until(nextTick);
            }
            // END -> START.
            // Start from size-2 so END is not commanded twice.
            for (int i = static_cast<int>(waypoints.size()) - 2;
                 i >= 0 && !stopRequested;
                 --i)
            {
                if (!sendWaypoint(
                        iCart,
                        waypoints[static_cast<std::size_t>(i)],
                        fixedOrientation,
                        waypointPeriod))
                {
                    std::cerr
                        << "[ERROR] Failed to send downward Cartesian waypoint "
                        << i
                        << '\n';
                    return false;
                }
                nextTick += period;
                std::this_thread::sleep_until(nextTick);
            }
        }
        if (stopRequested)
        {
            std::cout << "[CARTESIAN] Stop requested.\n";
            return true;
        }
        if (!iCart->waitMotionDone(0.02, 2.0))
        {
            std::cerr
                << "[WARNING] Final START target did not report motion-done "
                << "within 2 s\n";
        }
        std::cout
            << "[CARTESIAN] "
            << cycles
            << " cycles completed.\n";
        return true;
    }
} // namespace
int main(int argc, char *argv[])
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork(2.0))
    {
        std::cerr << "Could not connect to YARP network.\n";
        return 1;
    }
    Options opt;
    if (!parseArguments(argc, argv, opt))
    {
        return 1;
    }
    const auto waypoints =
        generateWaypoints(opt.intermediatePoints);
    const yarp::sig::Vector start =
        waypoints.front();
    const yarp::sig::Vector end =
        waypoints.back();
    std::cout
        << "\n========================================\n"
        << "ROBOT CARTESIAN CONTROL\n"
        << "========================================\n"
        << "Robot: "
        << opt.robot
        << '\n'
        << "P_start [m]: "
        << start.toString(6, 6)
        << '\n'
        << "P_end   [m]: "
        << end.toString(6, 6)
        << '\n';
    // --------------------------------------------------------
    // Open Cartesian controller client
    // --------------------------------------------------------
    yarp::os::Property options;
    options.put(
        "device",
        "cartesiancontrollerclient");
    options.put(
        "remote",
        "/" + opt.robot + "/cartesianController/right_arm");
    options.put(
        "local",
        "/primi_ball_balance/cartesian/right_arm");
    yarp::dev::PolyDriver cartesianDriver;
    if (!cartesianDriver.open(options))
    {
        std::cerr
            << "Could not open cartesiancontrollerclient for "
            << "/"
            << opt.robot
            << "/cartesianController/right_arm\n";
        return 1;
    }
    yarp::dev::ICartesianControl *iCart = nullptr;
    if (!cartesianDriver.view(iCart) || iCart == nullptr)
    {
        std::cerr << "Could not acquire ICartesianControl.\n";
        cartesianDriver.close();
        return 1;
    }
    yarp::os::Bottle cartinfo;
    iCart->getInfo(cartinfo);
    std::cout << "My Info:" + cartinfo.toString() << std::endl;
    // --------------------------------------------------------
    // Store the original Cartesian-controller configuration
    // --------------------------------------------------------
    // int contextId = -1;
    // if (!iCart->storeContext(&contextId))
    // {
    //     std::cerr << "Could not store Cartesian controller context.\n";
    //     cartesianDriver.close();
    //     return 1;
    // }
    // std::cout << "context id" << contextId << std::endl;
    bool ok = true;
    // Position is more important than orientation for this task.
    //ok = ok && iCart->setPosePriority("position");
    std::cout << ok << std::endl;
    // The initial move is a finite motion; enable tracking only for streaming.
    // ok = ok && iCart->setTrackingMode(false);
    std::cout << ok << std::endl;
    // Cartesian target tolerance in meters.
    ok = ok && iCart->setInTargetTol(opt.inTargetTol);
    std::cout << ok << std::endl;
    yarp::sig::Vector currentDof;
    if (iCart->getDOF(currentDof))
    {
        std::cout << currentDof.toString() << std::endl;
        // yarp::sig::Vector desiredDof =
        //     currentDof;
        // std::cout << currentDof << std::endl;
        // if (desiredDof.size() >= 3)
        // {
        //     desiredDof[0] = 0.0;
        //     desiredDof[1] = 0.0;
        //     desiredDof[2] = 0.0;
        //     for (std::size_t i = 3;
        //          i < desiredDof.size();
        //          ++i)
        //     {
        //         desiredDof[i] = 1.0;
        //     }
        //     yarp::sig::Vector resultingDof;
        //     if (!iCart->setDOF(
        //             desiredDof,
        //             resultingDof))
        //     {
        //         std::cerr
        //             << "[ERROR] Could not configure Cartesian DOFs.\n"
        //             << "Desired DOFs: "
        //             << desiredDof.size()
        //             << '\n'
        //             << "Resulting DOFs: "
        //             << resultingDof.size()
        //             << '\n';
        //         ok = false;
        //     }
        //     else
        //     {
        //         std::cout
        //             << "Cartesian DOFs: "
        //             << resultingDof.toString(0, 0)
        //             << '\n';
        //     }
        // }
    }
    if (!ok)
    {
        std::cerr << "Could not configure Cartesian controller" << std::endl;
        // iCart->restoreContext(contextId);
        //iCart->deleteContext(contextId);
        cartesianDriver.close();
        return 1;
    }


    // ========================================
    yarp::sig::Vector currentPos;
    yarp::sig::Vector fixedOrientation;

    std::cout << "[POSE] Waiting for initial Cartesian state...\n";

    bool poseAvailable = false;
    int attempts = 0;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);

    while (!stopRequested &&
        std::chrono::steady_clock::now() < deadline)
    {
        ++attempts;

        if (iCart->getPose(currentPos, fixedOrientation))
        {
            poseAvailable = true;
            break;
        }

        yarp::os::Time::delay(0.02);
    }

    std::cout << "[POSE] Attempts: " << attempts
            << ", success: " << std::boolalpha << poseAvailable
            << '\n';

    if (!poseAvailable)
    {
        std::cerr << "No valid Cartesian state received within 2 seconds.\n";
        cartesianDriver.close();
        return 1;
    }




    // ======================================



    // --------------------------------------------------------
    // Preserve the end-effector orientation
    // --------------------------------------------------------
    // yarp::sig::Vector currentPos;
    // yarp::sig::Vector fixedOrientation;

    // // std::cout << "Reading current Cartesian pose...\n"; 
    // // std::cout << "Current position [m]: " << currentPos.toString(6, 6) << std::endl;
    // // std::cout << "Fixed orientation [axis-angle]: " << fixedOrientation.toString(6, 6) << std::endl;
    
    // if (!iCart->getPose(
    //         currentPos,
    //         fixedOrientation))
    // {
    //     std::cerr << "Could not read current Cartesian pose.\n";
    //     // iCart->restoreContext(contextId);
    //     // iCart->deleteContext(contextId);
    //     cartesianDriver.close();
    //     return 1;
    // }
    // else{
    //     std::cout << "Reading current Cartesian pose...\n"; 
    //     std::cout << "Current position [m]: " << currentPos.toString(6, 6) << std::endl;
    //     std::cout << "Fixed orientation [axis-angle]: " << fixedOrientation.toString(6, 6) << std::endl;
    
    // }
    std::cout
        << "Current position [m]: "
        << currentPos.toString(6, 6)
        << '\n'
        << "Orientation held fixed [axis-angle]: "
        << fixedOrientation.toString(6, 6)
        << '\n';
    // --------------------------------------------------------
    // Move to START, then oscillate START <-> END
    // --------------------------------------------------------
    std::cout << "\n========================================\n"
              << "MOVING TO START\n"
              << "========================================\n";


    // ====================
    // Example only: replace with your desired absolute orientation.
    // This represents a rotation of +90 degrees around the root Y axis.
    // fixedOrientation.resize(4);
    // fixedOrientation[0] = 0.0;
    // fixedOrientation[1] = 1.0;
    // fixedOrientation[2] = 0.0;
    // fixedOrientation[3] =  PI / 4.0; // change these values to new orientation

    if (!iCart->setTrackingMode(false))
    {
        std::cerr << "Could not disable tracking for initialization.\n";
        return 1;
    }

    // =====================

    if (!moveToStart(
            iCart,
            // start,
            currentPos,
            fixedOrientation,
            opt.initTimeout,
            opt.inTargetTol))
    {
        iCart->stopControl();
        // iCart->restoreContext(contextId);
        // iCart->deleteContext(contextId);
        cartesianDriver.close();
        return 1;
    }
    else
    {
        std::cout << "[INIT] Reaching START position.\n";
    }




    if (!iCart->setTrackingMode(true))
    {
        std::cerr << "[ERROR] Could not enable Cartesian tracking mode.\n";
        iCart->stopControl();
        // iCart->restoreContext(contextId);
        // iCart->deleteContext(contextId);
        cartesianDriver.close();
        return 1;
    }
    bool motionOK = true;
    if (!stopRequested)
    {
        std::cout << "\n========================================\n"
              << "RUNNING OSCILLATIONS\n"
              << "========================================\n";
        motionOK =
            runOscillations(
                iCart,
                waypoints,
                fixedOrientation,
                opt.cycleDuration,
                opt.cycles);
    }
    // Stop the currently streamed Cartesian reference.
    iCart->stopControl();
    // Restore tracking mode, DOFs, trajectory settings, limits, etc.
    // if (!iCart->restoreContext(contextId))
    // {
    //     std::cerr
    //         << "[WARNING] Could not restore Cartesian controller context.\n";
    //     motionOK = false;
    // }
    //iCart->deleteContext(contextId);
    cartesianDriver.close();
    std::cout << "\nRobot_control_cartesian terminated.\n";
    return motionOK ? 0 : 1;
}
