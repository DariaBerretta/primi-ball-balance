#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>

#include <yarp/sig/Vector.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    std::string armType = "right";
    int firstJointArg = 1;

    // Optional:
    // ./joint_to_xyz --arm left q0 q1 ... q6
    if (argc >= 3 && std::string(argv[1]) == "--arm")
    {
        armType = argv[2];
        firstJointArg = 3;
    }

    if (armType != "left" && armType != "right")
    {
        std::cerr << "Arm must be 'left' or 'right'.\n";
        return 1;
    }

    if (argc - firstJointArg != 7)
    {
        std::cerr
            << "Usage:\n"
            << "  " << argv[0]
            << " q0 q1 q2 q3 q4 q5 q6\n\n"
            << "or:\n"
            << "  " << argv[0]
            << " --arm <left|right> q0 q1 q2 q3 q4 q5 q6\n\n"
            << "Joint angles must be given in DEGREES.\n";

        return 1;
    }

    // --------------------------------------------------------
    // Create nominal iCub arm kinematic model
    // --------------------------------------------------------

    iCub::iKin::iCubArm arm(armType);

    iCub::iKin::iKinChain* chain = arm.asChain();

    if (chain->getDOF() != 7)
    {
        std::cerr
            << "Unexpected arm DOFs: "
            << chain->getDOF()
            << " (expected 7)\n";

        return 1;
    }

    // --------------------------------------------------------
    // Read joint configuration from command line
    // --------------------------------------------------------

    yarp::sig::Vector q(7);

    try
    {
        for (int j = 0; j < 7; ++j)
        {
            const double qDeg =
                std::stod(argv[firstJointArg + j]);

            // iKin expects radians
            q[j] =
                qDeg * iCub::ctrl::CTRL_DEG2RAD;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "Could not parse joint values: "
            << e.what()
            << '\n';

        return 1;
    }

    // --------------------------------------------------------
    // Forward kinematics
    // --------------------------------------------------------

    chain->setAng(q);

    const yarp::sig::Vector xyz =
        chain->EndEffPosition();

    // --------------------------------------------------------
    // Print result
    // --------------------------------------------------------

    std::cout << std::fixed << std::setprecision(6);

    std::cout
        << "Arm: " << armType << '\n'
        << "\nJoint configuration [deg]:\n";

    for (int j = 0; j < 7; ++j)
    {
        std::cout
            << "q" << j << " = "
            << q[j] * iCub::ctrl::CTRL_RAD2DEG
            << '\n';
    }

    std::cout
        << "\nEnd-effector position [m]:\n"
        << "x = " << xyz[0] << '\n'
        << "y = " << xyz[1] << '\n'
        << "z = " << xyz[2] << '\n';

    std::cout
        << "\nEnd-effector position [mm]:\n"
        << "x = " << xyz[0] * 1000.0 << '\n'
        << "y = " << xyz[1] * 1000.0 << '\n'
        << "z = " << xyz[2] * 1000.0 << '\n';

    return 0;
}