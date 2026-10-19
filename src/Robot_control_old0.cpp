#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <atomic>
#include <cstddef>
#include <iostream>
#include <string>

namespace {

// State acquisition belongs to proprio_reader. This module only owns the
// remote_controlboard interfaces required to send motor commands.

// Set to true to loop the left arm between poseA and poseB.
// Set to false to move once to the home pose.
std::atomic<bool> FLAG{true};

class LeftArmController {
public:
    bool open(const std::string& robot)
    {
        yarp::os::Property armOptions;
        yarp::os::Property torsoOptions;
        yarp::os::Property headOptions;

        armOptions.put("device", "remote_controlboard");
        armOptions.put("local", "/primi-ball-balance/control/left_arm/client");
        armOptions.put("remote", "/" + robot + "/left_arm");

        torsoOptions.put("device", "remote_controlboard");
        torsoOptions.put("local", "/primi-ball-balance/control/torso/client");
        torsoOptions.put("remote", "/" + robot + "/torso");

        headOptions.put("device", "remote_controlboard");
        headOptions.put("local", "/primi-ball-balance/control/head/client");
        headOptions.put("remote", "/" + robot + "/head");

        if (!armDriver.open(armOptions) ||
            !torsoDriver.open(torsoOptions) ||
            !headDriver.open(headOptions)) {
            std::cerr << "Unable to connect to " << robot
                      << " joint drivers\n";
            close();
            return false;
        }

        const bool interfacesAvailable =
            armDriver.view(positionArm) &&
            armDriver.view(controlModeArm) &&
            torsoDriver.view(positionTorso) &&
            torsoDriver.view(controlModeTorso) &&
            headDriver.view(positionHead) &&
            headDriver.view(controlModeHead);

        if (!interfacesAvailable) {
            std::cerr << "Unable to open position/control-mode interfaces\n";
            close();
            return false;
        }

        return true;
    }

    void moveToPose(const yarp::sig::Vector& armPose,
                    double torsoPitch,
                    double headYaw)
    {
        controlModeTorso->setControlMode(2, VOCAB_CM_POSITION);
        positionTorso->setRefSpeed(2, 10.0);
        positionTorso->positionMove(2, torsoPitch);

        controlModeHead->setControlMode(0, VOCAB_CM_POSITION);
        positionHead->setRefSpeed(0, 10.0);
        positionHead->positionMove(0, headYaw);

        for (std::size_t joint = 0; joint < armPose.size(); ++joint) {
            const int jointIndex = static_cast<int>(joint);
            controlModeArm->setControlMode(jointIndex, VOCAB_CM_POSITION);
            positionArm->setRefSpeed(jointIndex, 40.0);
            positionArm->positionMove(jointIndex, armPose[joint]);
        }

        bool done = false;

        while (!done) {
            positionArm->checkMotionDone(&done);
            yarp::os::Time::delay(0.1);
        }
    }

    void close()
    {
        armDriver.close();
        torsoDriver.close();
        headDriver.close();

        positionArm = nullptr;
        positionTorso = nullptr;
        positionHead = nullptr;
        controlModeArm = nullptr;
        controlModeTorso = nullptr;
        controlModeHead = nullptr;
    }

private:
    yarp::dev::PolyDriver armDriver;
    yarp::dev::PolyDriver torsoDriver;
    yarp::dev::PolyDriver headDriver;

    yarp::dev::IPositionControl* positionArm = nullptr;
    yarp::dev::IPositionControl* positionTorso = nullptr;
    yarp::dev::IPositionControl* positionHead = nullptr;

    yarp::dev::IControlMode* controlModeArm = nullptr;
    yarp::dev::IControlMode* controlModeTorso = nullptr;
    yarp::dev::IControlMode* controlModeHead = nullptr;
};

} // namespace

int main(int argc, char* argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "Could not connect to the YARP network.\n";
        return 1;
    }

    const std::string robot = argc > 1 ? argv[1] : "icub";

    LeftArmController controller;

    if (!controller.open(robot)) {
        return 1;
    }

    const yarp::sig::Vector homeArm{
        -35.181, 30.196, 0.0, 50.038, 0.0, 0.0, 0.0};
    const double homeTorsoPitch = 0.0;
    const double homeHeadYaw = 0.0;

    const yarp::sig::Vector poseA = homeArm;
    const double poseATorsoPitch = homeTorsoPitch;
    const double poseAHeadYaw = homeHeadYaw;

    const yarp::sig::Vector poseB{
        7.24, 31.706, 4.479, 105.5, -20.396, -6.3, -6.746};
    const double poseBTorsoPitch = 0.0;
    const double poseBHeadYaw = 0.0;

    if (FLAG) {
        std::cout << "FLAG is true: looping between poseA and poseB.\n";

        while (FLAG) {
            controller.moveToPose(poseA, poseATorsoPitch, poseAHeadYaw);

            if (!FLAG) {
                break;
            }

            controller.moveToPose(poseB, poseBTorsoPitch, poseBHeadYaw);
        }
    } else {
        std::cout << "FLAG is false: moving to home pose.\n";
        controller.moveToPose(homeArm, homeTorsoPitch, homeHeadYaw);
    }

    controller.close();
    return 0;
}
