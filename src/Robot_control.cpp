#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/sig/Vector.h>

#include <atomic>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>


/*
USE: opens a thread of reading the encoders of left arm, right arm, and head.
If FLAG is True it loops between two positions in joint space.
If FLAG is false it goes to the home position and stays there.

Things to change:
- receive the position from the decision module of where to go for ballbalance
*/




std::mutex outputMutex;

// Set to true to loop the left arm between poseA and poseB.
// Set to false to send it to the home pose (and stay there).
std::atomic<bool> FLAG{true};

// ****************** Encoder reading (same pattern as Encoder_reader.cpp) ****************** //

void readEncoders(
    const std::string& remotePort,
    const std::string& localPort,
    const std::string& outputPortName,
    const std::string& partName)
{
    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", remotePort);
    options.put("local", localPort);

    yarp::dev::PolyDriver driver;

    if (!driver.open(options)) {
        std::scoped_lock lock(outputMutex);
        std::cerr << "Could not open remote_controlboard for " << partName
                  << " (remote " << remotePort << ")\n";
        return;
    }

    yarp::dev::IEncoders* iEncoders = nullptr;

    if (!driver.view(iEncoders) || iEncoders == nullptr) {
        std::scoped_lock lock(outputMutex);
        std::cerr << "Could not get IEncoders interface for " << partName << '\n';
        driver.close();
        return;
    }

    int numAxes = 0;
    iEncoders->getAxes(&numAxes);

    yarp::os::BufferedPort<yarp::sig::Vector> outputPort;

    if (!outputPort.open(outputPortName)) {
        std::scoped_lock lock(outputMutex);
        std::cerr << "Could not open " << outputPortName << '\n';
        driver.close();
        return;
    }

    yarp::sig::Vector encoders(static_cast<size_t>(numAxes), 0.0);
    std::size_t readCount = 0;

    while (true) {
        if (!iEncoders->getEncoders(encoders.data())) {
            yarp::os::Time::delay(0.01);
            continue;
        }

        readCount++;

        yarp::sig::Vector& outVec = outputPort.prepare();
        outVec = encoders;
        outputPort.write();

        if (readCount % 30 == 0) {
            std::scoped_lock lock(outputMutex);
            std::cout << partName << " (" << numAxes << " axes): [";
            for (int i = 0; i < numAxes; ++i) {
                std::cout << encoders[static_cast<size_t>(i)];
                if (i + 1 < numAxes) {
                    std::cout << ", ";
                }
            }
            std::cout << "] deg\n";
        }

        yarp::os::Time::delay(0.01);
    }
}

// ****************** Left arm joint-space control ****************** //

class LeftArmController {
public:
    bool open(const std::string& robot) {
        yarp::os::Property optArm, optTorso, optHead;

        optArm.put("device", "remote_controlboard");
        optArm.put("local", "/proprio_control/left_arm/client");
        optArm.put("remote", "/" + robot + "/left_arm");

        optTorso.put("device", "remote_controlboard");
        optTorso.put("local", "/proprio_control/torso/client");
        optTorso.put("remote", "/" + robot + "/torso");

        optHead.put("device", "remote_controlboard");
        optHead.put("local", "/proprio_control/head/client");
        optHead.put("remote", "/" + robot + "/head");

        if (!armDriver.open(optArm) || !torsoDriver.open(optTorso) || !headDriver.open(optHead)) {
            std::cerr << "Unable to connect to " << robot << " joint drivers\n";
            return false;
        }

        bool ok = armDriver.view(iPosArm) && armDriver.view(iModArm)
                  && torsoDriver.view(iPosTorso) && torsoDriver.view(iModTorso)
                  && headDriver.view(iPosHead) && headDriver.view(iModHead);

        if (!ok) {
            std::cerr << "Unable to open position/control-mode views\n";
            return false;
        }

        return true;
    }

    // Moves arm (7 joints), torso pitch (joint 2), and head yaw (joint 0) to the given
    // targets, then blocks until the arm motion is complete.
    void moveToPose(const yarp::sig::Vector& armPose, double torsoPitch, double headYaw) {
        iModTorso->setControlMode(2, VOCAB_CM_POSITION);
        iPosTorso->setRefSpeed(2, 10);
        iPosTorso->positionMove(2, torsoPitch);

        iModHead->setControlMode(0, VOCAB_CM_POSITION);
        iPosHead->setRefSpeed(0, 10);
        iPosHead->positionMove(0, headYaw);

        for (int j = 0; j < static_cast<int>(armPose.size()); ++j) {
            iModArm->setControlMode(j, VOCAB_CM_POSITION);
            iPosArm->setRefSpeed(j, 40);
            iPosArm->positionMove(j, armPose[static_cast<size_t>(j)]);
        }

        bool done = false;
        while (!done) {
            iPosArm->checkMotionDone(&done);
            yarp::os::Time::delay(0.1);
        }
    }

    void close() {
        armDriver.close();
        torsoDriver.close();
        headDriver.close();
    }

private:
    yarp::dev::PolyDriver armDriver, torsoDriver, headDriver;
    yarp::dev::IPositionControl* iPosArm = nullptr;
    yarp::dev::IPositionControl* iPosTorso = nullptr;
    yarp::dev::IPositionControl* iPosHead = nullptr;
    yarp::dev::IControlMode* iModArm = nullptr;
    yarp::dev::IControlMode* iModTorso = nullptr;
    yarp::dev::IControlMode* iModHead = nullptr;
};

int main(int argc, char* argv[]) {
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "Could not connect to the YARP network.\n";
        return 1;
    }

    const std::string robot = (argc > 1) ? argv[1] : "icub";

    // Start the encoder-reading threads - these run forever, independent of FLAG.
    std::thread leftArmEncThread(
        readEncoders,
        "/" + robot + "/left_arm",
        "/proprio_control/encoders/left_arm/client",
        "/proprio_control/encoders/left_arm:o",
        "left arm");
    std::thread rightArmEncThread(
        readEncoders,
        "/" + robot + "/right_arm",
        "/proprio_control/encoders/right_arm/client",
        "/proprio_control/encoders/right_arm:o",
        "right arm");
    std::thread headEncThread(
        readEncoders,
        "/" + robot + "/head",
        "/proprio_control/encoders/head/client",
        "/proprio_control/encoders/head:o",
        "head");

    // Set up the arm/torso/head controller.
    LeftArmController controller;
    if (!controller.open(robot)) {
        return 1;
    }

    // Two poses to loop between when FLAG is true (taken from the example program's
    // goHomePose/goInitPose targets), plus the "home" pose used when FLAG is false.
    yarp::sig::Vector homeArm{-35.181, 30.196, 0.0, 50.038, 0.0, 0.0, 0.0};
    const double homeTorsoPitch = 0.0;
    const double homeHeadYaw = 0.0;

    yarp::sig::Vector poseA = homeArm; // same as home pose
    const double poseATorsoPitch = homeTorsoPitch;
    const double poseAHeadYaw = homeHeadYaw;

    // In joint space, not cartesian space
    yarp::sig::Vector poseB{7.24, 31.706, 4.479, 105.5, -20.396, -6.3, -6.746};
    const double poseBTorsoPitch = 0.0;
    const double poseBHeadYaw = 0.0;

    if (FLAG) {
        std::cout << "FLAG is true: looping between poseA and poseB.\n"; // Otherwise it will loop between these two poses
        while (FLAG) {
            controller.moveToPose(poseA, poseATorsoPitch, poseAHeadYaw);
            if (!FLAG) break;
            controller.moveToPose(poseB, poseBTorsoPitch, poseBHeadYaw);
        }
    } else {
        std::cout << "FLAG is false: moving to home pose.\n"; // HERE is where I will send the pose if I want to change it online
        controller.moveToPose(homeArm, homeTorsoPitch, homeHeadYaw);
    }

    // Encoder reading keeps running - join blocks forever since those threads never return.
    leftArmEncThread.join();
    rightArmEncThread.join();
    headEncThread.join();

    controller.close();

    return 0;
}