#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

std::mutex outputMutex;

void readSkin(
    yarp::os::BufferedPort<yarp::sig::Vector>& port,
    const std::string& handName)
{
    std::size_t frameCount = 0;

    while (true) {
        auto* taxels = port.read(true);

        if (taxels == nullptr) {
            break;
        }

        frameCount++;

        // Avoid printing once per frame (skin streams at ~50 Hz).
        if (frameCount % 30 != 0) {
            continue;
        }

        // Find the taxel with the strongest activation for a quick readout.
        // Raw data: higher = more pressure. Compensated data: lower = more pressure
        // (0-255, 0 means no pressure) - adjust the comparison if you switch ports.
        std::size_t maxIndex = 0;
        double maxValue = 0.0;
        for (std::size_t i = 0; i < taxels->size(); ++i) {
            const double value = (*taxels)[i];
            if (value > maxValue) {
                maxValue = value;
                maxIndex = i;
            }
        }

        std::scoped_lock lock(outputMutex);

        std::cout << handName
                  << ": received " << taxels->size() << " taxels"
                  << " - strongest at #" << maxIndex << " (" << maxValue << ")"
                  << " - total frames: " << frameCount << '\n';
    }
}

int main() {
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "Could not connect to the YARP network.\n";
        return 1;
    }

    yarp::os::BufferedPort<yarp::sig::Vector> leftHandPort;
    yarp::os::BufferedPort<yarp::sig::Vector> rightHandPort;

    const std::string leftHandPortName = "/primi-ball-balance/skin/left_hand:i";
    const std::string rightHandPortName = "/primi-ball-balance/skin/right_hand:i";

    if (!leftHandPort.open(leftHandPortName)) {
        std::cerr << "Could not open " << leftHandPortName << '\n';
        return 1;
    }

    if (!rightHandPort.open(rightHandPortName)) {
        std::cerr << "Could not open " << rightHandPortName << '\n';
        leftHandPort.close();
        return 1;
    }


    // This is if we want to automatically connect the reader to the output port. 
    // If we do not want it there is the manual option below.
    // Another option is to have another script to connect the ports 

    // const std::string leftHandSource = "/icub/skin/left_hand_comp";
    // const std::string rightHandSource = "/icub/skin/right_hand_comp";

    // if (!yarp::os::Network::connect(leftHandSource, leftHandPortName)) {
    //     std::cerr << "Could not connect " << leftHandSource << " to " << leftHandPortName << '\n';
    //     return 1;
    // }

    // if (!yarp::os::Network::connect(rightHandSource, rightHandPortName)) {
    //     std::cerr << "Could not connect " << rightHandSource << " to " << rightHandPortName << '\n';
    //     return 1;
    // }    

    std::cout << "Waiting for left and right hand skin streams.\n";
    std::cout << "Connect e.g.:\n"
              << "  yarp connect /icub/skin/left_hand_comp "
              << leftHandPortName << '\n'
              << "  yarp connect /icub/skin/right_hand_comp "
              << rightHandPortName << '\n';

    std::thread leftThread(readSkin, std::ref(leftHandPort), "left hand");
    std::thread rightThread(readSkin, std::ref(rightHandPort), "right hand");

    leftThread.join();
    rightThread.join();

    leftHandPort.close();
    rightHandPort.close();

    return 0;
}