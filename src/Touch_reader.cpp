#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

namespace {

std::mutex outputMutex;

void readSkin(yarp::os::BufferedPort<yarp::sig::Vector>& port,
              const std::string& handName)
{
    std::size_t frameCount = 0;

    while (true) {
        const auto* taxels = port.read(true);

        if (taxels == nullptr) {
            break;
        }

        ++frameCount;

        if (taxels->size() == 0) {
            continue;
        }

        // The compensated skin streams normally rest near zero and increase
        // with contact pressure. Therefore, the largest value corresponds to
        // the strongest taxel activation.
        const auto strongest = std::max_element(taxels->begin(), taxels->end());
        const auto strongestIndex =
            static_cast<std::size_t>(std::distance(taxels->begin(), strongest));

        // Avoid printing every frame. Skin normally streams at approximately
        // 50 Hz, so this prints roughly once every 0.6 seconds.
        if (frameCount % 30 != 0) {
            continue;
        }

        std::scoped_lock lock(outputMutex);

        std::cout << handName
                  << ": received " << taxels->size() << " taxels"
                  << " - strongest at #" << strongestIndex
                  << " (" << *strongest << ")"
                  << " - total frames: " << frameCount << '\n';
    }
}

bool connectPort(const std::string& source,
                 const std::string& destination)
{
    if (yarp::os::Network::isConnected(source, destination)) {
        std::cout << "Already connected: "
                  << source << " -> " << destination << '\n';
        return true;
    }

    if (!yarp::os::Network::connect(source, destination, "tcp")) {
        std::cerr << "Could not connect "
                  << source << " -> " << destination << '\n';
        return false;
    }

    std::cout << "Connected: "
              << source << " -> " << destination << '\n';

    return true;
}

} // namespace

int main()
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "Could not connect to the YARP network.\n";
        return 1;
    }

    const std::string leftHandSource = "/icub/skin/left_hand_comp";
    const std::string rightHandSource = "/icub/skin/right_hand_comp";

    const std::string leftHandPortName =
        "/primi-ball-balance/skin/left_hand:i";

    const std::string rightHandPortName =
        "/primi-ball-balance/skin/right_hand:i";

    yarp::os::BufferedPort<yarp::sig::Vector> leftHandPort;
    yarp::os::BufferedPort<yarp::sig::Vector> rightHandPort;

    if (!leftHandPort.open(leftHandPortName)) {
        std::cerr << "Could not open " << leftHandPortName << '\n';
        return 1;
    }

    if (!rightHandPort.open(rightHandPortName)) {
        std::cerr << "Could not open " << rightHandPortName << '\n';
        leftHandPort.close();
        return 1;
    }

    if (!connectPort(leftHandSource, leftHandPortName)) {
        leftHandPort.close();
        rightHandPort.close();
        return 1;
    }

    if (!connectPort(rightHandSource, rightHandPortName)) {
        yarp::os::Network::disconnect(leftHandSource, leftHandPortName);
        leftHandPort.close();
        rightHandPort.close();
        return 1;
    }

    std::cout << "Reading compensated left and right hand skin streams.\n";
    std::cout << "Press Ctrl+C to stop.\n";

    std::thread leftThread(
        readSkin, std::ref(leftHandPort), "left hand");

    std::thread rightThread(
        readSkin, std::ref(rightHandPort), "right hand");

    leftThread.join();
    rightThread.join();

    yarp::os::Network::disconnect(leftHandSource, leftHandPortName);
    yarp::os::Network::disconnect(rightHandSource, rightHandPortName);

    leftHandPort.close();
    rightHandPort.close();

    return 0;
}