#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <string>
#include <thread>

namespace {

std::mutex outputMutex;

void readSkin(yarp::os::BufferedPort<yarp::sig::Vector>& inputPort,
              yarp::os::BufferedPort<yarp::sig::Vector>& outputPort,
              const std::string& handName)
{
    std::size_t frameCount = 0;

    while (true) {
        const auto* taxels = inputPort.read(true);

        if (taxels == nullptr) {
            break;
        }

        yarp::os::Stamp inputStamp;
        const bool hasInputStamp = inputPort.getEnvelope(inputStamp);

        auto& output = outputPort.prepare();
        output = *taxels;

        if (hasInputStamp) {
            outputPort.setEnvelope(inputStamp);
        }

        outputPort.write();
        ++frameCount;

        if (taxels->size() == 0 || frameCount % 30 != 0) {
            continue;
        }

        const auto strongest = std::max_element(taxels->begin(), taxels->end());
        const auto strongestIndex =
            static_cast<std::size_t>(std::distance(taxels->begin(), strongest));

        std::scoped_lock lock(outputMutex);

        std::cout << handName
                  << ": received " << taxels->size() << " taxels"
                  << " - strongest at #" << strongestIndex
                  << " (" << *strongest << ")"
                  << " - total frames: " << frameCount << '\n';
    }
}

} // namespace

int main()
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "Could not connect to the YARP network.\n";
        return 1;
    }

    const std::string leftInputName =
        "/primi-ball-balance/skin/left_hand:i";
    const std::string rightInputName =
        "/primi-ball-balance/skin/right_hand:i";

    const std::string leftOutputName =
        "/primi-ball-balance/skin/left_hand/compensated:o";
    const std::string rightOutputName =
        "/primi-ball-balance/skin/right_hand/compensated:o";

    yarp::os::BufferedPort<yarp::sig::Vector> leftInput;
    yarp::os::BufferedPort<yarp::sig::Vector> rightInput;
    yarp::os::BufferedPort<yarp::sig::Vector> leftOutput;
    yarp::os::BufferedPort<yarp::sig::Vector> rightOutput;

    if (!leftInput.open(leftInputName)) {
        std::cerr << "Could not open " << leftInputName << '\n';
        return 1;
    }

    if (!rightInput.open(rightInputName)) {
        std::cerr << "Could not open " << rightInputName << '\n';
        leftInput.close();
        return 1;
    }

    if (!leftOutput.open(leftOutputName)) {
        std::cerr << "Could not open " << leftOutputName << '\n';
        leftInput.close();
        rightInput.close();
        return 1;
    }

    if (!rightOutput.open(rightOutputName)) {
        std::cerr << "Could not open " << rightOutputName << '\n';
        leftInput.close();
        rightInput.close();
        leftOutput.close();
        return 1;
    }

    std::cout << "Waiting for compensated skin streams on:\n"
              << "  " << leftInputName << '\n'
              << "  " << rightInputName << '\n'
              << "Republishing locally on:\n"
              << "  " << leftOutputName << '\n'
              << "  " << rightOutputName << '\n';

    std::thread leftThread(
        readSkin, std::ref(leftInput), std::ref(leftOutput), "left hand");
    std::thread rightThread(
        readSkin, std::ref(rightInput), std::ref(rightOutput), "right hand");

    leftThread.join();
    rightThread.join();

    leftInput.close();
    rightInput.close();
    leftOutput.close();
    rightOutput.close();

    return 0;
}
