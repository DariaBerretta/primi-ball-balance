#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <string>

int main()
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "Could not connect to the YARP network.\n";
        return 1;
    }

    const std::string inputName =
        "/primi-ball-balance/skin/right_hand:i";
    const std::string outputName =
        "/primi-ball-balance/skin/right_hand/compensated:o";

    yarp::os::BufferedPort<yarp::sig::Vector> inputPort;
    yarp::os::BufferedPort<yarp::sig::Vector> outputPort;

    if (!inputPort.open(inputName)) {
        std::cerr << "Could not open " << inputName << '\n';
        return 1;
    }

    if (!outputPort.open(outputName)) {
        std::cerr << "Could not open " << outputName << '\n';
        inputPort.close();
        return 1;
    }

    std::cout << "Waiting for compensated right-hand skin data on:\n"
              << "  " << inputName << '\n'
              << "Republishing locally on:\n"
              << "  " << outputName << '\n';

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

        const auto strongest =
            std::max_element(taxels->begin(), taxels->end());
        const auto strongestIndex =
            static_cast<std::size_t>(
                std::distance(taxels->begin(), strongest));

        std::cout << "right hand: received " << taxels->size()
                  << " taxels - strongest at #" << strongestIndex
                  << " (" << *strongest << ")"
                  << " - total frames: " << frameCount << '\n';
    }

    inputPort.close();
    outputPort.close();

    return 0;
}