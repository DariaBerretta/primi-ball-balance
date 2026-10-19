#include <event-driven/core.h>
#include <yarp/os/Network.h>

#include <cstddef>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace {

struct PacketCounts {
    std::mutex mutex;
    std::optional<std::size_t> left;
    std::optional<std::size_t> right;
};

void forwardPacket(ev::packet<ev::AE>& inputPacket,
                   ev::BufferedPort<ev::AE>& outputPort)
{
    auto& outputPacket = outputPort.prepare();

    for (auto event = inputPacket.begin(); event != inputPacket.end(); ++event) {
        outputPacket.push_back(*event);
    }

    outputPacket.duration(inputPacket.duration());
    outputPacket.envelope() = inputPacket.envelope();
    outputPort.write();
}

void readCamera(ev::BufferedPort<ev::AE>& inputPort,
                ev::BufferedPort<ev::AE>& outputPort,
                bool isLeft,
                PacketCounts& counts)
{
    std::size_t packetCount = 0;

    while (true) {
        auto* packet = inputPort.read(true);

        if (packet == nullptr) {
            break;
        }

        ++packetCount;
        forwardPacket(*packet, outputPort);

        std::scoped_lock lock(counts.mutex);

        if (isLeft) {
            counts.left = packet->size();
        } else {
            counts.right = packet->size();
        }

        if (packetCount % 30 != 0) {
            continue;
        }

        std::cout << "left: ";

        if (counts.left.has_value()) {
            std::cout << *counts.left << " events";
        } else {
            std::cout << "waiting";
        }

        std::cout << " | right: ";

        if (counts.right.has_value()) {
            std::cout << *counts.right << " events";
        } else {
            std::cout << "waiting";
        }

        std::cout << '\n';
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
        "/primi-ball-balance/eventCam/left:i";
    const std::string rightInputName =
        "/primi-ball-balance/eventCam/right:i";

    const std::string leftOutputName =
        "/primi-ball-balance/eventCam/left/raw:o";
    const std::string rightOutputName =
        "/primi-ball-balance/eventCam/right/raw:o";

    ev::BufferedPort<ev::AE> leftInput;
    ev::BufferedPort<ev::AE> rightInput;
    ev::BufferedPort<ev::AE> leftOutput;
    ev::BufferedPort<ev::AE> rightOutput;

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

    std::cout << "Waiting for event-camera streams on:\n"
              << "  " << leftInputName << '\n'
              << "  " << rightInputName << '\n'
              << "Republishing locally on:\n"
              << "  " << leftOutputName << '\n'
              << "  " << rightOutputName << '\n';

    PacketCounts counts;

    std::thread leftThread(
        readCamera, std::ref(leftInput), std::ref(leftOutput),
        true, std::ref(counts));
    std::thread rightThread(
        readCamera, std::ref(rightInput), std::ref(rightOutput),
        false, std::ref(counts));

    leftThread.join();
    rightThread.join();

    leftInput.close();
    rightInput.close();
    leftOutput.close();
    rightOutput.close();

    return 0;
}
