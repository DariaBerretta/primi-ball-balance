#include <event-driven/core.h>
#include <yarp/os/Network.h>

#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

struct PacketCounts {
    std::mutex mutex;
    std::optional<std::size_t> left;
    std::optional<std::size_t> right;
};

void readCamera(ev::BufferedPort<ev::AE>& port, bool isLeft, PacketCounts& counts) {
    while (true) {
        ev::packet<ev::AE>* packet = port.read(true);

        if (packet == nullptr) {
            break;
        }

        std::scoped_lock lock(counts.mutex);

        if (isLeft) {
            counts.left = packet->size();
        } else {
            counts.right = packet->size();
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

        std::cout << std::endl;
    }
}

int main() {
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "Could not connect to the YARP network.\n";
        return 1;
    }

    const std::string leftPortName = "/primi-ball-balance/eventCam/left:i";
    const std::string rightPortName = "/primi-ball-balance/eventCam/right:i";

    ev::BufferedPort<ev::AE> leftPort;
    ev::BufferedPort<ev::AE> rightPort;

    if (!leftPort.open(leftPortName)) {
        std::cerr << "Could not open " << leftPortName << '\n';
        return 1;
    }

    if (!rightPort.open(rightPortName)) {
        std::cerr << "Could not open " << rightPortName << '\n';
        leftPort.close();
        return 1;
    }

    std::cout << "Waiting for event-camera streams on:\n"
              << "  " << leftPortName << '\n'
              << "  " << rightPortName << '\n';

    PacketCounts counts;

    std::thread leftThread(readCamera, std::ref(leftPort), true, std::ref(counts));
    std::thread rightThread(readCamera, std::ref(rightPort), false, std::ref(counts));

    leftThread.join();
    rightThread.join();

    leftPort.close();
    rightPort.close();

    return 0;
}