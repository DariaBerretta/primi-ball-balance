#include <event-driven/core.h>
#include <yarp/os/Network.h>

#include <functional>
#include <iostream>
#include <thread>

namespace {

// One remote subscription per eye. Forward the complete event packet and its
// timing metadata without filtering or changing the event sequence.
void forwardEventPackets(ev::BufferedPort<ev::AE>& eventSink,
                         ev::BufferedPort<ev::AE>& eventSource)
{
    while (auto* receivedPacket = eventSink.read(true)) {
        auto& outgoingPacket = eventSource.prepare();
        for (const auto& event : *receivedPacket) {
            outgoingPacket.push_back(event);
        }
        outgoingPacket.duration(receivedPacket->duration());
        outgoingPacket.envelope() = receivedPacket->envelope();
        eventSource.write();
    }
}

} // namespace

int main()
{
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "Could not connect to YARP.\n";
        return 1;
    }

    ev::BufferedPort<ev::AE> leftEventSink;
    ev::BufferedPort<ev::AE> rightEventSink;
    ev::BufferedPort<ev::AE> leftEventSource;
    ev::BufferedPort<ev::AE> rightEventSource;
    const bool opened =
        leftEventSink.open("/primi-ball-balance/eventCameraStream/left/sink:i") &&
        rightEventSink.open("/primi-ball-balance/eventCameraStream/right/sink:i") &&
        leftEventSource.open("/primi-ball-balance/eventCameraStream/left/source:o") &&
        rightEventSource.open("/primi-ball-balance/eventCameraStream/right/source:o");
    if (!opened) {
        std::cerr << "Could not open the stereo event relay ports.\n";
        leftEventSink.close();
        rightEventSink.close();
        leftEventSource.close();
        rightEventSource.close();
        return 1;
    }

    // Separate blocking reads prevent one camera from delaying the other.
    std::thread leftRelayThread(forwardEventPackets,
                                std::ref(leftEventSink), std::ref(leftEventSource));
    std::thread rightRelayThread(forwardEventPackets,
                                 std::ref(rightEventSink), std::ref(rightEventSource));
    leftRelayThread.join();
    rightRelayThread.join();

    leftEventSink.close();
    rightEventSink.close();
    leftEventSource.close();
    rightEventSource.close();
    return 0;
}
