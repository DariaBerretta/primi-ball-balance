#include <event-driven/core.h>
#include <yarp/os/Network.h>
 
#include <iostream>
#include <string>
 
int main(int argc, char* argv[]) {
    yarp::os::Network yarp;
 
    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "Could not connect to the YARP network.\n";
        return 1;
    }
 
    const std::string remotePort =
        argc > 1 ? argv[1] : "/zynqGrabber/left/AE:o";
 
    const std::string localPort =
        "/primi-ball-balance/eventCam:i";
 
    ev::BufferedPort<ev::AE> eventPort;
 
    if (!eventPort.open(localPort)) {
        std::cerr << "Could not open " << localPort << '\n';
        return 1;
    }
 
    if (!yarp::os::Network::connect(remotePort, localPort)) {
        std::cerr << "Could not connect " << remotePort << " to " << localPort << '\n';
 
        eventPort.close();
        return 1;
    }
 
    std::cout << "Reading events from " << remotePort << '\n';
 
    while (true) {
        ev::packet<ev::AE>* packet = eventPort.read(true);
 
        if (packet == nullptr) {
            break;
        }
 
        std::cout << "Received packet with " << packet->size() << " events\n";
    }
 
    eventPort.close();
    return 0;
}