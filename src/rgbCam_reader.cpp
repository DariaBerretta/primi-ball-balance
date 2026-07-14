#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

std::mutex outputMutex;

void readCamera(
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>>& port,
    const std::string& cameraName)
{
    std::size_t frameCount = 0;

    while (true) {
        auto* image = port.read(true);

        if (image == nullptr) {
            break;
        }

        frameCount++;

        // Avoid printing once per frame.
        if (frameCount % 30 != 0) {
            continue;
        }

        std::scoped_lock lock(outputMutex);

        std::cout << cameraName
                  << ": received frame "
                  << image->width() << "x" << image->height()
                  << " - total frames: " << frameCount << '\n';
    }
}

int main() {
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "Could not connect to the YARP network.\n";
        return 1;
    }

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> leftPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> rightPort;

    const std::string leftPortName = "/primi-ball-balance/rgbCam/left:i";
    const std::string rightPortName = "/primi-ball-balance/rgbCam/right:i";

    if (!leftPort.open(leftPortName)) {
        std::cerr << "Could not open " << leftPortName << '\n';
        return 1;
    }

    if (!rightPort.open(rightPortName)) {
        std::cerr << "Could not open " << rightPortName << '\n';
        leftPort.close();
        return 1;
    }

    std::cout << "Waiting for left and right RGB camera streams.\n";

    std::thread leftThread(readCamera, std::ref(leftPort), "left");
    std::thread rightThread(readCamera, std::ref(rightPort), "right");

    leftThread.join();
    rightThread.join();

    leftPort.close();
    rightPort.close();

    return 0;
}