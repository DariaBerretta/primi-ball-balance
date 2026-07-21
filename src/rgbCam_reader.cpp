#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>

#include <cstddef>
#include <iostream>
#include <string>

int main()
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "Could not connect to the YARP network.\n";
        return 1;
    }

    using RgbImage = yarp::sig::ImageOf<yarp::sig::PixelRgb>;

    yarp::os::BufferedPort<RgbImage> rgbPort;
    const std::string rgbPortName = "/primi-ball-balance/rgbCam:i";

    if (!rgbPort.open(rgbPortName)) {
        std::cerr << "Could not open " << rgbPortName << '\n';
        return 1;
    }

    std::cout << "Waiting for the RGB stream on "
              << rgbPortName << ".\n";

    std::size_t frameCount = 0;

    while (true) {
        RgbImage* image = rgbPort.read(true);

        if (image == nullptr) {
            break;
        }

        ++frameCount;

        // Avoid printing once per frame.
        if (frameCount % 30 == 0) {
            std::cout << "RGB: received frame "
                      << image->width() << 'x' << image->height()
                      << " - total frames: " << frameCount << '\n';
        }
    }

    rgbPort.close();
    return 0;
}