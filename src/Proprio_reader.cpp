#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

std::mutex outputMutex;

void readEncoders(
    const std::string& remotePort,
    const std::string& localPort,
    const std::string& outputPortName,
    const std::string& partName)
{
    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", remotePort);
    options.put("local", localPort);

    yarp::dev::PolyDriver driver;

    if (!driver.open(options)) {
        std::scoped_lock lock(outputMutex);
        std::cerr << "Could not open remote_controlboard for " << partName
                  << " (remote " << remotePort << ")\n";
        return;
    }

    yarp::dev::IEncoders* iEncoders = nullptr;

    if (!driver.view(iEncoders) || iEncoders == nullptr) {
        std::scoped_lock lock(outputMutex);
        std::cerr << "Could not get IEncoders interface for " << partName << '\n';
        driver.close();
        return;
    }

    int numAxes = 0;
    iEncoders->getAxes(&numAxes);

    yarp::os::BufferedPort<yarp::sig::Vector> outputPort;

    if (!outputPort.open(outputPortName)) {
        std::scoped_lock lock(outputMutex);
        std::cerr << "Could not open " << outputPortName << '\n';
        driver.close();
        return;
    }

    yarp::sig::Vector encoders(static_cast<size_t>(numAxes), 0.0);
    std::size_t readCount = 0;

    while (true) {
        // getEncoders() polls the control board for the latest joint values;
        // it returns false if a fresh reading isn't available yet.
        if (!iEncoders->getEncoders(encoders.data())) {
            yarp::os::Time::delay(0.01);
            continue;
        }

        readCount++;

        // Publish every fresh reading so the control process always has the
        // latest joint state - this is independent of the print rate below.
        yarp::sig::Vector& outVec = outputPort.prepare();
        outVec = encoders;
        outputPort.write();

        // Avoid printing on every poll - print roughly once every 30 reads.
        if (readCount % 30 != 0) {
            yarp::os::Time::delay(0.01);
            continue;
        }

        {
            std::scoped_lock lock(outputMutex);

            std::cout << partName << " (" << numAxes << " axes): [";
            for (int i = 0; i < numAxes; ++i) {
                std::cout << encoders[static_cast<size_t>(i)];
                if (i + 1 < numAxes) {
                    std::cout << ", ";
                }
            }
            std::cout << "] deg - total reads: " << readCount << '\n';
        }

        yarp::os::Time::delay(0.01);
    }

    outputPort.close();
    driver.close();
}

int main() {
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "Could not connect to the YARP network.\n";
        return 1;
    }

    // Adjust the "/icub/..." prefix to "/icubSim/..." if you're on the simulator.
    std::thread leftArmThread(
        readEncoders,
        "/icub/left_arm",
        "/primi-ball-balance/encoders/left_arm/client",
        "/primi-ball-balance/encoders/left_arm:o",
        "left arm");
    std::thread rightArmThread(
        readEncoders,
        "/icub/right_arm",
        "/primi-ball-balance/encoders/right_arm/client",
        "/primi-ball-balance/encoders/right_arm:o",
        "right arm");
    std::thread headThread(
        readEncoders,
        "/icub/head",
        "/primi-ball-balance/encoders/head/client",
        "/primi-ball-balance/encoders/head:o",
        "head");

    leftArmThread.join();
    rightArmThread.join();
    headThread.join();

    return 0;
}