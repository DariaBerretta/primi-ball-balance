#include "BallDetector.h"
#include "ImagePreprocessor.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cstddef>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
using RgbImage = yarp::sig::ImageOf<yarp::sig::PixelRgb>;

struct StereoFrame
{
    cv::Mat left;
    cv::Mat right;
};

struct EyePorts
{
    std::string eye;
    std::string rawName;
    std::string processedName;
    std::string detectionName;

    yarp::os::BufferedPort<RgbImage> raw;
    yarp::os::BufferedPort<RgbImage> processed;
    yarp::os::BufferedPort<yarp::sig::Vector> detection;
};

StereoFrame splitStereoFrame(const cv::Mat &combined)
{
    if(combined.empty())
        throw std::invalid_argument("Cannot split an empty stereo image");

    if(combined.cols % 2 != 0)
        throw std::runtime_error("Combined stereo image width must be even");

    const int eyeWidth = combined.cols / 2;

    return {
        combined(cv::Rect(0, 0, eyeWidth, combined.rows)),
        combined(cv::Rect(eyeWidth, 0, eyeWidth, combined.rows))
    };
}

void configureEyePorts(EyePorts &ports, const std::string &eye)
{
    const std::string baseName = "/primi-ball-balance/rgbCam/" + eye;

    ports.eye = eye;
    ports.rawName = baseName + "/raw:o";
    ports.processedName = baseName + "/processed:o";
    ports.detectionName = baseName + "/detection:o";
}

void closeEyePorts(EyePorts &ports)
{
    ports.raw.close();
    ports.processed.close();
    ports.detection.close();
}

bool openEyePorts(EyePorts &ports)
{
    if(!ports.raw.open(ports.rawName)) {
        std::cerr << "Could not open " << ports.rawName << '\n';
        return false;
    }

    if(!ports.processed.open(ports.processedName)) {
        std::cerr << "Could not open " << ports.processedName << '\n';
        closeEyePorts(ports);
        return false;
    }

    if(!ports.detection.open(ports.detectionName)) {
        std::cerr << "Could not open " << ports.detectionName << '\n';
        closeEyePorts(ports);
        return false;
    }

    return true;
}

void publishRaw(EyePorts &ports, const cv::Mat &rgbImage, bool hasStamp,
                yarp::os::Stamp &stamp)
{
    if(ports.raw.getOutputCount() == 0)
        return;

    RgbImage &output = ports.raw.prepare();
    output.resize(rgbImage.cols, rgbImage.rows);

    cv::Mat outputView(output.height(), output.width(), CV_8UC3,
                       output.getRawImage(), output.getRowSize());

    rgbImage.copyTo(outputView);

    if(hasStamp)
        ports.raw.setEnvelope(stamp);

    ports.raw.write();
}

void publishProcessed(EyePorts &ports, const PreprocessedFrame &frame,
                      const BallDetectionOutput &detection, bool hasStamp,
                      yarp::os::Stamp &stamp)
{
    if(ports.processed.getOutputCount() == 0)
        return;

    cv::Mat annotated = frame.bgr.clone();

    if(detection.valid) {
        cv::circle(annotated, detection.center, static_cast<int>(detection.radius),
                   cv::Scalar(0, 255, 0), 2);
        cv::circle(annotated, detection.center, 3, cv::Scalar(0, 0, 255), -1);

        const std::string label =
            ports.eye +
            " x=" + std::to_string(static_cast<int>(detection.center.x)) +
            " y=" + std::to_string(static_cast<int>(detection.center.y)) +
            " r=" + std::to_string(static_cast<int>(detection.radius));

        cv::putText(annotated, label, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2);
    } else {
        cv::putText(annotated, ports.eye + ": ball not detected",
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                    0.6, cv::Scalar(0, 0, 255), 2);
    }

    RgbImage &output = ports.processed.prepare();
    output.resize(annotated.cols, annotated.rows);

    cv::Mat outputView(output.height(), output.width(), CV_8UC3,
                       output.getRawImage(), output.getRowSize());

    cv::cvtColor(annotated, outputView, cv::COLOR_BGR2RGB);

    if(hasStamp)
        ports.processed.setEnvelope(stamp);

    ports.processed.write();
}

void publishDetection(EyePorts &ports, const BallDetectionOutput &detection,
                      double timestamp, bool hasStamp, yarp::os::Stamp &stamp)
{
    yarp::sig::Vector &data = ports.detection.prepare();
    data.resize(6);

    data[0] = detection.valid ? 1.0 : 0.0;
    data[1] = detection.valid ? detection.center.x : -1.0;
    data[2] = detection.valid ? detection.center.y : -1.0;
    data[3] = detection.valid ? detection.radius : -1.0;
    data[4] = detection.valid ? detection.confidence : 0.0;
    data[5] = timestamp;

    if(hasStamp)
        ports.detection.setEnvelope(stamp);

    ports.detection.write();
}

BallDetectionOutput processEye(EyePorts &ports, const cv::Mat &rgbImage,
                               double timestamp, bool hasStamp,
                               yarp::os::Stamp &stamp,
                               const ImagePreprocessor &preprocessor,
                               const BallDetector &detector)
{
    const PreprocessedFrame frame = preprocessor.process(rgbImage, timestamp);
    const BallDetectionOutput detection = detector.detect(frame);

    publishRaw(ports, rgbImage, hasStamp, stamp);
    publishProcessed(ports, frame, detection, hasStamp, stamp);
    publishDetection(ports, detection, timestamp, hasStamp, stamp);

    return detection;
}

void printDetection(const std::string &eye, const BallDetectionOutput &detection)
{
    std::cout << eye << ": ";

    if(!detection.valid) {
        std::cout << "not detected";
        return;
    }

    std::cout << '(' << detection.center.x << ", " << detection.center.y << ')'
              << " r=" << detection.radius
              << " confidence=" << detection.confidence;
}
}

int main()
{
    yarp::os::Network yarp;

    if(!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "Could not connect to the YARP network.\n";
        return 1;
    }

    const std::string inputPortName = "/primi-ball-balance/rgbCam/stereo:i";

    yarp::os::BufferedPort<RgbImage> inputPort;

    EyePorts leftPorts;
    EyePorts rightPorts;

    configureEyePorts(leftPorts, "left");
    configureEyePorts(rightPorts, "right");

    if(!inputPort.open(inputPortName)) {
        std::cerr << "Could not open " << inputPortName << '\n';
        return 1;
    }

    if(!openEyePorts(leftPorts)) {
        inputPort.close();
        return 1;
    }

    if(!openEyePorts(rightPorts)) {
        inputPort.close();
        closeEyePorts(leftPorts);
        return 1;
    }

    PreprocessingConfig preprocessingConfig;
    preprocessingConfig.inputColourOrder = InputColourOrder::RGB;
    preprocessingConfig.gaussianKernelSize = 5;
    preprocessingConfig.gaussianSigma = 0.0;

    BallDetectorConfig detectorConfig;
    detectorConfig.hsvRanges = {
        {{0, 35, 120}, {15, 255, 255}},
        {{165, 35, 120}, {179, 255, 255}}
    };
    detectorConfig.minContourArea = 100.0;
    detectorConfig.minCircularity = 0.65;
    detectorConfig.minRadius = 3.0f;
    detectorConfig.maxRadius = 1000.0f;
    detectorConfig.morphologyKernelSize = 5;
    detectorConfig.morphologyIterations = 1;

    try {
        const ImagePreprocessor preprocessor(preprocessingConfig);
        const BallDetector detector(detectorConfig);

        std::cout << "Waiting for combined RGB stereo frames on:\n"
                  << "  " << inputPortName << "\n\n"
                  << "Expected layout:\n"
                  << "  [left eye][right eye]\n\n"
                  << "Outputs:\n"
                  << "  " << leftPorts.rawName << '\n'
                  << "  " << rightPorts.rawName << '\n'
                  << "  " << leftPorts.processedName << '\n'
                  << "  " << rightPorts.processedName << '\n'
                  << "  " << leftPorts.detectionName << '\n'
                  << "  " << rightPorts.detectionName << '\n';

        std::size_t frameCount = 0;
        bool dimensionsPrinted = false;

        while(true) {
            RgbImage *inputImage = inputPort.read(true);

            if(inputImage == nullptr)
                break;

            ++frameCount;

            yarp::os::Stamp inputStamp;
            const bool hasStamp = inputPort.getEnvelope(inputStamp);
            const double timestamp = hasStamp
                ? inputStamp.getTime()
                : yarp::os::Time::now();

            cv::Mat combinedRgb(inputImage->height(), inputImage->width(), CV_8UC3,
                                inputImage->getRawImage(), inputImage->getRowSize());

            const StereoFrame stereo = splitStereoFrame(combinedRgb);

            if(!dimensionsPrinted) {
                std::cout << "Combined image: "
                          << combinedRgb.cols << 'x' << combinedRgb.rows
                          << "\nPer-eye image: "
                          << stereo.left.cols << 'x' << stereo.left.rows << '\n';

                dimensionsPrinted = true;
            }

            const BallDetectionOutput leftDetection =
                processEye(leftPorts, stereo.left, timestamp, hasStamp,
                           inputStamp, preprocessor, detector);

            const BallDetectionOutput rightDetection =
                processEye(rightPorts, stereo.right, timestamp, hasStamp,
                           inputStamp, preprocessor, detector);

            if(frameCount % 30 == 0) {
                printDetection("left", leftDetection);
                std::cout << " | ";
                printDetection("right", rightDetection);
                std::cout << std::endl;
            }
        }
    } catch(const std::exception &exception) {
        std::cerr << "RGB stereo processing failed: "
                  << exception.what() << '\n';

        inputPort.close();
        closeEyePorts(leftPorts);
        closeEyePorts(rightPorts);
        return 1;
    }

    inputPort.close();
    closeEyePorts(leftPorts);
    closeEyePorts(rightPorts);

    return 0;
}