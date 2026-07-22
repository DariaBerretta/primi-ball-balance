#include "BallDetector.h"
#include "ImagePreprocessor.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <exception>
#include <iostream>
#include <string>

int main(int argc, char **argv)
{
    if(argc != 2 && argc != 8) {
        std::cerr
            << "Usage:\n"
            << "  test_ball_detector <image>\n"
            << "  test_ball_detector <image> "
            << "<hMin> <sMin> <vMin> <hMax> <sMax> <vMax>\n";

        return 1;
    }

    const cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);

    if(image.empty()) {
        std::cerr << "Could not read image: " << argv[1] << '\n';
        return 1;
    }

    BallDetectorConfig detectorConfig;

    if(argc == 8) {
        try {
            detectorConfig.hsvRanges = {{
                {
                    std::stoi(argv[2]),
                    std::stoi(argv[3]),
                    std::stoi(argv[4])
                },
                {
                    std::stoi(argv[5]),
                    std::stoi(argv[6]),
                    std::stoi(argv[7])
                }
            }};
        } catch(const std::exception &) {
            std::cerr << "HSV limits must be integers\n";
            return 1;
        }
    }

    try {
        PreprocessingConfig preprocessingConfig;
        preprocessingConfig.inputColourOrder = InputColourOrder::BGR;
        preprocessingConfig.gaussianKernelSize = 5;

        const ImagePreprocessor preprocessor(preprocessingConfig);
        const BallDetector detector(detectorConfig);

        const PreprocessedFrame frame = preprocessor.process(image);

        cv::Mat mask;
        const BallDetectionOutput detection = detector.detect(frame, &mask);

        cv::Mat annotated = frame.bgr.clone();

        if(detection.valid) {
            cv::circle(
                annotated, detection.center,
                static_cast<int>(detection.radius),
                cv::Scalar(0, 255, 0), 2);

            cv::circle(
                annotated, detection.center, 3,
                cv::Scalar(0, 0, 255), -1);

            const std::string label =
                "center=(" +
                std::to_string(static_cast<int>(detection.center.x)) +
                ", " +
                std::to_string(static_cast<int>(detection.center.y)) +
                ") radius=" +
                std::to_string(static_cast<int>(detection.radius));

            cv::putText(
                annotated, label, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 0), 2);

            std::cout
                << "Ball detected\n"
                << "Center: "
                << detection.center.x << ", "
                << detection.center.y << '\n'
                << "Radius: " << detection.radius << " px\n"
                << "Confidence: " << detection.confidence << '\n';
        } else {
            cv::putText(
                annotated, "Ball not detected", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 0, 255), 2);

            std::cout << "Ball not detected\n";
        }

        if(!cv::imwrite("ball_mask.png", mask) ||
           !cv::imwrite("ball_detection.png", annotated)) {
            std::cerr << "Failed to save test outputs\n";
            return 1;
        }

        std::cout
            << "Saved ball_mask.png\n"
            << "Saved ball_detection.png\n";
    } catch(const std::exception &exception) {
        std::cerr << "Ball detection failed: "
                  << exception.what() << '\n';

        return 1;
    }

    return 0;
}