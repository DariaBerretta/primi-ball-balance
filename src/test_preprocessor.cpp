#include "ImagePreprocessor.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>

int main(int argc, char **argv)
{
    if(argc != 2) {
        std::cerr << "Usage: test_preprocessor <image-path>\n";
        return 1;
    }

    cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);

    if(image.empty()) {
        std::cerr << "Could not read image: " << argv[1] << '\n';
        return 1;
    }

    PreprocessingConfig config;
    config.inputColourOrder = InputColourOrder::BGR;
    config.gaussianKernelSize = 5;
    config.gaussianSigma = 0.0;

    try {
        ImagePreprocessor preprocessor(config);
        PreprocessedFrame frame = preprocessor.process(image);

        cv::imshow("Original", image);
        cv::imshow("Filtered BGR", frame.bgr);
        cv::imshow("HSV", frame.hsv);
        cv::imshow("Grayscale", frame.gray);

        cv::waitKey(0);
    } catch(const std::exception &exception) {
        std::cerr << "Preprocessing failed: " << exception.what() << '\n';
        return 1;
    }

    return 0;
}