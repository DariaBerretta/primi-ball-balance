#include "ImagePreprocessor.h"

#include <opencv2/imgproc.hpp>

#include <stdexcept>
#include <utility>

ImagePreprocessor::ImagePreprocessor(PreprocessingConfig config)
    : config(std::move(config))
{
    validateConfig();
}

void ImagePreprocessor::validateConfig() const
{
    const int kernelSize = config.gaussianKernelSize;

    if(kernelSize < 0)
        throw std::invalid_argument("Gaussian kernel size cannot be negative");

    if(kernelSize > 1 && kernelSize % 2 == 0)
        throw std::invalid_argument("Gaussian kernel size must be odd");

    if(config.gaussianSigma < 0.0)
        throw std::invalid_argument("Gaussian sigma cannot be negative");
}

void ImagePreprocessor::validateImage(const cv::Mat &image) const
{
    if(image.empty())
        throw std::invalid_argument("Cannot preprocess an empty image");

    if(image.depth() != CV_8U)
        throw std::invalid_argument("Input image must use 8-bit channels");

    if(image.channels() != 3)
        throw std::invalid_argument("Input image must contain three colour channels");
}

PreprocessedFrame ImagePreprocessor::process(
    const cv::Mat &image, double timestamp) const
{
    validateImage(image);

    cv::Mat normalisedBgr;

    if(config.inputColourOrder == InputColourOrder::RGB)
        cv::cvtColor(image, normalisedBgr, cv::COLOR_RGB2BGR);
    else
        image.copyTo(normalisedBgr);

    PreprocessedFrame frame;
    frame.timestamp = timestamp;

    if(config.gaussianKernelSize > 1) {
        cv::Size kernel(config.gaussianKernelSize, config.gaussianKernelSize);
        cv::GaussianBlur(normalisedBgr, frame.bgr, kernel, config.gaussianSigma);
    } else {
        frame.bgr = normalisedBgr;
    }

    cv::cvtColor(frame.bgr, frame.hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(frame.bgr, frame.gray, cv::COLOR_BGR2GRAY);

    return frame;
}