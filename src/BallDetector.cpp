#include "BallDetector.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
constexpr double PI = 3.14159265358979323846;

bool validHSVScalar(const cv::Scalar &value)
{
    return value[0] >= 0.0 && value[0] <= 179.0 &&
           value[1] >= 0.0 && value[1] <= 255.0 &&
           value[2] >= 0.0 && value[2] <= 255.0;
}
}

BallDetector::BallDetector(BallDetectorConfig config)
    : config(std::move(config))
{
    validateConfig();
}

void BallDetector::validateConfig() const
{
    if(config.hsvRanges.empty())
        throw std::invalid_argument("At least one HSV range is required");

    for(const HSVRange &range : config.hsvRanges) {
        if(!validHSVScalar(range.lower) || !validHSVScalar(range.upper))
            throw std::invalid_argument("HSV limits are outside the valid range");

        for(int channel = 0; channel < 3; ++channel) {
            if(range.lower[channel] > range.upper[channel])
                throw std::invalid_argument("HSV lower limit exceeds upper limit");
        }
    }

    if(config.minContourArea < 0.0)
        throw std::invalid_argument("Minimum contour area cannot be negative");

    if(config.minCircularity < 0.0 || config.minCircularity > 1.0)
        throw std::invalid_argument("Minimum circularity must be between 0 and 1");

    if(config.minRadius < 0.0f || config.maxRadius <= config.minRadius)
        throw std::invalid_argument("Invalid radius limits");

    if(config.morphologyKernelSize < 0)
        throw std::invalid_argument("Morphology kernel size cannot be negative");

    if(config.morphologyKernelSize > 1 &&
       config.morphologyKernelSize % 2 == 0)
        throw std::invalid_argument("Morphology kernel size must be odd");

    if(config.morphologyIterations < 0)
        throw std::invalid_argument("Morphology iterations cannot be negative");
}

cv::Mat BallDetector::createMask(const cv::Mat &hsvImage) const
{
    cv::Mat combinedMask = cv::Mat::zeros(
        hsvImage.rows, hsvImage.cols, CV_8UC1);

    for(const HSVRange &range : config.hsvRanges) {
        cv::Mat rangeMask;
        cv::inRange(hsvImage, range.lower, range.upper, rangeMask);
        cv::bitwise_or(combinedMask, rangeMask, combinedMask);
    }

    if(config.morphologyKernelSize > 1 &&
       config.morphologyIterations > 0) {
        const cv::Size kernelSize(
            config.morphologyKernelSize,
            config.morphologyKernelSize);

        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, kernelSize);

        cv::morphologyEx(
            combinedMask, combinedMask, cv::MORPH_OPEN, kernel,
            cv::Point(-1, -1), config.morphologyIterations);

        cv::morphologyEx(
            combinedMask, combinedMask, cv::MORPH_CLOSE, kernel,
            cv::Point(-1, -1), config.morphologyIterations);
    }

    return combinedMask;
}

BallDetectionOutput BallDetector::detect(
    const PreprocessedFrame &frame, cv::Mat *maskOutput) const
{
    BallDetectionOutput result;

    if(!frame.valid())
        return result;

    cv::Mat mask = createMask(frame.hsv);

    if(maskOutput != nullptr)
        mask.copyTo(*maskOutput);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(
        mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double bestArea = 0.0;

    for(const std::vector<cv::Point> &contour : contours) {
        const double area = cv::contourArea(contour);

        if(area < config.minContourArea)
            continue;

        const double perimeter = cv::arcLength(contour, true);

        if(perimeter <= 0.0)
            continue;

        const double circularity =
            4.0 * PI * area / (perimeter * perimeter);

        if(circularity < config.minCircularity)
            continue;

        cv::Point2f center;
        float radius = 0.0f;
        cv::minEnclosingCircle(contour, center, radius);

        if(radius < config.minRadius || radius > config.maxRadius)
            continue;

        const double enclosingCircleArea = PI * radius * radius;

        if(enclosingCircleArea <= 0.0)
            continue;

        const double fillRatio = std::clamp(
            area / enclosingCircleArea, 0.0, 1.0);

        if(area <= bestArea)
            continue;

        bestArea = area;

        result.valid = true;
        result.center = center;
        result.radius = radius;
        result.confidence = static_cast<float>(std::clamp(
            0.5 * circularity + 0.5 * fillRatio, 0.0, 1.0));
    }

    return result;
}