#pragma once

#include <opencv2/core.hpp>

// BGR is used for drawing; HSV is used for colour-based ball detection.
struct PreprocessedFrame
{
    cv::Mat bgr;
    cv::Mat hsv;

    bool valid() const noexcept { return !bgr.empty() && !hsv.empty(); }
};

enum class InputColourOrder { BGR, RGB };

struct PreprocessingConfig
{
    InputColourOrder inputColourOrder = InputColourOrder::BGR;
    int gaussianKernelSize = 5;
    double gaussianSigma = 0.0;
};

// Validate the input once, then convert and smooth each camera frame.
class ImagePreprocessor
{
public:
    explicit ImagePreprocessor(PreprocessingConfig config = {});
    PreprocessedFrame process(const cv::Mat &image) const;

private:
    PreprocessingConfig config;
    void validateConfig() const;
    void validateImage(const cv::Mat &image) const;
};
