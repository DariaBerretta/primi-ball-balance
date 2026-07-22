#pragma once

#include "PreprocessedFrame.h"

#include <opencv2/core.hpp>

enum class InputColourOrder
{
    BGR,
    RGB
};

struct PreprocessingConfig
{
    InputColourOrder inputColourOrder = InputColourOrder::BGR;
    int gaussianKernelSize = 5;
    double gaussianSigma = 0.0;
};

class ImagePreprocessor
{
public:
    explicit ImagePreprocessor(PreprocessingConfig config = {});

    PreprocessedFrame process(const cv::Mat &image, double timestamp = 0.0) const;

private:
    PreprocessingConfig config;

    void validateConfig() const;
    void validateImage(const cv::Mat &image) const;
};