#pragma once

#include <opencv2/core.hpp>

struct PreprocessedFrame
{
    cv::Mat bgr;
    cv::Mat hsv;
    cv::Mat gray;

    double timestamp = 0.0;

    bool valid() const noexcept
    {
        return !bgr.empty() && !hsv.empty() && !gray.empty();
    }
};