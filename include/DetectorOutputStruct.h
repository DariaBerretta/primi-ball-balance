#pragma once

#include <opencv2/core.hpp>

struct ImageDetectionOutput
{
    bool valid = false;
    cv::Point2f center = {-1.0f, -1.0f};
    float confidence = 0.0f;
};

struct BallDetectionOutput : public ImageDetectionOutput
{
    float radius = 0.0f;
};

struct ReferenceDetectionOutput : public ImageDetectionOutput
{
};