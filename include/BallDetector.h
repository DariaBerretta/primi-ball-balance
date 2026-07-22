#pragma once

#include "DetectorOutputStruct.h"
#include "PreprocessedFrame.h"

#include <opencv2/core.hpp>

#include <vector>

struct HSVRange
{
    cv::Scalar lower;
    cv::Scalar upper;
};

struct BallDetectorConfig
{
    std::vector<HSVRange> hsvRanges = {
        {{0, 35, 120}, {15, 255, 255}},
        {{165, 35, 120}, {179, 255, 255}}
    };

    double minContourArea = 100.0;
    double minCircularity = 0.65;

    float minRadius = 3.0f;
    float maxRadius = 1000.0f;

    int morphologyKernelSize = 5;
    int morphologyIterations = 1;
};

class BallDetector
{
public:
    explicit BallDetector(BallDetectorConfig config = {});

    BallDetectionOutput detect(
        const PreprocessedFrame &frame, cv::Mat *maskOutput = nullptr) const;

private:
    BallDetectorConfig config;

    void validateConfig() const;
    cv::Mat createMask(const cv::Mat &hsvImage) const;
};