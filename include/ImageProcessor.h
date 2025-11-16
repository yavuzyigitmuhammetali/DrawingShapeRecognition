#pragma once

#include <opencv2/opencv.hpp>

class ImageProcessor {
public:
    ImageProcessor() = default;
    ~ImageProcessor() = default;

    cv::Mat preProcessImage(const cv::Mat &frame) const;

private:
    static constexpr double kSigma = 0.33;
};
