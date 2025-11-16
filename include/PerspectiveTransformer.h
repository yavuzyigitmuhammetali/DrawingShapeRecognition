#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class PerspectiveTransformer {
public:
    PerspectiveTransformer() = default;
    ~PerspectiveTransformer() = default;

    cv::Mat warpImage(const cv::Mat &frame, const std::vector<cv::Point> &points) const;

private:
    std::vector<cv::Point> reOrderPoints(const std::vector<cv::Point> &points) const;

    static constexpr float kWarpWidth = 640.0F;
    static constexpr float kWarpHeight = 480.0F;
};
