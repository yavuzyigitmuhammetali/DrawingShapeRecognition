#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class ContourAnalyzer {
public:
    ContourAnalyzer() = default;
    ~ContourAnalyzer() = default;

    std::vector<cv::Point> getLargestContour(const cv::Mat &processedImage,
                                              cv::Size originalFrameSize) const;

private:
    static constexpr double kMinPaperAreaRatio = 0.05;
};
