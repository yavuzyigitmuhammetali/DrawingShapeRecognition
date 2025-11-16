#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct DetectedShape {
    std::string type{"Unknown"};
    double smoothness{0.0};
    cv::Rect boundingBox;
    std::vector<cv::Point> contour;
};

class ShapeClassifier {
public:
    ShapeClassifier() = default;
    ~ShapeClassifier() = default;

    std::vector<DetectedShape> findShapes(const cv::Mat &warpedImage) const;

private:
    static constexpr double kMinShapeAreaRatio = 0.0015;
    static constexpr double kCircularityThreshold = 0.85;
};
