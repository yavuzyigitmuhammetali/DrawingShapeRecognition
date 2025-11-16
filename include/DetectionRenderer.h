#pragma once

#include "ShapeClassifier.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class DetectionRenderer {
public:
    DetectionRenderer() = default;
    ~DetectionRenderer() = default;

    void drawDetections(cv::Mat &image, const std::vector<DetectedShape> &shapes) const;

private:
    std::string formatShapeLabel(const DetectedShape &shape, int precision = 2) const;
};
