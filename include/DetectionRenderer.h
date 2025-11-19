#pragma once

#include "ShapeClassifier.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// Forward declaration to avoid circular dependency
struct ReferenceShape;
struct FillingStats;

class DetectionRenderer {
public:
    DetectionRenderer() = default;
    ~DetectionRenderer() = default;

    // DRAWING mode: Draw detected shapes with bounding boxes and labels
    void drawDetections(cv::Mat &image, const std::vector<DetectedShape> &shapes) const;

    // FILLING mode: Draw reference shapes with fill progress visualization
    void drawFillingMode(cv::Mat &image,
                         const std::vector<ReferenceShape> &referenceShapes,
                         const std::vector<FillingStats> &fillingStats) const;

private:
    std::string formatShapeLabel(const DetectedShape &shape, int precision = 2) const;
    std::string formatFillingLabel(const FillingStats &stats, int precision = 0) const;
};
