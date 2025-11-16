#include "DetectionRenderer.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

void DetectionRenderer::drawDetections(cv::Mat &image,
                                        const std::vector<DetectedShape> &shapes) const {
    for (const auto &shape: shapes) {
        if (shape.type == "Unknown") {
            continue;
        }

        cv::rectangle(image, shape.boundingBox, cv::Scalar(0, 0, 255), 2);

        const std::string label = formatShapeLabel(shape);
        cv::Point labelOrigin(shape.boundingBox.x,
                              std::max(0, shape.boundingBox.y - 5));
        cv::putText(image, label, labelOrigin, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 255), 1);
    }
}

std::string DetectionRenderer::formatShapeLabel(const DetectedShape &shape, int precision) const {
    std::ostringstream stream;
    stream << shape.type;
    if (shape.type != "Unknown") {
        stream << " [" << std::fixed << std::setprecision(precision)
                << shape.smoothness << "]";
    }
    return stream.str();
}
