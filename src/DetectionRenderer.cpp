#include "DetectionRenderer.h"
#include "ShapeDetector.h"  // For ReferenceShape and FillingStats

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

void DetectionRenderer::drawFillingMode(cv::Mat &image,
                                         const std::vector<ReferenceShape> &referenceShapes,
                                         const std::vector<FillingStats> &fillingStats) const {
    if (referenceShapes.size() != fillingStats.size()) {
        return;  // Safety check
    }

    // Create overlay ONCE for semi-transparent visualization
    cv::Mat overlay = image.clone();

    for (size_t i = 0; i < referenceShapes.size(); ++i) {
        const auto &refShape = referenceShapes[i];
        const auto &stats = fillingStats[i];

        // Draw reference shape contour in BLUE directly onto overlay
        std::vector<std::vector<cv::Point>> contours = {refShape.contour};
        cv::drawContours(overlay, contours, 0, cv::Scalar(255, 0, 0), 2);

        // Overlay filled areas in GREEN (semi-transparent) - accumulate onto overlay
        if (!stats.filledMask.empty()) {
            cv::Mat greenMask = cv::Mat::zeros(image.size(), image.type());
            greenMask.setTo(cv::Scalar(0, 255, 0), stats.filledMask);
            cv::addWeighted(overlay, 1.0, greenMask, 0.3, 0, overlay);
        }

        // Overlay overflow areas in RED (semi-transparent) - accumulate onto overlay
        if (!stats.overflowMask.empty()) {
            cv::Mat redMask = cv::Mat::zeros(image.size(), image.type());
            redMask.setTo(cv::Scalar(0, 0, 255), stats.overflowMask);
            cv::addWeighted(overlay, 1.0, redMask, 0.4, 0, overlay);
        }

        // Draw statistics label next to the shape
        const std::string label = formatFillingLabel(stats);
        cv::Point labelOrigin(stats.boundingBox.x + stats.boundingBox.width + 10,
                              stats.boundingBox.y + stats.boundingBox.height / 2);

        // Draw background rectangle for better readability
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::rectangle(overlay,
                      cv::Point(labelOrigin.x - 2, labelOrigin.y - textSize.height - 2),
                      cv::Point(labelOrigin.x + textSize.width + 2, labelOrigin.y + baseline + 2),
                      cv::Scalar(255, 255, 255), cv::FILLED);

        // Draw text
        cv::putText(overlay, label, labelOrigin, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 0), 1);

        // Draw shape type label above the shape
        cv::Point typeLabel(stats.boundingBox.x, std::max(0, stats.boundingBox.y - 5));
        cv::putText(overlay, stats.shapeType, typeLabel, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 0, 0), 2);
    }

    // Copy final accumulated overlay back to image
    overlay.copyTo(image);
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

std::string DetectionRenderer::formatFillingLabel(const FillingStats &stats, int precision) const {
    std::ostringstream stream;
    stream << "Fill: " << std::fixed << std::setprecision(precision) << stats.fillPercentage << "%"
           << " | Err: " << std::fixed << std::setprecision(precision) << stats.overflowScore << "%";
    return stream.str();
}
