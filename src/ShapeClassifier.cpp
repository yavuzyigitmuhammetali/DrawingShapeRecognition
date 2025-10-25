#include "ShapeClassifier.hpp"

#include <opencv2/opencv.hpp>

using Params = ShapeClassifier::Parameters;

ShapeClassifier::Classification ShapeClassifier::classify(const ShapeSegmenter::Candidate& candidate) const {
    Classification result;

    if (candidate.contour.empty()) {
        return result;
    }

    double perimeter = cv::arcLength(candidate.contour, true);
    if (perimeter <= 0.0) {
        return result;
    }

    std::vector<cv::Point> approx;
    cv::approxPolyDP(candidate.contour, approx, Params::POLYGON_EPSILON * perimeter, true);
    const double area = cv::contourArea(candidate.contour);
    const int vertexCount = static_cast<int>(approx.size());

    if (vertexCount == 3) {
        result.type = ShapeType::Triangle;
        result.label = "Triangle";
        result.confidence = Params::CONFIDENCE_TRIANGLE;
        return result;
    }

    if (vertexCount == 4) {
        cv::Rect bounds = cv::boundingRect(approx);
        double aspectRatio = static_cast<double>(bounds.width) / static_cast<double>(bounds.height);
        aspectRatio = std::max(aspectRatio, 1.0 / aspectRatio);

        if (aspectRatio < Params::SQUARE_ASPECT_RATIO_THRESHOLD) {
            result.type = ShapeType::Square;
            result.label = "Square";
            result.confidence = Params::CONFIDENCE_SQUARE;
        } else {
            result.type = ShapeType::Rectangle;
            result.label = "Rectangle";
            result.confidence = Params::CONFIDENCE_RECTANGLE;
        }
        return result;
    }

    if (vertexCount == 6) {
        result.type = ShapeType::Hexagon;
        result.label = "Hexagon";
        result.confidence = Params::CONFIDENCE_HEXAGON;
        return result;
    }

    double circularity = 0.0;
    if (perimeter > 0.0) {
        circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
    }

    if (circularity > Params::CIRCLE_CIRCULARITY_THRESHOLD) {
        result.type = ShapeType::Circle;
        result.label = "Circle";
        result.confidence = std::min(1.0, circularity);
        return result;
    }

    result.confidence = circularity;
    return result;
}
