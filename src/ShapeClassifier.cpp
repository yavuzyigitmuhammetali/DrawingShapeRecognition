#include "ShapeClassifier.hpp"

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>

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
        result.label = toString(result.type);
        result.confidence = Params::CONFIDENCE_TRIANGLE;
        return result;
    }

    if (vertexCount == 4) {
        // Calculate all 4 side lengths
        double L1 = cv::norm(approx[0] - approx[1]);
        double L2 = cv::norm(approx[1] - approx[2]);
        double L3 = cv::norm(approx[2] - approx[3]);
        double L4 = cv::norm(approx[3] - approx[0]);

        // Calculate average length of opposite side pairs
        // This handles perspective distortion (e.g., 100x110x100x110 from a perfect 100x100 square)
        double avgPair1 = (L1 + L3) / 2.0;
        double avgPair2 = (L2 + L4) / 2.0;

        // Calculate aspect ratio of the averaged pairs
        double oppositePairRatio = 0.0;
        if (std::max(avgPair1, avgPair2) > 0) {
            oppositePairRatio = std::min(avgPair1, avgPair2) / std::max(avgPair1, avgPair2);
        }

        // Tolerance threshold: 10% difference allowed for squares
        // Example: 100/110 = 0.909 → Square, 100/130 = 0.769 → Rectangle
        const double SQUARE_TOLERANCE_RATIO = 0.90;

        if (oppositePairRatio > SQUARE_TOLERANCE_RATIO) {
            result.type = ShapeType::Square;
            result.label = toString(result.type);
            result.confidence = Params::CONFIDENCE_SQUARE;
        } else {
            result.type = ShapeType::Rectangle;
            result.label = toString(result.type);
            result.confidence = Params::CONFIDENCE_RECTANGLE;
        }
        return result;
    }

    if (vertexCount == 6) {
        result.type = ShapeType::Hexagon;
        result.label = toString(result.type);
        result.confidence = Params::CONFIDENCE_HEXAGON;
        return result;
    }

    double circularity = 0.0;
    if (perimeter > 0.0) {
        circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
    }

    if (circularity > Params::CIRCLE_CIRCULARITY_THRESHOLD) {
        result.type = ShapeType::Circle;
        result.label = toString(result.type);
        result.confidence = std::min(1.0, circularity);
        return result;
    }

    result.confidence = circularity;
    result.label = toString(result.type);
    return result;
}

std::string ShapeClassifier::toString(ShapeType type) {
    switch (type) {
    case ShapeType::Circle:
        return "Circle";
    case ShapeType::Triangle:
        return "Triangle";
    case ShapeType::Square:
        return "Square";
    case ShapeType::Rectangle:
        return "Rectangle";
    case ShapeType::Hexagon:
        return "Hexagon";
    default:
        return "Unknown";
    }
}
