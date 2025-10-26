#include "ShapeClassifier.hpp"

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>

using Params = ShapeClassifier::Parameters;

namespace {
// Helper: Calculate robust median-based affinity for squares
// Checks if 4 sides cluster around a single intended length
double calculateSquareAffinity(const std::vector<cv::Point>& polygon) {
    if (polygon.size() != 4) {
        return 0.0;
    }

    // Calculate all 4 side lengths
    std::vector<double> lengths;
    lengths.reserve(4);
    for (size_t i = 0; i < 4; ++i) {
        const cv::Point2f a = polygon[i];
        const cv::Point2f b = polygon[(i + 1) % 4];
        lengths.push_back(cv::norm(a - b));
    }

    // Sort to find median
    std::sort(lengths.begin(), lengths.end());

    // Calculate median (for 4 elements: average of middle two)
    const double median = (lengths[1] + lengths[2]) / 2.0;

    if (median <= 0.0) {
        return 0.0;
    }

    // Calculate total normalized absolute deviation from median
    double totalDeviation = 0.0;
    for (double L : lengths) {
        totalDeviation += std::abs(L - median);
    }

    // Normalize by total length (median * 4)
    const double normalizedError = totalDeviation / (median * 4.0);

    // Convert to affinity score with strictness factor
    const double affinity = std::max(0.0, 1.0 - normalizedError * 2.0);

    return affinity;
}

// Helper: Calculate opposite-side affinity for rectangles
// Checks if opposite sides are equal (pairs of equal sides)
double calculateRectangleAffinity(const std::vector<cv::Point>& polygon) {
    if (polygon.size() != 4) {
        return 0.0;
    }

    // Calculate all 4 side lengths
    std::vector<double> lengths;
    lengths.reserve(4);
    for (size_t i = 0; i < 4; ++i) {
        const cv::Point2f a = polygon[i];
        const cv::Point2f b = polygon[(i + 1) % 4];
        lengths.push_back(cv::norm(a - b));
    }

    const double L1 = lengths[0];
    const double L2 = lengths[1];
    const double L3 = lengths[2];
    const double L4 = lengths[3];

    // Check opposite sides: L1 vs L3, L2 vs L4
    const double mean13 = (L1 + L3) / 2.0;
    const double mean24 = (L2 + L4) / 2.0;

    if (mean13 <= 0.0 || mean24 <= 0.0) {
        return 0.0;
    }

    const double error13 = std::abs(L1 - L3) / mean13;
    const double error24 = std::abs(L2 - L4) / mean24;
    const double totalError = error13 + error24;

    // Convert to affinity score
    const double affinity = std::max(0.0, 1.0 - totalError);

    return affinity;
}
} // namespace

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
        // Calculate affinities for both square and rectangle interpretations
        const double squareAffinity = calculateSquareAffinity(approx);
        const double rectangleAffinity = calculateRectangleAffinity(approx);

        // Minimum threshold for a valid quadrilateral classification
        constexpr double MIN_AFFINITY_THRESH = 0.6;

        // Choose the classification based on which affinity is stronger
        if (squareAffinity > rectangleAffinity && squareAffinity > MIN_AFFINITY_THRESH) {
            result.type = ShapeType::Square;
            result.label = "Square";
            result.confidence = Params::CONFIDENCE_SQUARE * squareAffinity;
        } else if (rectangleAffinity > MIN_AFFINITY_THRESH) {
            result.type = ShapeType::Rectangle;
            result.label = "Rectangle";
            result.confidence = Params::CONFIDENCE_RECTANGLE * rectangleAffinity;
        } else {
            // It's a poor quadrilateral, mark as generic
            result.type = ShapeType::Unknown;
            result.label = "Quadrilateral";
            result.confidence = std::max(squareAffinity, rectangleAffinity);
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
