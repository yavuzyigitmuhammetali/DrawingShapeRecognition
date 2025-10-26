#include "ShapeQualityAnalyzer.hpp"

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace {
// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

// ============================================================================
// BASE METRIC CALCULATIONS (used as both positive and negative signals)
// ============================================================================

// Calculate solidity: Contour Area / Convex Hull Area
double calculateSolidityScore(const std::vector<cv::Point>& contour, double contourArea) {
    if (contour.size() < 3 || contourArea <= 0.0) {
        return 0.0;
    }

    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);

    const double hullArea = cv::contourArea(hull);
    if (hullArea <= 0.0) {
        return 0.0;
    }

    return clamp01(contourArea / hullArea);
}

// Calculate line waviness/straightness by measuring RMS distance from contour to ideal polygon edges
// Higher score = straighter lines
double calculateLineWavinessScore(const std::vector<cv::Point>& contour,
                                   const std::vector<cv::Point>& polygon,
                                   double perimeter) {
    if (contour.empty() || polygon.size() < 3 || perimeter <= 0.0) {
        return 0.0;
    }

    double sumSquaredDistances = 0.0;

    for (const auto& pt : contour) {
        const cv::Point2f ptf(static_cast<float>(pt.x), static_cast<float>(pt.y));

        // Find minimum distance from this contour point to any edge of the idealized polygon
        double minDist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < polygon.size(); ++i) {
            const cv::Point2f p1(static_cast<float>(polygon[i].x), static_cast<float>(polygon[i].y));
            const cv::Point2f p2(static_cast<float>(polygon[(i + 1) % polygon.size()].x),
                                 static_cast<float>(polygon[(i + 1) % polygon.size()].y));

            const cv::Point2f edge = p2 - p1;
            const cv::Point2f toPoint = ptf - p1;

            const double edgeLengthSq = edge.dot(edge);
            if (edgeLengthSq < 1e-6) {
                minDist = std::min(minDist, cv::norm(ptf - p1));
                continue;
            }

            const double t = clamp01(toPoint.dot(edge) / edgeLengthSq);
            const cv::Point2f projection = p1 + t * edge;
            const double dist = cv::norm(ptf - projection);

            minDist = std::min(minDist, dist);
        }

        sumSquaredDistances += minDist * minDist;
    }

    const double rmsDistance = std::sqrt(sumSquaredDistances / static_cast<double>(contour.size()));
    const double normalizedDeviation = rmsDistance / (perimeter + 1e-6);
    const double straightnessScore = std::exp(-normalizedDeviation * 50.0);

    return clamp01(straightnessScore);
}

// Calculate circularity: 4πA / P²
// Perfect circle = 1.0
double calculateCircularityScore(double area, double perimeter) {
    if (perimeter <= 0.0 || area <= 0.0) {
        return 0.0;
    }
    const double circularity = (4.0 * CV_PI * area) / (perimeter * perimeter + 1e-6);
    return clamp01(std::pow(circularity, 2.0)); // Quadratic penalty for sensitivity
}

// Calculate ellipse aspect ratio score
// Perfect circle has aspect ratio = 1.0
double calculateEllipseAspectRatioScore(const std::vector<cv::Point>& contour) {
    if (contour.size() < 5) {
        return 0.0;
    }

    try {
        const cv::RotatedRect ellipse = cv::fitEllipse(contour);
        const float majorAxis = std::max(ellipse.size.width, ellipse.size.height);
        const float minorAxis = std::min(ellipse.size.width, ellipse.size.height);

        if (majorAxis > 0.0f) {
            const double aspectRatio = static_cast<double>(minorAxis) / static_cast<double>(majorAxis);
            return clamp01(std::pow(aspectRatio, 3.0)); // Cubic penalty for high sensitivity
        }
    } catch (...) {
        // fitEllipse can throw if points are collinear
    }

    return 0.0;
}

// ============================================================================
// SHAPE-SPECIFIC AFFINITY CALCULATIONS
// ============================================================================

// Helper: Calculate angle variance for a polygon
double calculateAngleVariance(const std::vector<cv::Point>& polygon) {
    if (polygon.size() < 3) {
        return 1.0;
    }

    std::vector<double> angles;
    const size_t count = polygon.size();
    angles.reserve(count);

    for (size_t i = 0; i < count; ++i) {
        const cv::Point2f prev = polygon[(i + count - 1) % count];
        const cv::Point2f current = polygon[i];
        const cv::Point2f next = polygon[(i + 1) % count];

        const cv::Point2f v1 = prev - current;
        const cv::Point2f v2 = next - current;

        double dot = v1.dot(v2);
        double det = v1.x * v2.y - v1.y * v2.x;
        double angle = std::atan2(std::abs(det), dot);
        angles.push_back(angle);
    }

    const double mean = std::accumulate(angles.begin(), angles.end(), 0.0) / static_cast<double>(angles.size());
    if (mean <= 0.0) {
        return 1.0;
    }

    double variance = 0.0;
    for (double angle : angles) {
        const double diff = angle - mean;
        variance += diff * diff;
    }
    variance /= static_cast<double>(angles.size());
    return variance / (mean * mean + 1e-6);
}

// Helper: Calculate side length variance for a polygon
double calculateSideLengthVariance(const std::vector<cv::Point>& polygon) {
    if (polygon.size() < 2) {
        return 1.0;
    }

    std::vector<double> lengths;
    const size_t count = polygon.size();
    lengths.reserve(count);

    for (size_t i = 0; i < count; ++i) {
        const cv::Point2f a = polygon[i];
        const cv::Point2f b = polygon[(i + 1) % count];
        lengths.push_back(cv::norm(a - b));
    }

    const double mean = std::accumulate(lengths.begin(), lengths.end(), 0.0) / static_cast<double>(lengths.size());
    if (mean <= 0.0) {
        return 1.0;
    }

    double variance = 0.0;
    for (double len : lengths) {
        const double diff = len - mean;
        variance += diff * diff;
    }
    variance /= static_cast<double>(lengths.size());
    return variance / (mean * mean + 1e-6);
}

// Triangle affinity: good angle consistency
double calculateTriangleAffinity(const std::vector<cv::Point>& approx) {
    if (approx.size() != 3) {
        return 0.0;
    }
    const double angleVar = calculateAngleVariance(approx);
    return clamp01(1.0 - angleVar * 2.0); // Balanced strictness
}

// Square affinity: robust median-based side scoring
// BALANCED for fairness with rectangle scoring
double calculateSquareAffinity(const std::vector<cv::Point>& approx) {
    if (approx.size() != 4) {
        return 0.0;
    }

    // Calculate all 4 side lengths
    std::vector<double> lengths;
    lengths.reserve(4);
    for (size_t i = 0; i < 4; ++i) {
        const cv::Point2f a = approx[i];
        const cv::Point2f b = approx[(i + 1) % 4];
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

    // Convert to affinity with BALANCED strictness (1.5 instead of 2.0 for fairness)
    return std::max(0.0, 1.0 - normalizedError * 1.5);
}

// Rectangle affinity: opposite-side scoring
// BALANCED for fairness with square scoring
double calculateRectangleAffinity(const std::vector<cv::Point>& approx) {
    if (approx.size() != 4) {
        return 0.0;
    }

    // Calculate all 4 side lengths
    std::vector<double> lengths;
    lengths.reserve(4);
    for (size_t i = 0; i < 4; ++i) {
        const cv::Point2f a = approx[i];
        const cv::Point2f b = approx[(i + 1) % 4];
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

    // Convert to affinity with BALANCED strictness (1.5 multiplier)
    return std::max(0.0, 1.0 - totalError * 1.5);
}

// Hexagon affinity: side and angle consistency
double calculateHexagonAffinity(const std::vector<cv::Point>& approx) {
    if (approx.size() != 6) {
        return 0.0;
    }

    const double sideVar = calculateSideLengthVariance(approx);
    const double angleVar = calculateAngleVariance(approx);

    const double sideScore = clamp01(1.0 - sideVar * 2.0);
    const double angleScore = clamp01(1.0 - angleVar * 2.0);

    // Multiplicative: both must be good
    return sideScore * angleScore;
}
}

ShapeQualityAnalyzer::QualityScore ShapeQualityAnalyzer::evaluate(
    const ShapeSegmenter::Candidate& candidate,
    const ShapeClassifier::Classification& classification) const {

    using Params = ShapeQualityAnalyzer::Parameters;

    QualityScore quality;

    if (candidate.contour.empty()) {
        return quality;
    }

    const double area = cv::contourArea(candidate.contour);
    if (area <= 0.0) {
        return quality;
    }

    const double perimeter = cv::arcLength(candidate.contour, true);
    if (perimeter <= 0.0) {
        return quality;
    }

    // ========================================================================
    // STEP A: CALCULATE ALL BASE METRICS UPFRONT
    // ========================================================================
    // These metrics are used as both positive signals (for their "home" shape)
    // and negative penalties (for rival shapes)

    // Approximated polygon for shape-specific calculations
    std::vector<cv::Point> approx;
    cv::approxPolyDP(candidate.contour, approx, Params::POLYGON_EPSILON * perimeter, true);

    // Base quality metrics (apply to all shapes)
    const double solidityScore = calculateSolidityScore(candidate.contour, area);
    const double lineWavinessScore = calculateLineWavinessScore(candidate.contour, approx, perimeter);

    // Circle-specific metrics
    const double circularityScore = calculateCircularityScore(area, perimeter);
    const double ellipseAspectRatioScore = calculateEllipseAspectRatioScore(candidate.contour);

    // Polygon-specific affinity metrics
    const double triangleAffinity = calculateTriangleAffinity(approx);
    const double squareAffinity = calculateSquareAffinity(approx);
    const double rectangleAffinity = calculateRectangleAffinity(approx);
    const double hexagonAffinity = calculateHexagonAffinity(approx);

    // ========================================================================
    // STEP B: CALCULATE QUALITY SCORE
    // ========================================================================
    // positive_score: Quality metric based on shape-specific affinity and neatness
    // Both displayScore and systemScore are identical (no penalties applied)

    double positive_score = 0.0;

    switch (classification.type) {
    case ShapeClassifier::ShapeType::Circle: {
        // Circle quality: circularity + ellipse aspect ratio
        const double circle_goodness = (circularityScore + ellipseAspectRatioScore) / 2.0;
        positive_score = circle_goodness;
        break;
    }
    case ShapeClassifier::ShapeType::Triangle: {
        // Triangle quality: affinity × waviness × solidity
        positive_score = triangleAffinity * lineWavinessScore * solidityScore;
        break;
    }
    case ShapeClassifier::ShapeType::Square: {
        // Square quality: affinity × waviness × solidity
        positive_score = squareAffinity * lineWavinessScore * solidityScore;
        break;
    }
    case ShapeClassifier::ShapeType::Rectangle: {
        // Rectangle quality: affinity × waviness × solidity
        positive_score = rectangleAffinity * lineWavinessScore * solidityScore;
        break;
    }
    case ShapeClassifier::ShapeType::Hexagon: {
        // Hexagon quality: affinity × waviness × solidity
        positive_score = hexagonAffinity * lineWavinessScore * solidityScore;
        break;
    }
    default:
        positive_score = Params::SCORE_UNKNOWN_PENALTY;
        break;
    }

    // ========================================================================
    // CALCULATE SCORES (NO PENALTIES)
    // ========================================================================

    // systemScore: precise internal score for calculations
    // displayScore: rounded to nearest 10 for user display and logging (91 -> 100, 90 -> 90)
    quality.systemScore = clamp01(positive_score) * 100.0;
    quality.displayScore = std::ceil(quality.systemScore / 10.0) * 10.0;

    // Assign grade based on the user-facing displayScore
    if (quality.displayScore >= Params::GRADE_EXCELLENT) {
        quality.grade = "Excellent";
    } else if (quality.displayScore >= Params::GRADE_VERY_GOOD) {
        quality.grade = "Very Good";
    } else if (quality.displayScore >= Params::GRADE_GOOD) {
        quality.grade = "Good";
    } else if (quality.displayScore >= Params::GRADE_ACCEPTABLE) {
        quality.grade = "Acceptable";
    } else {
        quality.grade = "Needs Improvement";
    }

    return quality;
}
