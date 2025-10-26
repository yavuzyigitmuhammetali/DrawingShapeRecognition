#include "ShapeQualityAnalyzer.hpp"

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace {
double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

double sideLengthVariance(const std::vector<cv::Point>& polygon) {
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

double angleVariance(const std::vector<cv::Point>& polygon) {
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

// Calculate solidity: Contour Area / Convex Hull Area
// A higher solidity (closer to 1.0) indicates a more convex, solid shape
double calculateSolidity(const std::vector<cv::Point>& contour, double contourArea) {
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

// Calculate line straightness by measuring RMS distance from contour points to idealized polygon edges
// Returns a normalized score (0.0 = very wavy, 1.0 = perfectly straight)
double calculateLineStraightness(const std::vector<cv::Point>& contour, const std::vector<cv::Point>& polygon, double perimeter) {
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

            // Calculate perpendicular distance from point to line segment
            const cv::Point2f edge = p2 - p1;
            const cv::Point2f toPoint = ptf - p1;

            const double edgeLengthSq = edge.dot(edge);
            if (edgeLengthSq < 1e-6) {
                // Degenerate edge, use point-to-point distance
                minDist = std::min(minDist, cv::norm(ptf - p1));
                continue;
            }

            // Project point onto line (parameter t in [0, 1] for line segment)
            const double t = clamp01(toPoint.dot(edge) / edgeLengthSq);
            const cv::Point2f projection = p1 + t * edge;
            const double dist = cv::norm(ptf - projection);

            minDist = std::min(minDist, dist);
        }

        sumSquaredDistances += minDist * minDist;
    }

    // Calculate RMS distance
    const double rmsDistance = std::sqrt(sumSquaredDistances / static_cast<double>(contour.size()));

    // Normalize by perimeter to make it scale-invariant
    // A typical "good" drawing might have RMS distance around 1-2% of perimeter
    // We'll use an exponential decay to penalize deviation
    const double normalizedDeviation = rmsDistance / (perimeter + 1e-6);
    const double straightnessScore = std::exp(-normalizedDeviation * 50.0); // Aggressive penalty

    return clamp01(straightnessScore);
}

// Calculate distance from point to line segment
double pointToSegmentDistance(const cv::Point2f& pt, const cv::Point2f& segStart, const cv::Point2f& segEnd) {
    const cv::Point2f edge = segEnd - segStart;
    const cv::Point2f toPoint = pt - segStart;

    const double edgeLengthSq = edge.dot(edge);
    if (edgeLengthSq < 1e-6) {
        return cv::norm(pt - segStart);
    }

    const double t = clamp01(toPoint.dot(edge) / edgeLengthSq);
    const cv::Point2f projection = segStart + t * edge;
    return cv::norm(pt - projection);
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

    // Calculate solidity instead of fillRatio for rotation-invariant "neatness" metric
    const double solidity = calculateSolidity(candidate.contour, area);
    double score = Params::WEIGHT_SOLIDITY * solidity;

    switch (classification.type) {
    case ShapeClassifier::ShapeType::Circle: {
        cv::Point2f center;
        float radius = 0.0f;
        cv::minEnclosingCircle(candidate.contour, center, radius);
        if (radius > 0.0f) {
            // 1. Circularity metric (4πA / P²) with non-linear penalty
            const double circularity = (4.0 * CV_PI * area) / (perimeter * perimeter + 1e-6);
            const double circularityScore = clamp01(std::pow(circularity, 2.0)); // Quadratic penalty

            // 2. Mean deviation from ideal circle radius
            double sumDeviation = 0.0;
            for (const auto& pt : candidate.contour) {
                const double dist = cv::norm(cv::Point2f(static_cast<float>(pt.x), static_cast<float>(pt.y)) - center);
                sumDeviation += std::abs(dist - radius);
            }
            const double meanDeviation = sumDeviation / static_cast<double>(candidate.contour.size());
            const double deviationRatio = meanDeviation / (radius + 1e-6);
            const double deviationScore = clamp01(1.0 - deviationRatio * 2.0); // More aggressive penalty

            // 3. Ellipse aspect ratio (new metric)
            double ellipseScore = 0.0;
            if (candidate.contour.size() >= 5) { // fitEllipse requires at least 5 points
                try {
                    const cv::RotatedRect ellipse = cv::fitEllipse(candidate.contour);
                    const float majorAxis = std::max(ellipse.size.width, ellipse.size.height);
                    const float minorAxis = std::min(ellipse.size.width, ellipse.size.height);

                    if (majorAxis > 0.0f) {
                        const double aspectRatio = static_cast<double>(minorAxis) / static_cast<double>(majorAxis);
                        // Perfect circle has aspect ratio of 1.0, apply cubic penalty for deviation
                        ellipseScore = clamp01(std::pow(aspectRatio, 3.0));
                    }
                } catch (...) {
                    // fitEllipse can throw if points are collinear, use default score
                    ellipseScore = 0.0;
                }
            }

            // Weighted combination of circle metrics
            score += Params::WEIGHT_CIRCULARITY * circularityScore +
                     Params::WEIGHT_DEVIATION * deviationScore +
                     Params::WEIGHT_ELLIPSE_ASPECT * ellipseScore;
        }
        break;
    }
    case ShapeClassifier::ShapeType::Triangle:
    case ShapeClassifier::ShapeType::Square:
    case ShapeClassifier::ShapeType::Rectangle:
    case ShapeClassifier::ShapeType::Hexagon: {
        // Use approxPolyDP to find the idealized polygon vertices
        std::vector<cv::Point> approx;
        cv::approxPolyDP(candidate.contour, approx, Params::POLYGON_EPSILON * perimeter, true);
        if (approx.size() >= 3) {
            // 1. Geometric accuracy: measure side length and angle consistency of idealized polygon
            const double sideScore = clamp01(1.0 - sideLengthVariance(approx) * 2.0); // More aggressive
            const double angleScore = clamp01(1.0 - angleVariance(approx) * 2.0);     // More aggressive

            // 2. Line straightness: measure how well the original contour follows the idealized polygon
            const double straightnessScore = calculateLineStraightness(candidate.contour, approx, perimeter);

            // Weighted combination: geometry, straightness, and solidity
            score += Params::WEIGHT_SIDE_VARIANCE * sideScore +
                     Params::WEIGHT_ANGLE_VARIANCE * angleScore +
                     Params::WEIGHT_STRAIGHTNESS * straightnessScore;
        }
        break;
    }
    default:
        score += Params::SCORE_UNKNOWN_PENALTY;
        break;
    }

    score = clamp01(score);
    quality.score = score * 100.0;

    // REMOVED: Score rounding for precise scoring
    // quality.score = std::ceil(quality.score / Params::SCORE_ROUNDING_INTERVAL) * Params::SCORE_ROUNDING_INTERVAL;

    if (quality.score >= Params::GRADE_EXCELLENT) {
        quality.grade = "Excellent";
    } else if (quality.score >= Params::GRADE_VERY_GOOD) {
        quality.grade = "Very Good";
    } else if (quality.score >= Params::GRADE_GOOD) {
        quality.grade = "Good";
    } else if (quality.score >= Params::GRADE_ACCEPTABLE) {
        quality.grade = "Acceptable";
    } else {
        quality.grade = "Needs Improvement";
    }

    return quality;
}
