#include "ShapeQualityAnalyzer.hpp"

#include <opencv2/opencv.hpp>
#include <algorithm>
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
}

ShapeQualityAnalyzer::QualityScore ShapeQualityAnalyzer::evaluate(
    const ShapeSegmenter::Candidate& candidate,
    const ShapeClassifier::Classification& classification) const {

    QualityScore quality;

    if (candidate.contour.empty()) {
        return quality;
    }

    const double area = cv::contourArea(candidate.contour);
    if (area <= 0.0) {
        return quality;
    }

    const double perimeter = cv::arcLength(candidate.contour, true);
    const double boundingArea = static_cast<double>(candidate.boundingBox.area());
    const double fillRatio = boundingArea > 0.0 ? clamp01(area / boundingArea) : 0.0;

    double score = 0.2 * fillRatio;

    switch (classification.type) {
    case ShapeClassifier::ShapeType::Circle: {
        cv::Point2f center;
        float radius = 0.0f;
        cv::minEnclosingCircle(candidate.contour, center, radius);
        if (radius > 0.0f) {
            double sumDeviation = 0.0;
            for (const auto& pt : candidate.contour) {
                const double dist = cv::norm(cv::Point2f(static_cast<float>(pt.x), static_cast<float>(pt.y)) - center);
                sumDeviation += std::abs(dist - radius);
            }
            const double meanDeviation = sumDeviation / static_cast<double>(candidate.contour.size());
            const double deviationRatio = meanDeviation / (radius + 1e-6);

            const double circularity = (4.0 * CV_PI * area) / (perimeter * perimeter + 1e-6);
            const double circularityScore = clamp01(circularity);
            const double deviationScore = clamp01(1.0 - deviationRatio);

            score += 0.5 * circularityScore + 0.3 * deviationScore;
        }
        break;
    }
    case ShapeClassifier::ShapeType::Triangle:
    case ShapeClassifier::ShapeType::Square:
    case ShapeClassifier::ShapeType::Rectangle:
    case ShapeClassifier::ShapeType::Hexagon: {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(candidate.contour, approx, 0.02 * perimeter, true);
        if (approx.size() >= 3) {
            const double sideScore = clamp01(1.0 - sideLengthVariance(approx));
            const double angleScore = clamp01(1.0 - angleVariance(approx));
            score += 0.5 * sideScore + 0.3 * angleScore;
        }
        break;
    }
    default:
        score += 0.1;
        break;
    }

    score = clamp01(score);
    quality.score = score * 100.0;

    if (quality.score >= 90.0) {
        quality.grade = "Excellent";
    } else if (quality.score >= 75.0) {
        quality.grade = "Very Good";
    } else if (quality.score >= 60.0) {
        quality.grade = "Good";
    } else if (quality.score >= 40.0) {
        quality.grade = "Acceptable";
    } else {
        quality.grade = "Needs Improvement";
    }

    return quality;
}
