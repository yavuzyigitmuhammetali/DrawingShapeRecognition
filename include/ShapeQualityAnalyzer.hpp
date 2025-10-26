#pragma once

#include "ShapeClassifier.hpp"
#include "ShapeSegmenter.hpp"

class ShapeQualityAnalyzer {
public:
    struct QualityScore {
        double score = 0.0;
        std::string grade = "N/A";
    };

    struct Parameters {
        static constexpr double POLYGON_EPSILON = ShapeClassifier::Parameters::POLYGON_EPSILON;

        // Base metric: Solidity (replaces fillRatio)
        // Measures convexity: Contour Area / Convex Hull Area
        static constexpr double WEIGHT_SOLIDITY = 0.15;

        // Circle-specific weights
        static constexpr double WEIGHT_CIRCULARITY = 0.35;    // 4πA/P² metric with quadratic penalty
        static constexpr double WEIGHT_DEVIATION = 0.25;      // Mean radial deviation from ideal circle
        static constexpr double WEIGHT_ELLIPSE_ASPECT = 0.25; // Ellipse aspect ratio (1.0 = perfect circle)

        // Polygon-specific weights
        static constexpr double WEIGHT_SIDE_VARIANCE = 0.25;     // Side length consistency
        static constexpr double WEIGHT_ANGLE_VARIANCE = 0.25;    // Angle consistency
        static constexpr double WEIGHT_STRAIGHTNESS = 0.35;      // Line straightness (RMS deviation)

        // Grading thresholds (out of 100)
        static constexpr double GRADE_EXCELLENT = 85.0;    // Raised from 90
        static constexpr double GRADE_VERY_GOOD = 70.0;    // Raised from 75
        static constexpr double GRADE_GOOD = 55.0;         // Lowered from 60
        static constexpr double GRADE_ACCEPTABLE = 40.0;

        // REMOVED: SCORE_ROUNDING_INTERVAL (now using precise scores)

        static constexpr double SCORE_UNKNOWN_PENALTY = 0.1;
    };

    QualityScore evaluate(const ShapeSegmenter::Candidate& candidate,
                          const ShapeClassifier::Classification& classification) const;
};
