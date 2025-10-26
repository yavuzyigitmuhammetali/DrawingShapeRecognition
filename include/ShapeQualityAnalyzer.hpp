#pragma once

#include "ShapeClassifier.hpp"
#include "ShapeSegmenter.hpp"

class ShapeQualityAnalyzer {
public:
    struct QualityScore {
        double displayScore = 0.0; // Quality score (0-100) shown to the user
        double systemScore = 0.0;  // Quality score (0-100) for system use (currently identical to displayScore)
        std::string grade = "N/A";
    };

    struct Parameters {
        static constexpr double POLYGON_EPSILON = ShapeClassifier::Parameters::POLYGON_EPSILON;

        // Grading thresholds (out of 100)
        static constexpr double GRADE_EXCELLENT = 85.0;
        static constexpr double GRADE_VERY_GOOD = 70.0;
        static constexpr double GRADE_GOOD = 55.0;
        static constexpr double GRADE_ACCEPTABLE = 40.0;

        static constexpr double SCORE_UNKNOWN_PENALTY = 0.1;

        // Scoring Architecture:
        //   - Scores based purely on positive quality metrics (no rival penalties)
        //   - displayScore = systemScore = positive_score * 100
    };

    QualityScore evaluate(const ShapeSegmenter::Candidate& candidate,
                          const ShapeClassifier::Classification& classification) const;
};
