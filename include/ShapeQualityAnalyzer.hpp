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

        // Grading thresholds (out of 100)
        static constexpr double GRADE_EXCELLENT = 85.0;
        static constexpr double GRADE_VERY_GOOD = 70.0;
        static constexpr double GRADE_GOOD = 55.0;
        static constexpr double GRADE_ACCEPTABLE = 40.0;

        static constexpr double SCORE_UNKNOWN_PENALTY = 0.1;

        // NOTE: Weight parameters removed in favor of principled positive/negative architecture
        // Scoring now uses:
        //   - Positive metrics: shape-specific affinities (circularity for circles, etc.)
        //   - Negative penalties: rival shape affinities (polygon affinity penalizes circles, etc.)
        //   - Formula: final_score = positive_score * (1.0 - negative_penalty)
    };

    QualityScore evaluate(const ShapeSegmenter::Candidate& candidate,
                          const ShapeClassifier::Classification& classification) const;
};
