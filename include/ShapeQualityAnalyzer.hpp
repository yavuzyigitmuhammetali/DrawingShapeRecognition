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
        static constexpr double WEIGHT_FILL_RATIO = 0.2;
        static constexpr double WEIGHT_CIRCULARITY = 0.5;
        static constexpr double WEIGHT_DEVIATION = 0.3;
        static constexpr double WEIGHT_SIDE_VARIANCE = 0.5;
        static constexpr double WEIGHT_ANGLE_VARIANCE = 0.3;
        static constexpr double GRADE_EXCELLENT = 90.0;
        static constexpr double GRADE_VERY_GOOD = 75.0;
        static constexpr double GRADE_GOOD = 60.0;
        static constexpr double GRADE_ACCEPTABLE = 40.0;
        static constexpr double SCORE_ROUNDING_INTERVAL = 10.0;
        static constexpr double SCORE_UNKNOWN_PENALTY = 0.1;
    };

    QualityScore evaluate(const ShapeSegmenter::Candidate& candidate,
                          const ShapeClassifier::Classification& classification) const;
};
