#pragma once

#include "ShapeClassifier.hpp"
#include "ShapeSegmenter.hpp"

class ShapeQualityAnalyzer {
public:
    struct QualityScore {
        double score = 0.0;
        std::string grade = "N/A";
    };

    QualityScore evaluate(const ShapeSegmenter::Candidate& candidate,
                          const ShapeClassifier::Classification& classification) const;
};
