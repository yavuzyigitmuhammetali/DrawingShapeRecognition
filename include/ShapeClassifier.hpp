#pragma once

#include "ShapeSegmenter.hpp"

#include <string>

class ShapeClassifier {
public:
    enum class ShapeType {
        Unknown = 0,
        Circle,
        Triangle,
        Square,
        Rectangle,
        Hexagon
    };

    struct Classification {
        ShapeType type = ShapeType::Unknown;
        std::string label = "Unknown";
        double confidence = 0.0;
    };

    Classification classify(const ShapeSegmenter::Candidate& candidate) const;
};
