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

    struct Parameters {
        static constexpr double POLYGON_EPSILON = 0.04;                 // Fraction of perimeter for approxPolyDP
        static constexpr double CIRCLE_CIRCULARITY_THRESHOLD = 0.7;     // Minimum circularity to classify as circle
        static constexpr double SQUARE_ASPECT_RATIO_THRESHOLD = 1.15;   // Max aspect ratio to treat as square
        static constexpr double CONFIDENCE_TRIANGLE = 0.90;
        static constexpr double CONFIDENCE_SQUARE = 0.85;
        static constexpr double CONFIDENCE_RECTANGLE = 0.80;
        static constexpr double CONFIDENCE_HEXAGON = 0.75;
    };

    Classification classify(const ShapeSegmenter::Candidate& candidate) const;
};
