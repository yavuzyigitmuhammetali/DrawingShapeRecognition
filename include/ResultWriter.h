#pragma once

#include "ShapeClassifier.h"

#include <set>
#include <string>
#include <vector>

class ResultWriter {
public:
    ResultWriter();
    ~ResultWriter() = default;

    void startNewVideo(const std::string &videoPath);
    void addShape(const DetectedShape &shape);
    void finishVideo();

private:
    struct ShapeRecord {
        std::string type;
        double smoothness;

        bool operator<(const ShapeRecord &other) const {
            if (type != other.type) return type < other.type;
            return std::abs(smoothness - other.smoothness) > 0.01;
        }
    };

    std::set<ShapeRecord> uniqueShapes;
    std::string currentVideoName;
    std::string outputDir{"outputs/logs"};
};
