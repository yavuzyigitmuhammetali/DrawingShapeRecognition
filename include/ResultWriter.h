#pragma once

#include "ShapeClassifier.h"

#include <map>
#include <string>
#include <vector>

class ResultWriter {
public:
    explicit ResultWriter(const std::string &outputFileName = "detected_shapes.txt");
    ~ResultWriter() = default;

    void saveDetectionsToFile(const std::vector<DetectedShape> &shapes) const;

private:
    std::string formatShapeLabel(const DetectedShape &shape, int precision = 2) const;

    std::map<std::string, int> countKnownShapes(const std::vector<DetectedShape> &shapes,
                                                 int &unknownCount) const;

    std::string outputFileName;
};
