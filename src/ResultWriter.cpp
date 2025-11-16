#include "ResultWriter.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>

ResultWriter::ResultWriter(const std::string &outputFileName)
    : outputFileName(outputFileName) {
}

void ResultWriter::saveDetectionsToFile(const std::vector<DetectedShape> &shapes) const {
    std::ofstream outFile(outputFileName);
    if (!outFile.is_open()) {
        std::cerr << "ERROR: Unable to open output file (" << outputFileName << ")." << std::endl;
        return;
    }

    if (shapes.empty()) {
        outFile << "No shapes detected." << std::endl;
        return;
    }

    outFile << "Detected Shapes:" << std::endl;
    outFile << "-------------------------" << std::endl;

    int count = 1;
    for (const auto &shape: shapes) {
        if (shape.type == "Unknown") {
            continue;
        }

        outFile << "Shape #" << count++ << ":" << std::endl;
        outFile << "  Type          : " << shape.type << std::endl;
        outFile << "  Label         : " << formatShapeLabel(shape, 3) << std::endl;

        outFile << "  Box (x,y,w,h) : [" << shape.boundingBox.x << ", "
                << shape.boundingBox.y << ", " << shape.boundingBox.width << ", "
                << shape.boundingBox.height << "]" << std::endl;
        outFile << "-------------------------" << std::endl;
    }

    int unknownCount = 0;
    const std::map<std::string, int> counts = countKnownShapes(shapes, unknownCount);
    outFile << "Summary:" << std::endl;
    outFile << "  Known shapes : " << std::accumulate(
        counts.begin(), counts.end(), 0,
        [](int sum, const auto &entry) { return sum + entry.second; }) << std::endl;
    outFile << "  Unknown      : " << unknownCount << std::endl;
    if (!counts.empty()) {
        outFile << "  Breakdown    :" << std::endl;
        for (const auto &entry: counts) {
            outFile << "    - " << entry.first << ": " << entry.second << std::endl;
        }
    }
}

std::string ResultWriter::formatShapeLabel(const DetectedShape &shape, int precision) const {
    std::ostringstream stream;
    stream << shape.type;
    if (shape.type != "Unknown") {
        stream << " [" << std::fixed << std::setprecision(precision)
                << shape.smoothness << "]";
    }
    return stream.str();
}

std::map<std::string, int> ResultWriter::countKnownShapes(
    const std::vector<DetectedShape> &shapes, int &unknownCount) const {
    std::map<std::string, int> counts;
    unknownCount = 0;
    for (const auto &shape: shapes) {
        if (shape.type == "Unknown") {
            ++unknownCount;
            continue;
        }
        ++counts[shape.type];
    }
    return counts;
}
