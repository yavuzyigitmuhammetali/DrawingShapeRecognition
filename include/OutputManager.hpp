#pragma once

#include "ShapeClassifier.hpp"
#include "ShapeQualityAnalyzer.hpp"
#include "ShapeSegmenter.hpp"

#include <string>
#include <vector>

class OutputManager {
public:
    OutputManager();

    void logDetection(const ShapeSegmenter::Candidate& candidate,
                      const ShapeClassifier::Classification& classification,
                      const ShapeQualityAnalyzer::QualityScore& quality);

    void flush();

private:
    void ensureDirectories() const;

    std::vector<std::string> records_;
    std::string logFilePath_;
};
