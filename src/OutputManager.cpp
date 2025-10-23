#include "OutputManager.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>

namespace fs = std::filesystem;

OutputManager::OutputManager() {
    ensureDirectories();

    const auto timestamp = std::chrono::system_clock::now();
    const std::time_t timeValue = std::chrono::system_clock::to_time_t(timestamp);
    std::tm tmBuffer{};
#if defined(_WIN32)
    localtime_s(&tmBuffer, &timeValue);
#else
    localtime_r(&timeValue, &tmBuffer);
#endif

    std::ostringstream nameStream;
    nameStream << "output/logs/detections_"
               << std::put_time(&tmBuffer, "%Y%m%d_%H%M%S")
               << ".txt";

    logFilePath_ = nameStream.str();
}

void OutputManager::ensureDirectories() const {
    fs::create_directories("output/logs");
    fs::create_directories("output/videos");
}

void OutputManager::logDetection(const ShapeSegmenter::Candidate& candidate,
                                 const ShapeClassifier::Classification& classification,
                                 const ShapeQualityAnalyzer::QualityScore& quality) {
    std::ostringstream line;
    line << classification.label
         << " | confidence=" << std::fixed << std::setprecision(2) << classification.confidence
         << " | quality=" << std::setprecision(1) << quality.score
         << " (" << quality.grade << ")"
         << " | bbox=[" << candidate.boundingBox.x << "," << candidate.boundingBox.y
         << "," << candidate.boundingBox.width << "," << candidate.boundingBox.height << "]";

    records_.push_back(line.str());
}

void OutputManager::flush() {
    if (records_.empty()) {
        return;
    }

    std::ofstream file(logFilePath_, std::ios::out | std::ios::app);
    if (!file.is_open()) {
        return;
    }

    for (const auto& line : records_) {
        file << line << '\n';
    }

    records_.clear();
}
