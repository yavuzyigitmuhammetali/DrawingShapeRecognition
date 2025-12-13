#include "ResultWriter.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

ResultWriter::ResultWriter() {
    std::filesystem::create_directories(outputDir);
}

void ResultWriter::startNewVideo(const std::string &videoPath) {
    uniqueShapes.clear();

    // Extract video filename without extension
    std::filesystem::path path(videoPath);
    currentVideoName = path.stem().string();
}

void ResultWriter::addShape(const DetectedShape &shape) {
    if (currentVideoName.empty() || shape.type == "Unknown") {
        return;
    }

    ShapeRecord record{shape.type, shape.smoothness};
    uniqueShapes.insert(record);
}

void ResultWriter::finishVideo() {
    if (currentVideoName.empty() || uniqueShapes.empty()) {
        return;
    }

    std::string logPath = outputDir + "/" + currentVideoName + ".txt";
    std::ofstream outFile(logPath);

    if (!outFile.is_open()) {
        std::cerr << "ERROR: Unable to create log file: " << logPath << std::endl;
        return;
    }

    outFile << "Video: " << currentVideoName << "\n";
    outFile << "Detected Shapes:\n";
    outFile << "----------------\n";

    for (const auto &shape : uniqueShapes) {
        outFile << shape.type << " [" << std::fixed << std::setprecision(2)
                << shape.smoothness << "]\n";
    }

    outFile.close();
    std::cout << "Saved log: " << logPath << " (" << uniqueShapes.size() << " unique shapes)" << std::endl;

    uniqueShapes.clear();
    currentVideoName.clear();
}
