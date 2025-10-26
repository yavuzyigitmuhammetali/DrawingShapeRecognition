#include "OutputManager.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace fs = std::filesystem;

OutputManager::OutputManager()
    : state_(RecordingState::IDLE),
      recordingFps_(DEFAULT_FPS) {
    ensureDirectories();

    const auto timestamp = std::chrono::system_clock::now();
    const std::time_t timeValue = std::chrono::system_clock::to_time_t(timestamp);
    std::tm tmBuffer{};
#if defined(_WIN32)
    localtime_s(&tmBuffer, &timeValue);
#else
    localtime_r(&timeValue, &tmBuffer);
#endif

    std::ostringstream logNameStream;
    logNameStream << "output/logs/detections_"
                  << std::put_time(&tmBuffer, "%Y%m%d_%H%M%S")
                  << ".txt";
    logFilePath_ = logNameStream.str();
}

OutputManager::~OutputManager() {
    forceStopRecording();
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
         << " | quality=" << std::setprecision(1) << quality.displayScore
         << " (sys=" << std::setprecision(1) << quality.systemScore << ")"
         << " (" << quality.grade << ")"
         << " | bbox=[" << candidate.boundingBox.x << "," << candidate.boundingBox.y
         << "," << candidate.boundingBox.width << "," << candidate.boundingBox.height << "]";

    records_.push_back(line.str());
}

void OutputManager::processFrame(const cv::Mat& frame, bool arucoDetected) {
    if (frame.empty()) {
        return;
    }

    // Store frame size for future recordings
    if (recordingFrameSize_.width != frame.cols || recordingFrameSize_.height != frame.rows) {
        recordingFrameSize_ = frame.size();
    }

    switch (state_) {
        case RecordingState::IDLE:
            // Waiting for ArUco detection to start recording
            if (arucoDetected) {
                if (startNewRecording(recordingFrameSize_, recordingFps_)) {
                    state_ = RecordingState::RECORDING;
                    videoWriter_.write(frame);
                }
            }
            break;

        case RecordingState::RECORDING:
            // Active recording
            if (arucoDetected) {
                // ArUco still detected, continue recording
                videoWriter_.write(frame);
            } else {
                // ArUco lost, enter buffering state
                state_ = RecordingState::BUFFERING;
                bufferStartTime_ = std::chrono::steady_clock::now();
                videoWriter_.write(frame);  // Continue writing during buffer period
                std::cout << "ArUco lost. Buffer period started ("
                          << BUFFER_TIMEOUT_SECONDS << "s)..." << std::endl;
            }
            break;

        case RecordingState::BUFFERING:
            // Buffering: ArUco lost but waiting before stopping
            if (arucoDetected) {
                // ArUco detected again, resume recording
                state_ = RecordingState::RECORDING;
                videoWriter_.write(frame);
                std::cout << "ArUco detected again. Continuing recording." << std::endl;
            } else {
                // Check buffer timeout
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                    now - bufferStartTime_).count();

                if (elapsed < BUFFER_TIMEOUT_SECONDS) {
                    // Still within buffer time, continue writing
                    videoWriter_.write(frame);
                } else {
                    // Buffer timeout expired, stop recording
                    std::cout << "Buffer timeout reached. Stopping recording." << std::endl;
                    stopCurrentRecording();
                    state_ = RecordingState::IDLE;
                }
            }
            break;
    }
}

bool OutputManager::startNewRecording(const cv::Size& frameSize, double fps) {
    if (frameSize.width <= 0 || frameSize.height <= 0) {
        std::cerr << "Invalid frame size: " << frameSize.width
                  << "x" << frameSize.height << std::endl;
        return false;
    }

    currentVideoPath_ = generateVideoFilename();

    // H.264 codec (MP4 format)
    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    videoWriter_.open(currentVideoPath_, fourcc, fps, frameSize, true);

    if (!videoWriter_.isOpened()) {
        std::cerr << "Unable to open video writer: " << currentVideoPath_ << std::endl;
        return false;
    }

    // Record start time for duration check
    recordingStartTime_ = std::chrono::steady_clock::now();

    std::cout << "✅ Video recording started: " << currentVideoPath_ << std::endl;
    return true;
}

void OutputManager::stopCurrentRecording() {
    if (!videoWriter_.isOpened()) {
        return;
    }

    videoWriter_.release();

    // Calculate recording duration
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(
        now - recordingStartTime_).count();

    // Check if video is too short
    if (duration < MINIMUM_VIDEO_DURATION_SECONDS) {
        // Delete short video
        try {
            if (fs::exists(currentVideoPath_)) {
                fs::remove(currentVideoPath_);
                std::cout << "⚠️  Short video deleted (" << std::fixed << std::setprecision(1)
                          << duration << "s < " << MINIMUM_VIDEO_DURATION_SECONDS
                          << "s): " << currentVideoPath_ << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Failed to delete video: " << e.what() << std::endl;
        }
    } else {
        std::cout << "✅ Video recording completed (" << std::fixed << std::setprecision(1)
                  << duration << "s): " << currentVideoPath_ << std::endl;
    }
}

void OutputManager::forceStopRecording() {
    if (state_ != RecordingState::IDLE) {
        stopCurrentRecording();
        state_ = RecordingState::IDLE;
    }
}

bool OutputManager::isRecording() const {
    return state_ == RecordingState::RECORDING || state_ == RecordingState::BUFFERING;
}

std::string OutputManager::generateVideoFilename() const {
    const auto timestamp = std::chrono::system_clock::now();
    const std::time_t timeValue = std::chrono::system_clock::to_time_t(timestamp);
    std::tm tmBuffer{};
#if defined(_WIN32)
    localtime_s(&tmBuffer, &timeValue);
#else
    localtime_r(&timeValue, &tmBuffer);
#endif

    std::ostringstream nameStream;
    nameStream << "output/videos/output_"
               << std::put_time(&tmBuffer, "%Y%m%d_%H%M%S")
               << ".mp4";
    return nameStream.str();
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
