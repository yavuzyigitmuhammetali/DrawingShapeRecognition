#pragma once

#include "ShapeClassifier.hpp"
#include "ShapeQualityAnalyzer.hpp"
#include "ShapeSegmenter.hpp"

#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
#include <vector>

class OutputManager {
public:
    OutputManager();
    ~OutputManager();

    void logDetection(const ShapeSegmenter::Candidate& candidate,
                      const ShapeClassifier::Classification& classification,
                      const ShapeQualityAnalyzer::QualityScore& quality);

    // Video recording with smart buffering
    void processFrame(const cv::Mat& frame, bool arucoDetected);
    bool isRecording() const;
    void forceStopRecording();

    void flush();

private:
    enum class RecordingState {
        IDLE,       // No recording, waiting for ArUco
        RECORDING,  // Active recording, ArUco detected
        BUFFERING   // ArUco lost, waiting buffer timeout
    };

    void ensureDirectories() const;
    bool startNewRecording(const cv::Size& frameSize, double fps = 30.0);
    void stopCurrentRecording();
    std::string generateVideoFilename() const;

    std::vector<std::string> records_;
    std::string logFilePath_;
    std::string currentVideoPath_;
    cv::VideoWriter videoWriter_;

    // Recording state management
    RecordingState state_;
    cv::Size recordingFrameSize_;
    double recordingFps_;
    std::chrono::steady_clock::time_point bufferStartTime_;
    std::chrono::steady_clock::time_point recordingStartTime_;

    // Configuration
    static constexpr double BUFFER_TIMEOUT_SECONDS = 2.0;      // Buffer timeout when ArUco is lost
    static constexpr double MINIMUM_VIDEO_DURATION_SECONDS = 5.0;  // Minimum duration to save video
    static constexpr double DEFAULT_FPS = 30.0;
};
