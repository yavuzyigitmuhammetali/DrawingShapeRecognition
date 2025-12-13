#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>

class ResultWriter;

/**
 * Manages automatic video recording of bird's-eye view frames
 * Records only when valid ArUco markers are detected with smart buffering
 */
class VideoRecorder {
public:
    VideoRecorder(double minDuration = 5.0, double bufferDuration = 3.0, double fps = 30.0);
    ~VideoRecorder();

    /**
     * Update recording state and record frame if needed
     * @param hasValidFrame True if valid warped frame is available
     * @param frame The warped frame to record (if hasValidFrame is true)
     */
    void update(bool hasValidFrame, const cv::Mat &frame);

    void setResultWriter(ResultWriter *writer);
    bool isRecording() const { return state == RECORDING; }

private:
    void startRecording();
    void stopRecording(bool deleteFile);

    enum State { IDLE, RECORDING, BUFFERING };
    State state{IDLE};

    cv::VideoWriter videoWriter;
    std::string currentVideoPath;
    std::chrono::steady_clock::time_point recordingStartTime;
    std::chrono::steady_clock::time_point lastDetectionTime;

    const double minRecordDuration;  // Minimum recording duration (seconds)
    const double bufferDuration;     // Buffer tolerance (seconds)
    const double fps;
    const std::string outputDir{"outputs/videos"};

    ResultWriter *resultWriter{nullptr};
};
