#pragma once

#include "ArucoDetector.h"
#include "ArucoPerspectiveTransformer.h"
#include "DetectionRenderer.h"
#include "ResultWriter.h"
#include "ShapeClassifier.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>

class ShapeDetector {
public:
    ShapeDetector();

    ~ShapeDetector();

    void run();

private:
    cv::Mat processFrame(const cv::Mat &frame);

    // Video recording helpers
    void startRecording();
    void stopRecording(bool deleteFile);
    void recordFrame(const cv::Mat &warped);

    cv::VideoCapture cap;
    std::string windowName{"Shape Detector - ArUco Tracking"};
    std::string warpedWindowName{"Top-Down View"};

    ArucoDetector arucoDetector;
    ArucoPerspectiveTransformer arucoPerspectiveTransformer;
    ShapeClassifier shapeClassifier;
    DetectionRenderer detectionRenderer;
    ResultWriter resultWriter;

    // Video recording state
    enum RecordState { IDLE, RECORDING, BUFFERING };
    RecordState recordState{IDLE};
    cv::VideoWriter videoWriter;
    std::string currentVideoPath;
    std::chrono::steady_clock::time_point recordingStartTime;
    std::chrono::steady_clock::time_point lastDetectionTime;

    const double minRecordDuration = 5.0;  // n seconds
    const double bufferDuration = 3.0;     // m seconds
    const double fps = 30.0;
};
