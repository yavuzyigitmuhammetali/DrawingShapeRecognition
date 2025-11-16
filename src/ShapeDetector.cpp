#include "ShapeDetector.h"

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <filesystem>

ShapeDetector::ShapeDetector() {
    cap.open(0);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Failed to open the default camera." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::cout << "Camera opened successfully." << std::endl;
    cv::namedWindow(windowName);
    cv::namedWindow(warpedWindowName);
}

ShapeDetector::~ShapeDetector() {
    if (videoWriter.isOpened()) {
        double recordDuration = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - recordingStartTime).count();
        stopRecording(recordDuration < minRecordDuration);
    }
    cap.release();
    cv::destroyAllWindows();
}

void ShapeDetector::run() {
    cv::Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            break;
        }

        cv::Mat processedFrame = processFrame(frame);
        cv::imshow(windowName, processedFrame);

        const char key = static_cast<char>(cv::waitKey(1));
        if (key == 27) {
            break;
        }
    }
}

cv::Mat ShapeDetector::processFrame(const cv::Mat &frame) {
    cv::Mat outputFrame = frame.clone();

    // Detect markers and get inner corners
    std::vector<cv::Point2f> corners = arucoDetector.detectAndGetCorners(frame);
    arucoDetector.drawMarkers(outputFrame);

    std::vector<DetectedShape> allShapes;
    bool hasValidWarped = false;
    cv::Mat warped;

    if (!corners.empty()) {
        warped = arucoPerspectiveTransformer.warpImage(frame, corners);
        if (!warped.empty()) {
            hasValidWarped = true;
            allShapes = shapeClassifier.findShapes(warped);
            detectionRenderer.drawDetections(warped, allShapes);
            cv::imshow(warpedWindowName, warped);
        }
    }

    // Video recording state machine
    auto now = std::chrono::steady_clock::now();

    if (hasValidWarped) {
        if (recordState == IDLE) {
            startRecording();
            recordState = RECORDING;
        } else if (recordState == BUFFERING) {
            recordState = RECORDING;  // Resume recording
        }
        recordFrame(warped);
        lastDetectionTime = now;
    } else {
        if (recordState == RECORDING) {
            recordState = BUFFERING;  // Start buffering
            lastDetectionTime = now;
        } else if (recordState == BUFFERING) {
            double elapsed = std::chrono::duration<double>(now - lastDetectionTime).count();
            if (elapsed > bufferDuration) {
                double recordDuration = std::chrono::duration<double>(now - recordingStartTime).count();
                stopRecording(recordDuration < minRecordDuration);
                recordState = IDLE;
            }
        }
    }

    resultWriter.saveDetectionsToFile(allShapes);
    return outputFrame;
}

void ShapeDetector::startRecording() {
    // Create filename with timestamp
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << "recording_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".mp4";
    currentVideoPath = oss.str();

    // Initialize VideoWriter (685x1122 is warped size)
    videoWriter.open(currentVideoPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                     fps, cv::Size(685, 1122));

    if (videoWriter.isOpened()) {
        recordingStartTime = std::chrono::steady_clock::now();
        std::cout << "Started recording: " << currentVideoPath << std::endl;
    }
}

void ShapeDetector::stopRecording(bool deleteFile) {
    if (videoWriter.isOpened()) {
        videoWriter.release();

        if (deleteFile) {
            std::filesystem::remove(currentVideoPath);
            std::cout << "Deleted short recording: " << currentVideoPath << std::endl;
        } else {
            std::cout << "Saved recording: " << currentVideoPath << std::endl;
        }
    }
    currentVideoPath.clear();
}

void ShapeDetector::recordFrame(const cv::Mat &warped) {
    if (videoWriter.isOpened() && !warped.empty()) {
        videoWriter.write(warped);
    }
}
