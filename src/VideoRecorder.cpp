#include "VideoRecorder.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <filesystem>

VideoRecorder::VideoRecorder(double minDuration, double bufferDuration, double fps)
    : minRecordDuration(minDuration), bufferDuration(bufferDuration), fps(fps) {
    // Create output directory if it doesn't exist
    std::filesystem::create_directories(outputDir);
}

VideoRecorder::~VideoRecorder() {
    if (videoWriter.isOpened()) {
        double recordDuration = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - recordingStartTime).count();
        stopRecording(recordDuration < minRecordDuration);
    }
}

void VideoRecorder::update(bool hasValidFrame, const cv::Mat &frame) {
    auto now = std::chrono::steady_clock::now();

    if (hasValidFrame) {
        if (state == IDLE) {
            startRecording();
            state = RECORDING;
        } else if (state == BUFFERING) {
            state = RECORDING;
        }

        if (videoWriter.isOpened() && !frame.empty()) {
            videoWriter.write(frame);
        }
        lastDetectionTime = now;
    } else {
        if (state == RECORDING) {
            state = BUFFERING;
            lastDetectionTime = now;
        } else if (state == BUFFERING) {
            double elapsed = std::chrono::duration<double>(now - lastDetectionTime).count();
            if (elapsed > bufferDuration) {
                double recordDuration = std::chrono::duration<double>(now - recordingStartTime).count();
                stopRecording(recordDuration < minRecordDuration);
                state = IDLE;
            }
        }
    }
}

void VideoRecorder::startRecording() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << outputDir << "/recording_"
        << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".mp4";
    currentVideoPath = oss.str();

    videoWriter.open(currentVideoPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                     fps, cv::Size(685, 1122));

    if (videoWriter.isOpened()) {
        recordingStartTime = std::chrono::steady_clock::now();
        std::cout << "Started recording: " << currentVideoPath << std::endl;
    }
}

void VideoRecorder::stopRecording(bool deleteFile) {
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
