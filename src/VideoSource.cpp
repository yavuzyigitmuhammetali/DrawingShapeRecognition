#include "VideoSource.hpp"

#include <iostream>
#include <utility>

namespace {
constexpr int DEFAULT_WIDTH = 1280;
constexpr int DEFAULT_HEIGHT = 720;
}

VideoSource::VideoSource()
    : frameSize_(0, 0),
      isFileSource_(false) {}

VideoSource::~VideoSource() {
    release();
}

bool VideoSource::open(int cameraIndex) {
    cv::VideoCapture capture(cameraIndex);
    if (!capture.isOpened()) {
        std::cerr << "Failed to open camera: " << cameraIndex << std::endl;
        return false;
    }

    capture.set(cv::CAP_PROP_FRAME_WIDTH, DEFAULT_WIDTH);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, DEFAULT_HEIGHT);

    isFileSource_ = false;
    return openCapture(std::move(capture));
}

bool VideoSource::open(const std::string& videoPath) {
    cv::VideoCapture capture(videoPath);
    if (!capture.isOpened()) {
        std::cerr << "Unable to open video file: " << videoPath << std::endl;
        return false;
    }

    isFileSource_ = true;
    return openCapture(std::move(capture));
}

bool VideoSource::openCapture(cv::VideoCapture&& capture) {
    capture_ = std::move(capture);
    if (!capture_.isOpened()) {
        return false;
    }

    frameSize_ = cv::Size(
        static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH)),
        static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT))
    );

    if (frameSize_.width <= 0 || frameSize_.height <= 0) {
        frameSize_ = cv::Size(DEFAULT_WIDTH, DEFAULT_HEIGHT);
    }

    return true;
}

bool VideoSource::isOpened() const {
    return capture_.isOpened();
}

bool VideoSource::readFrame(cv::Mat& frame) {
    if (!capture_.isOpened()) {
        return false;
    }

    capture_ >> frame;
    if (frame.empty()) {
        return false;
    }

    return true;
}

void VideoSource::release() {
    if (capture_.isOpened()) {
        capture_.release();
    }
    frameSize_ = cv::Size(0, 0);
}

cv::Size VideoSource::frameSize() const {
    return frameSize_;
}

bool VideoSource::isFile() const {
    return isFileSource_;
}
