#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class VideoSource {
public:
    VideoSource();
    ~VideoSource();

    bool open(int cameraIndex);
    bool open(const std::string& videoPath);

    bool isOpened() const;
    bool readFrame(cv::Mat& frame);
    void release();

    cv::Size frameSize() const;
    bool isFile() const;

private:
    bool openCapture(cv::VideoCapture&& capture);

    cv::VideoCapture capture_;
    cv::Size frameSize_;
    bool isFileSource_;
};
