#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class ShapeSegmenter {
public:
    struct Candidate {
        cv::Rect boundingBox;
        std::vector<cv::Point> contour;
        cv::Mat mask;
    };

    struct Config {
        int blurKernelSize;
        double cannyLowThreshold;
        double cannyHighThreshold;
        int minimumContourArea;
        int morphKernelSize;
        int morphCloseIterations;

        Config()
            : blurKernelSize(5),
              cannyLowThreshold(50.0),
              cannyHighThreshold(150.0),
              minimumContourArea(500),
              morphKernelSize(3),
              morphCloseIterations(2) {}
    };

    explicit ShapeSegmenter(const Config& config = Config());

    std::vector<Candidate> segment(const cv::Mat& birdseyeFrame,
                                   const cv::Mat& mask = cv::Mat()) const;

private:
    Config config_;
};
