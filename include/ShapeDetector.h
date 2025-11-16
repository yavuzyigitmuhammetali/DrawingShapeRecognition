#pragma once

#include "ArucoDetector.h"
#include "ArucoPerspectiveTransformer.h"
#include "DetectionRenderer.h"
#include "ResultWriter.h"
#include "ShapeClassifier.h"

#include <opencv2/opencv.hpp>
#include <string>

class ShapeDetector {
public:
    ShapeDetector();

    ~ShapeDetector();

    void run();

private:
    cv::Mat processFrame(const cv::Mat &frame);

    cv::VideoCapture cap;
    std::string windowName{"Shape Detector - ArUco Tracking"};
    std::string warpedWindowName{"Top-Down View"};

    ArucoDetector arucoDetector;
    ArucoPerspectiveTransformer arucoPerspectiveTransformer;
    ShapeClassifier shapeClassifier;
    DetectionRenderer detectionRenderer;
    ResultWriter resultWriter;
};
