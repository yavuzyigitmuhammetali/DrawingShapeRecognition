#pragma once

#include "ContourAnalyzer.h"
#include "DetectionRenderer.h"
#include "ImageProcessor.h"
#include "PerspectiveTransformer.h"
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
    std::string windowName{"Shape Detector - Original"};
    std::string warpedWindowName{"Top-Down View"};

    ImageProcessor imageProcessor;
    ContourAnalyzer contourAnalyzer;
    PerspectiveTransformer perspectiveTransformer;
    ShapeClassifier shapeClassifier;
    DetectionRenderer detectionRenderer;
    ResultWriter resultWriter;
};
