#pragma once

#include "ArucoDetector.h"
#include "ArucoPerspectiveTransformer.h"
#include "DetectionRenderer.h"
#include "ResultWriter.h"
#include "ShapeClassifier.h"
#include "VideoRecorder.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// Detection mode enumeration
enum class DetectionMode {
    DRAWING,  // Default mode: detect and classify shapes
    FILLING   // Filling mode: analyze how well shapes are being filled
};

// Reference shape structure for FILLING mode
struct ReferenceShape {
    std::vector<cv::Point> contour;      // Original shape contour
    cv::Mat mask;                         // Binary mask (filled white polygon on black)
    int totalArea;                        // Total pixel area of the shape
    cv::Rect boundingBox;                 // Bounding box (ROI)
    std::string originalType;             // Original shape type (Triangle, Circle, etc.)

    ReferenceShape() : totalArea(0) {}
};

// Filling statistics for a single shape
struct FillingStats {
    double fillPercentage;                // Percentage of shape filled (0-100)
    double overflowScore;                 // Overflow penalty (0-100+)
    cv::Mat filledMask;                   // Mask of filled pixels
    cv::Mat overflowMask;                 // Mask of overflow pixels
    cv::Rect boundingBox;                 // Bounding box for rendering
    std::string shapeType;                // Original shape type

    FillingStats() : fillPercentage(0.0), overflowScore(0.0) {}
};

class ShapeDetector {
public:
    ShapeDetector();

    ~ShapeDetector();

    void run();

private:
    cv::Mat processFrame(const cv::Mat &frame);

    // Mode management
    void captureReferenceShapes(const std::vector<DetectedShape> &shapes, const cv::Size &imageSize);
    std::vector<FillingStats> analyzeFillProgress(const cv::Mat &warped);
    void resetToDrawingMode();

    // Ink extraction helper
    cv::Mat extractInkMask(const cv::Mat &warped) const;

    cv::VideoCapture cap;
    std::string windowName{"Shape Detector - ArUco Tracking"};
    std::string warpedWindowName{"Top-Down View"};

    ArucoDetector arucoDetector;
    ArucoPerspectiveTransformer arucoPerspectiveTransformer;
    ShapeClassifier shapeClassifier;
    DetectionRenderer detectionRenderer;
    ResultWriter resultWriter;
    VideoRecorder videoRecorder;

    // State machine variables
    DetectionMode currentMode{DetectionMode::DRAWING};
    std::vector<ReferenceShape> referenceShapes;
};
