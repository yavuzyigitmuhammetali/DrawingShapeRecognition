#include "ShapeDetector.h"
#include <cstdlib>
#include <iostream>

ShapeDetector::ShapeDetector() {
    cap.open(0);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Failed to open the default camera." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::cout << "Camera opened successfully.\n"
              << "Controls:\n"
              << "  'f' - Capture reference shapes and enter FILLING mode\n"
              << "  'r' - Reset and return to DRAWING mode\n"
              << "  ESC - Exit application" << std::endl;

    cv::namedWindow(windowName);
    cv::namedWindow(warpedWindowName);
    videoRecorder.setResultWriter(&resultWriter);
}

ShapeDetector::~ShapeDetector() {
    cap.release();
    cv::destroyAllWindows();
}

void ShapeDetector::run() {
    cv::Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::Mat processedFrame = processFrame(frame);
        cv::imshow(windowName, processedFrame);

        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27) break;  // ESC

        handleKeyPress(key);
    }
}

void ShapeDetector::handleKeyPress(char key) {
    if (key == 'f' || key == 'F') {
        std::cout << "Requesting transition to FILLING mode..." << std::endl;
    } else if (key == 'r' || key == 'R') {
        resetToDrawingMode();
        std::cout << "Reset to DRAWING mode." << std::endl;
    }
}

cv::Mat ShapeDetector::processFrame(const cv::Mat &frame) {
    cv::Mat outputFrame = frame.clone();

    // Detect ArUco markers
    std::vector<cv::Point2f> corners = arucoDetector.detectAndGetCorners(frame);
    arucoDetector.drawMarkers(outputFrame);

    if (corners.empty()) {
        return outputFrame;
    }

    // Warp image to top-down view
    cv::Mat warped = arucoPerspectiveTransformer.warpImage(frame, corners);
    if (warped.empty()) {
        return outputFrame;
    }

    // Process based on current mode
    std::vector<DetectedShape> allShapes;

    if (currentMode == DetectionMode::DRAWING) {
        allShapes = shapeClassifier.findShapes(warped);
        detectionRenderer.drawDetections(warped, allShapes);

        // Check for mode transition
        char key = static_cast<char>(cv::waitKey(1) & 0xFF);
        if (key == 'f' || key == 'F') {
            captureReferenceShapes(allShapes, warped.size());
        }
    } else {
        // FILLING mode
        auto fillingStats = fillingAnalyzer.analyze(warped, referenceShapes);
        detectionRenderer.drawFillingMode(warped, referenceShapes, fillingStats);
    }

    cv::imshow(warpedWindowName, warped);

    // Video recording
    videoRecorder.update(!warped.empty(), warped);

    if (videoRecorder.isRecording() && currentMode == DetectionMode::DRAWING) {
        for (const auto &shape : allShapes) {
            resultWriter.addShape(shape);
        }
    }

    return outputFrame;
}

void ShapeDetector::captureReferenceShapes(
    const std::vector<DetectedShape> &shapes,
    const cv::Size &imageSize) {

    if (shapes.empty()) {
        std::cerr << "WARNING: No shapes detected. Cannot enter FILLING mode." << std::endl;
        return;
    }

    referenceShapes.clear();

    for (const auto &shape : shapes) {
        if (shape.type == "Unknown") continue;

        ReferenceShape refShape;
        refShape.contour = shape.contour;
        refShape.boundingBox = shape.boundingBox;
        refShape.originalType = shape.type;

        // Create filled mask
        refShape.mask = cv::Mat::zeros(imageSize, CV_8UC1);
        std::vector<std::vector<cv::Point>> contours = {shape.contour};
        cv::drawContours(refShape.mask, contours, 0, cv::Scalar(255), cv::FILLED);
        refShape.totalArea = cv::countNonZero(refShape.mask);

        referenceShapes.push_back(refShape);
    }

    currentMode = DetectionMode::FILLING;
    std::cout << "Captured " << referenceShapes.size()
              << " reference shape(s). Entering FILLING mode." << std::endl;
}

void ShapeDetector::resetToDrawingMode() {
    currentMode = DetectionMode::DRAWING;
    referenceShapes.clear();
}
