#include "ShapeDetector.h"

#include <cstdlib>
#include <iostream>

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
    cv::Mat warped;

    if (!corners.empty()) {
        warped = arucoPerspectiveTransformer.warpImage(frame, corners);
        if (!warped.empty()) {
            allShapes = shapeClassifier.findShapes(warped);
            detectionRenderer.drawDetections(warped, allShapes);
            cv::imshow(warpedWindowName, warped);
        }
    }

    // Update video recording
    videoRecorder.update(!warped.empty(), warped);

    resultWriter.saveDetectionsToFile(allShapes);
    return outputFrame;
}
