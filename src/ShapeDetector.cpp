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

    // Detect ArUco markers
    std::map<int, cv::Point2f> markerCenters = arucoDetector.detectMarkers(frame);

    std::vector<DetectedShape> allShapes;
    if (arucoDetector.hasAllMarkers()) {
        // Draw markers on output frame for visualization
        arucoDetector.drawMarkers(outputFrame);

        // Get ordered corners (TL, TR, BR, BL)
        std::vector<cv::Point2f> orderedCorners = arucoDetector.getOrderedCorners();

        // Apply perspective transformation
        cv::Mat warped = arucoPerspectiveTransformer.warpImage(frame, orderedCorners);

        if (!warped.empty()) {
            // Detect shapes in warped (top-down) view
            allShapes = shapeClassifier.findShapes(warped);
            detectionRenderer.drawDetections(warped, allShapes);
            cv::imshow(warpedWindowName, warped);
        }
    } else {
        // Show error message when markers are not detected
        std::string errorMsg = "Place paper with ArUco markers in view";
        cv::putText(outputFrame, errorMsg, cv::Point(10, frame.rows - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }

    resultWriter.saveDetectionsToFile(allShapes);
    return outputFrame;
}

