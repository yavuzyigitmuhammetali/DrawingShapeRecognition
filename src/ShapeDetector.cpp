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
        if (frame.empty()) {
            break;
        }

        cv::Mat processedFrame = processFrame(frame);
        cv::imshow(windowName, processedFrame);

        const char key = static_cast<char>(cv::waitKey(1));
        if (key == 27) {
            break;
        } else if (key == 'c' || key == 'C') {
            // Capture baseline - enter Analysis Mode
            if (!frame.empty()) {
                std::vector<cv::Point2f> corners = arucoDetector.detectAndGetCorners(frame);
                if (!corners.empty()) {
                    cv::Mat warped = arucoPerspectiveTransformer.warpImage(frame, corners);
                    if (!warped.empty()) {
                        shapeClassifier.captureBaseline(warped);
                    } else {
                        std::cout << "Cannot capture: warped frame is empty" << std::endl;
                    }
                } else {
                    std::cout << "Cannot capture: no ArUco markers detected" << std::endl;
                }
            }
        } else if (key == 'r' || key == 'R') {
            // Reset - return to Scanning Mode
            shapeClassifier.reset();
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

            // Draw AR overlay: frozen contours in BLUE when in Analysis Mode
            if (shapeClassifier.isInAnalysisMode()) {
                const std::vector<FrozenShape>& frozenShapes = shapeClassifier.getFrozenShapes();

                // Draw ALL frozen shape contours
                for (const auto& frozen : frozenShapes) {
                    std::vector<std::vector<cv::Point>> contours = {frozen.contour};
                    cv::drawContours(warped, contours, 0, cv::Scalar(255, 0, 0), 3);
                }

                // Add status text
                std::string statusText = "ANALYSIS MODE (" +
                                        std::to_string(frozenShapes.size()) +
                                        " shapes) - Press 'R' to reset";
                cv::putText(warped, statusText,
                           cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                           0.7, cv::Scalar(255, 0, 0), 2);
            } else {
                // Add instruction text in Scanning Mode
                cv::putText(warped, "SCANNING MODE - Press 'C' to capture",
                           cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                           0.7, cv::Scalar(0, 255, 0), 2);
            }

            cv::imshow(warpedWindowName, warped);
        }
    }

    // Update video recording
    videoRecorder.update(!warped.empty(), warped);

    // Log shapes only when recording
    if (videoRecorder.isRecording()) {
        for (const auto &shape : allShapes) {
            resultWriter.addShape(shape);
        }
    }

    return outputFrame;
}
