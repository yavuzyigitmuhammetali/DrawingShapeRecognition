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
    std::cout << "Controls:" << std::endl;
    std::cout << "  'f' - Capture reference shapes and enter FILLING mode" << std::endl;
    std::cout << "  'r' - Reset and return to DRAWING mode" << std::endl;
    std::cout << "  ESC - Exit application" << std::endl;

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
        if (key == 27) {  // ESC key
            break;
        } else if (key == 'f' || key == 'F') {
            // This will be handled in processFrame to capture the current shapes
            // Set a flag or process immediately
            std::cout << "Requesting transition to FILLING mode..." << std::endl;
        } else if (key == 'r' || key == 'R') {
            resetToDrawingMode();
            std::cout << "Reset to DRAWING mode." << std::endl;
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
            if (currentMode == DetectionMode::DRAWING) {
                // DRAWING mode: detect and classify shapes
                allShapes = shapeClassifier.findShapes(warped);
                detectionRenderer.drawDetections(warped, allShapes);

                // Check if user wants to capture reference shapes
                const char key = static_cast<char>(cv::waitKey(1) & 0xFF);
                if (key == 'f' || key == 'F') {
                    captureReferenceShapes(allShapes, warped.size());
                }
            } else if (currentMode == DetectionMode::FILLING) {
                // FILLING mode: analyze fill progress
                std::vector<FillingStats> fillingStats = analyzeFillProgress(warped);
                detectionRenderer.drawFillingMode(warped, referenceShapes, fillingStats);
            }

            cv::imshow(warpedWindowName, warped);
        }
    }

    // Update video recording
    videoRecorder.update(!warped.empty(), warped);

    // Log shapes only when recording in DRAWING mode
    if (videoRecorder.isRecording() && currentMode == DetectionMode::DRAWING) {
        for (const auto &shape : allShapes) {
            resultWriter.addShape(shape);
        }
    }

    return outputFrame;
}

void ShapeDetector::captureReferenceShapes(const std::vector<DetectedShape> &shapes,
                                            const cv::Size &imageSize) {
    if (shapes.empty()) {
        std::cerr << "WARNING: No shapes detected. Cannot enter FILLING mode." << std::endl;
        return;
    }

    referenceShapes.clear();

    for (const auto &shape : shapes) {
        if (shape.type == "Unknown") {
            continue;
        }

        ReferenceShape refShape;
        refShape.contour = shape.contour;
        refShape.boundingBox = shape.boundingBox;
        refShape.originalType = shape.type;

        // Create binary mask with filled polygon
        refShape.mask = cv::Mat::zeros(imageSize, CV_8UC1);
        std::vector<std::vector<cv::Point>> contours = {shape.contour};
        cv::drawContours(refShape.mask, contours, 0, cv::Scalar(255), cv::FILLED);

        // Calculate total area
        refShape.totalArea = cv::countNonZero(refShape.mask);

        referenceShapes.push_back(refShape);
    }

    currentMode = DetectionMode::FILLING;
    std::cout << "Captured " << referenceShapes.size()
              << " reference shape(s). Entering FILLING mode." << std::endl;
}

std::vector<FillingStats> ShapeDetector::analyzeFillProgress(const cv::Mat &warped) {
    std::vector<FillingStats> results;

    if (referenceShapes.empty()) {
        return results;
    }

    // Extract current ink from the warped image (RAW - no dilation)
    cv::Mat currentInkMask = extractInkMask(warped);

    // Analyze each reference shape
    for (const auto &refShape : referenceShapes) {
        FillingStats stats;
        stats.boundingBox = refShape.boundingBox;
        stats.shapeType = refShape.originalType;

        // Create "Core Target" by excluding the outline itself
        // The blue reference contour acts as a neutral buffer - not expected to be filled
        cv::Mat coreTargetMask = refShape.mask.clone();
        std::vector<std::vector<cv::Point>> contours = {refShape.contour};
        cv::drawContours(coreTargetMask, contours, 0, cv::Scalar(0), 3);

        // Calculate fill percentage against the CORE target (not full mask)
        // This allows 100% achievement while strictly detecting real gaps
        cv::Mat intersectionMask;
        cv::bitwise_and(coreTargetMask, currentInkMask, intersectionMask);
        int filledPixels = cv::countNonZero(intersectionMask);
        int coreTargetArea = cv::countNonZero(coreTargetMask);

        // Avoid division by zero for very small shapes
        if (coreTargetArea > 0) {
            stats.fillPercentage = (static_cast<double>(filledPixels) / coreTargetArea) * 100.0;
        } else {
            stats.fillPercentage = 0.0;
        }
        stats.filledMask = intersectionMask;

        // Calculate overflow (pixels outside tolerance zone)
        // Use ORIGINAL currentInkMask (not dilated) for strict overflow detection
        cv::Mat toleranceZone;
        int dilationSize = 1;  // ~1 pixel (~0.2mm) - ZERO TOLERANCE mode
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                    cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1),
                                                    cv::Point(dilationSize, dilationSize));
        cv::dilate(refShape.mask, toleranceZone, kernel);

        // Find overflow pixels (in ink but outside tolerance zone)
        cv::Mat toleranceZoneInverted;
        cv::bitwise_not(toleranceZone, toleranceZoneInverted);
        cv::Mat overflowMask;
        cv::bitwise_and(currentInkMask, toleranceZoneInverted, overflowMask);

        // Only consider overflow within extended bounding box to avoid interference
        int margin = 30;
        cv::Rect extendedROI(
            std::max(0, refShape.boundingBox.x - margin),
            std::max(0, refShape.boundingBox.y - margin),
            std::min(warped.cols - std::max(0, refShape.boundingBox.x - margin),
                     refShape.boundingBox.width + 2 * margin),
            std::min(warped.rows - std::max(0, refShape.boundingBox.y - margin),
                     refShape.boundingBox.height + 2 * margin)
        );

        cv::Mat roiMask = cv::Mat::zeros(warped.size(), CV_8UC1);
        roiMask(extendedROI) = 255;
        cv::bitwise_and(overflowMask, roiMask, overflowMask);

        int overflowPixels = cv::countNonZero(overflowMask);
        stats.overflowScore = (static_cast<double>(overflowPixels) / refShape.totalArea) * 100.0;
        stats.overflowMask = overflowMask;

        results.push_back(stats);
    }

    return results;
}

void ShapeDetector::resetToDrawingMode() {
    currentMode = DetectionMode::DRAWING;
    referenceShapes.clear();
}

cv::Mat ShapeDetector::extractInkMask(const cv::Mat &warped) const {
    // Step 1: Pre-processing - Convert to grayscale
    cv::Mat gray;
    if (warped.channels() == 3) {
        cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = warped.clone();
    }

    // Apply Gaussian blur to reduce noise
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    // Step 2: Mask 1 - Otsu's Binarization for solid fills
    // This handles large solid filled regions correctly
    cv::Mat mask1;
    cv::threshold(gray, mask1, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    // Step 3: Mask 2 - Adaptive Threshold for edges and fine details
    // This preserves outlines and details that Otsu might miss due to shadows
    cv::Mat mask2;
    cv::adaptiveThreshold(gray, mask2, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV, 21, 10);

    // Step 4: Combine both masks using bitwise OR
    // This ensures we capture BOTH solid fills AND fine edges
    cv::Mat combinedMask;
    cv::bitwise_or(mask1, mask2, combinedMask);

    // Return combined mask directly - prioritize raw precision over gap filling
    // This preserves fine details and prevents merging of parallel strokes
    return combinedMask;
}
