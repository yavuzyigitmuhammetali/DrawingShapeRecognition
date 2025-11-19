#include "ShapeClassifier.h"
#include <iostream>

std::vector<DetectedShape> ShapeClassifier::findShapes(const cv::Mat &warpedImage) {
    if (!isAnalysisMode_) {
        // Stage 1: Scanning Mode - Standard detection
        return scanForShapes(warpedImage);
    } else {
        // Stage 2: Analysis Mode - Compare against all frozen references
        return analyzeColoring(warpedImage);
    }
}

std::vector<DetectedShape> ShapeClassifier::scanForShapes(const cv::Mat &warpedImage) const {
    std::vector<DetectedShape> shapes;
    cv::Mat gray, blurred, binary;

    cv::cvtColor(warpedImage, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 3, 0);

    cv::adaptiveThreshold(blurred, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV, 51, 9);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double warpedArea =
            static_cast<double>(warpedImage.cols) * warpedImage.rows;
    const double minShapeArea = warpedArea * kMinShapeAreaRatio;

    for (const auto &contour: contours) {
        const double area = cv::contourArea(contour);
        if (area < minShapeArea) {
            continue;
        }

        DetectedShape detected;
        detected.contour = contour;
        detected.boundingBox = cv::boundingRect(contour);

        const double perimeter = cv::arcLength(contour, true);
        if (perimeter == 0.0) {
            continue;
        }

        const double epsilon = 0.025 * perimeter;
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, epsilon, true);
        const int cornerCount = static_cast<int>(approx.size());

        const double circularity = (4.0 * CV_PI * area) / (perimeter * perimeter);
        const cv::RotatedRect minRect = cv::minAreaRect(contour);
        const double minRectArea = minRect.size.area();
        const double polygonCompactness = minRectArea > 0.0 ? area / minRectArea : 1.0;

        const double approxPerimeter = cv::arcLength(approx, true);
        const double perimeterRatio =
                approxPerimeter > 0.0 ? approxPerimeter / perimeter : 1.0;

        if (cornerCount == 3) {
            detected.type = "Triangle";
        } else if (cornerCount == 4) {
            if (circularity > kCircularityThreshold) {
                detected.type = "Circle";
            } else {
                const double epsilonPolygon = 0.04 * perimeter;
                std::vector<cv::Point> refinedApprox;
                cv::approxPolyDP(contour, refinedApprox, epsilonPolygon, true);

                if (refinedApprox.size() == 3) {
                    detected.type = "Triangle";
                } else {
                    const float aspect =
                            static_cast<float>(detected.boundingBox.width) /
                            static_cast<float>(detected.boundingBox.height);
                    if (aspect > 0.90F && aspect < 1.10F) {
                        detected.type = "Square";
                    } else {
                        detected.type = "Rectangle";
                    }
                }
            }
        } else if (cornerCount == 6) {
            detected.type = circularity > kCircularityThreshold ? "Circle" : "Hexagon";
        } else if (cornerCount > 6) {
            detected.type = "Circle";
        } else {
            detected.type = circularity > kCircularityThreshold ? "Circle" : "Unknown";
        }

        if (detected.type == "Triangle") {
            detected.smoothness = perimeterRatio;
        } else if (detected.type == "Circle") {
            detected.smoothness = circularity;
        } else if (detected.type == "Square" || detected.type == "Rectangle" ||
                   detected.type == "Hexagon") {
            detected.smoothness = polygonCompactness;
        } else {
            detected.smoothness = 0.0;
        }

        shapes.push_back(detected);
    }

    return shapes;
}

void ShapeClassifier::captureBaseline(const cv::Mat &warpedFrame) {
    if (warpedFrame.empty()) {
        std::cerr << "Cannot capture baseline: empty frame" << std::endl;
        return;
    }

    // Process frame to find ALL shapes using standard detection
    std::vector<DetectedShape> detectedShapes = scanForShapes(warpedFrame);

    if (detectedShapes.empty()) {
        std::cerr << "No shapes detected to capture" << std::endl;
        return;
    }

    // Clear previous frozen shapes
    frozenShapes.clear();

    // Step 1: Detect current ink (the outline) using Otsu thresholding
    // This is the ACTUAL ink on the paper at capture time
    cv::Mat gray, blurred, currentInkMask;
    cv::cvtColor(warpedFrame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 3, 0);
    cv::threshold(blurred, currentInkMask, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    // Create morphological kernels (shared across all shapes)
    cv::Mat dilateKernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * kDilationKernelSize + 1, 2 * kDilationKernelSize + 1)
    );

    cv::Mat erodeKernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * kErosionKernelSize + 1, 2 * kErosionKernelSize + 1)
    );

    // Process EACH detected shape
    for (const auto& detectedShape : detectedShapes) {
        FrozenShape frozen;

        // Store the frozen contour and type
        frozen.contour = detectedShape.contour;
        frozen.type = detectedShape.type;

        // Step 2: Create geometric mask (filled contour)
        frozen.binaryMask = cv::Mat::zeros(warpedFrame.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> contours = {frozen.contour};
        cv::drawContours(frozen.binaryMask, contours, 0, cv::Scalar(255), cv::FILLED);

        // Step 3: Calculate "True Target" - Empty Space Targeting
        // Subtract the current ink (outline) from the geometric shape
        // This gives us ONLY the white paper inside the shape
        cv::Mat whiteSpaceMask;
        cv::subtract(frozen.binaryMask, currentInkMask, whiteSpaceMask);

        // Step 4: Apply minimal erosion for jitter tolerance at ink-paper boundary
        cv::erode(whiteSpaceMask, frozen.innerFillMask, erodeKernel);

        // Calculate the inner area (true white space that needs to be filled)
        frozen.innerArea = cv::countNonZero(frozen.innerFillMask);

        if (frozen.innerArea == 0) {
            std::cerr << "Warning: Shape " << frozen.type
                      << " has no empty space (fully filled or too small). Skipping." << std::endl;
            continue; // Skip this shape
        }

        // Create safe spill mask (dilated from geometric boundary - STRICT: k=2)
        cv::dilate(frozen.binaryMask, frozen.safeSpillMask, dilateKernel);

        // Compute ROI with margin for optimization
        cv::Rect shapeBBox = detectedShape.boundingBox;
        frozen.boundingBox = cv::Rect(
            std::max(0, shapeBBox.x - kROIMargin),
            std::max(0, shapeBBox.y - kROIMargin),
            std::min(warpedFrame.cols - std::max(0, shapeBBox.x - kROIMargin),
                     shapeBBox.width + 2 * kROIMargin),
            std::min(warpedFrame.rows - std::max(0, shapeBBox.y - kROIMargin),
                     shapeBBox.height + 2 * kROIMargin)
        );

        frozenShapes.push_back(frozen);

        std::cout << "Captured: " << frozen.type
                  << " (White space area: " << frozen.innerArea << " pixels)" << std::endl;
    }

    // Activate analysis mode
    isAnalysisMode_ = true;

    std::cout << "Baseline captured: " << frozenShapes.size()
              << " shape(s) frozen for analysis" << std::endl;
}

std::vector<DetectedShape> ShapeClassifier::analyzeColoring(const cv::Mat &warpedFrame) const {
    std::vector<DetectedShape> results;

    // Convert current frame to binary to detect all ink (do this once)
    // Using OTSU thresholding to correctly identify solid ink masses
    // (Adaptive thresholding fails on solid black regions with no local contrast)
    cv::Mat gray, blurred, currentInkMask;
    cv::cvtColor(warpedFrame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 3, 0);
    cv::threshold(blurred, currentInkMask, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    // Process EACH frozen shape
    for (const auto& frozen : frozenShapes) {
        DetectedShape result;

        // Use the frozen type and contour
        result.type = frozen.type;
        result.contour = frozen.contour;
        result.boundingBox = cv::boundingRect(frozen.contour);
        result.smoothness = 0.0; // Not applicable in analysis mode

        // ROI Optimization: Extract the region of interest
        // This prevents ink from neighboring shapes from being counted as spills
        cv::Rect safeROI = frozen.boundingBox;

        // Ensure ROI is within frame bounds
        safeROI.x = std::max(0, safeROI.x);
        safeROI.y = std::max(0, safeROI.y);
        safeROI.width = std::min(warpedFrame.cols - safeROI.x, safeROI.width);
        safeROI.height = std::min(warpedFrame.rows - safeROI.y, safeROI.height);

        // Extract ROIs for this shape
        cv::Mat roiInkMask = currentInkMask(safeROI);
        cv::Mat roiInnerFillMask = frozen.innerFillMask(safeROI);
        cv::Mat roiSafeSpillMask = frozen.safeSpillMask(safeROI);

        // Calculate Filling: How much of the inner area is filled?
        cv::Mat filledPixels;
        cv::bitwise_and(roiInkMask, roiInnerFillMask, filledPixels);
        double filledCount = cv::countNonZero(filledPixels);
        result.fillingRatio = filledCount / frozen.innerArea;

        // Calculate Spill: How much ink is outside the safe zone?
        // Strictly penalize any ink beyond the TIGHT (k=2) dilated boundary
        cv::Mat spillPixels;
        cv::subtract(roiInkMask, roiSafeSpillMask, spillPixels);
        double spillCount = cv::countNonZero(spillPixels);
        result.spillRatio = spillCount / frozen.innerArea;

        // Clamp values to [0, 1] range for filling (spill can exceed 1.0)
        result.fillingRatio = std::min(1.0, std::max(0.0, result.fillingRatio));

        results.push_back(result);
    }

    return results;
}

void ShapeClassifier::reset() {
    // Clear all frozen shapes
    frozenShapes.clear();

    // Deactivate analysis mode
    isAnalysisMode_ = false;

    std::cout << "Baseline reset. Returning to scanning mode." << std::endl;
}
