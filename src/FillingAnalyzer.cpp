#include "FillingAnalyzer.h"
#include "ShapeDetector.h"

std::vector<FillingStats> FillingAnalyzer::analyze(
    const cv::Mat &warped,
    const std::vector<ReferenceShape> &referenceShapes) {

    std::vector<FillingStats> results;
    if (referenceShapes.empty()) {
        return results;
    }

    cv::Mat currentInkMask = extractInkMask(warped);

    for (const auto &refShape : referenceShapes) {
        results.push_back(calculateFillStats(refShape, currentInkMask, warped.size()));
    }

    return results;
}

FillingStats FillingAnalyzer::calculateFillStats(
    const ReferenceShape &refShape,
    const cv::Mat &currentInkMask,
    const cv::Size &imageSize) const {

    FillingStats stats;
    stats.boundingBox = refShape.boundingBox;
    stats.shapeType = refShape.originalType;

    // Create core target (exclude outline buffer)
    cv::Mat coreTarget = createCoreTarget(refShape);

    // Calculate fill percentage
    cv::bitwise_and(coreTarget, currentInkMask, stats.filledMask);
    stats.fillPercentage = calculateFillPercentage(coreTarget, currentInkMask);

    // Calculate overflow
    stats.overflowMask = calculateOverflow(refShape, currentInkMask, imageSize);
    int overflowPixels = cv::countNonZero(stats.overflowMask);
    stats.overflowScore = (static_cast<double>(overflowPixels) / refShape.totalArea) * 100.0;

    return stats;
}

cv::Mat FillingAnalyzer::createCoreTarget(const ReferenceShape &refShape) const {
    cv::Mat coreTarget = refShape.mask.clone();
    std::vector<std::vector<cv::Point>> contours = {refShape.contour};
    cv::drawContours(coreTarget, contours, 0, cv::Scalar(0), CONTOUR_THICKNESS);
    return coreTarget;
}

double FillingAnalyzer::calculateFillPercentage(
    const cv::Mat &coreTarget,
    const cv::Mat &inkMask) const {

    cv::Mat intersection;
    cv::bitwise_and(coreTarget, inkMask, intersection);

    int filledPixels = cv::countNonZero(intersection);
    int targetArea = cv::countNonZero(coreTarget);

    if (targetArea > 0) {
        return (static_cast<double>(filledPixels) / targetArea) * 100.0;
    }
    return 0.0;
}

cv::Mat FillingAnalyzer::calculateOverflow(
    const ReferenceShape &refShape,
    const cv::Mat &inkMask,
    const cv::Size &imageSize) const {

    // Create tolerance zone
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * OVERFLOW_TOLERANCE + 1, 2 * OVERFLOW_TOLERANCE + 1));

    cv::Mat toleranceZone;
    cv::dilate(refShape.mask, toleranceZone, kernel);

    // Find overflow pixels
    cv::Mat toleranceInverted;
    cv::bitwise_not(toleranceZone, toleranceInverted);

    cv::Mat overflow;
    cv::bitwise_and(inkMask, toleranceInverted, overflow);

    // Apply ROI mask to avoid interference
    cv::Rect roi(
        std::max(0, refShape.boundingBox.x - OVERFLOW_MARGIN),
        std::max(0, refShape.boundingBox.y - OVERFLOW_MARGIN),
        std::min(imageSize.width - std::max(0, refShape.boundingBox.x - OVERFLOW_MARGIN),
                 refShape.boundingBox.width + 2 * OVERFLOW_MARGIN),
        std::min(imageSize.height - std::max(0, refShape.boundingBox.y - OVERFLOW_MARGIN),
                 refShape.boundingBox.height + 2 * OVERFLOW_MARGIN)
    );

    cv::Mat roiMask = cv::Mat::zeros(imageSize, CV_8UC1);
    roiMask(roi) = 255;
    cv::bitwise_and(overflow, roiMask, overflow);

    return overflow;
}

cv::Mat FillingAnalyzer::extractInkMask(const cv::Mat &warped) const {
    // Convert to grayscale
    cv::Mat gray = (warped.channels() == 3)
        ? cv::Mat()
        : warped.clone();

    if (warped.channels() == 3) {
        cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
    }

    // Reduce noise
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    // Dual thresholding for solid fills and edges
    cv::Mat otsuMask, adaptiveMask;
    cv::threshold(gray, otsuMask, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::adaptiveThreshold(gray, adaptiveMask, 255,
                         cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                         cv::THRESH_BINARY_INV, 21, 10);

    // Combine both masks
    cv::Mat combinedMask;
    cv::bitwise_or(otsuMask, adaptiveMask, combinedMask);

    return combinedMask;
}
