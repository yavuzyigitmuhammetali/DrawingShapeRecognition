#include "ArucoPerspectiveTransformer.h"
#include <iostream>

ArucoPerspectiveTransformer::ArucoPerspectiveTransformer() {
    // Calculate output dimensions maintaining real-world aspect ratio
    int width = static_cast<int>(PAPER_WIDTH_CM * PIXELS_PER_CM);   // 13.7cm * 50 = 685px
    int height = static_cast<int>(PAPER_HEIGHT_CM * PIXELS_PER_CM); // 22.45cm * 50 = 1122px

    outputSize = cv::Size(width, height);

    std::cout << "ArucoPerspectiveTransformer initialized" << std::endl;
    std::cout << "  Real-world dimensions: " << PAPER_WIDTH_CM << "cm x " << PAPER_HEIGHT_CM << "cm" << std::endl;
    std::cout << "  Output resolution: " << outputSize.width << "x" << outputSize.height << " pixels" << std::endl;
    std::cout << "  Aspect ratio: " << (PAPER_WIDTH_CM / PAPER_HEIGHT_CM) << std::endl;
}

cv::Mat ArucoPerspectiveTransformer::warpImage(const cv::Mat &frame,
                                               const std::vector<cv::Point2f> &markerCenters) {
    cv::Mat warped;

    // Validate input
    if (frame.empty()) {
        std::cerr << "Error: Empty frame provided to warpImage" << std::endl;
        return warped;
    }

    if (!validateMarkers(markerCenters)) {
        std::cerr << "Error: Invalid marker centers (need exactly 4 points)" << std::endl;
        return warped;
    }

    // Source points: detected marker centers (TL, TR, BR, BL)
    std::vector<cv::Point2f> srcPoints = markerCenters;

    // Destination points: perfect rectangle with correct aspect ratio
    std::vector<cv::Point2f> dstPoints = createDestinationPoints();

    // Calculate perspective transformation matrix
    cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // Apply perspective transformation
    cv::warpPerspective(frame, warped, perspectiveMatrix, outputSize,
                        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

    return warped;
}

bool ArucoPerspectiveTransformer::validateMarkers(const std::vector<cv::Point2f> &markers) {
    if (markers.size() != 4) {
        std::cerr << "Error: Expected 4 markers, got " << markers.size() << std::endl;
        return false;
    }

    // Check that all points are valid (not negative or zero)
    for (const auto &point : markers) {
        if (point.x < 0 || point.y < 0) {
            std::cerr << "Error: Invalid marker coordinate: (" << point.x << ", " << point.y << ")" << std::endl;
            return false;
        }
    }

    return true;
}

std::vector<cv::Point2f> ArucoPerspectiveTransformer::createDestinationPoints() const {
    std::vector<cv::Point2f> dstPoints;

    // Create perfect rectangle with correct aspect ratio
    // Order: TL, TR, BR, BL
    dstPoints.push_back(cv::Point2f(0, 0));                                      // Top-Left
    dstPoints.push_back(cv::Point2f(outputSize.width - 1, 0));                   // Top-Right
    dstPoints.push_back(cv::Point2f(outputSize.width - 1, outputSize.height - 1)); // Bottom-Right
    dstPoints.push_back(cv::Point2f(0, outputSize.height - 1));                  // Bottom-Left

    return dstPoints;
}
