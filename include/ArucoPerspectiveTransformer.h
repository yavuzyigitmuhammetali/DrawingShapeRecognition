#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * Perspective transformer using ArUco marker positions
 * Creates normalized top-down view based on real-world measurements
 */
class ArucoPerspectiveTransformer {
public:
    ArucoPerspectiveTransformer();

    /**
     * Warp image to normalized top-down view
     * @param frame Input frame
     * @param markerCenters Ordered marker centers: TL, TR, BR, BL
     * @return Warped image with correct aspect ratio (empty if failed)
     */
    cv::Mat warpImage(const cv::Mat &frame, const std::vector<cv::Point2f> &markerCenters);

    /**
     * Get the output image dimensions
     */
    cv::Size getOutputSize() const { return outputSize; }

private:
    // Real-world measurements (in cm, from user's calibration)
    static constexpr float PAPER_WIDTH_CM = 13.7f;   // TL-TR, BL-BR distance
    static constexpr float PAPER_HEIGHT_CM = 22.45f; // TL-BL, TR-BR distance

    // Output image resolution (maintains aspect ratio)
    // Scale: 50 pixels per cm
    static constexpr int PIXELS_PER_CM = 50;
    cv::Size outputSize;

    /**
     * Validate marker centers (must have exactly 4 points)
     */
    static bool validateMarkers(const std::vector<cv::Point2f> &markers);

    /**
     * Create destination points for perspective transform
     */
    std::vector<cv::Point2f> createDestinationPoints() const;
};
