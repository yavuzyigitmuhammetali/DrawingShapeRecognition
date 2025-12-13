#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * Perspective transformer using ArUco marker positions
 * Warps to 685x1122 pixels (13.7cm x 22.45cm @ 50px/cm)
 */
class ArucoPerspectiveTransformer {
public:
    /**
     * Warp image to normalized top-down view
     * @param frame Input frame
     * @param corners Ordered marker corners: TL, TR, BR, BL
     * @return Warped image (685x1122) or empty if failed
     */
    cv::Mat warpImage(const cv::Mat &frame, const std::vector<cv::Point2f> &corners);
};
