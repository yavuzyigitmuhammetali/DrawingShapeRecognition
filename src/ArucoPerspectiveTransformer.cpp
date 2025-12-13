#include "ArucoPerspectiveTransformer.h"

cv::Mat ArucoPerspectiveTransformer::warpImage(const cv::Mat &frame,
                                               const std::vector<cv::Point2f> &corners) {
    if (frame.empty() || corners.size() != 4) {
        return {};
    }

    // Output: 13.7cm × 22.45cm @ 50px/cm = 685×1122 pixels
    constexpr int W = 685, H = 1122;
    std::vector<cv::Point2f> dst = {{0, 0}, {W - 1, 0}, {W - 1, H - 1}, {0, H - 1}};

    cv::Mat warped;
    cv::warpPerspective(frame, warped, cv::getPerspectiveTransform(corners, dst),
                        cv::Size(W, H), cv::INTER_LINEAR);
    return warped;
}
