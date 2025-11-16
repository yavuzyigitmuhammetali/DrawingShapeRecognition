#include "ContourAnalyzer.h"

std::vector<cv::Point> ContourAnalyzer::getLargestContour(const cv::Mat &processedImage,
                                                           cv::Size originalFrameSize) const {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(processedImage, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    const double minPaperArea =
            static_cast<double>(originalFrameSize.width) * originalFrameSize.height *
            kMinPaperAreaRatio;

    double maxArea = 0.0;
    std::vector<cv::Point> largestContour;

    for (const auto &contour: contours) {
        const double area = cv::contourArea(contour);
        if (area <= minPaperArea) {
            continue;
        }

        const double perimeter = cv::arcLength(contour, true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, 0.02 * perimeter, true);

        if (approx.size() == 4 && area > maxArea) {
            maxArea = area;
            largestContour = approx;
        }
    }

    return largestContour;
}
