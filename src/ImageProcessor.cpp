#include "ImageProcessor.h"

#include <algorithm>

cv::Mat ImageProcessor::preProcessImage(const cv::Mat &frame) const {
    cv::Mat gray, blurred, edges, dilated;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 5, 0);

    const double meanIntensity = cv::mean(blurred)[0];
    const int lower = std::max(0, static_cast<int>((1.0 - kSigma) * meanIntensity));
    const int upper = std::min(255, static_cast<int>((1.0 + kSigma) * meanIntensity));

    cv::Canny(blurred, edges, lower, upper);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(edges, dilated, kernel);
    return dilated;
}
