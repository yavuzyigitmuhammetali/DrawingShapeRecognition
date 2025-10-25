#include "ShapeSegmenter.hpp"

#include <algorithm>

ShapeSegmenter::ShapeSegmenter(const Config& config)
    : config_(config) {}

std::vector<ShapeSegmenter::Candidate> ShapeSegmenter::segment(const cv::Mat& birdseyeFrame,
                                                               const cv::Mat& mask) const {
    if (birdseyeFrame.empty()) {
        return {};
    }

    cv::Mat gray;
    if (birdseyeFrame.channels() == 3) {
        cv::cvtColor(birdseyeFrame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = birdseyeFrame.clone();
    }

    if (config_.blurKernelSize > 1) {
        cv::GaussianBlur(gray, gray, cv::Size(config_.blurKernelSize, config_.blurKernelSize), 0);
    }

    cv::Mat edges;
    cv::Canny(gray, edges, config_.cannyLowThreshold, config_.cannyHighThreshold);

    cv::Mat closed;
    const int morphKernelSize = std::max(1, config_.morphKernelSize);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                               cv::Size(morphKernelSize, morphKernelSize));
    cv::morphologyEx(edges, closed, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1),
                     std::max(0, config_.morphCloseIterations));

    cv::Mat working = closed;
    if (!mask.empty()) {
        cv::Mat clipped;
        cv::bitwise_and(working, mask, clipped);
        working = clipped;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(working, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Candidate> candidates;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < config_.minimumContourArea) {
            continue;
        }

        Candidate candidate;
        candidate.boundingBox = cv::boundingRect(contour);
        candidate.contour = contour;

        candidate.mask = cv::Mat::zeros(birdseyeFrame.size(), CV_8UC1);
        cv::drawContours(candidate.mask, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);

        candidates.push_back(std::move(candidate));
    }

    return candidates;
}
