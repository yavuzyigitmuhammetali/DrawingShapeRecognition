#include "PerspectiveTransformer.h"

#include <algorithm>

cv::Mat PerspectiveTransformer::warpImage(const cv::Mat &frame,
                                          const std::vector<cv::Point> &points) const {
    if (points.size() != 4) {
        return {};
    }

    const std::vector<cv::Point> orderedPoints = reOrderPoints(points);
    if (orderedPoints.size() != 4) {
        return {};
    }

    const cv::Point2f src[4] = {
        cv::Point2f(static_cast<float>(orderedPoints[0].x),
                    static_cast<float>(orderedPoints[0].y)),
        cv::Point2f(static_cast<float>(orderedPoints[1].x),
                    static_cast<float>(orderedPoints[1].y)),
        cv::Point2f(static_cast<float>(orderedPoints[2].x),
                    static_cast<float>(orderedPoints[2].y)),
        cv::Point2f(static_cast<float>(orderedPoints[3].x),
                    static_cast<float>(orderedPoints[3].y)),
    };

    const cv::Point2f dst[4] = {
        {0.0F, 0.0F},
        {kWarpWidth, 0.0F},
        {0.0F, kWarpHeight},
        {kWarpWidth, kWarpHeight},
    };

    cv::Mat transformMatrix = cv::getPerspectiveTransform(src, dst);

    cv::Mat warpedImage;
    cv::warpPerspective(frame, warpedImage, transformMatrix,
                        cv::Size(static_cast<int>(kWarpWidth), static_cast<int>(kWarpHeight)));
    return warpedImage;
}

std::vector<cv::Point> PerspectiveTransformer::reOrderPoints(
    const std::vector<cv::Point> &points) const {
    if (points.size() != 4) {
        return {};
    }

    std::vector<cv::Point> orderedPoints(4);
    std::vector<int> sumPoints;
    std::vector<int> diffPoints;
    sumPoints.reserve(points.size());
    diffPoints.reserve(points.size());

    for (const auto &pt: points) {
        sumPoints.push_back(pt.x + pt.y);
        diffPoints.push_back(pt.x - pt.y);
    }

    const auto topLeftIdx = static_cast<size_t>(
        std::distance(sumPoints.begin(), std::min_element(sumPoints.begin(), sumPoints.end())));
    const auto bottomRightIdx = static_cast<size_t>(
        std::distance(sumPoints.begin(), std::max_element(sumPoints.begin(), sumPoints.end())));
    const auto topRightIdx = static_cast<size_t>(
        std::distance(diffPoints.begin(), std::max_element(diffPoints.begin(), diffPoints.end())));
    const auto bottomLeftIdx = static_cast<size_t>(
        std::distance(diffPoints.begin(), std::min_element(diffPoints.begin(), diffPoints.end())));

    orderedPoints[0] = points[topLeftIdx];
    orderedPoints[3] = points[bottomRightIdx];
    orderedPoints[1] = points[topRightIdx];
    orderedPoints[2] = points[bottomLeftIdx];

    return orderedPoints;
}
