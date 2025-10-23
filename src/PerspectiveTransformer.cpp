#include "PerspectiveTransformer.hpp"

#include <algorithm>
#include <map>

namespace {
const std::vector<int> CORNER_MARKER_IDS = {0, 1, 2, 3};
const int ARUCO_DICT = cv::aruco::DICT_4X4_50;
}

PerspectiveTransformer::PerspectiveTransformer(const Config& config)
    : config_(config) {
    cv::aruco::Dictionary dictionaryData = cv::aruco::getPredefinedDictionary(ARUCO_DICT);
    dictionary_ = cv::makePtr<cv::aruco::Dictionary>(dictionaryData);
}

PerspectiveTransformer::Result PerspectiveTransformer::process(const cv::Mat& frame) const {
    Result result;
    if (frame.empty()) {
        return result;
    }

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    result.annotatedFrame = frame.clone();

    if (!detectMarkers(frame, markerIds, markerCorners)) {
        cv::putText(result.annotatedFrame,
                    "ArUco markerleri araniyor (IDs: 0,1,2,3)",
                    cv::Point(30, 50),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    cv::Scalar(0, 165, 255),
                    2);
        return result;
    }

    cv::aruco::drawDetectedMarkers(result.annotatedFrame, markerCorners, markerIds);

    std::vector<cv::Point2f> paperCorners;
    if (!extractPaperCorners(markerIds, markerCorners, paperCorners)) {
        cv::putText(result.annotatedFrame,
                    "Tum kose markerleri gorunur degil",
                    cv::Point(30, 50),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    cv::Scalar(0, 0, 255),
                    2);
        return result;
    }

    cv::Size outputSize;
    std::vector<cv::Point2f> dstPoints;
    cv::Mat homography = buildHomography(paperCorners, outputSize, dstPoints);
    if (homography.empty()) {
        return result;
    }

    result.markerCenters = paperCorners;
    drawPaperOverlay(result.annotatedFrame, paperCorners);

    cv::warpPerspective(frame, result.warped, homography,
                        outputSize, cv::INTER_LINEAR);

    generateMasks(frame, homography, dstPoints, outputSize,
                  result.paperOutlineImage, result.originalMask, result.warpedMask);

    result.success = !result.warped.empty();
    return result;
}

bool PerspectiveTransformer::detectMarkers(const cv::Mat& frame,
                                           std::vector<int>& markerIds,
                                           std::vector<std::vector<cv::Point2f>>& markerCorners) const {
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::makePtr<cv::aruco::DetectorParameters>();
    parameters->adaptiveThreshWinSizeMin = 3;
    parameters->adaptiveThreshWinSizeMax = 23;
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    parameters->cornerRefinementWinSize = 5;
    parameters->cornerRefinementMaxIterations = 30;
    parameters->cornerRefinementMinAccuracy = 0.01;

    cv::aruco::detectMarkers(frame, dictionary_, markerCorners, markerIds, parameters);
    return markerIds.size() >= CORNER_MARKER_IDS.size();
}

bool PerspectiveTransformer::extractPaperCorners(
    const std::vector<int>& markerIds,
    const std::vector<std::vector<cv::Point2f>>& markerCorners,
    std::vector<cv::Point2f>& paperCorners) const {

    std::map<int, cv::Point2f> markerCenters;

    for (size_t i = 0; i < markerIds.size(); ++i) {
        const int id = markerIds[i];
        if (std::find(CORNER_MARKER_IDS.begin(), CORNER_MARKER_IDS.end(), id) == CORNER_MARKER_IDS.end()) {
            continue;
        }

        cv::Point2f center(0.0f, 0.0f);
        for (const auto& corner : markerCorners[i]) {
            center += corner;
        }
        center.x /= 4.0f;
        center.y /= 4.0f;

        markerCenters[id] = center;
    }

    if (markerCenters.size() != CORNER_MARKER_IDS.size()) {
        return false;
    }

    paperCorners = {
        markerCenters.at(0), // Top-Left
        markerCenters.at(1), // Top-Right
        markerCenters.at(2), // Bottom-Right
        markerCenters.at(3)  // Bottom-Left
    };

    return true;
}

cv::Mat PerspectiveTransformer::applyBirdsEyeView(const cv::Mat& frame,
                                                  const std::vector<cv::Point2f>& paperCorners) const {
    if (paperCorners.size() != 4) {
        return {};
    }

    cv::Size outputSize;
    std::vector<cv::Point2f> dstPoints;
    cv::Mat homography = buildHomography(paperCorners, outputSize, dstPoints);
    if (homography.empty()) {
        return {};
    }

    cv::Mat warped;
    cv::warpPerspective(frame, warped, homography,
                        outputSize, cv::INTER_LINEAR);

    return warped;
}

void PerspectiveTransformer::drawPaperOverlay(cv::Mat& image,
                                              const std::vector<cv::Point2f>& paperCorners) const {
    if (paperCorners.size() != 4) {
        return;
    }

    std::vector<cv::Point> polygon;
    polygon.reserve(4);
    for (const auto& pt : paperCorners) {
        polygon.emplace_back(static_cast<int>(pt.x), static_cast<int>(pt.y));
    }

    cv::polylines(image, std::vector<std::vector<cv::Point>>{polygon},
                  true, cv::Scalar(0, 255, 0), 2);
}

cv::Mat PerspectiveTransformer::buildHomography(const std::vector<cv::Point2f>& paperCorners,
                                                cv::Size& outputSize,
                                                std::vector<cv::Point2f>& dstPoints) const {
    if (paperCorners.size() != 4) {
        outputSize = cv::Size();
        dstPoints.clear();
        return cv::Mat();
    }

    const float aspectRatio = config_.paperHeightCm / config_.paperWidthCm;
    const int outputWidth = std::max(config_.outputWidthPx, 200);
    const int outputHeight = static_cast<int>(std::round(outputWidth * aspectRatio));

    outputSize = cv::Size(outputWidth, outputHeight);

    const float scaleX = outputWidth / config_.paperWidthCm;
    const float scaleY = outputHeight / config_.paperHeightCm;

    const float markerOffsetX = (config_.paperWidthCm - config_.markerRectWidthCm) / 2.0f;
    const float markerOffsetY = (config_.paperHeightCm - config_.markerRectHeightCm) / 2.0f;
    const float halfMarkerSize = config_.markerSizeCm / 2.0f;

    const float tlx = (markerOffsetX + halfMarkerSize) * scaleX;
    const float tly = (markerOffsetY + halfMarkerSize) * scaleY;

    dstPoints = {
        {tlx, tly},
        {tlx + config_.markerRectWidthCm * scaleX, tly},
        {tlx + config_.markerRectWidthCm * scaleX, tly + config_.markerRectHeightCm * scaleY},
        {tlx, tly + config_.markerRectHeightCm * scaleY}
    };

    return cv::getPerspectiveTransform(paperCorners, dstPoints);
}

void PerspectiveTransformer::generateMasks(const cv::Mat& frame,
                                           const cv::Mat& homography,
                                           const std::vector<cv::Point2f>& dstPoints,
                                           const cv::Size& outputSize,
                                           std::vector<cv::Point2f>& paperOutlineImage,
                                           cv::Mat& originalMask,
                                           cv::Mat& warpedMask) const {
    paperOutlineImage.clear();
    originalMask = cv::Mat::zeros(frame.size(), CV_8UC1);
    warpedMask = cv::Mat::zeros(outputSize, CV_8UC1);

    if (homography.empty() || outputSize.width <= 0 || outputSize.height <= 0 || dstPoints.size() != 4) {
        return;
    }

    cv::Mat inverseHomography = homography.inv();
    cv::perspectiveTransform(dstPoints, paperOutlineImage, inverseHomography);

    std::vector<cv::Point> outlineInt;
    outlineInt.reserve(paperOutlineImage.size());
    for (const auto& pt : paperOutlineImage) {
        outlineInt.emplace_back(cvRound(pt.x), cvRound(pt.y));
    }
    cv::fillConvexPoly(originalMask, outlineInt, cv::Scalar(255));

    std::vector<cv::Point> warpedOutline;
    warpedOutline.reserve(dstPoints.size());
    for (const auto& pt : dstPoints) {
        warpedOutline.emplace_back(cvRound(pt.x), cvRound(pt.y));
    }
    cv::fillConvexPoly(warpedMask, warpedOutline, cv::Scalar(255));

    const int marginPixels = std::max(2, outputSize.width / 200);
    if (marginPixels > 0) {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                   cv::Size(2 * marginPixels + 1, 2 * marginPixels + 1));
        cv::dilate(warpedMask, warpedMask, kernel);
        cv::dilate(originalMask, originalMask, kernel);
    }
}
