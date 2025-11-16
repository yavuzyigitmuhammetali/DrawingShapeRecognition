#include "ArucoDetector.h"

ArucoDetector::ArucoDetector() {
    cv::aruco::DetectorParameters params;
    params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    arucoDetector = cv::aruco::ArucoDetector(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), params);
}

std::vector<cv::Point2f> ArucoDetector::detectAndGetCorners(const cv::Mat &frame) {
    markerIds.clear();
    markerCorners.clear();

    if (!frame.empty()) {
        arucoDetector.detectMarkers(frame, markerCorners, markerIds);
    }

    if (!hasAllMarkers()) {
        return {};
    }

    // Map: marker ID -> which corner to use (ArUco corners: 0=TL, 1=TR, 2=BR, 3=BL)
    constexpr int innerCornerMap[4] = {2, 3, 0, 1};  // TL→BR, TR→BL, BR→TL, BL→TR

    std::vector<cv::Point2f> corners(4);
    for (size_t i = 0; i < markerIds.size(); i++) {
        if (markerIds[i] < 4) {
            corners[markerIds[i]] = markerCorners[i][innerCornerMap[markerIds[i]]];
        }
    }

    return corners;
}

bool ArucoDetector::hasAllMarkers() const {
    if (markerIds.size() < 4) return false;
    bool found[4] = {false};
    for (int id : markerIds) {
        if (id >= 0 && id < 4) found[id] = true;
    }
    return found[0] && found[1] && found[2] && found[3];
}

void ArucoDetector::drawMarkers(cv::Mat &frame) const {
    if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
    }

    const char* status = hasAllMarkers() ? "TRACKING" : "SEARCHING...";
    cv::putText(frame, status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                0.8, hasAllMarkers() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 140, 255), 2);
}
