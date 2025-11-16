#include "ArucoDetector.h"
#include <iostream>

ArucoDetector::ArucoDetector() {
    // Use 4x4 dictionary with 50 markers (same as generate_markers.py)
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    // Create detector parameters with optimized settings
    cv::aruco::DetectorParameters detectorParams;

    // Optimize for better detection accuracy
    detectorParams.adaptiveThreshWinSizeMin = 3;
    detectorParams.adaptiveThreshWinSizeMax = 23;
    detectorParams.adaptiveThreshWinSizeStep = 10;
    detectorParams.minMarkerPerimeterRate = 0.03;  // Minimum marker size (3% of image)
    detectorParams.maxMarkerPerimeterRate = 4.0;   // Maximum marker size
    detectorParams.polygonalApproxAccuracyRate = 0.03;
    detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;  // Subpixel accuracy

    // Initialize ArucoDetector with dictionary and parameters
    arucoDetector = cv::aruco::ArucoDetector(dictionary, detectorParams);

    std::cout << "ArucoDetector initialized (DICT_4X4_50)" << std::endl;
}

std::map<int, cv::Point2f> ArucoDetector::detectMarkers(const cv::Mat &frame) {
    markerIds.clear();
    markerCorners.clear();
    markerCenters.clear();

    if (frame.empty()) {
        return markerCenters;
    }

    // Detect markers using the new API
    arucoDetector.detectMarkers(frame, markerCorners, markerIds);

    // Calculate centers for all detected markers
    for (size_t i = 0; i < markerIds.size(); i++) {
        int id = markerIds[i];
        cv::Point2f center = calculateCenter(markerCorners[i]);
        markerCenters[id] = center;
    }

    // Validate that we have all 4 required markers
    if (!validateMarkers()) {
        markerCenters.clear();
        return markerCenters;
    }

    return markerCenters;
}

std::vector<cv::Point2f> ArucoDetector::getOrderedCorners() const {
    std::vector<cv::Point2f> corners;

    if (!hasAllMarkers()) {
        return corners;
    }

    // Return in order: TL, TR, BR, BL
    corners.push_back(markerCenters.at(MARKER_TOP_LEFT));
    corners.push_back(markerCenters.at(MARKER_TOP_RIGHT));
    corners.push_back(markerCenters.at(MARKER_BOTTOM_RIGHT));
    corners.push_back(markerCenters.at(MARKER_BOTTOM_LEFT));

    return corners;
}

bool ArucoDetector::hasAllMarkers() const {
    return markerCenters.size() >= 4 &&
           markerCenters.count(MARKER_TOP_LEFT) > 0 &&
           markerCenters.count(MARKER_TOP_RIGHT) > 0 &&
           markerCenters.count(MARKER_BOTTOM_RIGHT) > 0 &&
           markerCenters.count(MARKER_BOTTOM_LEFT) > 0;
}

void ArucoDetector::drawMarkers(cv::Mat &frame) const {
    if (markerIds.empty()) {
        return;
    }

    // Draw detected markers with IDs
    cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

    // Draw centers and labels
    for (const auto &[id, center] : markerCenters) {
        // Draw center point
        cv::circle(frame, center, 5, cv::Scalar(0, 255, 0), -1);

        // Draw ID label
        std::string label;
        switch (id) {
            case MARKER_TOP_LEFT:
                label = "TL(0)";
                break;
            case MARKER_TOP_RIGHT:
                label = "TR(1)";
                break;
            case MARKER_BOTTOM_RIGHT:
                label = "BR(2)";
                break;
            case MARKER_BOTTOM_LEFT:
                label = "BL(3)";
                break;
            default:
                label = "ID:" + std::to_string(id);
        }

        cv::putText(frame, label, cv::Point(center.x + 10, center.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }

    // Draw status message
    std::string status = hasAllMarkers() ? "ALL 4 MARKERS DETECTED" :
                         "MISSING MARKERS (" + std::to_string(markerCenters.size()) + "/4)";
    cv::Scalar statusColor = hasAllMarkers() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::putText(frame, status, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, statusColor, 2);
}

cv::Point2f ArucoDetector::calculateCenter(const std::vector<cv::Point2f> &corners) {
    if (corners.size() != 4) {
        return cv::Point2f(0, 0);
    }

    // Calculate centroid of the 4 corners
    float x = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0f;
    float y = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0f;

    return cv::Point2f(x, y);
}

bool ArucoDetector::validateMarkers() const {
    // Check if we have all 4 required markers
    bool hasAll = hasAllMarkers();

    if (!hasAll && !markerIds.empty()) {
        std::cout << "Warning: Detected " << markerIds.size() << " markers, need 4. IDs: ";
        for (int id : markerIds) {
            std::cout << id << " ";
        }
        std::cout << std::endl;
    }

    return hasAll;
}
