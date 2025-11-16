#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

/**
 * ArUco marker detector for paper tracking
 * Detects 4 corner markers (IDs: 0=TL, 1=TR, 2=BR, 3=BL)
 */
class ArucoDetector {
public:
    ArucoDetector();

    /**
     * Detects ArUco markers and returns inner corners for perspective transformation
     * @param frame Input frame
     * @return Vector of 4 inner corner points: TL, TR, BR, BL (empty if not all markers found)
     */
    std::vector<cv::Point2f> detectAndGetCorners(const cv::Mat &frame);

    /**
     * Check if all 4 required markers were detected in last frame
     */
    bool hasAllMarkers() const;

    /**
     * Draw detected markers on frame
     */
    void drawMarkers(cv::Mat &frame) const;

private:
    cv::aruco::ArucoDetector arucoDetector;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
};