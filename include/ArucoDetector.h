#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>

/**
 * ArUco marker detector for paper tracking
 * Detects 4 corner markers (IDs: 0=TL, 1=TR, 2=BR, 3=BL)
 */
class ArucoDetector {
public:
    ArucoDetector();

    /**
     * Detects ArUco markers in the frame
     * @param frame Input frame
     * @return Map of marker ID to center point (empty if not all 4 markers found)
     */
    std::map<int, cv::Point2f> detectMarkers(const cv::Mat &frame);

    /**
     * Get ordered corners for perspective transformation
     * @return Vector of 4 points in order: TL, TR, BR, BL (empty if detection failed)
     */
    std::vector<cv::Point2f> getOrderedCorners() const;

    /**
     * Check if all 4 required markers were detected
     */
    bool hasAllMarkers() const;

    /**
     * Draw detected markers on frame (for debugging/visualization)
     */
    void drawMarkers(cv::Mat &frame) const;

private:
    cv::aruco::ArucoDetector arucoDetector;

    // Last detection results
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::map<int, cv::Point2f> markerCenters;

    // Required marker IDs
    static constexpr int MARKER_TOP_LEFT = 0;
    static constexpr int MARKER_TOP_RIGHT = 1;
    static constexpr int MARKER_BOTTOM_RIGHT = 2;
    static constexpr int MARKER_BOTTOM_LEFT = 3;

    /**
     * Calculate center point of a marker from its 4 corners
     */
    static cv::Point2f calculateCenter(const std::vector<cv::Point2f> &corners);

    /**
     * Validate that all 4 required markers are present
     */
    bool validateMarkers() const;
};