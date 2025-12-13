#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

// Forward declarations
struct ReferenceShape;
struct FillingStats;

class FillingAnalyzer {
public:
    FillingAnalyzer() = default;
    ~FillingAnalyzer() = default;

    // Main analysis method
    std::vector<FillingStats> analyze(const cv::Mat &warped,
                                      const std::vector<ReferenceShape> &referenceShapes);

private:
    // Ink extraction
    cv::Mat extractInkMask(const cv::Mat &warped) const;

    // Fill calculation
    FillingStats calculateFillStats(const ReferenceShape &refShape,
                                   const cv::Mat &currentInkMask,
                                   const cv::Size &imageSize) const;

    // Helper methods
    cv::Mat createCoreTarget(const ReferenceShape &refShape) const;
    double calculateFillPercentage(const cv::Mat &coreTarget,
                                   const cv::Mat &inkMask) const;
    cv::Mat calculateOverflow(const ReferenceShape &refShape,
                             const cv::Mat &inkMask,
                             const cv::Size &imageSize) const;

    // Constants
    static constexpr int CONTOUR_THICKNESS = 3;
    static constexpr int OVERFLOW_TOLERANCE = 1;  // pixels
    static constexpr int OVERFLOW_MARGIN = 30;    // pixels
};
