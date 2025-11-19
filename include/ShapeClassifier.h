#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct DetectedShape {
    std::string type{"Unknown"};
    double smoothness{0.0};
    cv::Rect boundingBox;
    std::vector<cv::Point> contour;
    double fillingRatio{0.0};  // How much of the inner area is filled
    double spillRatio{0.0};    // How much ink is outside the safe zone
};

struct FrozenShape {
    std::vector<cv::Point> contour;  // The frozen contour points
    std::string type;                // The frozen classification (e.g. "Triangle")
    cv::Mat binaryMask;              // The filled binary mask of the shape (Standard)
    cv::Mat safeSpillMask;           // Dilated mask (Tolerance zone for spills)
    cv::Mat innerFillMask;           // Eroded mask (Target zone for filling)
    double innerArea;                // Pixel count of innerFillMask
    cv::Rect boundingBox;            // Expanded ROI for optimization (with margin)
};

class ShapeClassifier {
public:
    ShapeClassifier() = default;
    ~ShapeClassifier() = default;

    std::vector<DetectedShape> findShapes(const cv::Mat &warpedImage);
    void captureBaseline(const cv::Mat &warpedFrame);
    void reset();
    bool isInAnalysisMode() const { return isAnalysisMode_; }
    const std::vector<FrozenShape>& getFrozenShapes() const { return frozenShapes; }

private:
    static constexpr double kMinShapeAreaRatio = 0.0015;
    static constexpr double kCircularityThreshold = 0.85;
    static constexpr int kDilationKernelSize = 2;  // Strict spill tolerance
    static constexpr int kErosionKernelSize = 3;   // Minimal erosion for precision (covers ~98% of shape)
    static constexpr int kROIMargin = 20;          // Margin for ROI expansion

    bool isAnalysisMode_ = false;
    std::vector<FrozenShape> frozenShapes;

    std::vector<DetectedShape> scanForShapes(const cv::Mat &warpedImage) const;
    std::vector<DetectedShape> analyzeColoring(const cv::Mat &warpedFrame) const;
};
