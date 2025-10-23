#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class PerspectiveTransformer {
public:
    struct Config {
        float paperWidthCm;
        float paperHeightCm;
        float markerSizeCm;
        float markerRectWidthCm;
        float markerRectHeightCm;
        int outputWidthPx;

        Config()
            : paperWidthCm(21.0f),
              paperHeightCm(29.7f),
              markerSizeCm(1.8f),
              markerRectWidthCm(13.7f),
              markerRectHeightCm(22.45f),
              outputWidthPx(1000) {}
    };

    struct Result {
        bool success = false;
        cv::Mat warped;
        cv::Mat warpedMask;
        cv::Mat originalMask;
        cv::Mat annotatedFrame;
        std::vector<cv::Point2f> markerCenters;
        std::vector<cv::Point2f> paperOutlineImage;
    };

    explicit PerspectiveTransformer(const Config& config = Config());
    Result process(const cv::Mat& frame) const;

private:
    bool detectMarkers(const cv::Mat& frame,
                       std::vector<int>& markerIds,
                       std::vector<std::vector<cv::Point2f>>& markerCorners) const;

    bool extractPaperCorners(const std::vector<int>& markerIds,
                             const std::vector<std::vector<cv::Point2f>>& markerCorners,
                             std::vector<cv::Point2f>& paperCorners) const;

    cv::Mat applyBirdsEyeView(const cv::Mat& frame,
                              const std::vector<cv::Point2f>& paperCorners) const;

    void drawPaperOverlay(cv::Mat& image,
                          const std::vector<cv::Point2f>& paperCorners) const;
    cv::Mat buildHomography(const std::vector<cv::Point2f>& paperCorners,
                            cv::Size& outputSize,
                            std::vector<cv::Point2f>& dstPoints) const;
    void generateMasks(const cv::Mat& frame,
                       const cv::Mat& homography,
                       const std::vector<cv::Point2f>& dstPoints,
                       const cv::Size& outputSize,
                       std::vector<cv::Point2f>& paperOutlineImage,
                       cv::Mat& originalMask,
                       cv::Mat& warpedMask) const;

    Config config_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
};
