#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <array>
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

    struct Parameters {
        static constexpr std::array<int, 4> CORNER_MARKER_IDS = {0, 1, 2, 3};
        static constexpr int ARUCO_DICT = cv::aruco::DICT_4X4_50;

        static constexpr int ADAPTIVE_THRESH_WIN_SIZE_MIN = 3;
        static constexpr int ADAPTIVE_THRESH_WIN_SIZE_MAX = 23;
        static constexpr int CORNER_REFINEMENT_WIN_SIZE = 5;
        static constexpr int CORNER_REFINEMENT_MAX_ITERATIONS = 30;
        static constexpr double CORNER_REFINEMENT_MIN_ACCURACY = 0.01;

        static constexpr int MIN_OUTPUT_WIDTH = 200;
        static constexpr int MASK_MARGIN_DIVISOR = 200;
        static constexpr int MIN_MASK_MARGIN = 2;
    };

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
