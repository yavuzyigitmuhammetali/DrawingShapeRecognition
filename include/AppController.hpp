#pragma once

#include "OutputManager.hpp"
#include "PerspectiveTransformer.hpp"
#include "ShapeClassifier.hpp"
#include "ShapeQualityAnalyzer.hpp"
#include "ShapeSegmenter.hpp"
#include "VideoSource.hpp"

#include <opencv2/opencv.hpp>
#include <deque>
#include <map>
#include <string>
#include <vector>

class AppController {
public:
    struct Parameters {
        inline static const cv::Scalar COLOR_BACKGROUND_DARK{30, 30, 30};
        inline static const cv::Scalar COLOR_BACKGROUND_VERY_DARK{25, 25, 25};
        inline static const cv::Scalar COLOR_ORANGE{0, 165, 255};
        inline static const cv::Scalar COLOR_GREEN{0, 255, 0};
        inline static const cv::Scalar COLOR_YELLOW{0, 255, 255};
        inline static const cv::Scalar COLOR_BLUE{255, 0, 0};
        inline static const cv::Scalar COLOR_CYAN{255, 255, 0};
        inline static const cv::Scalar COLOR_MAGENTA{255, 0, 255};
        inline static const cv::Scalar COLOR_WHITE{255, 255, 255};

        inline static const cv::Scalar COLOR_CIRCLE = COLOR_GREEN;
        inline static const cv::Scalar COLOR_TRIANGLE = COLOR_YELLOW;
        inline static const cv::Scalar COLOR_SQUARE = COLOR_BLUE;
        inline static const cv::Scalar COLOR_RECTANGLE = COLOR_CYAN;
        inline static const cv::Scalar COLOR_HEXAGON = COLOR_MAGENTA;
        inline static const cv::Scalar COLOR_UNKNOWN = COLOR_ORANGE;

        static constexpr int TEXT_MARGIN_X = 30;
        static constexpr int TEXT_MARGIN_Y_TOP = 80;
        static constexpr int TEXT_MARGIN_Y_CENTER = 240;
        static constexpr int MESSAGE_WINDOW_WIDTH = 640;
        static constexpr int MESSAGE_WINDOW_HEIGHT = 480;

        static constexpr double FONT_SCALE_NORMAL = 0.7;
        static constexpr double FONT_SCALE_LARGE = 0.9;
        static constexpr double FONT_SCALE_SMALL = 0.5;
        static constexpr int FONT_THICKNESS = 1;
        static constexpr int FONT_THICKNESS_BOLD = 2;
        static constexpr int TEXT_PADDING = 5;
        static constexpr int TEXT_PADDING_LARGE = 10;
        static constexpr int TEXT_BASELINE_OFFSET = 4;

        static constexpr int BBOX_THICKNESS = 2;

        static constexpr int KEY_ESC = 27;

        static constexpr int MAX_CAMERA_SCAN = 10;

        // Temporal stabilization for reducing classification flicker
        static constexpr int STABILIZATION_WINDOW_SIZE = 30;
        static constexpr double STABILIZATION_TRACKING_MAX_DISTANCE = 50.0;
        static constexpr int STABILIZATION_TRACKING_MAX_UNSEEN_FRAMES = 15;
    };

    AppController();
    int run(int argc, char** argv);

private:
    // Stabilization history entry
    struct ClassificationHistoryEntry {
        ShapeClassifier::ShapeType type;
        double systemScore;
    };

    // Stabilized result
    struct StabilizedResult {
        ShapeClassifier::ShapeType type;
        double averageScore;
        std::string label;
        std::string grade;
    };

    // Object tracker for stabilization
    class StabilizedShapeTracker {
    public:
        int id;
        cv::Point lastCentroid;
        int framesUnseen;
        std::deque<ClassificationHistoryEntry> history;
        ShapeSegmenter::Candidate lastCandidate;

        StabilizedShapeTracker() : id(-1), lastCentroid(0, 0), framesUnseen(0) {}

        void update(const ShapeSegmenter::Candidate& candidate,
                   const ShapeClassifier::Classification& classification,
                   const ShapeQualityAnalyzer::QualityScore& quality);

        StabilizedResult getStabilizedResult(const AppController* controller) const;
    };

    bool initialize(int argc, char** argv);
    void processStream();
    void processFrame(const cv::Mat& frame,
                      cv::Mat& cameraView,
                      cv::Mat& birdseyeView,
                      bool& hasBirdseye);
    void annotateDetections(cv::Mat& frame,
                            const std::vector<ShapeSegmenter::Candidate>& candidates,
                            const std::vector<ShapeClassifier::Classification>& classifications,
                            const std::vector<ShapeQualityAnalyzer::QualityScore>& qualities) const;

    std::vector<int> detectAvailableCameras(int maxCameras = Parameters::MAX_CAMERA_SCAN) const;
    int promptCameraSelection(const std::vector<int>& cameras) const;

    // Stabilization helper method
    std::string getGradeFromScore(double score) const;

    VideoSource videoSource_;
    PerspectiveTransformer perspectiveTransformer_;
    ShapeSegmenter segmenter_;
    ShapeClassifier classifier_;
    ShapeQualityAnalyzer qualityAnalyzer_;
    OutputManager outputManager_;
    bool running_;

    // Object tracking for stabilization
    std::vector<StabilizedShapeTracker> trackers_;
    int nextTrackerId_;
};
