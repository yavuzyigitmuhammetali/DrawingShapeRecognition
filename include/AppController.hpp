#pragma once

#include "OutputManager.hpp"
#include "PerspectiveTransformer.hpp"
#include "ShapeClassifier.hpp"
#include "ShapeQualityAnalyzer.hpp"
#include "ShapeSegmenter.hpp"
#include "VideoSource.hpp"

#include <string>
#include <vector>

class AppController {
public:
    AppController();
    int run(int argc, char** argv);

private:
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

    std::vector<int> detectAvailableCameras(int maxCameras = 10) const;
    int promptCameraSelection(const std::vector<int>& cameras) const;

    VideoSource videoSource_;
    PerspectiveTransformer perspectiveTransformer_;
    ShapeSegmenter segmenter_;
    ShapeClassifier classifier_;
    ShapeQualityAnalyzer qualityAnalyzer_;
    OutputManager outputManager_;
    bool running_;
};
