#include "AppController.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>

using Params = AppController::Parameters;

namespace {

cv::Point computeCentroid(const ShapeSegmenter::Candidate& candidate) {
    const cv::Moments moments = cv::moments(candidate.contour);
    if (moments.m00 != 0.0) {
        return {
            static_cast<int>(moments.m10 / moments.m00),
            static_cast<int>(moments.m01 / moments.m00)
        };
    }

    return {
        candidate.boundingBox.x + candidate.boundingBox.width / 2,
        candidate.boundingBox.y + candidate.boundingBox.height / 2
    };
}

cv::Scalar colorForShape(ShapeClassifier::ShapeType type) {
    switch (type) {
    case ShapeClassifier::ShapeType::Circle:
        return Params::COLOR_CIRCLE;
    case ShapeClassifier::ShapeType::Triangle:
        return Params::COLOR_TRIANGLE;
    case ShapeClassifier::ShapeType::Square:
        return Params::COLOR_SQUARE;
    case ShapeClassifier::ShapeType::Rectangle:
        return Params::COLOR_RECTANGLE;
    case ShapeClassifier::ShapeType::Hexagon:
        return Params::COLOR_HEXAGON;
    default:
        return Params::COLOR_UNKNOWN;
    }
}

std::string buildAnnotationText(const ShapeClassifier::Classification& classification,
                                const ShapeQualityAnalyzer::QualityScore& quality) {
    std::ostringstream label;
    label << classification.label << " | " << std::fixed << std::setprecision(1)
          << quality.displayScore << "% (" << quality.grade << ")";
    return label.str();
}

double euclideanDistance(const cv::Point& a, const cv::Point& b) {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    return std::hypot(dx, dy);
}

} // namespace

AppController::AppController()
    : running_(false), nextTrackerId_(0) {}

int AppController::run(int argc, char** argv) {
    if (!initialize(argc, argv)) {
        std::cerr << "Initialization failed." << std::endl;
        return -1;
    }

    processStream();
    outputManager_.forceStopRecording();
    outputManager_.flush();
    return 0;
}

bool AppController::initialize(int argc, char** argv) {
    std::cout << "Starting Drawing Shape Recognition..." << std::endl;

    bool opened = false;
    bool argumentProvided = argc > 1;

    if (argc > 1) {
        std::string argument = argv[1];
        bool isNumeric = !argument.empty() &&
                         std::all_of(argument.begin(), argument.end(), [](unsigned char ch) { return std::isdigit(ch); });
        if (isNumeric) {
            int cameraIndex = std::stoi(argument);
            opened = videoSource_.open(cameraIndex);
            if (!opened) {
                std::cout << "Failed to open camera. Searching for available cameras.\n";
            }
        } else {
            opened = videoSource_.open(argument);
        }
    } else {
        auto cameras = detectAvailableCameras();
        if (cameras.empty()) {
            std::cerr << "No available camera found." << std::endl;
            return false;
        }
        int selection = promptCameraSelection(cameras);
        if (selection < 0) {
            return false;
        }
        opened = videoSource_.open(selection);
    }

    if (!opened) {
        if (argumentProvided) {
            auto cameras = detectAvailableCameras();
            if (cameras.empty()) {
                std::cerr << "No available camera found." << std::endl;
                return false;
            }
            int selection = promptCameraSelection(cameras);
            if (selection < 0) {
                return false;
            }
            opened = videoSource_.open(selection);
        }
    }

    if (!opened) {
        std::cerr << "Could not open input source." << std::endl;
        return false;
    }

    running_ = true;
    std::cout << "✅ System ready. Waiting for ArUco marker detection..." << std::endl;
    return true;
}

void AppController::processStream() {
    cv::namedWindow("Camera View", cv::WINDOW_NORMAL);
    cv::namedWindow("Bird's Eye View", cv::WINDOW_NORMAL);

    cv::Mat frame;
    while (running_) {
        if (!videoSource_.readFrame(frame)) {
            if (videoSource_.isFile()) {
                break;
            }
            continue;
        }

        cv::Mat cameraView;
        cv::Mat birdseyeView;
        bool hasBirdseye = false;
        processFrame(frame, cameraView, birdseyeView, hasBirdseye);

        if (!cameraView.empty()) {
            cv::imshow("Camera View", cameraView);
        }

        if (hasBirdseye) {
            cv::imshow("Bird's Eye View", birdseyeView);
        } else {
            cv::Mat message(Params::MESSAGE_WINDOW_HEIGHT, Params::MESSAGE_WINDOW_WIDTH, CV_8UC3,
                            Params::COLOR_BACKGROUND_DARK);
            cv::putText(message,
                        "Waiting for markers...",
                        cv::Point(Params::TEXT_MARGIN_X, Params::TEXT_MARGIN_Y_CENTER),
                        cv::FONT_HERSHEY_SIMPLEX,
                        Params::FONT_SCALE_LARGE,
                        Params::COLOR_ORANGE,
                        Params::FONT_THICKNESS_BOLD);
            cv::imshow("Bird's Eye View", message);
        }

        int key = cv::waitKey(1);
        if (key == Params::KEY_ESC) {
            running_ = false;
        }
    }

    cv::destroyWindow("Camera View");
    cv::destroyWindow("Bird's Eye View");
}

void AppController::processFrame(const cv::Mat& frame,
                                 cv::Mat& cameraView,
                                 cv::Mat& birdseyeView,
                                 bool& hasBirdseye) {
    auto perspectiveResult = perspectiveTransformer_.process(frame);
    prepareCameraView(frame, perspectiveResult, cameraView);

    hasBirdseye = perspectiveResult.success;
    if (!hasBirdseye) {
        renderMarkerPrompt(cameraView);
        outputManager_.processFrame(cameraView, false);
        return;
    }

    hasBirdseye = prepareBirdseyeView(perspectiveResult, birdseyeView);
    if (!hasBirdseye) {
        outputManager_.processFrame(cameraView, false);
        return;
    }

    std::vector<ShapeSegmenter::Candidate> candidates;
    std::vector<ShapeClassifier::Classification> classifications;
    std::vector<ShapeQualityAnalyzer::QualityScore> qualities;

    analyzeDetections(perspectiveResult, candidates, classifications, qualities);
    updateTrackers(candidates, classifications, qualities);

    std::vector<ShapeSegmenter::Candidate> stabilizedCandidates;
    std::vector<ShapeClassifier::Classification> stabilizedClassifications;
    std::vector<ShapeQualityAnalyzer::QualityScore> stabilizedQualities;
    collectStabilizedDetections(stabilizedCandidates, stabilizedClassifications, stabilizedQualities);

    annotateDetections(birdseyeView, stabilizedCandidates, stabilizedClassifications, stabilizedQualities);
    outputManager_.processFrame(birdseyeView, true);
}

void AppController::prepareCameraView(const cv::Mat& frame,
                                      const PerspectiveTransformer::Result& perspectiveResult,
                                      cv::Mat& cameraView) const {
    cameraView = perspectiveResult.annotatedFrame.empty()
                     ? frame.clone()
                     : perspectiveResult.annotatedFrame.clone();

    if (cameraView.empty() || perspectiveResult.originalMask.empty()) {
        return;
    }

    if (cameraView.channels() == 1) {
        cv::cvtColor(cameraView, cameraView, cv::COLOR_GRAY2BGR);
    }

    cv::Mat outsideMask;
    cv::bitwise_not(perspectiveResult.originalMask, outsideMask);
    cameraView.setTo(Params::COLOR_BACKGROUND_VERY_DARK, outsideMask);

    if (!perspectiveResult.paperOutlineImage.empty()) {
        std::vector<cv::Point> polygon;
        polygon.reserve(perspectiveResult.paperOutlineImage.size());
        for (const auto& pt : perspectiveResult.paperOutlineImage) {
            polygon.emplace_back(cvRound(pt.x), cvRound(pt.y));
        }
        cv::polylines(cameraView, std::vector<std::vector<cv::Point>>{polygon},
                      true, Params::COLOR_GREEN, Params::BBOX_THICKNESS);
    }
}

bool AppController::prepareBirdseyeView(const PerspectiveTransformer::Result& perspectiveResult,
                                        cv::Mat& birdseyeView) const {
    if (!perspectiveResult.success || perspectiveResult.warped.empty()) {
        birdseyeView.release();
        return false;
    }

    birdseyeView = perspectiveResult.warped.clone();

    if (birdseyeView.channels() == 1) {
        cv::cvtColor(birdseyeView, birdseyeView, cv::COLOR_GRAY2BGR);
    }

    if (!perspectiveResult.warpedMask.empty()) {
        cv::Mat outsideMask;
        cv::bitwise_not(perspectiveResult.warpedMask, outsideMask);
        birdseyeView.setTo(Params::COLOR_BACKGROUND_DARK, outsideMask);
    }

    return true;
}

void AppController::renderMarkerPrompt(cv::Mat& cameraView) const {
    if (cameraView.empty()) {
        return;
    }

    cv::putText(cameraView,
                "Place the markers so they are visible.",
                cv::Point(Params::TEXT_MARGIN_X, Params::TEXT_MARGIN_Y_TOP),
                cv::FONT_HERSHEY_SIMPLEX,
                Params::FONT_SCALE_NORMAL,
                Params::COLOR_ORANGE,
                Params::FONT_THICKNESS_BOLD);
}

void AppController::analyzeDetections(const PerspectiveTransformer::Result& perspectiveResult,
                                      std::vector<ShapeSegmenter::Candidate>& candidates,
                                      std::vector<ShapeClassifier::Classification>& classifications,
                                      std::vector<ShapeQualityAnalyzer::QualityScore>& qualities) {
    candidates = segmenter_.segment(perspectiveResult.warped, perspectiveResult.warpedMask);

    classifications.clear();
    qualities.clear();
    classifications.reserve(candidates.size());
    qualities.reserve(candidates.size());

    for (const auto& candidate : candidates) {
        const auto classification = classifier_.classify(candidate);
        const auto quality = qualityAnalyzer_.evaluate(candidate, classification);

        classifications.push_back(classification);
        qualities.push_back(quality);
        outputManager_.logDetection(candidate, classification, quality);
    }
}

void AppController::updateTrackers(
    const std::vector<ShapeSegmenter::Candidate>& candidates,
    const std::vector<ShapeClassifier::Classification>& classifications,
    const std::vector<ShapeQualityAnalyzer::QualityScore>& qualities) {

    std::vector<cv::Point> candidateCentroids;
    candidateCentroids.reserve(candidates.size());
    std::transform(candidates.begin(), candidates.end(), std::back_inserter(candidateCentroids),
                   [](const ShapeSegmenter::Candidate& candidate) { return computeCentroid(candidate); });

    std::vector<bool> candidateMatched(candidates.size(), false);
    std::vector<bool> trackerMatched(trackers_.size(), false);

    for (size_t trackerIdx = 0; trackerIdx < trackers_.size(); ++trackerIdx) {
        double minDistance = std::numeric_limits<double>::max();
        int bestCandidateIdx = -1;

        for (size_t candidateIdx = 0; candidateIdx < candidates.size(); ++candidateIdx) {
            if (candidateMatched[candidateIdx]) {
                continue;
            }

            const double distance = euclideanDistance(candidateCentroids[candidateIdx],
                                                      trackers_[trackerIdx].lastCentroid);
            if (distance < minDistance) {
                minDistance = distance;
                bestCandidateIdx = static_cast<int>(candidateIdx);
            }
        }

        if (bestCandidateIdx >= 0 && minDistance < Params::STABILIZATION_TRACKING_MAX_DISTANCE) {
            trackers_[trackerIdx].update(candidates[bestCandidateIdx],
                                         classifications[bestCandidateIdx],
                                         qualities[bestCandidateIdx]);
            trackerMatched[trackerIdx] = true;
            candidateMatched[bestCandidateIdx] = true;
        }
    }

    for (int trackerIdx = static_cast<int>(trackers_.size()) - 1; trackerIdx >= 0; --trackerIdx) {
        if (trackerMatched[trackerIdx]) {
            continue;
        }

        auto& tracker = trackers_[trackerIdx];
        tracker.framesUnseen++;
        if (tracker.framesUnseen > Params::STABILIZATION_TRACKING_MAX_UNSEEN_FRAMES) {
            trackers_.erase(trackers_.begin() + trackerIdx);
        }
    }

    for (size_t candidateIdx = 0; candidateIdx < candidates.size(); ++candidateIdx) {
        if (candidateMatched[candidateIdx]) {
            continue;
        }

        StabilizedShapeTracker tracker;
        tracker.id = nextTrackerId_++;
        tracker.update(candidates[candidateIdx], classifications[candidateIdx], qualities[candidateIdx]);
        trackers_.push_back(tracker);
    }
}

void AppController::collectStabilizedDetections(
    std::vector<ShapeSegmenter::Candidate>& candidates,
    std::vector<ShapeClassifier::Classification>& classifications,
    std::vector<ShapeQualityAnalyzer::QualityScore>& qualities) const {

    candidates.clear();
    classifications.clear();
    qualities.clear();

    candidates.reserve(trackers_.size());
    classifications.reserve(trackers_.size());
    qualities.reserve(trackers_.size());

    for (const auto& tracker : trackers_) {
        const StabilizedResult stabilized = tracker.getStabilizedResult(this);

        candidates.push_back(tracker.lastCandidate);

        ShapeClassifier::Classification classification;
        classification.type = stabilized.type;
        classification.label = ShapeClassifier::toString(stabilized.type);
        classification.confidence = 0.0;
        classifications.push_back(classification);

        const double averageScore = std::max(0.0, stabilized.averageScore);

        ShapeQualityAnalyzer::QualityScore quality;
        quality.systemScore = averageScore;
        quality.displayScore = std::ceil(averageScore / 10.0) * 10.0;
        quality.grade = stabilized.grade;
        qualities.push_back(quality);
    }
}

void AppController::annotateDetections(
    cv::Mat& frame,
    const std::vector<ShapeSegmenter::Candidate>& candidates,
    const std::vector<ShapeClassifier::Classification>& classifications,
    const std::vector<ShapeQualityAnalyzer::QualityScore>& qualities) const {

    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& candidate = candidates[i];
        const auto& classification = classifications[i];
        const auto& quality = qualities[i];

        const cv::Scalar color = colorForShape(classification.type);

        cv::rectangle(frame, candidate.boundingBox, color, Params::BBOX_THICKNESS);
        cv::drawContours(frame, std::vector<std::vector<cv::Point>>{candidate.contour}, -1, color,
                         Params::BBOX_THICKNESS);

        const std::string label = buildAnnotationText(classification, quality);

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                            Params::FONT_SCALE_SMALL, Params::FONT_THICKNESS, &baseline);
        cv::Rect textBg(candidate.boundingBox.x,
                        std::max(candidate.boundingBox.y - textSize.height - (2 * Params::TEXT_PADDING), 0),
                        textSize.width + (2 * Params::TEXT_PADDING),
                        textSize.height + Params::TEXT_PADDING + baseline);

        cv::rectangle(frame, textBg, cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(frame, label,
                    cv::Point(textBg.x + Params::TEXT_PADDING,
                              textBg.y + textBg.height - Params::TEXT_BASELINE_OFFSET),
                    cv::FONT_HERSHEY_SIMPLEX,
                    Params::FONT_SCALE_SMALL,
                    Params::COLOR_WHITE,
                    Params::FONT_THICKNESS);
    }
}

std::vector<int> AppController::detectAvailableCameras(int maxCameras) const {
    std::vector<int> cameras;

    std::cout << "Scanning cameras..." << std::endl;
    for (int index = 0; index < maxCameras; ++index) {
        cv::VideoCapture probe(index);
        if (!probe.isOpened()) {
            continue;
        }

        cv::Mat frame;
        probe >> frame;
        if (frame.empty()) {
            continue;
        }

        int width = static_cast<int>(probe.get(cv::CAP_PROP_FRAME_WIDTH));
        int height = static_cast<int>(probe.get(cv::CAP_PROP_FRAME_HEIGHT));

        std::cout << "  Camera " << index << ": " << width << "x" << height;
#if CV_VERSION_MAJOR >= 4
        std::string backend = probe.getBackendName();
        if (!backend.empty()) {
            std::cout << " (" << backend << ")";
        }
#endif
        std::cout << std::endl;

        cameras.push_back(index);
    }

    return cameras;
}

int AppController::promptCameraSelection(const std::vector<int>& cameras) const {
    if (cameras.empty()) {
        return -1;
    }

    if (cameras.size() == 1) {
        std::cout << "Single camera found. Using camera " << cameras.front() << '.' << std::endl;
        return cameras.front();
    }

    std::cout << "Multiple cameras found:" << std::endl;
    for (size_t i = 0; i < cameras.size(); ++i) {
        std::cout << "  [" << i << "] Camera " << cameras[i] << std::endl;
    }

    while (true) {
        std::cout << "Your selection (0-" << cameras.size() - 1 << "): ";
        std::string input;
        if (!std::getline(std::cin, input)) {
            return -1;
        }

        std::stringstream ss(input);
        int choice = -1;
        if (ss >> choice && choice >= 0 && choice < static_cast<int>(cameras.size())) {
            return cameras[choice];
        }

        std::cout << "Invalid selection, try again." << std::endl;
    }
}

std::string AppController::getGradeFromScore(double score) const {
    if (score >= ShapeQualityAnalyzer::Parameters::GRADE_EXCELLENT) {
        return "Excellent";
    } else if (score >= ShapeQualityAnalyzer::Parameters::GRADE_VERY_GOOD) {
        return "Very Good";
    } else if (score >= ShapeQualityAnalyzer::Parameters::GRADE_GOOD) {
        return "Good";
    } else if (score >= ShapeQualityAnalyzer::Parameters::GRADE_ACCEPTABLE) {
        return "Acceptable";
    } else {
        return "Poor";
    }
}

// StabilizedShapeTracker implementation
void AppController::StabilizedShapeTracker::update(
    const ShapeSegmenter::Candidate& candidate,
    const ShapeClassifier::Classification& classification,
    const ShapeQualityAnalyzer::QualityScore& quality) {

    lastCentroid = computeCentroid(candidate);

    ClassificationHistoryEntry entry{classification.type, quality.systemScore};
    history.push_back(entry);

    while (history.size() > static_cast<size_t>(Params::STABILIZATION_WINDOW_SIZE)) {
        history.pop_front();
    }

    lastCandidate = candidate;
    framesUnseen = 0;
}

AppController::StabilizedResult AppController::StabilizedShapeTracker::getStabilizedResult(
    const AppController* controller) const {

    StabilizedResult result;
    result.type = ShapeClassifier::ShapeType::Unknown;
    result.averageScore = 0.0;
    result.label = "Unknown";
    result.grade = "N/A";

    if (history.empty()) {
        return result;
    }

    std::map<ShapeClassifier::ShapeType, std::vector<double>> scoresByType;
    for (const auto& entry : history) {
        scoresByType[entry.type].push_back(entry.systemScore);
    }

    ShapeClassifier::ShapeType bestType = ShapeClassifier::ShapeType::Unknown;
    double bestAverageScore = -1.0;

    for (const auto& [type, scores] : scoresByType) {
        if (scores.empty()) {
            continue;
        }

        const double total = std::accumulate(scores.begin(), scores.end(), 0.0);
        const double averageScore = total / static_cast<double>(scores.size());

        if (averageScore > bestAverageScore) {
            bestAverageScore = averageScore;
            bestType = type;
        }
    }

    result.type = bestType;
    result.averageScore = bestAverageScore;
    result.label = ShapeClassifier::toString(bestType);
    result.grade = controller->getGradeFromScore(bestAverageScore);

    return result;
}
