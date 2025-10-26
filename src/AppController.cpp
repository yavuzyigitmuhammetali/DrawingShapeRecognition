#include "AppController.hpp"

#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>

using Params = AppController::Parameters;

AppController::AppController()
    : running_(false) {}

int AppController::run(int argc, char** argv) {
    if (!initialize(argc, argv)) {
        std::cerr << "Baslatma basarisiz." << std::endl;
        return -1;
    }

    processStream();
    outputManager_.forceStopRecording();
    outputManager_.flush();
    return 0;
}

bool AppController::initialize(int argc, char** argv) {
    std::cout << "Drawing Shape Recognition baslatiliyor..." << std::endl;

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
                std::cout << "Kamera acilamadi. Kullanilabilir kameralar aranacak.\n";
            }
        } else {
            opened = videoSource_.open(argument);
        }
    } else {
        auto cameras = detectAvailableCameras();
        if (cameras.empty()) {
            std::cerr << "Kullanilabilir kamera bulunamadi." << std::endl;
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
                std::cerr << "Kullanilabilir kamera bulunamadi." << std::endl;
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
        std::cerr << "Girdi kaynagi acilamadi." << std::endl;
        return false;
    }

    running_ = true;
    std::cout << "✅ Sistem hazır. ArUco marker tespiti bekleniyor..." << std::endl;
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
                        "Markerler bekleniyor...",
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

    cameraView = perspectiveResult.annotatedFrame.empty()
                 ? frame.clone()
                 : perspectiveResult.annotatedFrame.clone();

    if (!cameraView.empty() && !perspectiveResult.originalMask.empty()) {
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

    hasBirdseye = perspectiveResult.success;
    if (!hasBirdseye) {
        cv::putText(cameraView,
                    "Markerleri gorunecek sekilde yerlestirin.",
                    cv::Point(Params::TEXT_MARGIN_X, Params::TEXT_MARGIN_Y_TOP),
                    cv::FONT_HERSHEY_SIMPLEX,
                    Params::FONT_SCALE_NORMAL,
                    Params::COLOR_ORANGE,
                    Params::FONT_THICKNESS_BOLD);
        // No ArUco detected - let OutputManager handle recording state
        outputManager_.processFrame(cameraView, false);
        return;
    }

    birdseyeView = perspectiveResult.warped.clone();
    if (birdseyeView.empty()) {
        hasBirdseye = false;
        outputManager_.processFrame(cameraView, false);
        return;
    }

    if (birdseyeView.channels() == 1) {
        cv::cvtColor(birdseyeView, birdseyeView, cv::COLOR_GRAY2BGR);
    }

    if (!perspectiveResult.warpedMask.empty()) {
        cv::Mat outsideMask;
        cv::bitwise_not(perspectiveResult.warpedMask, outsideMask);
        birdseyeView.setTo(Params::COLOR_BACKGROUND_DARK, outsideMask);
    }

    auto candidates = segmenter_.segment(perspectiveResult.warped, perspectiveResult.warpedMask);
    std::vector<ShapeClassifier::Classification> classifications;
    std::vector<ShapeQualityAnalyzer::QualityScore> qualities;

    classifications.reserve(candidates.size());
    qualities.reserve(candidates.size());

    for (const auto& candidate : candidates) {
        auto classification = classifier_.classify(candidate);
        auto quality = qualityAnalyzer_.evaluate(candidate, classification);

        classifications.push_back(classification);
        qualities.push_back(quality);

        outputManager_.logDetection(candidate, classification, quality);
    }

    annotateDetections(birdseyeView, candidates, classifications, qualities);

    // ArUco detected successfully - send birdseye view to OutputManager
    outputManager_.processFrame(birdseyeView, true);
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

        cv::Scalar color;
        switch (classification.type) {
        case ShapeClassifier::ShapeType::Circle:
            color = Params::COLOR_CIRCLE;
            break;
        case ShapeClassifier::ShapeType::Triangle:
            color = Params::COLOR_TRIANGLE;
            break;
        case ShapeClassifier::ShapeType::Square:
            color = Params::COLOR_SQUARE;
            break;
        case ShapeClassifier::ShapeType::Rectangle:
            color = Params::COLOR_RECTANGLE;
            break;
        case ShapeClassifier::ShapeType::Hexagon:
            color = Params::COLOR_HEXAGON;
            break;
        default:
            color = Params::COLOR_UNKNOWN;
            break;
        }

        cv::rectangle(frame, candidate.boundingBox, color, Params::BBOX_THICKNESS);
        cv::drawContours(frame, std::vector<std::vector<cv::Point>>{candidate.contour}, -1, color,
                         Params::BBOX_THICKNESS);

        std::ostringstream label;
        label << classification.label << " | Q:" << std::fixed << std::setprecision(0) << quality.displayScore
              << " (S:" << std::setprecision(0) << quality.systemScore << ") | "
              << quality.grade;

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX,
                                            Params::FONT_SCALE_SMALL, Params::FONT_THICKNESS, &baseline);
        cv::Rect textBg(candidate.boundingBox.x,
                        std::max(candidate.boundingBox.y - textSize.height - (2 * Params::TEXT_PADDING), 0),
                        textSize.width + (2 * Params::TEXT_PADDING),
                        textSize.height + Params::TEXT_PADDING + baseline);

        cv::rectangle(frame, textBg, cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(frame, label.str(),
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

    std::cout << "Kameralar taraniyor..." << std::endl;
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

        std::cout << "  Kamera " << index << ": " << width << "x" << height;
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
        std::cout << "Tek kamera bulundu. Kamera " << cameras.front() << " kullanilacak." << std::endl;
        return cameras.front();
    }

    std::cout << "Birden fazla kamera bulundu:" << std::endl;
    for (size_t i = 0; i < cameras.size(); ++i) {
        std::cout << "  [" << i << "] Kamera " << cameras[i] << std::endl;
    }

    while (true) {
        std::cout << "Seciminiz (0-" << cameras.size() - 1 << "): ";
        std::string input;
        if (!std::getline(std::cin, input)) {
            return -1;
        }

        std::stringstream ss(input);
        int choice = -1;
        if (ss >> choice && choice >= 0 && choice < static_cast<int>(cameras.size())) {
            return cameras[choice];
        }

        std::cout << "Gecersiz secim, tekrar deneyin." << std::endl;
    }
}
