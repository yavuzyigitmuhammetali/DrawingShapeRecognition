#include "AppController.hpp"

#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>

AppController::AppController()
    : running_(false) {}

int AppController::run(int argc, char** argv) {
    if (!initialize(argc, argv)) {
        std::cerr << "Baslatma basarisiz." << std::endl;
        return -1;
    }

    processStream();
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
            cv::Mat message(480, 640, CV_8UC3, cv::Scalar(30, 30, 30));
            cv::putText(message,
                        "Markerler bekleniyor...",
                        cv::Point(30, 240),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.9,
                        cv::Scalar(0, 165, 255),
                        2);
            cv::imshow("Bird's Eye View", message);
        }

        int key = cv::waitKey(1);
        if (key == 27) { // ESC
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

    hasBirdseye = perspectiveResult.success;
    if (!hasBirdseye) {
        cv::putText(cameraView,
                    "Markerleri gorunecek sekilde yerlestirin.",
                    cv::Point(30, 80),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(0, 165, 255),
                    2);
        return;
    }

    birdseyeView = perspectiveResult.warped.clone();
    if (birdseyeView.empty()) {
        hasBirdseye = false;
        return;
    }

    if (birdseyeView.channels() == 1) {
        cv::cvtColor(birdseyeView, birdseyeView, cv::COLOR_GRAY2BGR);
    }

    auto candidates = segmenter_.segment(perspectiveResult.warped);
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
            color = cv::Scalar(0, 255, 0);
            break;
        case ShapeClassifier::ShapeType::Triangle:
            color = cv::Scalar(0, 255, 255);
            break;
        case ShapeClassifier::ShapeType::Square:
            color = cv::Scalar(255, 0, 0);
            break;
        case ShapeClassifier::ShapeType::Rectangle:
            color = cv::Scalar(255, 255, 0);
            break;
        case ShapeClassifier::ShapeType::Hexagon:
            color = cv::Scalar(255, 0, 255);
            break;
        default:
            color = cv::Scalar(0, 165, 255);
            break;
        }

        cv::rectangle(frame, candidate.boundingBox, color, 2);
        cv::drawContours(frame, std::vector<std::vector<cv::Point>>{candidate.contour}, -1, color, 2);

        std::ostringstream label;
        label << classification.label << " | " << std::fixed << std::setprecision(1)
              << quality.score << "% (" << quality.grade << ")";

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::Rect textBg(candidate.boundingBox.x,
                        std::max(candidate.boundingBox.y - textSize.height - 8, 0),
                        textSize.width + 10,
                        textSize.height + 6);

        cv::rectangle(frame, textBg, cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(frame, label.str(),
                    cv::Point(textBg.x + 5, textBg.y + textBg.height - 4),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(255, 255, 255),
                1);
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
