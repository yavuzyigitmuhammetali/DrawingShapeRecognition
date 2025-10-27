#include "ShapeDetector.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>

namespace {
constexpr double kSigma = 0.33;
constexpr double kMinPaperAreaRatio = 0.05;
constexpr double kMinShapeAreaRatio = 0.0015;
constexpr double kCircularityThreshold = 0.85;
constexpr float kWarpWidth = 640.0F;
constexpr float kWarpHeight = 480.0F;
}  // namespace

ShapeDetector::ShapeDetector() {
    cap.open(0);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Failed to open the default camera." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::cout << "Camera opened successfully." << std::endl;
    cv::namedWindow(windowName);
    cv::namedWindow(warpedWindowName);
}

ShapeDetector::~ShapeDetector() {
    cap.release();
    cv::destroyAllWindows();
}

void ShapeDetector::run() {
    cv::Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            break;
        }

        cv::Mat processedFrame = processFrame(frame);
        cv::imshow(windowName, processedFrame);

        const char key = static_cast<char>(cv::waitKey(1));
        if (key == 27) {
            break;
        }
    }
}

cv::Mat ShapeDetector::processFrame(const cv::Mat& frame) {
    cv::Mat outputFrame = frame.clone();

    cv::Mat processedImage = preProcessImage(frame);
    std::vector<cv::Point> paperContour = getLargestContour(processedImage, frame.size());

    std::vector<DetectedShape> allShapes;
    if (!paperContour.empty()) {
        cv::drawContours(outputFrame, std::vector<std::vector<cv::Point>>{paperContour}, -1,
                         cv::Scalar(0, 255, 0), 3);

        cv::Mat warped = warpImage(frame, paperContour);
        if (!warped.empty()) {
            allShapes = findShapes(warped);
            drawDetections(warped, allShapes);
            cv::imshow(warpedWindowName, warped);
        }
    }

    annotateSummary(outputFrame, allShapes);
    saveDetectionsToFile(allShapes);
    return outputFrame;
}

cv::Mat ShapeDetector::preProcessImage(const cv::Mat& frame) {
    cv::Mat gray, blurred, edges, dilated;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 5, 0);

    const double meanIntensity = cv::mean(blurred)[0];
    const int lower = std::max(0, static_cast<int>((1.0 - kSigma) * meanIntensity));
    const int upper = std::min(255, static_cast<int>((1.0 + kSigma) * meanIntensity));

    cv::Canny(blurred, edges, lower, upper);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(edges, dilated, kernel);
    return dilated;
}

std::vector<cv::Point> ShapeDetector::getLargestContour(const cv::Mat& processedImage,
                                                        cv::Size originalFrameSize) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(processedImage, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    const double minPaperArea =
        static_cast<double>(originalFrameSize.width) * originalFrameSize.height *
        kMinPaperAreaRatio;

    double maxArea = 0.0;
    std::vector<cv::Point> largestContour;

    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area <= minPaperArea) {
            continue;
        }

        const double perimeter = cv::arcLength(contour, true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, 0.02 * perimeter, true);

        if (approx.size() == 4 && area > maxArea) {
            maxArea = area;
            largestContour = approx;
        }
    }

    return largestContour;
}

std::vector<cv::Point> ShapeDetector::reOrderPoints(const std::vector<cv::Point>& points) {
    if (points.size() != 4) {
        return {};
    }

    std::vector<cv::Point> orderedPoints(4);
    std::vector<int> sumPoints;
    std::vector<int> diffPoints;
    sumPoints.reserve(points.size());
    diffPoints.reserve(points.size());

    for (const auto& pt : points) {
        sumPoints.push_back(pt.x + pt.y);
        diffPoints.push_back(pt.x - pt.y);
    }

    const auto topLeftIdx = static_cast<size_t>(
        std::distance(sumPoints.begin(), std::min_element(sumPoints.begin(), sumPoints.end())));
    const auto bottomRightIdx = static_cast<size_t>(
        std::distance(sumPoints.begin(), std::max_element(sumPoints.begin(), sumPoints.end())));
    const auto topRightIdx = static_cast<size_t>(
        std::distance(diffPoints.begin(), std::max_element(diffPoints.begin(), diffPoints.end())));
    const auto bottomLeftIdx = static_cast<size_t>(
        std::distance(diffPoints.begin(), std::min_element(diffPoints.begin(), diffPoints.end())));

    orderedPoints[0] = points[topLeftIdx];
    orderedPoints[3] = points[bottomRightIdx];
    orderedPoints[1] = points[topRightIdx];
    orderedPoints[2] = points[bottomLeftIdx];

    return orderedPoints;
}

cv::Mat ShapeDetector::warpImage(const cv::Mat& frame, const std::vector<cv::Point>& points) {
    if (points.size() != 4) {
        return {};
    }

    const std::vector<cv::Point> orderedPoints = reOrderPoints(points);
    if (orderedPoints.size() != 4) {
        return {};
    }

    const cv::Point2f src[4] = {
        cv::Point2f(static_cast<float>(orderedPoints[0].x),
                    static_cast<float>(orderedPoints[0].y)),
        cv::Point2f(static_cast<float>(orderedPoints[1].x),
                    static_cast<float>(orderedPoints[1].y)),
        cv::Point2f(static_cast<float>(orderedPoints[2].x),
                    static_cast<float>(orderedPoints[2].y)),
        cv::Point2f(static_cast<float>(orderedPoints[3].x),
                    static_cast<float>(orderedPoints[3].y)),
    };

    const cv::Point2f dst[4] = {
        {0.0F, 0.0F},
        {kWarpWidth, 0.0F},
        {0.0F, kWarpHeight},
        {kWarpWidth, kWarpHeight},
    };

    cv::Mat transformMatrix = cv::getPerspectiveTransform(src, dst);

    cv::Mat warpedImage;
    cv::warpPerspective(frame, warpedImage, transformMatrix,
                        cv::Size(static_cast<int>(kWarpWidth), static_cast<int>(kWarpHeight)));
    return warpedImage;
}

std::vector<DetectedShape> ShapeDetector::findShapes(const cv::Mat& warpedImage) {
    std::vector<DetectedShape> shapes;
    cv::Mat gray, blurred, binary;

    cv::cvtColor(warpedImage, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 3, 0);

    cv::adaptiveThreshold(blurred, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV, 51, 9);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double warpedArea =
        static_cast<double>(warpedImage.cols) * warpedImage.rows;
    const double minShapeArea = warpedArea * kMinShapeAreaRatio;

    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < minShapeArea) {
            continue;
        }

        DetectedShape detected;
        detected.contour = contour;
        detected.boundingBox = cv::boundingRect(contour);

        const double perimeter = cv::arcLength(contour, true);
        if (perimeter == 0.0) {
            continue;
        }

        const double epsilon = 0.025 * perimeter;
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, epsilon, true);
        const int cornerCount = static_cast<int>(approx.size());

        const double circularity = (4.0 * CV_PI * area) / (perimeter * perimeter);
        const cv::RotatedRect minRect = cv::minAreaRect(contour);
        const double minRectArea = minRect.size.area();
        const double polygonCompactness = minRectArea > 0.0 ? area / minRectArea : 1.0;

        const double approxPerimeter = cv::arcLength(approx, true);
        const double perimeterRatio =
            approxPerimeter > 0.0 ? approxPerimeter / perimeter : 1.0;

        if (cornerCount == 3) {
            detected.type = "Triangle";
        } else if (cornerCount == 4) {
            if (circularity > kCircularityThreshold) {
                detected.type = "Circle";
            } else {
                const double epsilonPolygon = 0.04 * perimeter;
                std::vector<cv::Point> refinedApprox;
                cv::approxPolyDP(contour, refinedApprox, epsilonPolygon, true);

                if (refinedApprox.size() == 3) {
                    detected.type = "Triangle";
                } else {
                    const float aspect =
                        static_cast<float>(detected.boundingBox.width) /
                        static_cast<float>(detected.boundingBox.height);
                    if (aspect > 0.90F && aspect < 1.10F) {
                        detected.type = "Square";
                    } else {
                        detected.type = "Rectangle";
                    }
                }
            }
        } else if (cornerCount == 6) {
            detected.type = circularity > kCircularityThreshold ? "Circle" : "Hexagon";
        } else if (cornerCount > 6) {
            detected.type = "Circle";
        } else {
            detected.type = circularity > kCircularityThreshold ? "Circle" : "Unknown";
        }

        if (detected.type == "Triangle") {
            detected.smoothness = perimeterRatio;
        } else if (detected.type == "Circle") {
            detected.smoothness = circularity;
        } else if (detected.type == "Square" || detected.type == "Rectangle" ||
                   detected.type == "Hexagon") {
            detected.smoothness = polygonCompactness;
        } else {
            detected.smoothness = 0.0;
        }

        shapes.push_back(detected);
    }

    return shapes;
}

void ShapeDetector::drawDetections(cv::Mat& image, const std::vector<DetectedShape>& shapes) {
    for (const auto& shape : shapes) {
        if (shape.type == "Unknown") {
            continue;
        }

        cv::rectangle(image, shape.boundingBox, cv::Scalar(0, 0, 255), 2);

        std::stringstream labelStream;
        labelStream << shape.type << " [" << std::fixed << std::setprecision(2)
                    << shape.smoothness << "]";
        const std::string label = labelStream.str();

        cv::Point labelOrigin(shape.boundingBox.x,
                              std::max(0, shape.boundingBox.y - 5));
        cv::putText(image, label, labelOrigin, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 255), 1);
    }
}

void ShapeDetector::annotateSummary(cv::Mat& image, const std::vector<DetectedShape>& shapes) {
    const cv::Point origin{10, 25};
    const double fontScale = 0.6;
    const int thickness = 1;

    if (image.empty()) {
        return;
    }

    if (shapes.empty()) {
        cv::putText(image, "No shapes detected", origin, cv::FONT_HERSHEY_SIMPLEX,
                    fontScale, cv::Scalar(0, 255, 255), thickness);
        return;
    }

    std::map<std::string, int> counts;
    int unknownCount = 0;
    for (const auto& shape : shapes) {
        if (shape.type == "Unknown") {
            ++unknownCount;
            continue;
        }
        ++counts[shape.type];
    }

    const int knownCount =
        std::accumulate(counts.begin(), counts.end(), 0,
                        [](int sum, const auto& entry) { return sum + entry.second; });

    std::stringstream header;
    header << "Shapes detected: " << knownCount;
    if (unknownCount > 0) {
        header << " (Unknown: " << unknownCount << ")";
    }

    cv::putText(image, header.str(), origin, cv::FONT_HERSHEY_SIMPLEX, fontScale,
                cv::Scalar(0, 255, 255), thickness);

    if (counts.empty()) {
        return;
    }

    int line = 1;
    for (const auto& entry : counts) {
        std::stringstream lineStream;
        lineStream << "  " << entry.first << ": " << entry.second;
        cv::Point lineOrigin{origin.x, origin.y + line * 20};
        cv::putText(image, lineStream.str(), lineOrigin, cv::FONT_HERSHEY_SIMPLEX, fontScale,
                    cv::Scalar(0, 255, 255), thickness);
        ++line;
        if (line > 5) {
            cv::putText(image, "  ...", {origin.x, origin.y + line * 20},
                        cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 255, 255),
                        thickness);
            break;
        }
    }
}

void ShapeDetector::saveDetectionsToFile(const std::vector<DetectedShape>& shapes) {
    std::ofstream outFile(outputFileName);
    if (!outFile.is_open()) {
        std::cerr << "ERROR: Unable to open output file (" << outputFileName << ")." << std::endl;
        return;
    }

    if (shapes.empty()) {
        outFile << "No shapes detected." << std::endl;
        return;
    }

    outFile << "Detected Shapes:" << std::endl;
    outFile << "-------------------------" << std::endl;

    int count = 1;
    for (const auto& shape : shapes) {
        if (shape.type == "Unknown") {
            continue;
        }

        outFile << "Shape #" << count++ << ":" << std::endl;
        outFile << "  Type          : " << shape.type << std::endl;

        std::stringstream smoothStream;
        smoothStream << std::fixed << std::setprecision(3) << shape.smoothness;
        outFile << "  Smoothness    : " << smoothStream.str() << std::endl;

        outFile << "  Box (x,y,w,h) : [" << shape.boundingBox.x << ", "
                << shape.boundingBox.y << ", " << shape.boundingBox.width << ", "
                << shape.boundingBox.height << "]" << std::endl;
        outFile << "-------------------------" << std::endl;
    }
}
