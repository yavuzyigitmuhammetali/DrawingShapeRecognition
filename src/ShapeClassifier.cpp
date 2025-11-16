#include "ShapeClassifier.h"

std::vector<DetectedShape> ShapeClassifier::findShapes(const cv::Mat &warpedImage) const {
    std::vector<DetectedShape> shapes;
    cv::Mat gray, blurred, binary;

    cv::cvtColor(warpedImage, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 3, 0);

    cv::adaptiveThreshold(blurred, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV, 51, 9);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double warpedArea =
            static_cast<double>(warpedImage.cols) * warpedImage.rows;
    const double minShapeArea = warpedArea * kMinShapeAreaRatio;

    for (const auto &contour: contours) {
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
