#pragma once

#include <opencv2/opencv.hpp>
#include <map>
#include <string>
#include <vector>

struct DetectedShape {
    std::string type{"Unknown"};
    double smoothness{0.0};
    cv::Rect boundingBox;
    std::vector<cv::Point> contour;
};

class ShapeDetector {
public:
    ShapeDetector();
    ~ShapeDetector();

    void run();

private:
    cv::Mat processFrame(const cv::Mat& frame);

    cv::Mat preProcessImage(const cv::Mat& frame);
    std::vector<cv::Point> getLargestContour(const cv::Mat& processedImage,
                                             cv::Size originalFrameSize);
    std::vector<cv::Point> reOrderPoints(const std::vector<cv::Point>& points);
    cv::Mat warpImage(const cv::Mat& frame, const std::vector<cv::Point>& points);

    std::vector<DetectedShape> findShapes(const cv::Mat& warpedImage);

    void drawDetections(cv::Mat& image, const std::vector<DetectedShape>& shapes);
    void saveDetectionsToFile(const std::vector<DetectedShape>& shapes);
    void annotateSummary(cv::Mat& image, const std::vector<DetectedShape>& shapes);
    std::string formatShapeLabel(const DetectedShape& shape, int precision = 2) const;
    std::map<std::string, int> countKnownShapes(const std::vector<DetectedShape>& shapes,
                                                int& unknownCount) const;

    cv::VideoCapture cap;
    std::string windowName{"Shape Detector - Original"};
    std::string warpedWindowName{"Top-Down View"};
    std::string outputFileName{"detected_shapes.txt"};
};
