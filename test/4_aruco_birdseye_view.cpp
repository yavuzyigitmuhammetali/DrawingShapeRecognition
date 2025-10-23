/**
 * ArUco-based Bird's Eye View Tool
 *
 * This tool creates a bird's-eye view of paper using only ArUco markers,
 * WITHOUT requiring camera calibration. It works by detecting the 4 corner
 * markers and applying perspective transformation.
 *
 * Usage:
 *   ./aruco_birdseye_view [camera_id]
 *
 * Arguments:
 *   camera_id - Camera index (0, 1, 2, ...) - optional, will prompt if not provided
 *
 * Controls:
 *   SPACE - Toggle measurement display
 *   's'   - Save current frame
 *   ESC   - Exit
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <sstream>

// ArUco configuration (matching the template from generate_markers.py)
const int ARUCO_DICT = cv::aruco::DICT_4X4_50;
const std::vector<int> CORNER_MARKER_IDS = {0, 1, 2, 3}; // TL, TR, BR, BL

// === PHYSICAL MEASUREMENTS (from user) ===
// Standard A4 paper size
const float A4_WIDTH_CM = 21.0f;
const float A4_HEIGHT_CM = 29.7f;

// ArUco marker size (actual printed size)
const float MARKER_SIZE_CM = 1.8f;  // 1.8cm x 1.8cm

// Rectangle formed by the 4 corner markers (measured distances)
// These are the distances between marker CENTERS
const float MARKERS_RECT_WIDTH_CM = 13.7f;   // TL-TR or BL-BR horizontal distance
const float MARKERS_RECT_HEIGHT_CM = 22.45f; // TL-BL or TR-BR vertical distance

// Calculate marker positions on A4 paper (centers)
// Markers are centered on the page
const float MARKER_OFFSET_X_CM = (A4_WIDTH_CM - MARKERS_RECT_WIDTH_CM) / 2.0f;
const float MARKER_OFFSET_Y_CM = (A4_HEIGHT_CM - MARKERS_RECT_HEIGHT_CM) / 2.0f;

// Camera selection functions
std::vector<int> detectAvailableCameras(int maxCamerasToCheck = 10) {
    std::vector<int> availableCameras;

    std::cout << "Scanning for available cameras...\n";

    for (int i = 0; i < maxCamerasToCheck; i++) {
        cv::VideoCapture cap(i);
        if (cap.isOpened()) {
            cv::Mat frame;
            cap >> frame;

            if (!frame.empty()) {
                availableCameras.push_back(i);

                int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
                int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

                std::cout << "  Camera " << i << ": "
                         << width << "x" << height << " ";

                std::string backend = cap.getBackendName();
                if (!backend.empty()) {
                    std::cout << "(" << backend << ")";
                }
                std::cout << "\n";
            }
            cap.release();
        }
    }

    return availableCameras;
}

int selectCamera(const std::vector<int>& availableCameras) {
    if (availableCameras.empty()) {
        std::cerr << "ERROR: No cameras detected!\n";
        return -1;
    }

    if (availableCameras.size() == 1) {
        std::cout << "\nOnly one camera detected. Using Camera " << availableCameras[0] << "\n";
        return availableCameras[0];
    }

    std::cout << "\nMultiple cameras detected. Please select:\n";
    for (size_t i = 0; i < availableCameras.size(); i++) {
        std::cout << "  [" << i << "] Camera " << availableCameras[i] << "\n";
    }

    int selection = -1;
    while (true) {
        std::cout << "\nEnter selection (0-" << (availableCameras.size() - 1) << "): ";
        std::string input;
        std::getline(std::cin, input);

        std::stringstream ss(input);
        if (ss >> selection && selection >= 0 && selection < static_cast<int>(availableCameras.size())) {
            break;
        }
        std::cout << "Invalid selection. Please try again.\n";
    }

    return availableCameras[selection];
}

class BirdsEyeViewer {
private:
    cv::aruco::Dictionary dictionaryData;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    bool showMeasurements;

public:
    BirdsEyeViewer() : showMeasurements(true) {
        dictionaryData = cv::aruco::getPredefinedDictionary(ARUCO_DICT);
        dictionary = cv::makePtr<cv::aruco::Dictionary>(dictionaryData);

        std::cout << "\n========================================\n";
        std::cout << "  ArUco Bird's Eye View Tool\n";
        std::cout << "========================================\n";
        std::cout << "Template Configuration:\n";
        std::cout << "  Paper size: " << A4_WIDTH_CM << " x " << A4_HEIGHT_CM << " cm (A4)\n";
        std::cout << "  Marker size: " << MARKER_SIZE_CM << " x " << MARKER_SIZE_CM << " cm\n";
        std::cout << "  Markers rectangle: " << MARKERS_RECT_WIDTH_CM << " x "
                  << MARKERS_RECT_HEIGHT_CM << " cm\n";
        std::cout << "  Marker IDs: 0 (TL), 1 (TR), 2 (BR), 3 (BL)\n";
        std::cout << "========================================\n\n";
    }

    bool detectMarkers(const cv::Mat& frame,
                      std::vector<int>& markerIds,
                      std::vector<std::vector<cv::Point2f>>& markerCorners) {
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::makePtr<cv::aruco::DetectorParameters>();

        // Optimize for accuracy
        parameters->adaptiveThreshWinSizeMin = 3;
        parameters->adaptiveThreshWinSizeMax = 23;
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        parameters->cornerRefinementWinSize = 5;
        parameters->cornerRefinementMaxIterations = 30;
        parameters->cornerRefinementMinAccuracy = 0.01;

        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters);

        return markerIds.size() >= 4;
    }

    bool extractPaperCorners(const std::vector<int>& markerIds,
                            const std::vector<std::vector<cv::Point2f>>& markerCorners,
                            std::vector<cv::Point2f>& paperCorners) {

        // Find the 4 corner markers and get their centers
        std::map<int, cv::Point2f> markerCenters;

        for (size_t i = 0; i < markerIds.size(); i++) {
            int id = markerIds[i];

            // Check if this is a corner marker (0, 1, 2, 3)
            if (std::find(CORNER_MARKER_IDS.begin(), CORNER_MARKER_IDS.end(), id)
                != CORNER_MARKER_IDS.end()) {

                // Calculate marker center (average of 4 corners)
                cv::Point2f center(0, 0);
                for (const auto& corner : markerCorners[i]) {
                    center += corner;
                }
                center.x /= 4.0f;
                center.y /= 4.0f;

                markerCenters[id] = center;
            }
        }

        // Need all 4 markers
        if (markerCenters.size() != 4) {
            return false;
        }

        // Extract in order: TL (0), TR (1), BR (2), BL (3)
        paperCorners.clear();
        paperCorners.push_back(markerCenters[0]); // Top-Left
        paperCorners.push_back(markerCenters[1]); // Top-Right
        paperCorners.push_back(markerCenters[2]); // Bottom-Right
        paperCorners.push_back(markerCenters[3]); // Bottom-Left

        return true;
    }

    cv::Mat applyBirdsEyeView(const cv::Mat& frame,
                             const std::vector<cv::Point2f>& paperCorners,
                             int outputWidth = 1000) {

        // Calculate output height maintaining A4 aspect ratio
        float aspectRatio = A4_HEIGHT_CM / A4_WIDTH_CM;
        int outputHeight = static_cast<int>(outputWidth * aspectRatio);

        // Define destination points (full A4 paper in output)
        // We map marker centers to their physical positions on A4
        float scaleX = outputWidth / A4_WIDTH_CM;
        float scaleY = outputHeight / A4_HEIGHT_CM;

        std::vector<cv::Point2f> dstPoints;

        // Top-Left marker center position on A4
        float tl_x = MARKER_OFFSET_X_CM + MARKER_SIZE_CM / 2.0f;
        float tl_y = MARKER_OFFSET_Y_CM + MARKER_SIZE_CM / 2.0f;

        dstPoints.push_back(cv::Point2f(tl_x * scaleX, tl_y * scaleY));  // TL
        dstPoints.push_back(cv::Point2f((tl_x + MARKERS_RECT_WIDTH_CM) * scaleX, tl_y * scaleY));  // TR
        dstPoints.push_back(cv::Point2f((tl_x + MARKERS_RECT_WIDTH_CM) * scaleX,
                                       (tl_y + MARKERS_RECT_HEIGHT_CM) * scaleY));  // BR
        dstPoints.push_back(cv::Point2f(tl_x * scaleX, (tl_y + MARKERS_RECT_HEIGHT_CM) * scaleY));  // BL

        // Get perspective transformation matrix
        cv::Mat transformMatrix = cv::getPerspectiveTransform(paperCorners, dstPoints);

        // Apply transformation
        cv::Mat warped;
        cv::warpPerspective(frame, warped, transformMatrix,
                           cv::Size(outputWidth, outputHeight),
                           cv::INTER_LINEAR);

        return warped;
    }

    bool detectCircle(const cv::Mat& birdseye,
                     cv::Point2f& center,
                     float& radius,
                     std::vector<cv::Point>& contour) {

        // Convert to grayscale
        cv::Mat gray;
        cv::cvtColor(birdseye, gray, cv::COLOR_BGR2GRAY);

        // Apply Gaussian blur
        cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2.0);

        // Canny edge detection
        cv::Mat edges;
        cv::Canny(gray, edges, 30, 100, 3);

        // Morphological closing to connect broken edges
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, kernel);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        // Find the best circular contour
        double maxScore = 0;
        int bestIdx = -1;

        for (size_t i = 0; i < contours.size(); i++) {
            if (contours[i].size() < 50) {
                continue;
            }

            double area = cv::contourArea(contours[i]);

            // Filter by area
            if (area < 2000 || area > birdseye.rows * birdseye.cols * 0.5) {
                continue;
            }

            // Check circularity
            double perimeter = cv::arcLength(contours[i], true);
            double circularity = 4 * M_PI * area / (perimeter * perimeter);

            // Check fit to enclosing circle
            cv::Point2f testCenter;
            float testRadius;
            cv::minEnclosingCircle(contours[i], testCenter, testRadius);

            double enclosingArea = M_PI * testRadius * testRadius;
            double fitScore = area / enclosingArea;

            // Combined score
            double score = circularity * fitScore;

            if (circularity > 0.75 && fitScore > 0.75 && score > maxScore) {
                maxScore = score;
                bestIdx = i;
            }
        }

        if (bestIdx == -1) {
            return false;
        }

        contour = contours[bestIdx];
        cv::minEnclosingCircle(contour, center, radius);

        // Adjust radius to match contour better
        double sumDist = 0;
        for (const auto& pt : contour) {
            double dist = cv::norm(cv::Point2f(pt) - center);
            sumDist += dist;
        }
        radius = static_cast<float>(sumDist / contour.size());

        return true;
    }

    void measurePerpendicularDiameters(const std::vector<cv::Point>& contour,
                                      const cv::Point2f& center,
                                      float& diameter1,
                                      float& diameter2,
                                      float& angle1,
                                      cv::Point2f& p1a, cv::Point2f& p1b,
                                      cv::Point2f& p2a, cv::Point2f& p2b) {

        // Find the longest diameter
        float maxDist = 0;
        cv::Point2f farthestA, farthestB;

        for (size_t i = 0; i < contour.size(); i++) {
            for (size_t j = i + 1; j < contour.size(); j++) {
                cv::Point2f ptA(contour[i].x, contour[i].y);
                cv::Point2f ptB(contour[j].x, contour[j].y);
                float dist = cv::norm(ptA - ptB);

                if (dist > maxDist) {
                    maxDist = dist;
                    farthestA = ptA;
                    farthestB = ptB;
                }
            }
        }

        p1a = farthestA;
        p1b = farthestB;
        diameter1 = maxDist;

        // Calculate intersection point (midpoint of first diameter)
        cv::Point2f intersectionPoint = (p1a + p1b) / 2.0f;

        // Calculate angle
        cv::Point2f vec1 = p1a - p1b;
        angle1 = atan2(vec1.y, vec1.x);

        // Find perpendicular diameter
        float angle2 = angle1 + M_PI / 2.0;
        cv::Point2f direction(cos(angle2), sin(angle2));

        // Find furthest points along perpendicular direction
        cv::Point2f bestPos, bestNeg;
        float maxDistPos = 0;
        float maxDistNeg = 0;

        for (const auto& pt : contour) {
            cv::Point2f ptf(pt.x, pt.y);
            cv::Point2f vec = ptf - intersectionPoint;

            float dotProduct = vec.x * direction.x + vec.y * direction.y;

            if (dotProduct > 0) {
                if (dotProduct > maxDistPos) {
                    maxDistPos = dotProduct;
                    bestPos = ptf;
                }
            } else if (dotProduct < 0) {
                float absProjection = std::abs(dotProduct);
                if (absProjection > maxDistNeg) {
                    maxDistNeg = absProjection;
                    bestNeg = ptf;
                }
            }
        }

        p2a = bestPos;
        p2b = bestNeg;
        diameter2 = cv::norm(p2a - p2b);
    }

    void drawMeasurements(cv::Mat& output,
                         const cv::Point2f& center,
                         float radius,
                         float diameter1,
                         float diameter2,
                         const cv::Point2f& p1a,
                         const cv::Point2f& p1b,
                         const cv::Point2f& p2a,
                         const cv::Point2f& p2b,
                         const std::vector<cv::Point>& contour) {

        // Draw the detected contour
        cv::drawContours(output, std::vector<std::vector<cv::Point>>{contour},
                        0, cv::Scalar(0, 255, 0), 2);

        if (showMeasurements) {
            // Calculate intersection point
            cv::Point2f intersectionPoint = (p1a + p1b) / 2.0f;

            // Draw diameter 1 (red)
            cv::line(output, p1a, p1b, cv::Scalar(0, 0, 255), 2);
            cv::circle(output, p1a, 5, cv::Scalar(0, 0, 255), -1);
            cv::circle(output, p1b, 5, cv::Scalar(0, 0, 255), -1);

            // Draw diameter 2 (blue)
            cv::line(output, p2a, p2b, cv::Scalar(255, 0, 0), 2);
            cv::circle(output, p2a, 5, cv::Scalar(255, 0, 0), -1);
            cv::circle(output, p2b, 5, cv::Scalar(255, 0, 0), -1);

            // Draw intersection point (yellow)
            cv::circle(output, intersectionPoint, 6, cv::Scalar(0, 255, 255), -1);
            cv::circle(output, intersectionPoint, 8, cv::Scalar(0, 255, 255), 2);

            // Calculate ratio
            float ratio = diameter1 / diameter2;
            float error = std::abs(1.0f - ratio) * 100.0f;

            // Display measurements
            int y = 30;
            cv::putText(output, "Bird's Eye View Results:",
                       cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       cv::Scalar(255, 255, 255), 2);
            y += 35;

            cv::putText(output, "Diameter 1 (Red):   " + std::to_string(static_cast<int>(diameter1)) + " px",
                       cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                       cv::Scalar(0, 0, 255), 2);
            y += 30;

            cv::putText(output, "Diameter 2 (Blue):  " + std::to_string(static_cast<int>(diameter2)) + " px",
                       cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                       cv::Scalar(255, 0, 0), 2);
            y += 30;

            char ratioStr[50];
            snprintf(ratioStr, sizeof(ratioStr), "Ratio: %.4f", ratio);
            cv::Scalar ratioColor = (error < 1.0f) ? cv::Scalar(0, 255, 0) :
                                   (error < 3.0f) ? cv::Scalar(0, 255, 255) :
                                   cv::Scalar(0, 0, 255);
            cv::putText(output, ratioStr, cv::Point(10, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, ratioColor, 2);
            y += 30;

            char errorStr[50];
            snprintf(errorStr, sizeof(errorStr), "Error: %.2f%%", error);
            cv::putText(output, errorStr, cv::Point(10, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, ratioColor, 2);
            y += 35;

            // Quality assessment
            std::string quality;
            if (error < 0.5f) quality = "Excellent!";
            else if (error < 1.0f) quality = "Very Good";
            else if (error < 2.0f) quality = "Good";
            else if (error < 5.0f) quality = "Acceptable";
            else quality = "Poor";

            cv::putText(output, "Quality: " + quality, cv::Point(10, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, ratioColor, 2);
        }
    }

    void processFrame(const cv::Mat& frame, cv::Mat& originalOutput, cv::Mat& birdseyeOutput) {
        // Detect ArUco markers
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;

        if (!detectMarkers(frame, markerIds, markerCorners)) {
            originalOutput = frame.clone();
            birdseyeOutput = cv::Mat();

            std::string msg = "Searching for ArUco markers (need IDs: 0,1,2,3)...";
            cv::putText(originalOutput, msg,
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       cv::Scalar(0, 165, 255), 2);
            return;
        }

        // Draw detected markers
        originalOutput = frame.clone();
        cv::aruco::drawDetectedMarkers(originalOutput, markerCorners, markerIds);

        // Extract paper corners
        std::vector<cv::Point2f> paperCorners;
        if (!extractPaperCorners(markerIds, markerCorners, paperCorners)) {
            birdseyeOutput = cv::Mat();
            cv::putText(originalOutput, "Need all 4 corner markers (IDs: 0,1,2,3)",
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       cv::Scalar(0, 165, 255), 2);
            return;
        }

        // Draw paper boundary
        std::vector<cv::Point> paperCornersInt;
        for (const auto& pt : paperCorners) {
            paperCornersInt.push_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
        }
        cv::polylines(originalOutput, std::vector<std::vector<cv::Point>>{paperCornersInt},
                     true, cv::Scalar(0, 255, 0), 2);

        // Apply bird's-eye view transformation
        cv::Mat birdseye = applyBirdsEyeView(frame, paperCorners);

        // Try to detect circle
        cv::Point2f center;
        float radius;
        std::vector<cv::Point> contour;

        if (!detectCircle(birdseye, center, radius, contour)) {
            birdseyeOutput = birdseye.clone();
            cv::putText(birdseyeOutput, "No circle detected (optional)",
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       cv::Scalar(0, 165, 255), 2);
            return;
        }

        // Measure perpendicular diameters
        float diameter1, diameter2, angle1;
        cv::Point2f p1a, p1b, p2a, p2b;
        measurePerpendicularDiameters(contour, center, diameter1, diameter2, angle1,
                                     p1a, p1b, p2a, p2b);

        // Draw measurements
        birdseyeOutput = birdseye.clone();
        drawMeasurements(birdseyeOutput, center, radius, diameter1, diameter2,
                        p1a, p1b, p2a, p2b, contour);
    }

    void toggleMeasurements() {
        showMeasurements = !showMeasurements;
    }
};

void displayInstructions() {
    std::cout << "Instructions:\n";
    std::cout << "  1. Place the printed ArUco marker template in view\n";
    std::cout << "  2. Ensure all 4 corner markers are visible (IDs: 0,1,2,3)\n";
    std::cout << "  3. The tool will create a bird's-eye view automatically\n";
    std::cout << "  4. Draw a circle on the paper to test accuracy (optional)\n\n";

    std::cout << "Controls:\n";
    std::cout << "  SPACE - Toggle measurement display\n";
    std::cout << "  's'   - Save current frame\n";
    std::cout << "  ESC   - Exit\n\n";
}

int main(int argc, char** argv) {
    int cameraId = -1;

    // Parse command line arguments
    if (argc > 1) {
        std::stringstream ss(argv[1]);
        ss >> cameraId;
    }

    std::cout << "\n========================================\n";
    std::cout << "  ArUco Bird's Eye View\n";
    std::cout << "========================================\n\n";

    // Detect and select camera if not specified
    if (cameraId == -1) {
        std::vector<int> availableCameras = detectAvailableCameras();
        cameraId = selectCamera(availableCameras);

        if (cameraId == -1) {
            return -1;
        }
    }

    BirdsEyeViewer viewer;
    displayInstructions();

    // Open camera
    std::cout << "Opening Camera " << cameraId << "...\n";
    cv::VideoCapture cap(cameraId);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Cannot open camera " << cameraId << "!\n";
        return -1;
    }

    // Set camera resolution
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    int actualWidth = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int actualHeight = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    std::cout << "Camera " << cameraId << " opened successfully!\n";
    std::cout << "Resolution: " << actualWidth << "x" << actualHeight << "\n";
    std::cout << "Starting live feed...\n\n";

    // Create windows
    cv::namedWindow("Camera View", cv::WINDOW_NORMAL);
    cv::namedWindow("Bird's Eye View", cv::WINDOW_NORMAL);

    cv::moveWindow("Camera View", 50, 50);
    cv::moveWindow("Bird's Eye View", 700, 50);

    cv::Mat frame, originalOutput, birdseyeOutput;
    int frameCount = 0;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "ERROR: Cannot read frame!\n";
            break;
        }

        viewer.processFrame(frame, originalOutput, birdseyeOutput);

        if (!originalOutput.empty()) {
            cv::imshow("Camera View", originalOutput);
        }

        if (!birdseyeOutput.empty()) {
            cv::imshow("Bird's Eye View", birdseyeOutput);
        }

        int key = cv::waitKey(1);

        if (key == 27) { // ESC
            std::cout << "Exiting...\n";
            break;
        } else if (key == ' ') { // SPACE
            viewer.toggleMeasurements();
        } else if (key == 's' || key == 'S') { // Save
            if (!birdseyeOutput.empty()) {
                std::string filename = "birdseye_" + std::to_string(frameCount) + ".jpg";
                cv::imwrite(filename, birdseyeOutput);
                std::cout << "Saved bird's-eye view: " << filename << "\n";
            }
            if (!originalOutput.empty()) {
                std::string filename = "camera_" + std::to_string(frameCount) + ".jpg";
                cv::imwrite(filename, originalOutput);
                std::cout << "Saved camera view: " << filename << "\n";
            }
            frameCount++;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
