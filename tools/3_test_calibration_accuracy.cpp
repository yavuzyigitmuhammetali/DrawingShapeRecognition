/**
 * Calibration Accuracy Test Tool
 *
 * This tool tests camera calibration accuracy by:
 * 1. Detecting ArUco markers on a test paper
 * 2. Applying undistortion using calibration data
 * 3. Creating bird's-eye view transformation
 * 4. Detecting a perfect circle drawn on the paper
 * 5. Measuring two perpendicular diameters and comparing their ratio
 *
 * A perfect circle should have a diameter ratio of ~1.0
 * If calibration is accurate, the transformed circle will remain circular.
 *
 * Usage:
 *   ./test_calibration_accuracy [camera_id] [calibration_file.yml]
 *
 * Arguments:
 *   camera_id - Camera index (0, 1, 2, ...) - optional, will prompt if not provided
 *   calibration_file - Calibration file path (default: camera_calibration.yml)
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

// ArUco configuration (matching the template)
const int ARUCO_DICT = cv::aruco::DICT_4X4_50;
const std::vector<int> CORNER_MARKER_IDS = {0, 1, 2, 3}; // TL, TR, BR, BL

// Physical dimensions (based on actual measurements)
const float A4_WIDTH_CM = 21.0f;
const float A4_HEIGHT_CM = 29.7f;
const float MARKER_SIZE_CM = 1.8f;  // Actual printed ArUco marker size (1.8cm x 1.8cm)

// Measured dimensions of the rectangle formed by 4 corner markers
// (outermost points of the 4 markers)
const float MARKERS_RECT_WIDTH_CM = 13.7f;   // Horizontal span of 4 markers
const float MARKERS_RECT_HEIGHT_CM = 22.45f; // Vertical span of 4 markers

// Calculate marker positions on A4 paper
// Center the markers rectangle on the A4 paper
const float MARKER_OFFSET_X_CM = (A4_WIDTH_CM - MARKERS_RECT_WIDTH_CM) / 2.0f;
const float MARKER_OFFSET_Y_CM = (A4_HEIGHT_CM - MARKERS_RECT_HEIGHT_CM) / 2.0f;

// Camera selection functions
std::vector<int> detectAvailableCameras(int maxCamerasToCheck = 10) {
    std::vector<int> availableCameras;

    std::cout << "Scanning for available cameras...\n";

    for (int i = 0; i < maxCamerasToCheck; i++) {
        cv::VideoCapture cap(i);
        if (cap.isOpened()) {
            // Try to read a frame to confirm it's working
            cv::Mat frame;
            cap >> frame;

            if (!frame.empty()) {
                availableCameras.push_back(i);

                // Get camera properties
                int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
                int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

                std::cout << "  Camera " << i << ": "
                         << width << "x" << height << " ";

                // Try to get camera backend info
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

class CalibrationTester {
private:
    cv::aruco::Dictionary dictionaryData;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Size imageSize;
    bool calibrationLoaded;
    bool showMeasurements;

    // Calibration statistics
    double reprojectionError;
    int framesUsed;

public:
    CalibrationTester() : calibrationLoaded(false), showMeasurements(true) {
        dictionaryData = cv::aruco::getPredefinedDictionary(ARUCO_DICT);
        dictionary = cv::makePtr<cv::aruco::Dictionary>(dictionaryData);

        std::cout << "\n========================================\n";
        std::cout << "  Calibration Accuracy Test Tool\n";
        std::cout << "========================================\n\n";
    }

    bool loadCalibration(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);

        if (!fs.isOpened()) {
            std::cerr << "ERROR: Cannot open calibration file: " << filename << "\n";
            return false;
        }

        fs["camera_matrix"] >> cameraMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        fs["image_width"] >> imageSize.width;
        fs["image_height"] >> imageSize.height;
        fs["reprojection_error"] >> reprojectionError;
        fs["frames_used"] >> framesUsed;

        fs.release();

        if (cameraMatrix.empty() || distCoeffs.empty()) {
            std::cerr << "ERROR: Invalid calibration data in file!\n";
            return false;
        }

        calibrationLoaded = true;

        std::cout << "Calibration loaded successfully:\n";
        std::cout << "  File: " << filename << "\n";
        std::cout << "  Image size: " << imageSize.width << " x " << imageSize.height << "\n";
        std::cout << "  Reprojection error: " << reprojectionError << " pixels\n";
        std::cout << "  Frames used: " << framesUsed << "\n";
        std::cout << "  Camera fx: " << cameraMatrix.at<double>(0, 0) << "\n";
        std::cout << "  Camera fy: " << cameraMatrix.at<double>(1, 1) << "\n";
        std::cout << "========================================\n\n";

        return true;
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

        // Find the 4 corner markers
        std::map<int, cv::Point2f> markerCenters;

        for (size_t i = 0; i < markerIds.size(); i++) {
            int id = markerIds[i];

            // Check if this is a corner marker
            if (std::find(CORNER_MARKER_IDS.begin(), CORNER_MARKER_IDS.end(), id)
                != CORNER_MARKER_IDS.end()) {

                // Calculate marker center
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

        // Extract in order: TL, TR, BR, BL
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

        // Define destination points for A4 paper
        std::vector<cv::Point2f> dstPoints;
        dstPoints.push_back(cv::Point2f(0, 0));                              // TL
        dstPoints.push_back(cv::Point2f(outputWidth - 1, 0));               // TR
        dstPoints.push_back(cv::Point2f(outputWidth - 1, outputHeight - 1)); // BR
        dstPoints.push_back(cv::Point2f(0, outputHeight - 1));              // BL

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

        // Apply Gaussian blur to reduce noise
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.5);

        // Adaptive threshold for robust detection
        cv::Mat binary;
        cv::adaptiveThreshold(gray, binary, 255,
                            cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                            cv::THRESH_BINARY_INV, 15, 2);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Find the largest circular contour
        double maxArea = 0;
        int bestIdx = -1;

        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);

            // Filter by area (circle should be reasonably sized)
            if (area < 1000 || area > birdseye.rows * birdseye.cols * 0.5) {
                continue;
            }

            // Check circularity
            double perimeter = cv::arcLength(contours[i], true);
            double circularity = 4 * M_PI * area / (perimeter * perimeter);

            // Perfect circle has circularity = 1.0
            if (circularity > 0.7 && area > maxArea) {
                maxArea = area;
                bestIdx = i;
            }
        }

        if (bestIdx == -1) {
            return false;
        }

        // Get circle parameters
        contour = contours[bestIdx];
        cv::minEnclosingCircle(contour, center, radius);

        return true;
    }

    void measurePerpendicularDiameters(const std::vector<cv::Point>& contour,
                                      const cv::Point2f& center,
                                      float& diameter1,
                                      float& diameter2,
                                      float& angle1,
                                      cv::Point2f& p1a, cv::Point2f& p1b,
                                      cv::Point2f& p2a, cv::Point2f& p2b) {

        // Fit ellipse to get orientation
        cv::RotatedRect ellipse = cv::fitEllipse(contour);

        // Primary axis angle
        angle1 = ellipse.angle * M_PI / 180.0;
        float angle2 = angle1 + M_PI / 2.0; // Perpendicular

        // Major axis
        float majorAxis = std::max(ellipse.size.width, ellipse.size.height);

        // Calculate endpoints for both diameters
        float length = majorAxis * 1.2; // Extend beyond contour

        // Diameter 1 (along major axis orientation)
        p1a = cv::Point2f(
            center.x + length * cos(angle1),
            center.y + length * sin(angle1)
        );
        p1b = cv::Point2f(
            center.x - length * cos(angle1),
            center.y - length * sin(angle1)
        );

        // Diameter 2 (perpendicular)
        p2a = cv::Point2f(
            center.x + length * cos(angle2),
            center.y + length * sin(angle2)
        );
        p2b = cv::Point2f(
            center.x - length * cos(angle2),
            center.y - length * sin(angle2)
        );

        // Find actual intersections with contour
        auto findIntersections = [&contour](cv::Point2f center, float angle) -> std::pair<float, float> {
            float maxDist1 = 0, maxDist2 = 0;

            for (const auto& pt : contour) {
                cv::Point2f vec(pt.x - center.x, pt.y - center.y);
                float dotProduct = vec.x * cos(angle) + vec.y * sin(angle);
                float dist = sqrt(vec.x * vec.x + vec.y * vec.y);

                if (dotProduct > 0 && dist > maxDist1) {
                    maxDist1 = dist;
                } else if (dotProduct < 0 && dist > maxDist2) {
                    maxDist2 = dist;
                }
            }

            return {maxDist1, maxDist2};
        };

        auto [d1a, d1b] = findIntersections(center, angle1);
        auto [d2a, d2b] = findIntersections(center, angle2);

        diameter1 = d1a + d1b;
        diameter2 = d2a + d2b;

        // Update endpoints to actual intersection points
        p1a = cv::Point2f(center.x + d1a * cos(angle1), center.y + d1a * sin(angle1));
        p1b = cv::Point2f(center.x - d1b * cos(angle1), center.y - d1b * sin(angle1));
        p2a = cv::Point2f(center.x + d2a * cos(angle2), center.y + d2a * sin(angle2));
        p2b = cv::Point2f(center.x - d2b * cos(angle2), center.y - d2b * sin(angle2));
    }

    void drawMeasurements(cv::Mat& output,
                         const cv::Point2f& center,
                         float radius,
                         float diameter1,
                         float diameter2,
                         const cv::Point2f& p1a,
                         const cv::Point2f& p1b,
                         const cv::Point2f& p2a,
                         const cv::Point2f& p2b) {

        // Draw circle
        cv::circle(output, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
        cv::circle(output, center, 5, cv::Scalar(0, 255, 0), -1);

        if (showMeasurements) {
            // Draw diameter 1 (red)
            cv::line(output, p1a, p1b, cv::Scalar(0, 0, 255), 2);
            cv::circle(output, p1a, 5, cv::Scalar(0, 0, 255), -1);
            cv::circle(output, p1b, 5, cv::Scalar(0, 0, 255), -1);

            // Draw diameter 2 (blue)
            cv::line(output, p2a, p2b, cv::Scalar(255, 0, 0), 2);
            cv::circle(output, p2a, 5, cv::Scalar(255, 0, 0), -1);
            cv::circle(output, p2b, 5, cv::Scalar(255, 0, 0), -1);

            // Calculate ratio
            float ratio = diameter1 / diameter2;
            float error = std::abs(1.0f - ratio) * 100.0f; // Percentage error

            // Display measurements
            int y = 30;
            cv::putText(output, "Calibration Test Results:",
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
            else quality = "Poor - Recalibrate";

            cv::putText(output, "Quality: " + quality, cv::Point(10, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, ratioColor, 2);
        }
    }

    void processFrame(const cv::Mat& frame, cv::Mat& output) {
        if (!calibrationLoaded) {
            output = frame.clone();
            cv::putText(output, "ERROR: No calibration loaded!",
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       cv::Scalar(0, 0, 255), 2);
            return;
        }

        // Undistort frame
        cv::Mat undistorted;
        cv::undistort(frame, undistorted, cameraMatrix, distCoeffs);

        // Detect ArUco markers
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;

        if (!detectMarkers(undistorted, markerIds, markerCorners)) {
            output = undistorted.clone();
            cv::putText(output, "Searching for ArUco markers...",
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       cv::Scalar(0, 165, 255), 2);
            return;
        }

        // Draw detected markers
        output = undistorted.clone();
        cv::aruco::drawDetectedMarkers(output, markerCorners, markerIds);

        // Extract paper corners
        std::vector<cv::Point2f> paperCorners;
        if (!extractPaperCorners(markerIds, markerCorners, paperCorners)) {
            cv::putText(output, "Need all 4 corner markers (IDs: 0,1,2,3)",
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       cv::Scalar(0, 165, 255), 2);
            return;
        }

        // Apply bird's-eye view transformation
        cv::Mat birdseye = applyBirdsEyeView(undistorted, paperCorners);

        // Detect circle
        cv::Point2f center;
        float radius;
        std::vector<cv::Point> contour;

        if (!detectCircle(birdseye, center, radius, contour)) {
            output = birdseye.clone();
            cv::putText(output, "No circle detected - draw a circle on paper!",
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       cv::Scalar(0, 0, 255), 2);
            return;
        }

        // Measure perpendicular diameters
        float diameter1, diameter2, angle1;
        cv::Point2f p1a, p1b, p2a, p2b;
        measurePerpendicularDiameters(contour, center, diameter1, diameter2, angle1,
                                     p1a, p1b, p2a, p2b);

        // Draw measurements
        output = birdseye.clone();
        drawMeasurements(output, center, radius, diameter1, diameter2, p1a, p1b, p2a, p2b);
    }

    void toggleMeasurements() {
        showMeasurements = !showMeasurements;
    }
};

void displayInstructions() {
    std::cout << "Instructions:\n";
    std::cout << "  1. Place the printed test paper with ArUco markers in view\n";
    std::cout << "  2. Ensure all 4 corner markers are visible\n";
    std::cout << "  3. The tool will detect the circle and measure its diameters\n";
    std::cout << "  4. Ratio close to 1.0 indicates good calibration\n\n";

    std::cout << "Controls:\n";
    std::cout << "  SPACE - Toggle measurement display\n";
    std::cout << "  's'   - Save current frame\n";
    std::cout << "  ESC   - Exit\n\n";
}

int main(int argc, char** argv) {
    int cameraId = -1;
    std::string calibrationFile = "camera_calibration.yml";

    // Parse command line arguments
    if (argc > 1) {
        // Try to parse first argument as camera ID
        std::stringstream ss(argv[1]);
        if (!(ss >> cameraId)) {
            // If not a number, treat as calibration file
            calibrationFile = argv[1];
            cameraId = -1;
        }
    }

    if (argc > 2) {
        // Second argument is calibration file
        calibrationFile = argv[2];
    }

    std::cout << "\n========================================\n";
    std::cout << "  Calibration Accuracy Test\n";
    std::cout << "========================================\n\n";

    // Detect and select camera if not specified
    if (cameraId == -1) {
        std::vector<int> availableCameras = detectAvailableCameras();
        cameraId = selectCamera(availableCameras);

        if (cameraId == -1) {
            return -1;
        }
    }

    CalibrationTester tester;

    if (!tester.loadCalibration(calibrationFile)) {
        std::cerr << "Failed to load calibration. Please run calibration first:\n";
        std::cerr << "  ./calibrate_camera_charuco\n";
        return -1;
    }

    displayInstructions();

    // Open selected camera
    std::cout << "Opening Camera " << cameraId << "...\n";
    cv::VideoCapture cap(cameraId);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Cannot open camera " << cameraId << "!\n";
        return -1;
    }

    // Set camera resolution
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    // Get actual resolution
    int actualWidth = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int actualHeight = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    std::cout << "Camera " << cameraId << " opened successfully!\n";
    std::cout << "Resolution: " << actualWidth << "x" << actualHeight << "\n";
    std::cout << "Starting test...\n\n";

    cv::Mat frame, output;
    int frameCount = 0;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "ERROR: Cannot read frame!\n";
            break;
        }

        tester.processFrame(frame, output);

        cv::imshow("Calibration Test", output);

        int key = cv::waitKey(1);

        if (key == 27) { // ESC
            std::cout << "Exiting...\n";
            break;
        } else if (key == ' ') { // SPACE
            tester.toggleMeasurements();
        } else if (key == 's' || key == 'S') { // Save
            std::string filename = "calibration_test_" + std::to_string(frameCount++) + ".jpg";
            cv::imwrite(filename, output);
            std::cout << "Saved: " << filename << "\n";
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
