/**
 * ChArUco Camera Calibration Tool
 *
 * This tool calibrates the camera using a ChArUco calibration board.
 * The calibration compensates for lens distortion and provides accurate
 * parameters for perspective correction in the main shape recognition system.
 *
 * Usage:
 *   ./calibrate_camera_charuco [camera_id] [output_calibration_file.yml]
 *
 * Arguments:
 *   camera_id - Camera index (0, 1, 2, ...) - optional, will prompt if not provided
 *   output_file - Output calibration file path (default: camera_calibration.yml)
 *
 * Controls:
 *   SPACE - Capture current frame for calibration
 *   'c'   - Perform calibration with captured frames
 *   ESC   - Exit
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

// ChArUco Board Configuration (based on actual printed measurements)
const int SQUARES_X = 7;
const int SQUARES_Y = 5;
const float SQUARE_LENGTH = 0.0207f;  // 2.07 cm in meters (actual printed size)
const float MARKER_LENGTH = 0.014f;   // 1.4 cm in meters (actual printed size)
const int ARUCO_DICT = cv::aruco::DICT_4X4_50;

// Calibration parameters
const int MIN_FRAMES_FOR_CALIBRATION = 15;
const int MAX_FRAMES_FOR_CALIBRATION = 50;

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

class CharucoCalibrator {
private:
    cv::aruco::Dictionary dictionaryData;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::CharucoBoard> board;
    std::vector<std::vector<cv::Point2f>> allCharucoCorners;
    std::vector<std::vector<int>> allCharucoIds;
    cv::Size imageSize;

public:
    CharucoCalibrator() {
        // Initialize ArUco dictionary and ChArUco board
        dictionaryData = cv::aruco::getPredefinedDictionary(ARUCO_DICT);
        dictionary = cv::makePtr<cv::aruco::Dictionary>(dictionaryData);

        // Create CharucoBoard manually on heap
        board = cv::Ptr<cv::aruco::CharucoBoard>(
            new cv::aruco::CharucoBoard(
                cv::Size(SQUARES_X, SQUARES_Y),
                SQUARE_LENGTH, MARKER_LENGTH,
                dictionaryData
            )
        );

        std::cout << "\n========================================\n";
        std::cout << "  ChArUco Camera Calibration Tool\n";
        std::cout << "========================================\n";
        std::cout << "Board Configuration:\n";
        std::cout << "  Squares: " << SQUARES_X << " x " << SQUARES_Y << "\n";
        std::cout << "  Square size: " << SQUARE_LENGTH * 100 << " cm\n";
        std::cout << "  Marker size: " << MARKER_LENGTH * 100 << " cm\n";
        std::cout << "  ArUco Dict: DICT_4X4_50\n";
        std::cout << "========================================\n\n";
    }

    bool detectCharucoCorners(const cv::Mat& frame, cv::Mat& outputFrame) {
        outputFrame = frame.clone();

        // Detect ArUco markers
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::makePtr<cv::aruco::DetectorParameters>();

        // Optimize detection parameters for better accuracy
        parameters->adaptiveThreshWinSizeMin = 3;
        parameters->adaptiveThreshWinSizeMax = 23;
        parameters->adaptiveThreshWinSizeStep = 10;
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters);

        // If at least one marker detected
        if (markerIds.size() > 0) {
            cv::aruco::drawDetectedMarkers(outputFrame, markerCorners, markerIds);

            // Interpolate ChArUco corners
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco::interpolateCornersCharuco(
                markerCorners, markerIds, frame, board,
                charucoCorners, charucoIds
            );

            // Draw detected ChArUco corners
            if (charucoIds.size() > 0) {
                cv::aruco::drawDetectedCornersCharuco(outputFrame, charucoCorners, charucoIds);

                // Display detection info
                std::string info = "Detected: " + std::to_string(charucoIds.size()) + " corners";
                cv::putText(outputFrame, info, cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                return true;
            }
        }

        // No detection
        cv::putText(outputFrame, "No ChArUco board detected", cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        return false;
    }

    bool captureCalibrationFrame(const cv::Mat& frame) {
        if (allCharucoCorners.size() >= MAX_FRAMES_FOR_CALIBRATION) {
            std::cout << "Maximum frames (" << MAX_FRAMES_FOR_CALIBRATION << ") reached!\n";
            return false;
        }

        // Detect ArUco markers
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::makePtr<cv::aruco::DetectorParameters>();
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters);

        if (markerIds.size() > 0) {
            // Interpolate ChArUco corners
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco::interpolateCornersCharuco(
                markerCorners, markerIds, frame, board,
                charucoCorners, charucoIds
            );

            // Need at least 4 corners for good calibration
            if (charucoIds.size() >= 4) {
                allCharucoCorners.push_back(charucoCorners);
                allCharucoIds.push_back(charucoIds);
                imageSize = frame.size();

                std::cout << "Frame " << allCharucoCorners.size() << " captured ("
                         << charucoIds.size() << " corners detected)\n";
                return true;
            } else {
                std::cout << "Not enough corners detected. Try different angle/distance.\n";
                return false;
            }
        }

        std::cout << "No ChArUco board detected in frame!\n";
        return false;
    }

    bool calibrate(cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                   std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
                   double& reprojectionError) {

        if (allCharucoCorners.size() < MIN_FRAMES_FOR_CALIBRATION) {
            std::cout << "ERROR: Need at least " << MIN_FRAMES_FOR_CALIBRATION
                     << " frames, but only " << allCharucoCorners.size() << " captured.\n";
            return false;
        }

        std::cout << "\n========================================\n";
        std::cout << "  Starting Calibration...\n";
        std::cout << "========================================\n";
        std::cout << "Using " << allCharucoCorners.size() << " frames\n";
        std::cout << "Image size: " << imageSize.width << " x " << imageSize.height << "\n";
        std::cout << "Calibrating... ";
        std::cout.flush();

        // Calibrate camera
        reprojectionError = cv::aruco::calibrateCameraCharuco(
            allCharucoCorners, allCharucoIds, board, imageSize,
            cameraMatrix, distCoeffs, rvecs, tvecs,
            cv::CALIB_RATIONAL_MODEL  // Use more accurate distortion model
        );

        std::cout << "Done!\n";
        std::cout << "========================================\n\n";

        return true;
    }

    void printCalibrationResults(const cv::Mat& cameraMatrix,
                                 const cv::Mat& distCoeffs,
                                 double reprojectionError) {
        std::cout << "========================================\n";
        std::cout << "  Calibration Results\n";
        std::cout << "========================================\n";
        std::cout << "Reprojection Error: " << reprojectionError << " pixels\n";
        std::cout << "(Lower is better, <0.5 is excellent)\n\n";

        std::cout << "Camera Matrix (Intrinsics):\n";
        std::cout << "  fx: " << cameraMatrix.at<double>(0, 0) << "\n";
        std::cout << "  fy: " << cameraMatrix.at<double>(1, 1) << "\n";
        std::cout << "  cx: " << cameraMatrix.at<double>(0, 2) << "\n";
        std::cout << "  cy: " << cameraMatrix.at<double>(1, 2) << "\n\n";

        std::cout << "Distortion Coefficients:\n";
        std::cout << "  k1: " << distCoeffs.at<double>(0) << "\n";
        std::cout << "  k2: " << distCoeffs.at<double>(1) << "\n";
        std::cout << "  p1: " << distCoeffs.at<double>(2) << "\n";
        std::cout << "  p2: " << distCoeffs.at<double>(3) << "\n";
        std::cout << "  k3: " << distCoeffs.at<double>(4) << "\n";

        if (distCoeffs.rows >= 8) {
            std::cout << "  k4: " << distCoeffs.at<double>(5) << "\n";
            std::cout << "  k5: " << distCoeffs.at<double>(6) << "\n";
            std::cout << "  k6: " << distCoeffs.at<double>(7) << "\n";
        }
        std::cout << "========================================\n\n";
    }

    bool saveCalibration(const std::string& filename,
                        const cv::Mat& cameraMatrix,
                        const cv::Mat& distCoeffs,
                        const cv::Size& imageSize,
                        double reprojectionError) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);

        if (!fs.isOpened()) {
            std::cerr << "ERROR: Cannot open file for writing: " << filename << "\n";
            return false;
        }

        // Get current timestamp
        time_t now = time(0);
        char timestamp[100];
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", localtime(&now));

        // Write calibration data
        fs << "calibration_date" << timestamp;
        fs << "image_width" << imageSize.width;
        fs << "image_height" << imageSize.height;
        fs << "board_width" << SQUARES_X;
        fs << "board_height" << SQUARES_Y;
        fs << "square_size" << SQUARE_LENGTH;
        fs << "marker_size" << MARKER_LENGTH;
        fs << "frames_used" << (int)allCharucoCorners.size();
        fs << "reprojection_error" << reprojectionError;
        fs << "camera_matrix" << cameraMatrix;
        fs << "distortion_coefficients" << distCoeffs;

        fs.release();

        std::cout << "Calibration saved to: " << filename << "\n";
        return true;
    }

    int getCapturedFramesCount() const {
        return allCharucoCorners.size();
    }
};

void displayInstructions() {
    std::cout << "Instructions:\n";
    std::cout << "  1. Hold the printed ChArUco board in front of the camera\n";
    std::cout << "  2. Press SPACE to capture frames from different angles/distances\n";
    std::cout << "  3. Capture at least " << MIN_FRAMES_FOR_CALIBRATION << " frames\n";
    std::cout << "  4. Press 'c' to calibrate when ready\n";
    std::cout << "  5. Press ESC to exit\n\n";

    std::cout << "Tips for best results:\n";
    std::cout << "  - Ensure good lighting (no shadows on board)\n";
    std::cout << "  - Keep board flat and stable when capturing\n";
    std::cout << "  - Capture from various angles (tilted left/right/up/down)\n";
    std::cout << "  - Capture at various distances (near, far, medium)\n";
    std::cout << "  - Cover all areas of the camera frame\n";
    std::cout << "  - Wait for clear detection (green corners) before capturing\n\n";
}

int main(int argc, char** argv) {
    int cameraId = -1;
    std::string outputFile = "camera_calibration.yml";

    // Parse command line arguments
    if (argc > 1) {
        // Try to parse first argument as camera ID
        std::stringstream ss(argv[1]);
        if (!(ss >> cameraId)) {
            // If not a number, treat as output file
            outputFile = argv[1];
            cameraId = -1;
        }
    }

    if (argc > 2) {
        // Second argument is output file
        outputFile = argv[2];
    }

    std::cout << "\n========================================\n";
    std::cout << "  ChArUco Camera Calibration\n";
    std::cout << "========================================\n\n";

    // Detect and select camera if not specified
    if (cameraId == -1) {
        std::vector<int> availableCameras = detectAvailableCameras();
        cameraId = selectCamera(availableCameras);

        if (cameraId == -1) {
            return -1;
        }
    }

    CharucoCalibrator calibrator;
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
    std::cout << "Output file: " << outputFile << "\n";
    std::cout << "Starting live feed...\n\n";

    cv::Mat frame, displayFrame;
    bool calibrationDone = false;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "ERROR: Cannot read frame from camera!\n";
            break;
        }

        // Detect and visualize ChArUco board
        calibrator.detectCharucoCorners(frame, displayFrame);

        // Display frame count
        std::string frameInfo = "Captured: " + std::to_string(calibrator.getCapturedFramesCount()) +
                               "/" + std::to_string(MIN_FRAMES_FOR_CALIBRATION) + " min";
        cv::putText(displayFrame, frameInfo, cv::Point(10, displayFrame.rows - 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

        // Display instructions
        cv::putText(displayFrame, "SPACE: Capture | C: Calibrate | ESC: Exit",
                   cv::Point(10, displayFrame.rows - 50),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        cv::imshow("ChArUco Calibration", displayFrame);

        int key = cv::waitKey(1);

        if (key == 27) { // ESC
            std::cout << "Exiting...\n";
            break;
        } else if (key == ' ') { // SPACE - Capture frame
            if (calibrator.captureCalibrationFrame(frame)) {
                // Visual feedback
                cv::Mat flash = displayFrame.clone();
                cv::rectangle(flash, cv::Point(0, 0),
                            cv::Point(flash.cols, flash.rows),
                            cv::Scalar(0, 255, 0), 20);
                cv::imshow("ChArUco Calibration", flash);
                cv::waitKey(100);
            }
        } else if (key == 'c' || key == 'C') { // Calibrate
            cv::Mat cameraMatrix, distCoeffs;
            std::vector<cv::Mat> rvecs, tvecs;
            double reprojError;

            if (calibrator.calibrate(cameraMatrix, distCoeffs, rvecs, tvecs, reprojError)) {
                calibrator.printCalibrationResults(cameraMatrix, distCoeffs, reprojError);

                cv::Size imgSize = frame.size();
                if (calibrator.saveCalibration(outputFile, cameraMatrix, distCoeffs,
                                              imgSize, reprojError)) {
                    calibrationDone = true;
                    std::cout << "\n✓ Calibration completed successfully!\n";
                    std::cout << "\nYou can now use this calibration file with:\n";
                    std::cout << "  ./test_calibration_accuracy " << outputFile << "\n\n";
                    break;
                }
            }
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return calibrationDone ? 0 : 1;
}
