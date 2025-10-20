#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <string>

/**
 * Bird's-Eye View Calibration Verification Tool
 *
 * Purpose: Verify that camera calibration and perspective correction work correctly
 *
 * What it does:
 * 1. Loads camera calibration data (camera_calibration.yml)
 * 2. Detects 4 ArUco markers on the corners of the template
 * 3. Applies undistortion (removes lens distortion)
 * 4. Applies perspective transform (converts to bird's-eye view)
 *
 * Expected result:
 * - A circle drawn on paper should appear as a PERFECT circle in bird's-eye view
 * - Even when camera is at an angle, the bird's-eye view should be straight-on
 * - No stretching, squishing, or perspective distortion
 */

struct CalibrationData {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat optimal_camera_matrix;
    cv::Size image_size;
    bool loaded = false;
};

// Camera info struct
struct CameraInfo {
    int index;
    int width;
    int height;
    int fps;
    std::string backend;
    std::string description;
};

// Scan all available cameras
std::vector<CameraInfo> scanAllCameras(int max_cameras = 10) {
    std::vector<CameraInfo> cameras;
    std::cout << "🔍 Scanning for cameras..." << std::endl;

    for (int i = 0; i < max_cameras; i++) {
        cv::VideoCapture cap(i);
        if (cap.isOpened()) {
            cv::Mat frame;
            bool ret = cap.read(frame);
            if (ret && !frame.empty()) {
                CameraInfo info;
                info.index = i;
                info.width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
                info.height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
                info.fps = cap.get(cv::CAP_PROP_FPS);
                info.backend = cap.getBackendName();

                if (info.width == 640 && info.height == 480) {
                    info.description = "Standard Webcam";
                } else if (info.width == 1280 && info.height == 720) {
                    info.description = "HD Camera";
                } else if (info.width >= 1920) {
                    info.description = "Full HD Camera";
                } else {
                    info.description = "Generic Camera";
                }
                cameras.push_back(info);
            }
            cap.release();
        }
    }
    return cameras;
}

// Let user select camera
int selectCamera() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   BIRD'S-EYE VIEW VERIFICATION - Camera Selection     ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝\n" << std::endl;

    auto cameras = scanAllCameras();
    if (cameras.empty()) {
        std::cerr << "❌ No cameras found!" << std::endl;
        return -1;
    }

    std::cout << "Available cameras:\n" << std::endl;
    for (const auto& cam : cameras) {
        std::cout << "  [" << cam.index << "] " << cam.description
                  << " (" << cam.width << "x" << cam.height << " @ " << cam.fps << " fps)"
                  << std::endl;
    }

    if (cameras.size() == 1) {
        std::cout << "\n✅ Only one camera found, auto-selected: Camera " << cameras[0].index << std::endl;
        return cameras[0].index;
    }

    std::cout << "\nEnter camera ID: ";
    int selected;
    std::cin >> selected;

    for (const auto& cam : cameras) {
        if (cam.index == selected) {
            std::cout << "✅ Camera " << selected << " selected." << std::endl;
            return selected;
        }
    }

    std::cerr << "❌ Invalid camera ID!" << std::endl;
    return -1;
}

// Load camera calibration from YML file
CalibrationData loadCalibration(const std::string& filename) {
    CalibrationData data;

    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "❌ Could not open calibration file: " << filename << std::endl;
        std::cerr << "   Make sure you ran camera_calibration first!" << std::endl;
        return data;
    }

    int width, height;
    fs["camera_matrix"] >> data.camera_matrix;
    fs["distortion_coefficients"] >> data.dist_coeffs;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    data.image_size = cv::Size(width, height);

    // Try to load optimal camera matrix (if available)
    if (!fs["optimal_camera_matrix"].empty()) {
        fs["optimal_camera_matrix"] >> data.optimal_camera_matrix;
        std::cout << "✅ Using optimal camera matrix (square pixels)" << std::endl;
    } else {
        data.optimal_camera_matrix = data.camera_matrix.clone();
        std::cout << "⚠️  Using standard camera matrix (optimal not found)" << std::endl;
    }

    fs.release();
    data.loaded = true;

    std::cout << "✅ Calibration loaded successfully" << std::endl;
    std::cout << "   Image size: " << width << "x" << height << std::endl;

    return data;
}

// ArUco marker detection
struct MarkerCorners {
    cv::Point2f top_left;
    cv::Point2f top_right;
    cv::Point2f bottom_right;
    cv::Point2f bottom_left;
    bool all_detected = false;
};

MarkerCorners detectTemplateMarkers(const cv::Mat& frame) {
    MarkerCorners corners;

    // ArUco detection setup (same as template generator)
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters params = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, params);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    detector.detectMarkers(frame, marker_corners, marker_ids);

    if (marker_ids.size() < 4) {
        return corners;  // Not all markers detected
    }

    // Extract corners for each marker (we want the CENTER of each marker)
    // Marker IDs: 0=TOP_LEFT, 1=TOP_RIGHT, 2=BOTTOM_RIGHT, 3=BOTTOM_LEFT
    for (size_t i = 0; i < marker_ids.size(); i++) {
        int id = marker_ids[i];

        // Calculate center of marker
        cv::Point2f center(0, 0);
        for (int j = 0; j < 4; j++) {
            center += marker_corners[i][j];
        }
        center *= 0.25f;

        if (id == 0) corners.top_left = center;
        else if (id == 1) corners.top_right = center;
        else if (id == 2) corners.bottom_right = center;
        else if (id == 3) corners.bottom_left = center;
    }

    // Check if all 4 corners were found
    corners.all_detected = (corners.top_left.x > 0 && corners.top_right.x > 0 &&
                           corners.bottom_right.x > 0 && corners.bottom_left.x > 0);

    return corners;
}

// Apply bird's-eye view transformation
// PRESERVE ACTUAL MARKER DISTANCES (don't force to square!)
cv::Mat applyBirdsEyeView(const cv::Mat& frame, const MarkerCorners& corners, int max_size = 800) {
    if (!corners.all_detected) {
        return cv::Mat::zeros(max_size, max_size, frame.type());
    }

    // Source points (detected marker positions)
    std::vector<cv::Point2f> src_points = {
        corners.top_left,
        corners.top_right,
        corners.bottom_right,
        corners.bottom_left
    };

    // Calculate ACTUAL distances between markers
    float width_top = cv::norm(corners.top_right - corners.top_left);
    float width_bottom = cv::norm(corners.bottom_right - corners.bottom_left);
    float height_left = cv::norm(corners.bottom_left - corners.top_left);
    float height_right = cv::norm(corners.bottom_right - corners.top_right);

    float avg_width = (width_top + width_bottom) / 2.0f;
    float avg_height = (height_left + height_right) / 2.0f;

    // Calculate scale to fit in max_size while preserving aspect ratio
    float scale = std::min(max_size / avg_width, max_size / avg_height);

    int dst_width = static_cast<int>(avg_width * scale);
    int dst_height = static_cast<int>(avg_height * scale);

    // Destination points - PRESERVE THE ASPECT RATIO!
    std::vector<cv::Point2f> dst_points = {
        cv::Point2f(0, 0),
        cv::Point2f(dst_width, 0),
        cv::Point2f(dst_width, dst_height),
        cv::Point2f(0, dst_height)
    };

    // Compute perspective transform matrix
    cv::Mat transform_matrix = cv::getPerspectiveTransform(src_points, dst_points);

    // Apply transformation
    cv::Mat warped;
    cv::warpPerspective(frame, warped, transform_matrix, cv::Size(dst_width, dst_height));

    return warped;
}

// Circle detection and diameter measurement
struct CircleMeasurement {
    cv::Point2f center;
    float horizontal_diameter;
    float vertical_diameter;
    float ratio;
    bool detected;
};

CircleMeasurement detectAndMeasureCircle(const cv::Mat& birdseye_view) {
    CircleMeasurement result;
    result.detected = false;

    if (birdseye_view.empty()) return result;

    // Convert to grayscale
    cv::Mat gray;
    if (birdseye_view.channels() == 3) {
        cv::cvtColor(birdseye_view, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = birdseye_view.clone();
    }

    // Apply binary threshold to find dark shapes (pen drawings)
    cv::Mat binary;
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return result;

    // Find the largest contour (should be the circle)
    double max_area = 0;
    int largest_idx = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area && area > 1000) {  // Minimum area threshold
            max_area = area;
            largest_idx = i;
        }
    }

    if (largest_idx < 0) return result;

    // Fit ellipse to the contour (works for both circles and ovals)
    if (contours[largest_idx].size() >= 5) {
        cv::RotatedRect ellipse = cv::fitEllipse(contours[largest_idx]);

        result.center = ellipse.center;

        // Get the two axes (major and minor)
        float axis1 = ellipse.size.width;
        float axis2 = ellipse.size.height;

        // For a rotated ellipse, we need to determine which is horizontal/vertical
        // For simplicity, we'll use the ellipse axes as-is
        result.horizontal_diameter = std::max(axis1, axis2);
        result.vertical_diameter = std::min(axis1, axis2);

        // Calculate ratio (should be 1.0 for perfect circle)
        result.ratio = result.vertical_diameter / result.horizontal_diameter;

        result.detected = true;
    }

    return result;
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     BIRD'S-EYE VIEW CALIBRATION VERIFICATION          ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝\n" << std::endl;

    // Load calibration
    std::cout << "📂 Loading camera calibration..." << std::endl;
    CalibrationData calib = loadCalibration("camera_calibration.yml");

    if (!calib.loaded) {
        std::cerr << "\n❌ Failed to load calibration!" << std::endl;
        std::cerr << "   Please run camera_calibration first." << std::endl;
        return -1;
    }

    // Select camera
    int camera_index = selectCamera();
    if (camera_index < 0) return -1;

    // Open camera
    std::cout << "\n🎥 Opening camera " << camera_index << "..." << std::endl;
    cv::VideoCapture cap(camera_index);

    if (!cap.isOpened()) {
        std::cerr << "❌ Cannot open camera!" << std::endl;
        return -1;
    }

    std::cout << "✅ Camera opened successfully\n" << std::endl;

    std::cout << "╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                    INSTRUCTIONS                        ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║  1. Place your printed template with the circle       ║" << std::endl;
    std::cout << "║     in front of the camera                             ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  2. Try different angles:                              ║" << std::endl;
    std::cout << "║     • Tilt the paper left/right                       ║" << std::endl;
    std::cout << "║     • Tilt the paper up/down                          ║" << std::endl;
    std::cout << "║     • View from different distances                    ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  3. Observe the result:                                ║" << std::endl;
    std::cout << "║     LEFT window  = Original distorted view             ║" << std::endl;
    std::cout << "║     RIGHT window = Corrected bird's-eye view          ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  ✅ SUCCESS: Circle looks perfectly round in RIGHT     ║" << std::endl;
    std::cout << "║  ❌ FAIL: Circle is stretched/squished in RIGHT        ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  Press 'q' to quit                                     ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝\n" << std::endl;

    std::cout << "▶️  Starting verification...\n" << std::endl;

    const int OUTPUT_SIZE = 800;  // Size of bird's-eye view output

    while (true) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) break;

        // Step 1: Undistort using calibration
        cv::Mat undistorted;
        cv::undistort(frame, undistorted, calib.camera_matrix, calib.dist_coeffs,
                     calib.optimal_camera_matrix);

        // Step 2: Detect ArUco markers
        MarkerCorners corners = detectTemplateMarkers(undistorted);

        // Step 3: Apply bird's-eye view transformation
        cv::Mat birdseye;
        if (corners.all_detected) {
            birdseye = applyBirdsEyeView(undistorted, corners, OUTPUT_SIZE);

            // Draw corner markers on original for visualization
            cv::circle(undistorted, corners.top_left, 8, cv::Scalar(0, 255, 0), -1);
            cv::circle(undistorted, corners.top_right, 8, cv::Scalar(0, 255, 0), -1);
            cv::circle(undistorted, corners.bottom_right, 8, cv::Scalar(0, 255, 0), -1);
            cv::circle(undistorted, corners.bottom_left, 8, cv::Scalar(0, 255, 0), -1);

            // Draw lines between corners
            cv::line(undistorted, corners.top_left, corners.top_right, cv::Scalar(0, 255, 0), 2);
            cv::line(undistorted, corners.top_right, corners.bottom_right, cv::Scalar(0, 255, 0), 2);
            cv::line(undistorted, corners.bottom_right, corners.bottom_left, cv::Scalar(0, 255, 0), 2);
            cv::line(undistorted, corners.bottom_left, corners.top_left, cv::Scalar(0, 255, 0), 2);

            // Status overlay
            cv::putText(undistorted, "MARKERS DETECTED", cv::Point(20, 40),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

            // Step 4: Detect and measure circle
            CircleMeasurement circle = detectAndMeasureCircle(birdseye);

            if (circle.detected) {
                // Draw the detected ellipse
                cv::RotatedRect ellipse_rect(
                    circle.center,
                    cv::Size2f(circle.horizontal_diameter, circle.vertical_diameter),
                    0
                );
                cv::ellipse(birdseye, ellipse_rect, cv::Scalar(0, 255, 0), 2);

                // Draw center point
                cv::circle(birdseye, circle.center, 5, cv::Scalar(255, 0, 0), -1);

                // Draw horizontal diameter (red)
                float half_h = circle.horizontal_diameter / 2.0f;
                cv::Point2f h_start(circle.center.x - half_h, circle.center.y);
                cv::Point2f h_end(circle.center.x + half_h, circle.center.y);
                cv::line(birdseye, h_start, h_end, cv::Scalar(0, 0, 255), 3);
                cv::putText(birdseye, "H", cv::Point(h_end.x + 10, h_end.y),
                           cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

                // Draw vertical diameter (blue)
                float half_v = circle.vertical_diameter / 2.0f;
                cv::Point2f v_start(circle.center.x, circle.center.y - half_v);
                cv::Point2f v_end(circle.center.x, circle.center.y + half_v);
                cv::line(birdseye, v_start, v_end, cv::Scalar(255, 0, 0), 3);
                cv::putText(birdseye, "V", cv::Point(v_end.x + 10, v_end.y),
                           cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);

                // Display measurements (top-left info box)
                cv::rectangle(birdseye, cv::Point(10, 10), cv::Point(380, 180),
                             cv::Scalar(0, 0, 0), -1);
                cv::rectangle(birdseye, cv::Point(10, 10), cv::Point(380, 180),
                             cv::Scalar(255, 255, 255), 2);

                int y_pos = 40;
                cv::putText(birdseye, "CIRCLE MEASUREMENT:", cv::Point(20, y_pos),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

                y_pos += 30;
                char h_text[100];
                snprintf(h_text, sizeof(h_text), "H-Diameter: %.1f px", circle.horizontal_diameter);
                cv::putText(birdseye, h_text, cv::Point(20, y_pos),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

                y_pos += 30;
                char v_text[100];
                snprintf(v_text, sizeof(v_text), "V-Diameter: %.1f px", circle.vertical_diameter);
                cv::putText(birdseye, v_text, cv::Point(20, y_pos),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

                y_pos += 30;
                char ratio_text[100];
                snprintf(ratio_text, sizeof(ratio_text), "Ratio (V/H): %.3f", circle.ratio);
                cv::Scalar ratio_color;
                if (circle.ratio > 0.98 && circle.ratio < 1.02) {
                    ratio_color = cv::Scalar(0, 255, 0);  // Green = perfect
                } else if (circle.ratio > 0.95 && circle.ratio < 1.05) {
                    ratio_color = cv::Scalar(0, 255, 255);  // Yellow = good
                } else {
                    ratio_color = cv::Scalar(0, 0, 255);  // Red = needs work
                }
                cv::putText(birdseye, ratio_text, cv::Point(20, y_pos),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, ratio_color, 2);

                y_pos += 30;
                std::string quality_text;
                if (circle.ratio > 0.98 && circle.ratio < 1.02) {
                    quality_text = "Quality: PERFECT!";
                } else if (circle.ratio > 0.95 && circle.ratio < 1.05) {
                    quality_text = "Quality: GOOD";
                } else if (circle.ratio > 0.90 && circle.ratio < 1.10) {
                    quality_text = "Quality: ACCEPTABLE";
                } else {
                    quality_text = "Quality: NEEDS RECALIB";
                }
                cv::putText(birdseye, quality_text, cv::Point(20, y_pos),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, ratio_color, 2);

            } else {
                // Circle not detected
                cv::putText(birdseye, "Circle NOT detected", cv::Point(20, 40),
                           cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
                cv::putText(birdseye, "Make sure circle is drawn", cv::Point(20, 80),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }

        } else {
            // No markers detected - create placeholder
            birdseye = cv::Mat::zeros(OUTPUT_SIZE, OUTPUT_SIZE, undistorted.type());

            cv::putText(undistorted, "NO MARKERS DETECTED", cv::Point(20, 40),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            cv::putText(undistorted, "Show all 4 corners to camera", cv::Point(20, 80),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

            cv::putText(birdseye, "WAITING FOR MARKERS...", cv::Point(150, OUTPUT_SIZE/2),
                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        }

        // Resize both to same height for side-by-side comparison
        int display_height = OUTPUT_SIZE;

        // Resize original
        cv::Mat undistorted_resized;
        cv::resize(undistorted, undistorted_resized, cv::Size(OUTPUT_SIZE, display_height));

        // Resize birdseye to same height (maintain aspect ratio)
        cv::Mat birdseye_resized;
        if (!birdseye.empty() && birdseye.rows > 0) {
            float bird_aspect = static_cast<float>(birdseye.cols) / birdseye.rows;
            int bird_width = static_cast<int>(display_height * bird_aspect);
            cv::resize(birdseye, birdseye_resized, cv::Size(bird_width, display_height));
        } else {
            birdseye_resized = cv::Mat::zeros(display_height, OUTPUT_SIZE, undistorted.type());
        }

        // Create side-by-side comparison
        cv::Mat comparison;
        cv::hconcat(undistorted_resized, birdseye_resized, comparison);

        // Add labels
        cv::putText(comparison, "ORIGINAL (Undistorted)", cv::Point(20, display_height - 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
        cv::putText(comparison, "BIRD'S-EYE VIEW (Corrected)", cv::Point(OUTPUT_SIZE + 20, display_height - 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        // Add instruction
        cv::putText(comparison, "Press 'q' to quit", cv::Point(comparison.cols/2 - 100, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

        cv::imshow("Calibration Verification - Compare Views", comparison);

        char key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            break;
        }
    }

    std::cout << "\n╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                  VERIFICATION GUIDE                    ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║  ✅ CALIBRATION WORKING if you saw:                    ║" << std::endl;
    std::cout << "║     • Circle appears perfectly round in bird's-eye     ║" << std::endl;
    std::cout << "║     • No distortion at edges of bird's-eye view        ║" << std::endl;
    std::cout << "║     • Square template appears perfectly square         ║" << std::endl;
    std::cout << "║     • Straight lines remain straight                   ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  ❌ RECALIBRATE NEEDED if you saw:                     ║" << std::endl;
    std::cout << "║     • Circle is oval or stretched                      ║" << std::endl;
    std::cout << "║     • Lines are curved or bent                         ║" << std::endl;
    std::cout << "║     • Corners are distorted                            ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  Next step: Use this in your shape recognition!       ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝" << std::endl;

    cap.release();
    cv::destroyAllWindows();

    return 0;
}