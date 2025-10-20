#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <string>

/**
 * Interactive Camera Calibration Adjuster
 *
 * Allows manual fine-tuning of camera calibration parameters in real-time
 * to achieve perfect bird's-eye view circle correction.
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
    std::cout << "║     INTERACTIVE CALIBRATION - Camera Selection        ║" << std::endl;
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

// Global variables for trackbars
struct CalibrationParams {
    int fx;           // Focal length X (x10 for precision)
    int fy;           // Focal length Y (x10 for precision)
    int cx;           // Principal point X
    int cy;           // Principal point Y
    int k1;           // Radial distortion k1 (x1000 for precision)
    int k2;           // Radial distortion k2 (x1000 for precision)
    int k3;           // Radial distortion k3 (x1000 for precision)
    int p1;           // Tangential distortion p1 (x1000 for precision)
    int p2;           // Tangential distortion p2 (x1000 for precision)
    int use_square_pixels; // 0 or 1: force fx=fy for square pixels
};

CalibrationParams g_params;
bool g_params_changed = false;

// Trackbar callback
void onTrackbarChange(int, void*) {
    g_params_changed = true;
}

// Load initial calibration
CalibrationData loadCalibration(const std::string& filename) {
    CalibrationData data;

    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "❌ Could not open calibration file: " << filename << std::endl;
        return data;
    }

    int width, height;
    fs["camera_matrix"] >> data.camera_matrix;
    fs["distortion_coefficients"] >> data.dist_coeffs;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    data.image_size = cv::Size(width, height);

    if (!fs["optimal_camera_matrix"].empty()) {
        fs["optimal_camera_matrix"] >> data.optimal_camera_matrix;
    } else {
        data.optimal_camera_matrix = data.camera_matrix.clone();
    }

    fs.release();
    data.loaded = true;

    std::cout << "✅ Calibration loaded from " << filename << std::endl;

    return data;
}

// Initialize trackbar values from calibration
void initializeParams(const CalibrationData& calib) {
    double fx = calib.optimal_camera_matrix.at<double>(0, 0);
    double fy = calib.optimal_camera_matrix.at<double>(1, 1);
    double cx = calib.optimal_camera_matrix.at<double>(0, 2);
    double cy = calib.optimal_camera_matrix.at<double>(1, 2);

    g_params.fx = static_cast<int>(fx * 10);
    g_params.fy = static_cast<int>(fy * 10);
    g_params.cx = static_cast<int>(cx);
    g_params.cy = static_cast<int>(cy);

    g_params.k1 = static_cast<int>(calib.dist_coeffs.at<double>(0) * 1000);
    g_params.k2 = static_cast<int>(calib.dist_coeffs.at<double>(1) * 1000);
    g_params.p1 = static_cast<int>(calib.dist_coeffs.at<double>(2) * 1000);
    g_params.p2 = static_cast<int>(calib.dist_coeffs.at<double>(3) * 1000);
    g_params.k3 = calib.dist_coeffs.cols >= 5 ?
                  static_cast<int>(calib.dist_coeffs.at<double>(4) * 1000) : 0;

    g_params.use_square_pixels = (std::abs(fx - fy) < 1.0) ? 1 : 0;

    std::cout << "\n📊 Initial parameters:" << std::endl;
    std::cout << "  fx: " << fx << std::endl;
    std::cout << "  fy: " << fy << std::endl;
    std::cout << "  cx: " << cx << std::endl;
    std::cout << "  cy: " << cy << std::endl;
    std::cout << "  k1: " << calib.dist_coeffs.at<double>(0) << std::endl;
    std::cout << "  k2: " << calib.dist_coeffs.at<double>(1) << std::endl;
    std::cout << "  p1: " << calib.dist_coeffs.at<double>(2) << std::endl;
    std::cout << "  p2: " << calib.dist_coeffs.at<double>(3) << std::endl;
}

// Create trackbars
void createTrackbars(const cv::Size& image_size) {
    cv::namedWindow("Calibration Controls", cv::WINDOW_NORMAL);
    cv::resizeWindow("Calibration Controls", 600, 400);

    // Camera matrix parameters
    cv::createTrackbar("fx (x10)", "Calibration Controls", &g_params.fx, 20000, onTrackbarChange);
    cv::createTrackbar("fy (x10)", "Calibration Controls", &g_params.fy, 20000, onTrackbarChange);
    cv::createTrackbar("cx", "Calibration Controls", &g_params.cx, image_size.width, onTrackbarChange);
    cv::createTrackbar("cy", "Calibration Controls", &g_params.cy, image_size.height, onTrackbarChange);

    // Distortion coefficients (scaled by 1000 for precision)
    cv::createTrackbar("k1 (x1000)", "Calibration Controls", &g_params.k1, 2000, onTrackbarChange);
    cv::setTrackbarMin("k1 (x1000)", "Calibration Controls", -1000);

    cv::createTrackbar("k2 (x1000)", "Calibration Controls", &g_params.k2, 2000, onTrackbarChange);
    cv::setTrackbarMin("k2 (x1000)", "Calibration Controls", -1000);

    cv::createTrackbar("k3 (x1000)", "Calibration Controls", &g_params.k3, 2000, onTrackbarChange);
    cv::setTrackbarMin("k3 (x1000)", "Calibration Controls", -1000);

    cv::createTrackbar("p1 (x1000)", "Calibration Controls", &g_params.p1, 200, onTrackbarChange);
    cv::setTrackbarMin("p1 (x1000)", "Calibration Controls", -100);

    cv::createTrackbar("p2 (x1000)", "Calibration Controls", &g_params.p2, 200, onTrackbarChange);
    cv::setTrackbarMin("p2 (x1000)", "Calibration Controls", -100);

    // Square pixels option
    cv::createTrackbar("Square Pixels", "Calibration Controls", &g_params.use_square_pixels, 1, onTrackbarChange);
}

// Build camera matrix and dist coeffs from current parameters
void buildCalibrationMatrices(cv::Mat& camera_matrix, cv::Mat& dist_coeffs) {
    double fx = g_params.fx / 10.0;
    double fy = g_params.fy / 10.0;

    // Apply square pixel constraint if enabled
    if (g_params.use_square_pixels) {
        double f_avg = (fx + fy) / 2.0;
        fx = f_avg;
        fy = f_avg;
    }

    double cx = g_params.cx;
    double cy = g_params.cy;

    camera_matrix = (cv::Mat_<double>(3, 3) <<
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1
    );

    dist_coeffs = (cv::Mat_<double>(1, 5) <<
        g_params.k1 / 1000.0,
        g_params.k2 / 1000.0,
        g_params.p1 / 1000.0,
        g_params.p2 / 1000.0,
        g_params.k3 / 1000.0
    );
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

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters params = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, params);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    detector.detectMarkers(frame, marker_corners, marker_ids);

    if (marker_ids.size() < 4) {
        return corners;
    }

    for (size_t i = 0; i < marker_ids.size(); i++) {
        int id = marker_ids[i];

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

// Circle detection and measurement
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

    cv::Mat gray;
    if (birdseye_view.channels() == 3) {
        cv::cvtColor(birdseye_view, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = birdseye_view.clone();
    }

    cv::Mat binary;
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return result;

    double max_area = 0;
    int largest_idx = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area && area > 1000) {
            max_area = area;
            largest_idx = i;
        }
    }

    if (largest_idx < 0) return result;

    if (contours[largest_idx].size() >= 5) {
        cv::RotatedRect ellipse = cv::fitEllipse(contours[largest_idx]);

        result.center = ellipse.center;

        float axis1 = ellipse.size.width;
        float axis2 = ellipse.size.height;

        result.horizontal_diameter = std::max(axis1, axis2);
        result.vertical_diameter = std::min(axis1, axis2);
        result.ratio = result.vertical_diameter / result.horizontal_diameter;

        result.detected = true;
    }

    return result;
}

// Save calibration to file
void saveCalibration(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
                     const cv::Size& image_size, const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    fs << "calibration_date" << std::ctime(&now_time);
    fs << "camera_index" << 0;
    fs << "image_width" << image_size.width;
    fs << "image_height" << image_size.height;
    fs << "camera_matrix" << camera_matrix;
    fs << "distortion_coefficients" << dist_coeffs;
    fs << "optimal_camera_matrix" << camera_matrix;
    fs << "pixel_aspect_ratio_correction_applied" << (g_params.use_square_pixels == 1);
    fs << "manually_adjusted" << true;

    fs.release();

    std::cout << "\n💾 Calibration saved to: " << filename << std::endl;
    std::cout << "   Camera matrix:" << std::endl << camera_matrix << std::endl;
    std::cout << "   Distortion coeffs:" << std::endl << dist_coeffs << std::endl;
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     INTERACTIVE CALIBRATION ADJUSTER                   ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝\n" << std::endl;

    // Load initial calibration
    std::cout << "📂 Loading initial calibration..." << std::endl;
    CalibrationData calib = loadCalibration("camera_calibration.yml");

    if (!calib.loaded) {
        std::cerr << "\n❌ Failed to load calibration!" << std::endl;
        std::cerr << "   Please run camera_calibration first." << std::endl;
        return -1;
    }

    // Initialize parameters
    initializeParams(calib);

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

    // Create trackbars
    createTrackbars(calib.image_size);

    std::cout << "\n╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                    INSTRUCTIONS                        ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║  1. Place template with circle in front of camera     ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  2. Adjust calibration sliders to make circle         ║" << std::endl;
    std::cout << "║     perfectly round in bird's-eye view                ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  CONTROLS:                                             ║" << std::endl;
    std::cout << "║    • Adjust sliders in 'Calibration Controls' window  ║" << std::endl;
    std::cout << "║    • fx/fy: Focal length (adjust for aspect ratio)   ║" << std::endl;
    std::cout << "║    • cx/cy: Image center point                        ║" << std::endl;
    std::cout << "║    • k1/k2/k3: Radial distortion (barrel/pincushion) ║" << std::endl;
    std::cout << "║    • p1/p2: Tangential distortion                     ║" << std::endl;
    std::cout << "║    • Square Pixels: Force fx=fy (recommended)         ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  KEYBOARD SHORTCUTS (Fine adjustment):                 ║" << std::endl;
    std::cout << "║    '1'/'2' = Decrease/Increase fx                     ║" << std::endl;
    std::cout << "║    '3'/'4' = Decrease/Increase fy                     ║" << std::endl;
    std::cout << "║    '5'/'6' = Decrease/Increase k1 (distortion)        ║" << std::endl;
    std::cout << "║    'SPACE' = Toggle Square Pixels mode                ║" << std::endl;
    std::cout << "║    's'     = Save calibration to file                 ║" << std::endl;
    std::cout << "║    'r'     = Reset to original values                 ║" << std::endl;
    std::cout << "║    'q'     = Quit                                      ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  GOAL: Ratio (V/H) = 1.000 for perfect circle!       ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝\n" << std::endl;

    const int OUTPUT_SIZE = 800;
    cv::Mat current_camera_matrix, current_dist_coeffs;

    // Build initial calibration matrices from loaded config
    buildCalibrationMatrices(current_camera_matrix, current_dist_coeffs);

    while (true) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) break;

        // Rebuild calibration matrices if parameters changed
        if (g_params_changed) {
            buildCalibrationMatrices(current_camera_matrix, current_dist_coeffs);
            g_params_changed = false;
        }

        // Apply undistortion
        // Use original camera matrix as input, updated one as output
        cv::Mat undistorted;
        cv::undistort(frame, undistorted, current_camera_matrix, current_dist_coeffs);

        // Detect markers
        MarkerCorners corners = detectTemplateMarkers(undistorted);

        // Apply bird's-eye view
        cv::Mat birdseye;
        if (corners.all_detected) {
            birdseye = applyBirdsEyeView(undistorted, corners, OUTPUT_SIZE);

            // Draw markers
            cv::circle(undistorted, corners.top_left, 8, cv::Scalar(0, 255, 0), -1);
            cv::circle(undistorted, corners.top_right, 8, cv::Scalar(0, 255, 0), -1);
            cv::circle(undistorted, corners.bottom_right, 8, cv::Scalar(0, 255, 0), -1);
            cv::circle(undistorted, corners.bottom_left, 8, cv::Scalar(0, 255, 0), -1);

            cv::line(undistorted, corners.top_left, corners.top_right, cv::Scalar(0, 255, 0), 2);
            cv::line(undistorted, corners.top_right, corners.bottom_right, cv::Scalar(0, 255, 0), 2);
            cv::line(undistorted, corners.bottom_right, corners.bottom_left, cv::Scalar(0, 255, 0), 2);
            cv::line(undistorted, corners.bottom_left, corners.top_left, cv::Scalar(0, 255, 0), 2);

            // Detect circle
            CircleMeasurement circle = detectAndMeasureCircle(birdseye);

            if (circle.detected) {
                // Draw ellipse
                cv::RotatedRect ellipse_rect(
                    circle.center,
                    cv::Size2f(circle.horizontal_diameter, circle.vertical_diameter),
                    0
                );
                cv::ellipse(birdseye, ellipse_rect, cv::Scalar(0, 255, 0), 2);

                // Draw center
                cv::circle(birdseye, circle.center, 5, cv::Scalar(255, 0, 0), -1);

                // Draw diameters
                float half_h = circle.horizontal_diameter / 2.0f;
                cv::Point2f h_start(circle.center.x - half_h, circle.center.y);
                cv::Point2f h_end(circle.center.x + half_h, circle.center.y);
                cv::line(birdseye, h_start, h_end, cv::Scalar(0, 0, 255), 3);

                float half_v = circle.vertical_diameter / 2.0f;
                cv::Point2f v_start(circle.center.x, circle.center.y - half_v);
                cv::Point2f v_end(circle.center.x, circle.center.y + half_v);
                cv::line(birdseye, v_start, v_end, cv::Scalar(255, 0, 0), 3);

                // Display measurements
                cv::rectangle(birdseye, cv::Point(10, 10), cv::Point(400, 200),
                             cv::Scalar(0, 0, 0), -1);
                cv::rectangle(birdseye, cv::Point(10, 10), cv::Point(400, 200),
                             cv::Scalar(255, 255, 255), 2);

                int y_pos = 40;
                cv::putText(birdseye, "CIRCLE MEASUREMENT:", cv::Point(20, y_pos),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

                y_pos += 35;
                char h_text[100];
                snprintf(h_text, sizeof(h_text), "H: %.1f px", circle.horizontal_diameter);
                cv::putText(birdseye, h_text, cv::Point(20, y_pos),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

                y_pos += 30;
                char v_text[100];
                snprintf(v_text, sizeof(v_text), "V: %.1f px", circle.vertical_diameter);
                cv::putText(birdseye, v_text, cv::Point(20, y_pos),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

                y_pos += 35;
                char ratio_text[100];
                snprintf(ratio_text, sizeof(ratio_text), "RATIO: %.4f", circle.ratio);
                cv::Scalar ratio_color;
                if (circle.ratio > 0.99 && circle.ratio < 1.01) {
                    ratio_color = cv::Scalar(0, 255, 0);
                } else if (circle.ratio > 0.95 && circle.ratio < 1.05) {
                    ratio_color = cv::Scalar(0, 255, 255);
                } else {
                    ratio_color = cv::Scalar(0, 0, 255);
                }
                cv::putText(birdseye, ratio_text, cv::Point(20, y_pos),
                           cv::FONT_HERSHEY_SIMPLEX, 0.8, ratio_color, 2);

                y_pos += 35;
                std::string quality_text;
                if (circle.ratio > 0.99 && circle.ratio < 1.01) {
                    quality_text = "PERFECT!";
                } else if (circle.ratio > 0.95 && circle.ratio < 1.05) {
                    quality_text = "GOOD";
                } else {
                    quality_text = "ADJUST!";
                }
                cv::putText(birdseye, quality_text, cv::Point(20, y_pos),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, ratio_color, 2);
            }
        } else {
            birdseye = cv::Mat::zeros(OUTPUT_SIZE, OUTPUT_SIZE, undistorted.type());
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

        cv::Mat comparison;
        cv::hconcat(undistorted_resized, birdseye_resized, comparison);

        cv::putText(comparison, "ORIGINAL", cv::Point(20, display_height - 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
        cv::putText(comparison, "BIRD'S-EYE (CROPPED)", cv::Point(OUTPUT_SIZE + 20, display_height - 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        cv::putText(comparison, "'s'=Save 'r'=Reset 'q'=Quit", cv::Point(comparison.cols/2 - 150, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

        cv::imshow("Interactive Calibration", comparison);

        char key = cv::waitKey(10);

        // Keyboard controls for fine adjustment
        bool updated = false;
        const int STEP = 10;  // Adjustment step size

        if (key == 'q' || key == 'Q') {
            break;
        } else if (key == 's' || key == 'S') {
            std::string filename = "camera_calibration_adjusted.yml";
            saveCalibration(current_camera_matrix, current_dist_coeffs,
                          calib.image_size, filename);
        } else if (key == 'r' || key == 'R') {
            std::cout << "\n🔄 Resetting to original calibration..." << std::endl;
            initializeParams(calib);
            cv::setTrackbarPos("fx (x10)", "Calibration Controls", g_params.fx);
            cv::setTrackbarPos("fy (x10)", "Calibration Controls", g_params.fy);
            cv::setTrackbarPos("k1 (x1000)", "Calibration Controls", g_params.k1);
            cv::setTrackbarPos("k2 (x1000)", "Calibration Controls", g_params.k2);
            cv::setTrackbarPos("Square Pixels", "Calibration Controls", g_params.use_square_pixels);
            updated = true;
        } else if (key == '1') {
            g_params.fx = std::max(0, g_params.fx - STEP);
            cv::setTrackbarPos("fx (x10)", "Calibration Controls", g_params.fx);
            std::cout << "fx: " << (g_params.fx / 10.0) << std::endl;
            updated = true;
        } else if (key == '2') {
            g_params.fx = std::min(20000, g_params.fx + STEP);
            cv::setTrackbarPos("fx (x10)", "Calibration Controls", g_params.fx);
            std::cout << "fx: " << (g_params.fx / 10.0) << std::endl;
            updated = true;
        } else if (key == '3') {
            g_params.fy = std::max(0, g_params.fy - STEP);
            cv::setTrackbarPos("fy (x10)", "Calibration Controls", g_params.fy);
            std::cout << "fy: " << (g_params.fy / 10.0) << std::endl;
            updated = true;
        } else if (key == '4') {
            g_params.fy = std::min(20000, g_params.fy + STEP);
            cv::setTrackbarPos("fy (x10)", "Calibration Controls", g_params.fy);
            std::cout << "fy: " << (g_params.fy / 10.0) << std::endl;
            updated = true;
        } else if (key == '5') {
            g_params.k1 = std::max(-1000, g_params.k1 - 1);
            cv::setTrackbarPos("k1 (x1000)", "Calibration Controls", g_params.k1);
            std::cout << "k1: " << (g_params.k1 / 1000.0) << std::endl;
            updated = true;
        } else if (key == '6') {
            g_params.k1 = std::min(2000, g_params.k1 + 1);
            cv::setTrackbarPos("k1 (x1000)", "Calibration Controls", g_params.k1);
            std::cout << "k1: " << (g_params.k1 / 1000.0) << std::endl;
            updated = true;
        } else if (key == ' ') {
            g_params.use_square_pixels = 1 - g_params.use_square_pixels;
            cv::setTrackbarPos("Square Pixels", "Calibration Controls", g_params.use_square_pixels);
            std::cout << "Square Pixels: " << (g_params.use_square_pixels ? "ON" : "OFF") << std::endl;
            updated = true;
        }

        if (updated) {
            g_params_changed = true;
        }
    }

    std::cout << "\n✅ Interactive calibration session ended." << std::endl;

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
