#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <chrono>
#include <limits>
#include <cmath>
#include <sstream>
#include <algorithm>

// Perspective transform için köşeleri sırala
std::vector<cv::Point2f> orderPoints(const std::vector<cv::Point2f>& pts) {
    std::vector<cv::Point2f> ordered(4);
    std::vector<float> sums(4);
    std::vector<float> diffs(4);

    for (int i = 0; i < 4; i++) {
        sums[i] = pts[i].x + pts[i].y;
        diffs[i] = pts[i].y - pts[i].x;
    }

    auto tl_idx = std::min_element(sums.begin(), sums.end()) - sums.begin();
    ordered[0] = pts[tl_idx];

    auto br_idx = std::max_element(sums.begin(), sums.end()) - sums.begin();
    ordered[2] = pts[br_idx];

    auto tr_idx = std::min_element(diffs.begin(), diffs.end()) - diffs.begin();
    ordered[1] = pts[tr_idx];

    auto bl_idx = std::max_element(diffs.begin(), diffs.end()) - diffs.begin();
    ordered[3] = pts[bl_idx];

    return ordered;
}

// Kamera bilgisi struct
struct CameraInfo {
    int index;
    int width;
    int height;
    int fps;
    std::string backend;
    std::string description;
};

// Tüm kameraları tara
std::vector<CameraInfo> scanAllCameras(int max_cameras = 10) {
    std::vector<CameraInfo> cameras;
    std::cout << "🔍 Kameralar taranıyor..." << std::endl;

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
                    info.description = "External Webcam (Low Res)";
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

// Kullanıcıya kamera seçtir
int selectCamera() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              KAMERA SEÇİMİ                            ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝\n" << std::endl;

    auto cameras = scanAllCameras();
    if (cameras.empty()) {
        std::cerr << "❌ Hiçbir kamera bulunamadı!" << std::endl;
        return -1;
    }

    std::cout << "Bulunan kameralar:\n" << std::endl;
    for (const auto& cam : cameras) {
        std::cout << "  [" << cam.index << "] " << cam.description
                  << " (" << cam.width << "x" << cam.height << " @ " << cam.fps << " fps)"
                  << std::endl;
    }

    if (cameras.size() == 1) {
        std::cout << "\n✅ Tek kamera bulundu, otomatik seçildi: Kamera " << cameras[0].index << std::endl;
        return cameras[0].index;
    }

    std::cout << "\nKamera ID girin: ";
    int selected;
    std::cin >> selected;

    for (const auto& cam : cameras) {
        if (cam.index == selected) {
            std::cout << "✅ Kamera " << selected << " seçildi." << std::endl;
            return selected;
        }
    }

    std::cerr << "❌ Geçersiz kamera ID'si!" << std::endl;
    return -1;
}

// Daire tespit ve aspect ratio ölçümü
struct CircleInfo {
    bool found = false;
    cv::Point2f center;
    float width = 0.0f;
    float height = 0.0f;
    float aspect_ratio = 0.0f;
    cv::RotatedRect rect;
};

CircleInfo detectCircle(const cv::Mat& warped) {
    CircleInfo info;

    // Gray conversion
    cv::Mat gray;
    if (warped.channels() == 3) {
        cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = warped.clone();
    }

    // Threshold
    cv::Mat binary;
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Find largest contour (likely the drawn circle)
    if (contours.empty()) return info;

    size_t largest_idx = 0;
    double largest_area = 0;

    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > largest_area) {
            largest_area = area;
            largest_idx = i;
        }
    }

    // Must have at least 5 points to fit ellipse
    if (contours[largest_idx].size() < 5) return info;

    // Fit ellipse
    info.rect = cv::fitEllipse(contours[largest_idx]);
    info.center = info.rect.center;
    info.width = info.rect.size.width;
    info.height = info.rect.size.height;

    // Aspect ratio (always > 1)
    if (info.width > info.height) {
        info.aspect_ratio = info.width / info.height;
    } else {
        info.aspect_ratio = info.height / info.width;
    }

    info.found = true;
    return info;
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║       ASPECT RATIO DEBUG TOOL                         ║" << std::endl;
    std::cout << "║       Measure Real Circle Distortion                  ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝\n" << std::endl;

    // Kamera seç
    int camera_index = selectCamera();
    if (camera_index < 0) return -1;

    std::cout << "\n🎥 Kamera açılıyor..." << std::endl;
    cv::VideoCapture cap(camera_index);
    if (!cap.isOpened()) {
        std::cerr << "❌ Kamera açılamadı!" << std::endl;
        return -1;
    }
    std::cout << "✅ Kamera " << camera_index << " başlatıldı!\n" << std::endl;

    // Load calibration
    cv::Mat camera_matrix, dist_coeffs, optimal_camera_matrix;
    bool calibration_loaded = false;
    std::string calib_file = "camera_calibration.yml";

    cv::FileStorage fs(calib_file, cv::FileStorage::READ);
    if (fs.isOpened()) {
        int img_width, img_height;
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> dist_coeffs;
        fs["image_width"] >> img_width;
        fs["image_height"] >> img_height;
        fs.release();

        if (!camera_matrix.empty() && !dist_coeffs.empty()) {
            calibration_loaded = true;

            double fx_orig = camera_matrix.at<double>(0, 0);
            double fy_orig = camera_matrix.at<double>(1, 1);

            // Try to load optimal matrix from YML (if available)
            cv::Mat optimal_from_file;
            fs.open(calib_file, cv::FileStorage::READ);
            if (fs.isOpened()) {
                fs["optimal_camera_matrix"] >> optimal_from_file;
                fs.release();
            }

            if (!optimal_from_file.empty()) {
                optimal_camera_matrix = optimal_from_file;
                double fx_opt = optimal_camera_matrix.at<double>(0, 0);
                double fy_opt = optimal_camera_matrix.at<double>(1, 1);

                std::cout << "✅ Kalibrasyon yüklendi (with optimal matrix):" << std::endl;
                std::cout << "   Original fx: " << std::fixed << std::setprecision(2) << fx_orig << std::endl;
                std::cout << "   Original fy: " << fy_orig << std::endl;
                std::cout << "   Original fy/fx: " << std::setprecision(4) << (fy_orig/fx_orig) << std::endl;
                std::cout << "   ✨ PRE-CALCULATED optimal matrix loaded!" << std::endl;
                std::cout << "   Optimal fx = fy: " << std::setprecision(2) << fx_opt << std::endl;
                std::cout << "   Optimal fy/fx: " << std::setprecision(4) << (fy_opt/fx_opt) << " (= 1.0000)" << std::endl;
            } else {
                double cx = camera_matrix.at<double>(0, 2);
                double cy = camera_matrix.at<double>(1, 2);
                double f_avg = (fx_orig + fy_orig) / 2.0;

                optimal_camera_matrix = (cv::Mat_<double>(3, 3) <<
                    f_avg, 0, cx,
                    0, f_avg, cy,
                    0, 0, 1
                );

                std::cout << "✅ Kalibrasyon yüklendi:" << std::endl;
                std::cout << "   Original fx: " << std::fixed << std::setprecision(2) << fx_orig << std::endl;
                std::cout << "   Original fy: " << fy_orig << std::endl;
                std::cout << "   Original fy/fx: " << std::setprecision(4) << (fy_orig/fx_orig) << std::endl;
                std::cout << "   MANUAL Square Pixel Matrix!" << std::endl;
                std::cout << "   New fx = fy: " << std::setprecision(2) << f_avg << std::endl;
                std::cout << "   New fy/fx: " << std::setprecision(4) << (1.0) << " (= 1.0000)" << std::endl;
            }
            std::cout << std::endl;
        }
    } else {
        std::cout << "⚠️  Kalibrasyon yok, ham görüntü kullanılacak." << std::endl;
    }

    std::cout << "TALİMATLAR:" << std::endl;
    std::cout << "  1. ArUco template'i göster (4 marker)" << std::endl;
    std::cout << "  2. Ortaya bir DAIRE çiz (pergel veya elle)" << std::endl;
    std::cout << "  3. Program dairenin GERÇEK aspect ratio'sunu ölçecek" << std::endl;
    std::cout << "  4. 1.00 = mükemmel daire!" << std::endl;
    std::cout << "\nKlavye: 'q' = Çık, 'f' = Freeze (ölçümü dondur)\n" << std::endl;

    // ArUco setup
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    const int OUTPUT_WIDTH = 800;

    bool frozen = false;
    cv::Mat frozen_warped;

    while (true) {
        cv::Mat frame;
        if (!frozen) {
            cap >> frame;
            if (frame.empty()) break;
        }

        cv::Mat frame_undistorted;
        if (calibration_loaded && !frozen) {
            // Undistort with optimal matrix
            cv::Mat temp;
            cv::undistort(frame, temp, camera_matrix, dist_coeffs, optimal_camera_matrix);

            // CRITICAL FIX: Resize output to correct aspect ratio!
            // Because fx changed but output size stayed same, we need to resize
            double fx_orig = camera_matrix.at<double>(0, 0);
            double fy_orig = camera_matrix.at<double>(1, 1);

            // Calculate new height to compensate for focal length change
            int new_height = static_cast<int>(temp.rows * (fx_orig / fy_orig));
            cv::resize(temp, frame_undistorted, cv::Size(temp.cols, new_height));

            std::cout << "🔧 Undistort + Resize: " << temp.cols << "x" << temp.rows
                     << " → " << frame_undistorted.cols << "x" << frame_undistorted.rows << std::endl;
        } else if (!frozen) {
            frame_undistorted = frame.clone();
        }

        cv::Mat display, warped;

        if (!frozen) {
            display = frame_undistorted.clone();

            // ArUco detection
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;

            detector.detectMarkers(frame_undistorted, markerCorners, markerIds);

            if (!markerIds.empty()) {
                cv::aruco::drawDetectedMarkers(display, markerCorners, markerIds);

                bool has_0 = false, has_1 = false, has_2 = false, has_3 = false;
                for (int id : markerIds) {
                    if (id == 0) has_0 = true;
                    if (id == 1) has_1 = true;
                    if (id == 2) has_2 = true;
                    if (id == 3) has_3 = true;
                }

                if (has_0 && has_1 && has_2 && has_3) {
                    // Find corners
                    std::vector<cv::Point2f> corners(4);
                    std::vector<bool> corner_found(4, false);

                    std::vector<cv::Point2f> marker_centers(markerIds.size());
                    cv::Point2f board_center(0.f, 0.f);

                    for (size_t i = 0; i < markerIds.size(); ++i) {
                        cv::Point2f center(0.f, 0.f);
                        for (int j = 0; j < 4; ++j) {
                            center += markerCorners[i][j];
                        }
                        center *= 0.25f;
                        marker_centers[i] = center;
                        board_center += center;
                    }
                    board_center *= 1.0f / static_cast<float>(marker_centers.size());

                    for (size_t i = 0; i < markerIds.size(); ++i) {
                        cv::Point2f dir = marker_centers[i] - board_center;
                        float dir_norm = std::sqrt(dir.dot(dir));
                        if (dir_norm < 1e-3f) {
                            dir = cv::Point2f(0.f, -1.f);
                        } else {
                            dir *= (1.0f / dir_norm);
                        }

                        float best_score = -std::numeric_limits<float>::infinity();
                        cv::Point2f best_corner;
                        for (int j = 0; j < 4; ++j) {
                            cv::Point2f vec = markerCorners[i][j] - marker_centers[i];
                            float score = vec.dot(dir);
                            if (score > best_score) {
                                best_score = score;
                                best_corner = markerCorners[i][j];
                            }
                        }

                        int target = -1;
                        if (markerIds[i] == 0) target = 0;
                        else if (markerIds[i] == 1) target = 1;
                        else if (markerIds[i] == 2) target = 2;
                        else if (markerIds[i] == 3) target = 3;

                        if (target >= 0) {
                            corners[target] = best_corner;
                            corner_found[target] = true;
                        }
                    }

                    if (corner_found[0] && corner_found[1] && corner_found[2] && corner_found[3]) {
                        corners = orderPoints(corners);

                        // Draw corners
                        for (int i = 0; i < 4; i++) {
                            cv::line(display, corners[i], corners[(i+1)%4], cv::Scalar(0, 255, 0), 3);
                        }

                        // Calculate output height based on INPUT aspect ratio (preserve square pixels!)
                        float input_aspect = (float)frame_undistorted.rows / frame_undistorted.cols;
                        int output_height = static_cast<int>(OUTPUT_WIDTH * input_aspect);

                        // Perspective transform
                        std::vector<cv::Point2f> dst_points = {
                            cv::Point2f(0, 0),
                            cv::Point2f(OUTPUT_WIDTH - 1, 0),
                            cv::Point2f(OUTPUT_WIDTH - 1, output_height - 1),
                            cv::Point2f(0, output_height - 1)
                        };

                        cv::Mat transform_matrix = cv::getPerspectiveTransform(corners, dst_points);
                        cv::warpPerspective(frame_undistorted, warped, transform_matrix,
                                           cv::Size(OUTPUT_WIDTH, output_height));
                    }
                }
            }

            cv::imshow("Camera View", display);
        } else {
            warped = frozen_warped.clone();
        }

        if (!warped.empty()) {
            cv::Mat warped_display = warped.clone();

            // Detect circle and measure aspect ratio
            CircleInfo circle = detectCircle(warped);

            if (circle.found) {
                // Draw fitted ellipse
                cv::ellipse(warped_display, circle.rect, cv::Scalar(255, 0, 255), 3);

                // Draw axes
                cv::line(warped_display,
                         cv::Point(circle.center.x - circle.width/2, circle.center.y),
                         cv::Point(circle.center.x + circle.width/2, circle.center.y),
                         cv::Scalar(0, 0, 255), 2);
                cv::line(warped_display,
                         cv::Point(circle.center.x, circle.center.y - circle.height/2),
                         cv::Point(circle.center.x, circle.center.y + circle.height/2),
                         cv::Scalar(0, 255, 0), 2);

                // Info overlay
                cv::rectangle(warped_display, cv::Point(0, 0), cv::Point(warped_display.cols, 200),
                             cv::Scalar(0, 0, 0), -1);

                cv::putText(warped_display, "CIRCLE ASPECT RATIO MEASUREMENT",
                           cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

                std::ostringstream width_ss;
                width_ss << "Width (RED):   " << std::fixed << std::setprecision(1) << circle.width << " px";
                cv::putText(warped_display, width_ss.str(),
                           cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

                std::ostringstream height_ss;
                height_ss << "Height (GREEN): " << std::fixed << std::setprecision(1) << circle.height << " px";
                cv::putText(warped_display, height_ss.str(),
                           cv::Point(20, 100), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

                std::ostringstream ratio_ss;
                ratio_ss << "Aspect Ratio:  " << std::fixed << std::setprecision(4) << circle.aspect_ratio;
                cv::Scalar ratio_color;
                if (std::abs(circle.aspect_ratio - 1.0f) < 0.01f) {
                    ratio_color = cv::Scalar(0, 255, 0); // Perfect!
                } else if (std::abs(circle.aspect_ratio - 1.0f) < 0.03f) {
                    ratio_color = cv::Scalar(0, 255, 255); // Good
                } else {
                    ratio_color = cv::Scalar(0, 165, 255); // Needs work
                }
                cv::putText(warped_display, ratio_ss.str(),
                           cv::Point(20, 135), cv::FONT_HERSHEY_SIMPLEX, 0.7, ratio_color, 2);

                // Quality assessment
                float deviation = std::abs(circle.aspect_ratio - 1.0f) * 100.0f;
                std::ostringstream quality_ss;
                quality_ss << "Deviation: " << std::fixed << std::setprecision(2) << deviation << "% from perfect";
                cv::putText(warped_display, quality_ss.str(),
                           cv::Point(20, 170), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

                // Direction indicator
                std::string direction;
                if (circle.width > circle.height) {
                    float diff_pct = ((circle.width / circle.height) - 1.0f) * 100.0f;
                    direction = "Horizontally stretched (" + std::to_string((int)diff_pct) + "% wider)";
                } else if (circle.height > circle.width) {
                    float diff_pct = ((circle.height / circle.width) - 1.0f) * 100.0f;
                    direction = "Vertically stretched (" + std::to_string((int)diff_pct) + "% taller)";
                } else {
                    direction = "PERFECT CIRCLE!";
                }
                cv::putText(warped_display, direction,
                           cv::Point(20, warped_display.rows - 20),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

            } else {
                cv::putText(warped_display, "NO CIRCLE DETECTED",
                           cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
                cv::putText(warped_display, "Draw a circle in the center",
                           cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);
            }

            if (frozen) {
                cv::putText(warped_display, "FROZEN - Press 'f' to unfreeze",
                           cv::Point(20, warped_display.rows - 50),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            }

            cv::imshow("Bird's Eye View - Circle Analysis", warped_display);
        }

        char key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            break;
        } else if (key == 'f' || key == 'F') {
            frozen = !frozen;
            if (frozen && !warped.empty()) {
                frozen_warped = warped.clone();
                std::cout << "🔒 Görüntü donduruldu - Analiz ediliyor..." << std::endl;
            } else {
                std::cout << "▶️  Canlı görüntü devam ediyor..." << std::endl;
            }
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
