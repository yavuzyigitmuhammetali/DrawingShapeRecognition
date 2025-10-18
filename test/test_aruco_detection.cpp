#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>        // ← EKLENDİ
#include <chrono>        // ← EKLENDİ (emin olmak için)
#include <limits>
#include <cmath>
#include <sstream>
#include <algorithm>

// Kamera bilgisi struct
struct CameraInfo {
    int index;
    int width;
    int height;
    int fps;
    std::string backend;
    std::string description;
};

// Tüm kameraları tara ve detaylı bilgi topla
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

                // Kamerayı tanımla (heuristic)
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


// Kullanıcıya kamera seçtir (detaylı menü)
int selectCameraDetailed() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              KAMERA SEÇİMİ                            ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝\n" << std::endl;

    auto cameras = scanAllCameras();

    if (cameras.empty()) {
        std::cerr << "❌ Hiçbir kamera bulunamadı!" << std::endl;
        return -1;
    }

    std::cout << "Bulunan kameralar:\n" << std::endl;
    std::cout << "┌──────┬─────────────┬──────────┬──────────────────────┐" << std::endl;
    std::cout << "│ ID   │ Çözünürlük  │ FPS      │ Açıklama             │" << std::endl;
    std::cout << "├──────┼─────────────┼──────────┼──────────────────────┤" << std::endl;

    for (const auto& cam : cameras) {
        std::cout << "│ [" << cam.index << "]  │ ";
        std::cout << std::setw(4) << cam.width << "x" << std::setw(4) << std::left << cam.height << " │ ";
        std::cout << std::setw(8) << std::left << (std::to_string(cam.fps) + " fps") << " │ ";
        std::cout << std::setw(20) << std::left << cam.description << " │" << std::endl;
    }

    std::cout << "└──────┴─────────────┴──────────┴──────────────────────┘" << std::endl;

    // Tek kamera varsa otomatik seç
    if (cameras.size() == 1) {
        std::cout << "\n✅ Tek kamera bulundu, otomatik seçildi: Kamera " << cameras[0].index << std::endl;
        return cameras[0].index;
    }

    // Çoklu kamera - kullanıcıdan seçim iste
    std::cout << "\n📌 Hangi kamerayı kullanmak istersiniz?" << std::endl;
    std::cout << "   (Test için önerilen: Harici Webcam)" << std::endl;
    std::cout << "\nKamera ID girin [";
    for (size_t i = 0; i < cameras.size(); i++) {
        std::cout << cameras[i].index;
        if (i < cameras.size() - 1) std::cout << "/";
    }
    std::cout << "]: ";

    int selected;
    std::cin >> selected;

    // Geçerli mi kontrol et
    bool valid = false;
    for (const auto& cam : cameras) {
        if (cam.index == selected) {
            valid = true;
            std::cout << "\n✅ Kamera " << selected << " seçildi: " << cam.description << std::endl;
            std::cout << "   (" << cam.width << "x" << cam.height << " @ " << cam.fps << " fps)" << std::endl;
            break;
        }
    }

    if (!valid) {
        std::cerr << "\n❌ Geçersiz kamera ID'si!" << std::endl;
        return -1;
    }

    return selected;
}

// Perspective transform için köşeleri sırala
std::vector<cv::Point2f> orderPoints(const std::vector<cv::Point2f>& pts) {
    std::vector<cv::Point2f> ordered(4);

    // Toplamı en küçük = sol üst, en büyük = sağ alt
    std::vector<float> sums(4);
    std::vector<float> diffs(4);

    for (int i = 0; i < 4; i++) {
        sums[i] = pts[i].x + pts[i].y;
        diffs[i] = pts[i].y - pts[i].x;
    }

    // Top-left: minimum sum
    auto tl_idx = std::min_element(sums.begin(), sums.end()) - sums.begin();
    ordered[0] = pts[tl_idx];

    // Bottom-right: maximum sum
    auto br_idx = std::max_element(sums.begin(), sums.end()) - sums.begin();
    ordered[2] = pts[br_idx];

    // Top-right: minimum diff
    auto tr_idx = std::min_element(diffs.begin(), diffs.end()) - diffs.begin();
    ordered[1] = pts[tr_idx];

    // Bottom-left: maximum diff
    auto bl_idx = std::max_element(diffs.begin(), diffs.end()) - diffs.begin();
    ordered[3] = pts[bl_idx];

    return ordered;
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║        ARUCO MARKER DETECTION TEST                    ║" << std::endl;
    std::cout << "║        Template Quality Assessment                    ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝\n" << std::endl;

    // Kamera seç (detaylı menü)
    int camera_index = selectCameraDetailed();
    if (camera_index < 0) return -1;

    std::cout << "\n🎥 Kamera açılıyor..." << std::endl;
    cv::VideoCapture cap(camera_index);

    if (!cap.isOpened()) {
        std::cerr << "❌ Kamera açılamadı!" << std::endl;
        return -1;
    }

    std::cout << "✅ Kamera başlatıldı!" << std::endl;
    std::cout << "\n╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                    TALIMATLAR                          ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║  1. Yazdırılmış template'i kameraya gösterin          ║" << std::endl;
    std::cout << "║  2. 4 ArUco marker tespit edilmeye çalışılacak        ║" << std::endl;
    std::cout << "║  3. Kağıdı farklı açılardan gösterin                  ║" << std::endl;
    std::cout << "║  4. Farklı mesafelerden test edin                     ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  🟢 Yeşil = Tüm marker'lar tespit edildi             ║" << std::endl;
    std::cout << "║  🟠 Turuncu = Bazı marker'lar eksik                  ║" << std::endl;
    std::cout << "║  🔴 Kırmızı = Marker tespit edilemedi                ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  Klavye:                                               ║" << std::endl;
    std::cout << "║    'q' = Çıkış                                        ║" << std::endl;
    std::cout << "║    's' = Screenshot kaydet                            ║" << std::endl;
    std::cout << "║    'r' = İstatistikleri sıfırla                       ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝\n" << std::endl;

    std::cout << "⏳ Hazırlanıyor..." << std::endl;

    // Load camera calibration (if available)
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
                // Use pre-calculated optimal matrix from calibration
                optimal_camera_matrix = optimal_from_file;
                double fx_opt = optimal_camera_matrix.at<double>(0, 0);
                double fy_opt = optimal_camera_matrix.at<double>(1, 1);

                std::cout << "✅ Kamera kalibrasyonu yüklendi: " << calib_file << std::endl;
                std::cout << "   Lens distorsiyonu düzeltmesi AKTİF!" << std::endl;
                std::cout << "   Original fx/fy: " << std::fixed << std::setprecision(2)
                         << fx_orig << "/" << fy_orig << " (ratio: "
                         << std::setprecision(4) << (fy_orig/fx_orig) << ")" << std::endl;
                std::cout << "   ✨ Using PRE-CALCULATED optimal matrix from calibration!" << std::endl;
                std::cout << "   Optimal fx/fy: " << std::setprecision(2) << fx_opt << "/" << fy_opt
                         << " (ratio: " << std::setprecision(4) << (fy_opt/fx_opt) << ")" << std::endl;
            } else {
                // Fallback: Create optimal matrix manually (backward compatibility)
                double cx = camera_matrix.at<double>(0, 2);
                double cy = camera_matrix.at<double>(1, 2);
                double f_avg = (fx_orig + fy_orig) / 2.0;

                optimal_camera_matrix = (cv::Mat_<double>(3, 3) <<
                    f_avg, 0, cx,
                    0, f_avg, cy,
                    0, 0, 1
                );

                std::cout << "✅ Kamera kalibrasyonu yüklendi: " << calib_file << std::endl;
                std::cout << "   Lens distorsiyonu düzeltmesi AKTİF!" << std::endl;
                std::cout << "   Original fx/fy: " << std::fixed << std::setprecision(2)
                         << fx_orig << "/" << fy_orig << " (ratio: "
                         << std::setprecision(4) << (fy_orig/fx_orig) << ")" << std::endl;
                std::cout << "   ⚠️  Optimal matrix not in YML, creating manually..." << std::endl;
                std::cout << "   MANUAL Square Pixel Matrix: fx = fy = "
                         << std::setprecision(2) << f_avg << std::endl;
            }
        } else {
            std::cout << "⚠️  Kalibrasyon dosyası bozuk, kullanılmayacak." << std::endl;
        }
    } else {
        std::cout << "⚠️  Kalibrasyon dosyası bulunamadı: " << calib_file << std::endl;
        std::cout << "   Lens distorsiyonu düzeltmesi KAPALI (ham kamera görüntüsü)" << std::endl;
        std::cout << "   Kalibrasyon için: ./camera_calibration çalıştırın" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "▶️  Başladı!\n" << std::endl;

    // ArUco setup
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    // İstatistikler
    int frame_count = 0;
    int detection_count = 0;
    int four_markers_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    // Çıktı boyutu (bird's eye view) - Input aspect ratio'yu koru (square pixels!)
    const int OUTPUT_WIDTH = 800;

    std::cout << "📐 Bird's Eye View will preserve INPUT aspect ratio (square pixels)" << std::endl;

    while (true) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) break;

        frame_count++;

        // Apply lens distortion correction (if calibration available)
        cv::Mat frame_undistorted;
        if (calibration_loaded) {
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
        } else {
            frame_undistorted = frame.clone();
        }

        // ArUco detection (on undistorted frame)
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

        detector.detectMarkers(frame_undistorted, markerCorners, markerIds, rejectedCandidates);

        // Kopyalar (visualization için)
        cv::Mat display = frame_undistorted.clone();
        cv::Mat warped;

        // Tespit edilen marker sayısı
        std::string status_text;
        cv::Scalar status_color;

        if (!markerIds.empty()) {
            detection_count++;

            // Marker'ları çiz
            cv::aruco::drawDetectedMarkers(display, markerCorners, markerIds);

            // ID'leri kontrol et
            bool has_0 = false, has_1 = false, has_2 = false, has_3 = false;

            for (int id : markerIds) {
                if (id == 0) has_0 = true;
                if (id == 1) has_1 = true;
                if (id == 2) has_2 = true;
                if (id == 3) has_3 = true;
            }

            status_text = "Markers: ";
            if (has_0) status_text += "[0] ";
            if (has_1) status_text += "[1] ";
            if (has_2) status_text += "[2] ";
            if (has_3) status_text += "[3] ";

            // 4 marker bulundu mu?
            if (has_0 && has_1 && has_2 && has_3) {
                four_markers_count++;
                status_color = cv::Scalar(0, 255, 0); // Green
                status_text += ">>> ALL 4 DETECTED! <<<";

                // Köşe noktalarını bul (marker dış köşeleri)
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
                    if (markerIds[i] == 0) target = 0;       // Top-Left marker
                    else if (markerIds[i] == 1) target = 1;  // Top-Right marker
                    else if (markerIds[i] == 2) target = 2;  // Bottom-Right marker
                    else if (markerIds[i] == 3) target = 3;  // Bottom-Left marker

                    if (target >= 0) {
                        corners[target] = best_corner;
                        corner_found[target] = true;
                    }
                }

                if (!corner_found[0] || !corner_found[1] || !corner_found[2] || !corner_found[3]) {
                    status_color = cv::Scalar(0, 165, 255);
                    status_text = "Marker corners missing";
                    continue;
                }

                // Köşeleri sırala (güvenlik için)
                corners = orderPoints(corners);

                // Calculate output height based on INPUT frame aspect ratio (preserve square pixels!)
                float input_aspect = (float)frame_undistorted.rows / frame_undistorted.cols;
                int current_output_height = static_cast<int>(OUTPUT_WIDTH * input_aspect);

                // Kontur çiz (kağıt sınırı) - KALIN YEŞİL ÇİZGİ
                for (int i = 0; i < 4; i++) {
                    cv::line(display, corners[i], corners[(i+1)%4],
                            cv::Scalar(0, 255, 0), 4);
                    cv::circle(display, corners[i], 10, cv::Scalar(255, 0, 0), -1);
                    cv::circle(display, corners[i], 12, cv::Scalar(0, 255, 0), 2);
                }

                // Perspective transform - Bird's eye view
                std::vector<cv::Point2f> dst_points = {
                    cv::Point2f(0, 0),
                    cv::Point2f(OUTPUT_WIDTH - 1, 0),
                    cv::Point2f(OUTPUT_WIDTH - 1, current_output_height - 1),
                    cv::Point2f(0, current_output_height - 1)
                };

                cv::Mat transform_matrix = cv::getPerspectiveTransform(corners, dst_points);
                cv::warpPerspective(frame_undistorted, warped, transform_matrix,
                                   cv::Size(OUTPUT_WIDTH, current_output_height));

            } else {
                status_color = cv::Scalar(0, 165, 255); // Orange
                int missing = 4 - (has_0 + has_1 + has_2 + has_3);
                status_text += "(" + std::to_string(missing) + " missing)";
            }

        } else {
            status_text = "NO MARKERS DETECTED";
            status_color = cv::Scalar(0, 0, 255); // Red
        }

        // FPS hesapla
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        double fps = (elapsed > 0) ? (double)frame_count / elapsed : 0;

        // Bilgileri ekrana yaz - Daha büyük ve okunabilir
        cv::rectangle(display, cv::Point(0, 0), cv::Point(display.cols, 140),
                     cv::Scalar(20, 20, 20), -1);
        cv::rectangle(display, cv::Point(0, 0), cv::Point(display.cols, 140),
                     status_color, 3);

        cv::putText(display, status_text, cv::Point(15, 35),
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2);

        std::string info = "Camera " + std::to_string(camera_index) +
                          " | FPS: " + std::to_string((int)fps) +
                          " | Frames: " + std::to_string(frame_count);
        if (calibration_loaded) {
            info += " | CALIBRATED | SQUARE PIXELS";
        }
        cv::putText(display, info, cv::Point(15, 70),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

        // İstatistikler
        double detection_rate = frame_count > 0 ?
            (double)detection_count / frame_count * 100 : 0;
        double four_marker_rate = frame_count > 0 ?
            (double)four_markers_count / frame_count * 100 : 0;

        std::string stats = "Detection: " + std::to_string((int)detection_rate) +
                           "% | Perfect (4/4): " + std::to_string((int)four_marker_rate) + "%";
        cv::putText(display, stats, cv::Point(15, 100),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(100, 255, 255), 2);

        cv::putText(display, "'q'=Quit  's'=Screenshot  'r'=Reset Stats",
                   cv::Point(15, 130),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);

        // Göster
        cv::imshow("ArUco Detection Test - Camera " + std::to_string(camera_index), display);

        if (!warped.empty()) {
            cv::Mat warped_display = warped.clone();

            int warped_width = warped.cols;
            int warped_height = warped.rows;
            int center_x = warped_width / 2;
            int center_y = warped_height / 2;
            int radius = std::max(10, std::min(warped_width, warped_height) / 6);

            // Aspect ratio doğrulaması için rehber çizimler
            cv::circle(warped_display, cv::Point(center_x, center_y), radius,
                       cv::Scalar(255, 0, 255), 3);
            cv::line(warped_display, cv::Point(center_x - radius, center_y),
                     cv::Point(center_x + radius, center_y), cv::Scalar(0, 255, 255), 2);
            cv::line(warped_display, cv::Point(center_x, center_y - radius),
                     cv::Point(center_x, center_y + radius), cv::Scalar(0, 255, 255), 2);

            std::string title = calibration_loaded ?
                "Bird's Eye View (SQUARE PIXELS)" :
                "Bird's Eye View (Corrected)";
            cv::putText(warped_display, title,
                        cv::Point(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);

            if (calibration_loaded) {
                cv::putText(warped_display, "Circle should be PERFECTLY ROUND!",
                            cv::Point(20, 80),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            } else {
                cv::putText(warped_display, "Template Quality: GOOD",
                            cv::Point(20, 80),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            }

            std::ostringstream resolution_ss;
            resolution_ss << "Resolution: " << warped_width << "x" << warped_height;

            std::ostringstream info_ss;
            info_ss << "Square pixels preserved! Circles = ROUND!";

            cv::putText(warped_display, resolution_ss.str(),
                        cv::Point(20, warped_height - 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
            cv::putText(warped_display, info_ss.str(),
                        cv::Point(20, warped_height - 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

            cv::imshow("Bird's Eye View (Corrected)", warped_display);
        }

        // Klavye kontrolü
        char key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            break;
        } else if (key == 's' || key == 'S') {
            // Screenshot kaydet
            std::string timestamp = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
            std::string filename = "test_screenshot_" + timestamp + ".jpg";
            cv::imwrite(filename, display);

            if (!warped.empty()) {
                std::string warped_filename = "test_warped_" + timestamp + ".jpg";
                cv::imwrite(warped_filename, warped);
                std::cout << "✅ Screenshots saved: " << filename << ", " << warped_filename << std::endl;
            } else {
                std::cout << "✅ Screenshot saved: " << filename << std::endl;
            }
        } else if (key == 'r' || key == 'R') {
            // İstatistikleri sıfırla
            frame_count = 0;
            detection_count = 0;
            four_markers_count = 0;
            start_time = std::chrono::steady_clock::now();
            std::cout << "🔄 İstatistikler sıfırlandı." << std::endl;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    // Final istatistikler
    std::cout << "\n╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                  TEST SONUÇLARI                        ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║  Kamera ID:           " << std::setw(30) << std::left << camera_index << "║" << std::endl;
    std::cout << "║  Toplam Frame:        " << std::setw(30) << std::left << frame_count << "║" << std::endl;
    std::cout << "║  Any Marker Detected: " << std::setw(30) << std::left << detection_count << "║" << std::endl;
    std::cout << "║  All 4 Detected:      " << std::setw(30) << std::left << four_markers_count << "║" << std::endl;

    double detection_rate = frame_count > 0 ? (double)detection_count / frame_count * 100 : 0;
    double four_rate = frame_count > 0 ? (double)four_markers_count / frame_count * 100 : 0;

    std::cout << "║  Detection Rate:      " << std::setw(30) << std::left
              << (std::to_string((int)detection_rate) + "%") << "║" << std::endl;
    std::cout << "║  Perfect Rate (4/4):  " << std::setw(30) << std::left
              << (std::to_string((int)four_rate) + "%") << "║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝" << std::endl;

    // Değerlendirme
    std::cout << "\n📊 TEMPLATE PERFORMANS DEĞERLENDİRMESİ:\n" << std::endl;

    if (four_rate >= 95) {
        std::cout << "🏆 MÜKEMMEL! (A+)" << std::endl;
        std::cout << "   Template'iniz kusursuz çalışıyor!" << std::endl;
        std::cout << "   %95+ tespit oranı - gerçek zamanlı işleme için ideal." << std::endl;
        std::cout << "   ✅ Üretime hazır!" << std::endl;
    } else if (four_rate >= 85) {
        std::cout << "✅ ÇOK İYİ! (A)" << std::endl;
        std::cout << "   Template güvenilir bir şekilde çalışıyor." << std::endl;
        std::cout << "   %85+ tespit oranı - kullanılabilir." << std::endl;
        std::cout << "   ✅ Projeye devam edebilirsiniz." << std::endl;
    } else if (four_rate >= 70) {
        std::cout << "👍 İYİ (B)" << std::endl;
        std::cout << "   Template çalışıyor ama optimize edilebilir." << std::endl;
        std::cout << "   Öneriler:" << std::endl;
        std::cout << "   - Daha iyi aydınlatma kullanın" << std::endl;
        std::cout << "   - Marker'ları daha net yazdırın" << std::endl;
    } else if (four_rate >= 50) {
        std::cout << "⚠️  ORTA (C)" << std::endl;
        std::cout << "   Kullanılabilir ama iyileştirme şart." << std::endl;
        std::cout << "   Kontrol edin:" << std::endl;
        std::cout << "   - Aydınlatma düzgün mü?" << std::endl;
        std::cout << "   - Kağıt düz mü?" << std::endl;
        std::cout << "   - Marker boyutları doğru mu? (2.5x2.5 cm)" << std::endl;
    } else {
        std::cout << "❌ DÜŞÜK (D/F)" << std::endl;
        std::cout << "   Template veya ortam ciddi iyileştirme gerekiyor." << std::endl;
        std::cout << "   ZORUNLU kontroller:" << std::endl;
        std::cout << "   ✗ Marker'lar DOĞRU yazdırıldı mı? (2.5cm x 2.5cm)" << std::endl;
        std::cout << "   ✗ Yazdırma kalitesi YETERLİ mi? (net, keskin)" << std::endl;
        std::cout << "   ✗ Kağıt BEYAZ mi?" << std::endl;
        std::cout << "   ✗ Aydınlatma İYİ mi? (gölgesiz)" << std::endl;
        std::cout << "   ✗ Mesafe UYGUN mu? (30-60cm)" << std::endl;
    }

    std::cout << "\n" << std::string(60, '=') << std::endl;

    return 0;
}
