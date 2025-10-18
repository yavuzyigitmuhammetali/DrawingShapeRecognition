#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include <thread>

// Kamera bilgisi struct
struct CameraInfo {
    int index;
    int width;
    int height;
    int fps;
    std::string backend;
    std::string description;
};

// Config dosyasını oku
struct CharucoBoardConfig {
    int squares_x = 7;
    int squares_y = 5;
    float square_length = 0.03f;  // meters
    float marker_length = 0.02f;  // meters
    cv::aruco::PredefinedDictionaryType dict_type = cv::aruco::DICT_4X4_50;
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

// Kullanıcıya kamera seçtir
int selectCamera() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║         KAMERA KALİBRASYONU - Kamera Seçimi          ║" << std::endl;
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

// Config dosyasını oku
CharucoBoardConfig loadConfig(const std::string& filename) {
    CharucoBoardConfig config;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cout << "⚠️  Config dosyası bulunamadı, default değerler kullanılıyor." << std::endl;
        return config;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line[0] == '#' || line.empty()) continue;

        size_t pos = line.find('=');
        if (pos != std::string::npos) {
            std::string key = line.substr(0, pos);
            std::string value = line.substr(pos + 1);

            if (key == "SQUARES_X") config.squares_x = std::stoi(value);
            else if (key == "SQUARES_Y") config.squares_y = std::stoi(value);
            else if (key == "SQUARE_LENGTH") config.square_length = std::stof(value);
            else if (key == "MARKER_LENGTH") config.marker_length = std::stof(value);
            else if (key == "ARUCO_DICT") config.dict_type = static_cast<cv::aruco::PredefinedDictionaryType>(std::stoi(value));
        }
    }

    file.close();
    return config;
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║        INTERACTIVE CAMERA CALIBRATION TOOL            ║" << std::endl;
    std::cout << "║              ChArUco Board Method                      ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝\n" << std::endl;

    // Config yükle
    CharucoBoardConfig config = loadConfig("tools/charuco_board_config.txt");

    std::cout << "📋 Board Configuration:" << std::endl;
    std::cout << "  Squares: " << config.squares_x << "x" << config.squares_y << std::endl;
    std::cout << "  Square: " << (config.square_length * 100) << " cm" << std::endl;
    std::cout << "  Marker: " << (config.marker_length * 100) << " cm\n" << std::endl;

    // Kamera seç
    int camera_index = selectCamera();
    if (camera_index < 0) return -1;

    std::cout << "\n🎥 Kamera açılıyor..." << std::endl;
    cv::VideoCapture cap(camera_index);

    if (!cap.isOpened()) {
        std::cerr << "❌ Kamera açılamadı!" << std::endl;
        return -1;
    }

    // ChArUco board setup
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(config.dict_type);
    cv::Ptr<cv::aruco::CharucoBoard> board = new cv::aruco::CharucoBoard(
        cv::Size(config.squares_x, config.squares_y),
        config.square_length,
        config.marker_length,
        dictionary
    );
    cv::aruco::CharucoDetector detector(*board);

    std::cout << "\n╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                    TALİMATLAR                          ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║  1. Yazdırılmış ChArUco board'u kameraya gösterin     ║" << std::endl;
    std::cout << "║  2. Board tamamen görünür olmalı (4 köşe de dahil)    ║" << std::endl;
    std::cout << "║  3. En az 15 farklı pozisyondan fotoğraf çekin        ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  ÖNEMLİ POZİSYONLAR:                                  ║" << std::endl;
    std::cout << "║    • Düz (merkez)                                     ║" << std::endl;
    std::cout << "║    • Sola yatık                                       ║" << std::endl;
    std::cout << "║    • Sağa yatık                                       ║" << std::endl;
    std::cout << "║    • Yukarı yatık                                     ║" << std::endl;
    std::cout << "║    • Aşağı yatık                                      ║" << std::endl;
    std::cout << "║    • Dört köşeden (çapraz)                            ║" << std::endl;
    std::cout << "║    • Yakın mesafe                                     ║" << std::endl;
    std::cout << "║    • Orta mesafe                                      ║" << std::endl;
    std::cout << "║    • Uzak mesafe                                      ║" << std::endl;
    std::cout << "║                                                        ║" << std::endl;
    std::cout << "║  KLAVYE:                                               ║" << std::endl;
    std::cout << "║    SPACE  = Fotoğraf çek                              ║" << std::endl;
    std::cout << "║    'c'    = Kalibrasyonu başlat (min 10 fotoğraf)    ║" << std::endl;
    std::cout << "║    'r'    = Tüm fotoğrafları sil, yeniden başla       ║" << std::endl;
    std::cout << "║    'q'    = İptal ve çık                              ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝\n" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "▶️  Başladı! Board'u gösterin...\n" << std::endl;

    // Kalibrasyon verileri
    std::vector<std::vector<cv::Point2f>> all_charuco_corners;
    std::vector<std::vector<int>> all_charuco_ids;
    cv::Size image_size;

    const int MIN_IMAGES = 10;
    const int RECOMMENDED_IMAGES = 20;

    // Ana döngü
    bool running = true;
    while (running) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) break;

        if (image_size.width == 0) {
            image_size = frame.size();
        }

        cv::Mat display = frame.clone();

        // ChArUco detection
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<cv::Point2f> charuco_corners;
        std::vector<int> charuco_ids;

        detector.detectBoard(frame, charuco_corners, charuco_ids, marker_corners, marker_ids);

        // Status bar
        cv::Scalar status_color;
        std::string status_text;

        int captured_count = all_charuco_corners.size();

        if (charuco_corners.size() > 3) {
            status_color = cv::Scalar(0, 255, 0);
            status_text = "BOARD DETECTED - Press SPACE to capture";

            // Draw detections
            if (!marker_ids.empty()) {
                cv::aruco::drawDetectedMarkers(display, marker_corners, marker_ids);
            }
            if (!charuco_corners.empty()) {
                cv::aruco::drawDetectedCornersCharuco(display, charuco_corners, charuco_ids);
            }
        } else {
            status_color = cv::Scalar(0, 0, 255);
            status_text = "NO BOARD - Show full board to camera";
        }

        // Info overlay
        cv::rectangle(display, cv::Point(0, 0), cv::Point(display.cols, 150),
                     cv::Scalar(20, 20, 20), -1);
        cv::rectangle(display, cv::Point(0, 0), cv::Point(display.cols, 150),
                     status_color, 3);

        cv::putText(display, status_text, cv::Point(15, 35),
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2);

        // Progress
        std::string progress = "Captured: " + std::to_string(captured_count) + " / " +
                               std::to_string(RECOMMENDED_IMAGES) + " (min: " +
                               std::to_string(MIN_IMAGES) + ")";
        cv::Scalar progress_color = captured_count >= MIN_IMAGES ?
                                    cv::Scalar(0, 255, 0) : cv::Scalar(100, 255, 255);
        cv::putText(display, progress, cv::Point(15, 70),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, progress_color, 2);

        // Progress bar
        int bar_width = display.cols - 40;
        float progress_ratio = std::min(1.0f, (float)captured_count / RECOMMENDED_IMAGES);
        cv::rectangle(display, cv::Point(20, 85), cv::Point(20 + bar_width, 105),
                     cv::Scalar(60, 60, 60), -1);
        cv::rectangle(display, cv::Point(20, 85),
                     cv::Point(20 + (int)(bar_width * progress_ratio), 105),
                     progress_color, -1);

        // Instructions
        cv::putText(display, "SPACE=Capture  'c'=Calibrate  'r'=Reset  'q'=Quit",
                   cv::Point(15, 130), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                   cv::Scalar(200, 200, 200), 1);

        cv::imshow("Camera Calibration", display);

        char key = cv::waitKey(1);

        if (key == ' ' && charuco_corners.size() > 3) {
            // Capture
            all_charuco_corners.push_back(charuco_corners);
            all_charuco_ids.push_back(charuco_ids);

            std::cout << "📸 Fotoğraf " << captured_count + 1 << " çekildi! ";

            // Pozisyon önerisi
            if (captured_count < 9) {
                std::string suggestions[] = {
                    "Şimdi: Sola yatırın",
                    "Şimdi: Sağa yatırın",
                    "Şimdi: Yukarı yatırın",
                    "Şimdi: Aşağı yatırın",
                    "Şimdi: Sol üst köşeden gösterin",
                    "Şimdi: Sağ üst köşeden gösterin",
                    "Şimdi: Sol alt köşeden gösterin",
                    "Şimdi: Sağ alt köşeden gösterin",
                    "Şimdi: Yakından gösterin"
                };
                std::cout << suggestions[captured_count];
            }
            std::cout << std::endl;

        } else if (key == 'c' || key == 'C') {
            if (captured_count < MIN_IMAGES) {
                std::cout << "⚠️  En az " << MIN_IMAGES << " fotoğraf gerekli! (Şu an: "
                         << captured_count << ")" << std::endl;
            } else {
                std::cout << "\n🔧 Kalibrasyon başlatılıyor..." << std::endl;
                running = false;
            }
        } else if (key == 'r' || key == 'R') {
            all_charuco_corners.clear();
            all_charuco_ids.clear();
            std::cout << "🔄 Tüm fotoğraflar silindi. Yeniden başlayın." << std::endl;
        } else if (key == 'q' || key == 'Q') {
            std::cout << "\n❌ Kalibrasyon iptal edildi." << std::endl;
            return 0;
        }
    }

    // Kalibrasyon hesaplama
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "KALIBRASYON HESAPLANIYOR..." << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "  Toplam fotoğraf: " << all_charuco_corners.size() << std::endl;
    std::cout << "  Görüntü boyutu: " << image_size.width << "x" << image_size.height << std::endl;

    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    try {
        double reprojection_error = cv::aruco::calibrateCameraCharuco(
            all_charuco_corners,
            all_charuco_ids,
            board,
            image_size,
            camera_matrix,
            dist_coeffs,
            rvecs,
            tvecs
        );

        std::cout << "\n✅ Kalibrasyon başarılı!" << std::endl;
        std::cout << "  Reprojection error: " << reprojection_error << " pixels" << std::endl;

        // Sonuçları göster
        std::cout << "\n📊 KALIBRASYON SONUÇLARI:" << std::endl;
        std::cout << std::string(60, '-') << std::endl;
        std::cout << "Camera Matrix (focal length & principal point):" << std::endl;
        std::cout << camera_matrix << std::endl;
        std::cout << "\nDistortion Coefficients (k1, k2, p1, p2, k3):" << std::endl;
        std::cout << dist_coeffs << std::endl;

        // Kalite değerlendirmesi
        std::cout << "\n📈 KALİTE DEĞERLENDİRMESİ:" << std::endl;
        if (reprojection_error < 0.5) {
            std::cout << "  🏆 MÜKEMMEL! (< 0.5 px error)" << std::endl;
        } else if (reprojection_error < 1.0) {
            std::cout << "  ✅ ÇOK İYİ! (< 1.0 px error)" << std::endl;
        } else if (reprojection_error < 2.0) {
            std::cout << "  👍 İYİ (< 2.0 px error)" << std::endl;
        } else {
            std::cout << "  ⚠️  KABUL EDİLEBİLİR (ama daha iyi olabilir)" << std::endl;
        }

        // Create OPTIMAL square pixel matrix
        double fx = camera_matrix.at<double>(0, 0);
        double fy = camera_matrix.at<double>(1, 1);
        double cx = camera_matrix.at<double>(0, 2);
        double cy = camera_matrix.at<double>(1, 2);
        double f_avg = (fx + fy) / 2.0;

        cv::Mat optimal_camera_matrix = (cv::Mat_<double>(3, 3) <<
            f_avg, 0, cx,
            0, f_avg, cy,
            0, 0, 1
        );

        std::cout << "\n📐 OPTIMAL SQUARE PIXEL MATRIX:" << std::endl;
        std::cout << "   Original fx: " << fx << std::endl;
        std::cout << "   Original fy: " << fy << std::endl;
        std::cout << "   Optimal f (average): " << f_avg << std::endl;
        std::cout << "   → fx = fy = " << f_avg << " (perfect square pixels!)" << std::endl;

        // YML dosyasına kaydet
        std::string output_filename = "camera_calibration.yml";
        cv::FileStorage fs(output_filename, cv::FileStorage::WRITE);

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);

        fs << "calibration_date" << std::ctime(&now_time);
        fs << "camera_index" << camera_index;
        fs << "image_width" << image_size.width;
        fs << "image_height" << image_size.height;
        fs << "number_of_images" << (int)all_charuco_corners.size();
        fs << "reprojection_error" << reprojection_error;
        fs << "camera_matrix" << camera_matrix;
        fs << "distortion_coefficients" << dist_coeffs;
        fs << "optimal_camera_matrix" << optimal_camera_matrix;  // ← YENİ!
        fs << "pixel_aspect_ratio_correction_applied" << true;   // ← YENİ!

        fs.release();

        std::cout << "\n💾 Kalibrasyon dosyası kaydedildi: " << output_filename << std::endl;

        // Demo: Önce/Sonra gösterimi
        std::cout << "\n🔍 Distortion düzeltme demo'su gösteriliyor..." << std::endl;
        std::cout << "   (Herhangi bir tuşa basarak çıkın)" << std::endl;

        while (true) {
            cv::Mat frame;
            cap >> frame;
            if (frame.empty()) break;

            cv::Mat undistorted;
            cv::undistort(frame, undistorted, camera_matrix, dist_coeffs);

            // Side by side
            cv::Mat combined;
            cv::hconcat(frame, undistorted, combined);

            cv::putText(combined, "BEFORE (Original)", cv::Point(20, 40),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            cv::putText(combined, "AFTER (Undistorted)", cv::Point(frame.cols + 20, 40),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

            cv::imshow("Before/After - Press any key to exit", combined);

            if (cv::waitKey(1) >= 0) break;
        }

        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "✅ KALIBRASYON TAMAMLANDI!" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "\nNext steps:" << std::endl;
        std::cout << "  1. '" << output_filename << "' dosyası oluşturuldu" << std::endl;
        std::cout << "  2. Bu dosyayı shape recognition kodunda kullanın" << std::endl;
        std::cout << "  3. Her karede cv::undistort() ile düzeltme yapın" << std::endl;
        std::cout << std::string(60, '=') << std::endl;

    } catch (const cv::Exception& e) {
        std::cerr << "\n❌ Kalibrasyon hatası: " << e.what() << std::endl;
        return -1;
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
