#include <iostream>
#include <opencv2/opencv.hpp> // OpenCV'yi test etmek için ekleyelim

int main() {
    // OpenCV versiyonunu yazdırarak kütüphanenin çalıştığını test edelim
    std::cout << "OpenCV projesi basladi. Version: " << CV_VERSION << std::endl;

    // Bir pencere oluşturmayı deneyelim
    cv::namedWindow("Test Penceresi", cv::WINDOW_AUTOSIZE);
    cv::waitKey(0); // Bir tuşa basılana kadar bekle

    return 0;
}