# 🎨 Drawing Shape Recognition

Real-time geometric shape detection and quality assessment using OpenCV and ArUco markers.

## 📖 Project Description

This project analyzes real-time video feed to detect and classify hand-drawn geometric shapes on paper. It uses ArUco markers for perspective correction and OpenCV for shape detection and classification.

### 🎯 Features

- ✅ Real-time video processing (webcam or video file)
- ✅ Bird's eye view transformation using ArUco markers
- ✅ Shape detection: Circle, Triangle, Square, Rectangle, Hexagon
- ✅ Shape quality assessment (perfectness score)
- ✅ Bounding box visualization with labels
- ✅ Video and text file output

### 👥 Team Members

- **Buse Yüsra Köse**
- **Serdar Sarı**
- **Taha Yiğit Göksu**
- **Muhammet Ali Yavuzyiğit**

### 🛠️ Technologies

- **Language:** C++17
- **Library:** OpenCV 4.12.0 (with contrib/ArUco)
- **Build System:** CMake 3.20+
- **Platform:** macOS (M1/M2), Linux, Windows

## 📋 Requirements

### System Requirements
- C++17 compatible compiler
- CMake 3.20 or higher
- OpenCV 4.x with contrib modules (ArUco)

### Hardware Requirements
- Webcam (recommended: 720p @ 30fps or higher)
- Printer (for ArUco markers)