cat > README.md << 'EOF'
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
- **Muhammed Ali Yavuzyiğit**

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

## 🚀 Installation

### macOS (M1/M2)
```bash
# Install Homebrew (if not installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install dependencies
brew install cmake opencv

# Clone repository
git clone https://github.com/YOUR_USERNAME/DrawingShapeRecognition.git
cd DrawingShapeRecognition

# Build
mkdir build && cd build
cmake ..
make

# Run
./shape_recognition
```

### Linux (Ubuntu/Debian)
```bash
# Install dependencies
sudo apt update
sudo apt install build-essential cmake libopencv-dev libopencv-contrib-dev

# Clone and build (same as macOS)
```

### Windows

See `docs/INSTALLATION_WINDOWS.md`

## 📖 Usage

### Basic Usage
```bash
# Interactive camera selection
./shape_recognition

# Specify camera index
./shape_recognition 0

# Use video file
./shape_recognition video.mp4
```

### Preparing ArUco Markers

1. Print markers from `aruco_markers/template.pdf`
2. Cut out the 4 markers
3. Paste them on the corners of white A4 paper
4. Draw shapes in the middle area
5. Keep at least 3cm margin from markers

## 🏗️ Project Architecture
```
DrawingShapeRecognition/
├── src/                      # Source files
│   ├── main.cpp
│   ├── VideoCapture.cpp      # Camera/video handling
│   ├── PerspectiveTransformer.cpp  # ArUco & bird's eye view
│   ├── ShapeDetector.cpp     # Contour detection
│   ├── ShapeClassifier.cpp   # Shape classification
│   └── OutputManager.cpp     # Save video & logs
├── include/                  # Header files
├── aruco_markers/            # Printable ArUco markers
├── output/                   # Generated outputs
│   ├── videos/
│   └── logs/
└── docs/                     # Documentation
```

## 📚 Documentation

- [Installation Guide](docs/INSTALLATION.md)
- [Usage Guide](docs/USAGE.md)
- [Architecture](docs/ARCHITECTURE.md)
- [API Reference](docs/API.md)

## 🧪 Testing
```bash
# Run tests
cd build
./run_tests
```

## 📊 Project Status

- [x] Week 1: Project setup and VideoCapture
- [ ] Week 2: ArUco detection and perspective transform
- [ ] Week 3: Shape detection
- [ ] Week 4: Shape classification
- [ ] Week 5: Quality assessment
- [ ] Week 6: Output and testing
- [ ] Week 7: Documentation and demo

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Course:** SWE 417 - Digital Image Processing
- **Instructor:** [Instructor Name]
- **Institution:** [University Name]

## 📧 Contact

For questions or issues, please open an issue on GitHub or contact the team members.

---

Made with ❤️ by Team [Your Team Name]
EOF