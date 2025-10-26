# 🎨 Drawing Shape Recognition

Real-time detection, classification, and quality analysis of hand-drawn geometric shapes for the SWE 417 Digital Image Processing course project.

## 🔍 Overview
- Processes live webcam feeds or prerecorded videos.
- Uses four printed ArUco markers to build a bird's-eye view of an A4 sheet (21 × 29.7 cm) and isolates its drawing area.
- Segments candidate contours, classifies shapes (circle, triangle, square, rectangle, hexagon), and scores drawing neatness.
- Annotates both the normalized and original views while logging detections and recording annotated clips.

## 🚀 Key Capabilities
- **Flexible input**: command-line argument for camera index or video path, with automatic camera scanning/prompting when none is supplied.
- **Precise rectification**: bird's-eye view computed from marker IDs 0–3 sized 1.8 cm; spacing: 13.7 cm (horizontal) and 22.45 cm (vertical).
- **Robust segmentation**: grayscale conversion, Gaussian blur, Canny edges, and morphological closing constrained by the transformed paper mask.
- **Deterministic classification**: polygon approximation + circularity checks discriminate among circle, triangle, square, rectangle, and hexagon.
- **Neatness scoring**: solidity, edge straightness, and shape-specific affinity metrics produce a 0–100 score and grade (Excellent/Very Good/Good/Acceptable/Needs Improvement).
- **Temporal stabilization**: centroid-based tracking aggregates per-shape history to eliminate flicker and average quality over time.
- **Smart output management**: detections appended to `output/logs/detections_<timestamp>.txt`; annotated videos stored under `output/videos/` only while markers are visible (2 s buffer, 5 s minimum duration).

## 🛠 Requirements
- C++17-compatible compiler (tested with Clang/GCC).
- CMake 3.20 or newer.
- OpenCV 4.x **with** ArUco (`opencv_contrib`) modules available.
- Webcam capable of 720p@30fps (recommended) or a prerecorded test video.

### Install dependencies
**macOS (Homebrew)**
```bash
brew install cmake opencv
```

**Ubuntu / Debian**
```bash
sudo apt update
sudo apt install build-essential cmake libopencv-dev libopencv-contrib-dev
```

**Windows**
- Install CMake and OpenCV (with contrib/aruco) and ensure their bin/include/lib folders are on your PATH/CMAKE_PREFIX_PATH.

## 🧱 Build
```bash
mkdir -p build
cd build
cmake ..
cmake --build .
```

Two executables are produced:
- `shape_recognition` – the main application.
- `aruco_birdseye_view` – a calibration/diagnostic tool for verifying marker detection and warping.

## ▶️ Run
```bash
./shape_recognition          # Scan cameras and ask which one to use
./shape_recognition 0        # Use camera index 0
./shape_recognition sample.mp4
```

- `ESC` closes the application.
- `aruco_birdseye_view [camera_id]` launches the diagnostic helper (SPACE toggles measurements, `s` saves a frame).

## 🧾 Preparing the Template
1. Print `aruco_markers/template.pdf` on portrait-oriented A4 paper without scaling.
2. Marker IDs must remain in their original order: 0 (top-left), 1 (top-right), 2 (bottom-right), 3 (bottom-left).
3. Physical measurements the pipeline expects:
   - Marker squares: `1.8 cm × 1.8 cm`.
   - Horizontal center-to-center distance (TL ↔ TR, BL ↔ BR): `13.7 cm`.
   - Vertical center-to-center distance (TL ↔ BL, TR ↔ BR): `22.45 cm`.
4. Draw shapes with a dark 1 mm felt-tip pen inside the inner rectangle; leave margin around the markers so they remain fully visible.
5. Provide diffuse lighting to reduce glare and keep the paper roughly parallel to the camera.

## 📤 Outputs
- **Camera View**: original feed with darkened surroundings and the detected paper outline.
- **Bird's Eye View**: perspective-corrected drawing surface with overlays.
- **Bounding boxes & contours**: colored by shape type; label format `Shape | XX% (Grade)`.
- **Logs**: appended to `output/logs/detections_<timestamp>.txt` (auto-created on first run).
- **Videos**: annotated recordings in `output/videos/output_<timestamp>.mp4`. Clips shorter than 5 seconds are discarded automatically.

## 📐 Quality Grades
| Grade           | Threshold (display score) |
|-----------------|---------------------------|
| Excellent       | ≥ 85                      |
| Very Good       | ≥ 70                      |
| Good            | ≥ 55                      |
| Acceptable      | ≥ 40                      |
| Needs Improvement | < 40                   |

`displayScore` is rounded up to the nearest 10, while internal scoring keeps full precision for stabilization.

## 🧩 Architecture Snapshot
| Component | Responsibility |
|-----------|----------------|
| `AppController` | Initializes the pipeline, handles CLI input, manages streaming loop, runs stabilization, and orchestrates annotation. |
| `VideoSource` | Opens/reads camera or file streams, maintaining frame size metadata. |
| `PerspectiveTransformer` | Detects ArUco markers, builds the bird's-eye homography using measured dimensions, and produces masks. |
| `ShapeSegmenter` | Generates candidate contours via grayscale filtering, edge detection, and morphological cleanup. |
| `ShapeClassifier` | Classifies candidates using polygon approximation heuristics and circularity metrics. |
| `ShapeQualityAnalyzer` | Scores drawing neatness with Solidity/straightness/shape-affinity metrics and applies grade thresholds. |
| `OutputManager` | Persists detection logs, manages buffered video recording, and ensures output directories exist. |

## 📁 Repository Layout
```
.
├── CMakeLists.txt
├── include/                # Public headers for each module
├── src/                    # Core implementation files
│   ├── AppController.cpp
│   ├── OutputManager.cpp
│   ├── PerspectiveTransformer.cpp
│   ├── ShapeClassifier.cpp
│   ├── ShapeQualityAnalyzer.cpp
│   ├── ShapeSegmenter.cpp
│   ├── VideoSource.cpp
│   └── main.cpp
├── test/
│   └── 4_aruco_birdseye_view.cpp
├── aruco_markers/          # Printable markers and generator script
├── test_images/            # Placeholder for sample stills (empty by default)
├── test_videos/            # Placeholder for sample videos (empty by default)
└── output/                 # Generated logs/videos (created at runtime)
```

## 👥 Team
- Buse Yüsra Köse
- Serdar Sarı
- Taha Yiğit Göksu
- Muhammet Ali Yavuzyiğit

## 📚 Course
- **Course:** SWE 417 — Digital Image Processing
- **Institution:** To be filled per submission requirements

For issues or weekly progress tracking, follow the GitHub repository linked to this project.
