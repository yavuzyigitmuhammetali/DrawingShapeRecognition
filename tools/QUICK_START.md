# Quick Start: Camera Calibration

## 1. Generate Calibration Board (One Time)

```bash
cd tools
python3 1_generate_charuco_calibration_board.py
```

Print `charuco_calibration_board.pdf` at **100% scale** on A4 paper.

## 2. Calibrate Camera

**Interactive camera selection (recommended):**
```bash
cd ..
./build/calibrate_camera_charuco
```
The tool will scan and list all available cameras, then prompt you to select one.

**Or specify camera directly:**
```bash
./build/calibrate_camera_charuco 0  # Use camera 0
./build/calibrate_camera_charuco 1 calibration_data/camera1_calibration.yml  # Camera 1
```

**Calibration process:**
- The tool will scan for available cameras
- Select your camera from the list
- Press `SPACE` to capture frames (15-20 minimum)
- Vary angles and distances
- Press `c` when ready to calibrate
- Target: reprojection error < 0.5 pixels

## 3. Generate Test Paper (One Time)

```bash
cd aruco_markers
python3 generate_markers.py
```

Print `template.pdf` at **100% scale** on A4 paper.
Draw a **perfect circle** in the center using a compass.

## 4. Test Calibration

**Interactive camera selection:**
```bash
cd ..
./build/test_calibration_accuracy
```

**Or specify camera directly:**
```bash
./build/test_calibration_accuracy 0  # Use camera 0
./build/test_calibration_accuracy 1 calibration_data/camera1_calibration.yml  # Camera 1
```

- Show test paper to camera
- Check diameter ratio (target: ~1.0000)
- Press `s` to save results
- Press `ESC` to exit

## 5. Use in Your Project

```cpp
#include <opencv2/opencv.hpp>

// Load calibration
cv::Mat cameraMatrix, distCoeffs;
cv::FileStorage fs("calibration_data/camera_calibration.yml", cv::FileStorage::READ);
fs["camera_matrix"] >> cameraMatrix;
fs["distortion_coefficients"] >> distCoeffs;
fs.release();

// Undistort frame
cv::Mat frame, undistorted;
cap >> frame;
cv::undistort(frame, undistorted, cameraMatrix, distCoeffs);
```

## Quality Targets

- **Reprojection Error:** < 0.5 pixels (excellent), < 1.0 pixels (good)
- **Circle Diameter Ratio:** 1.0000 ± 0.01 (excellent), ± 0.02 (good)

See `CALIBRATION_GUIDE.md` for detailed instructions.
