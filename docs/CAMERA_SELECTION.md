# Camera Selection Guide

## Overview

Both calibration tools support multiple cameras and provide automatic camera detection and selection.

## Usage Methods

### 1. Interactive Selection (Recommended)

Simply run the tool without arguments:

```bash
./build/calibrate_camera_charuco
./build/test_calibration_accuracy
```

The tool will:
1. Scan for available cameras (0-9)
2. Display each camera with its resolution and backend
3. Prompt you to select which camera to use

**Example output:**
```
Scanning for available cameras...
  Camera 0: 1280x720 (AVFoundation)
  Camera 1: 1920x1080 (AVFoundation)
  Camera 2: 640x480 (AVFoundation)

Multiple cameras detected. Please select:
  [0] Camera 0
  [1] Camera 1
  [2] Camera 2

Enter selection (0-2): 1
```

### 2. Specify Camera via Command Line

**Calibration tool:**
```bash
# Use camera 0 with default output file
./build/calibrate_camera_charuco 0

# Use camera 1 with specific output file
./build/calibrate_camera_charuco 1 calibration_data/camera1_calibration.yml

# Use camera 2 with custom name
./build/calibrate_camera_charuco 2 calibration_data/external_camera.yml
```

**Test tool:**
```bash
# Use camera 0 with default calibration file
./build/test_calibration_accuracy 0

# Use camera 1 with specific calibration file
./build/test_calibration_accuracy 1 calibration_data/camera1_calibration.yml

# Use camera 2 with custom calibration file
./build/test_calibration_accuracy 2 calibration_data/external_camera.yml
```

## Common Camera Setups

### MacBook (Built-in + External)

```bash
# Camera 0: Built-in FaceTime HD Camera
./build/calibrate_camera_charuco 0 calibration_data/builtin_camera.yml

# Camera 1: External USB Camera
./build/calibrate_camera_charuco 1 calibration_data/usb_camera.yml

# Camera 2: External USB Camera 2
./build/calibrate_camera_charuco 2 calibration_data/usb_camera2.yml
```

### Desktop with Multiple Webcams

```bash
# Camera 0: Primary webcam
./build/calibrate_camera_charuco 0 calibration_data/primary_webcam.yml

# Camera 1: Secondary webcam
./build/calibrate_camera_charuco 1 calibration_data/secondary_webcam.yml
```

## Best Practices

### 1. Naming Convention

Use descriptive names for calibration files:

```
calibration_data/
├── camera0_builtin_720p.yml       # Built-in camera at 720p
├── camera1_logitech_1080p.yml     # External Logitech at 1080p
├── camera2_usb_640p.yml           # USB camera at 640p
└── README.md
```

### 2. Calibrate Each Camera Separately

Each camera has unique lens distortion characteristics:

```bash
# Calibrate built-in camera
./build/calibrate_camera_charuco 0 calibration_data/camera0_builtin.yml

# Calibrate external camera
./build/calibrate_camera_charuco 1 calibration_data/camera1_external.yml
```

### 3. Test with Correct Calibration File

Always test a camera with its own calibration file:

```bash
# Test built-in camera
./build/test_calibration_accuracy 0 calibration_data/camera0_builtin.yml

# Test external camera
./build/test_calibration_accuracy 1 calibration_data/camera1_external.yml
```

### 4. Match Resolution

If you change camera resolution, recalibrate:

```bash
# Calibrated at 1280x720
./build/calibrate_camera_charuco 0 calibration_data/camera0_720p.yml

# If switching to 1920x1080, recalibrate
./build/calibrate_camera_charuco 0 calibration_data/camera0_1080p.yml
```

## Troubleshooting

### Camera Not Detected

**Problem:** Camera exists but not listed

**Solutions:**
- Ensure camera is connected and turned on
- Check camera permissions (macOS: System Preferences → Security & Privacy → Camera)
- Close other applications using the camera
- Try unplugging and reconnecting USB cameras
- Restart the computer

### Wrong Camera Selected

**Problem:** Selected wrong camera interactively

**Solutions:**
- Press Ctrl+C to exit
- Restart with camera ID specified:
  ```bash
  ./build/calibrate_camera_charuco 1  # Force camera 1
  ```

### Camera Index Changed

**Problem:** Camera IDs change after reconnecting

**Solutions:**
- Always verify camera with interactive selection first
- Note the resolution and backend to identify cameras
- Reconnect cameras in same order
- Use USB hub to maintain consistent port assignment

### Multiple Identical Cameras

**Problem:** Can't distinguish between identical camera models

**Solutions:**
- Use interactive selection and test each one
- Connect cameras one at a time
- Label cameras physically (e.g., "Camera A", "Camera B")
- Create test frame with each camera to identify

## Camera Information

### Camera Backends

Common backends you might see:

- **AVFoundation** (macOS): Native macOS camera framework
- **V4L2** (Linux): Video4Linux2
- **DirectShow** (Windows): Microsoft DirectShow
- **MSMF** (Windows): Microsoft Media Foundation

### Resolution Notes

- Tool requests 1280x720 by default
- Actual resolution depends on camera capabilities
- Higher resolution = more detail but slower processing
- Lower resolution = faster but less accurate

## Integration with Main Project

When using calibration in your main shape recognition program:

```cpp
// Load calibration for specific camera
int cameraId = 1;  // Your selected camera
cv::Mat cameraMatrix, distCoeffs;

std::string calibFile = "calibration_data/camera" +
                       std::to_string(cameraId) + "_calibration.yml";

cv::FileStorage fs(calibFile, cv::FileStorage::READ);
if (fs.isOpened()) {
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    // Open the same camera
    cv::VideoCapture cap(cameraId);

    // Undistort frames
    cv::Mat frame, undistorted;
    cap >> frame;
    cv::undistort(frame, undistorted, cameraMatrix, distCoeffs);
}
```

## Command Reference

### Calibration Tool

```bash
# Interactive
./build/calibrate_camera_charuco

# With camera ID
./build/calibrate_camera_charuco <camera_id>

# With camera ID and output file
./build/calibrate_camera_charuco <camera_id> <output_file.yml>

# With output file only (will prompt for camera)
./build/calibrate_camera_charuco <output_file.yml>
```

### Test Tool

```bash
# Interactive
./build/test_calibration_accuracy

# With camera ID
./build/test_calibration_accuracy <camera_id>

# With camera ID and calibration file
./build/test_calibration_accuracy <camera_id> <calibration_file.yml>

# With calibration file only (will prompt for camera)
./build/test_calibration_accuracy <calibration_file.yml>
```

## Examples

### Complete Workflow for Two Cameras

```bash
# 1. Generate calibration board (once)
cd tools
python3 1_generate_charuco_calibration_board.py
cd ..

# 2. Calibrate Camera 0 (built-in)
./build/calibrate_camera_charuco 0 calibration_data/camera0.yml
# Follow on-screen instructions, capture frames, press 'c'

# 3. Calibrate Camera 1 (external)
./build/calibrate_camera_charuco 1 calibration_data/camera1.yml
# Follow on-screen instructions, capture frames, press 'c'

# 4. Generate test paper (once)
cd aruco_markers
python3 generate_markers.py
# Draw circle in center
cd ..

# 5. Test Camera 0
./build/test_calibration_accuracy 0 calibration_data/camera0.yml
# Check ratio ≈ 1.0

# 6. Test Camera 1
./build/test_calibration_accuracy 1 calibration_data/camera1.yml
# Check ratio ≈ 1.0
```

### Quick Recalibration

```bash
# If you already know camera IDs
./build/calibrate_camera_charuco 1 calibration_data/camera1.yml
./build/test_calibration_accuracy 1 calibration_data/camera1.yml
```
