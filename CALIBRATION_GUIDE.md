# Camera Calibration Guide

This guide explains how to calibrate your camera for accurate shape recognition with perspective correction.

## Overview

Camera calibration is essential for:
1. **Lens distortion correction** - Removes barrel/pincushion distortion
2. **Accurate perspective transformation** - Enables bird's-eye view correction
3. **Precise measurements** - Improves shape detection and classification

## What You Need

1. **Printed ChArUco Calibration Board** (A4 size)
2. **Printed ArUco Test Paper** (A4 size with corner markers)
3. **Drawing materials** (black marker/pen for circle)
4. **Good lighting** (even, bright light without shadows)
5. **Flat surface** to place the papers

## Step-by-Step Process

### Step 1: Generate and Print ChArUco Calibration Board

```bash
cd tools
python3 1_generate_charuco_calibration_board.py
```

This creates:
- `charuco_calibration_board.pdf` - Print this at **100% scale** (NOT fit-to-page)
- `charuco_calibration_board.png` - For reference
- `charuco_board_config.txt` - Configuration file

**Important Printing Instructions:**
- Use standard A4 white paper (21 x 29.7 cm)
- Print at 100% scale (no scaling/fit-to-page)
- High quality print mode
- Verify after printing: Measure one black square, should be ~2.07 cm × 2.07 cm

### Step 2: Calibrate Your Camera

**Interactive camera selection (recommended):**
```bash
./build/calibrate_camera_charuco
```

**Or specify camera and output file:**
```bash
./build/calibrate_camera_charuco 0  # Use camera 0, default output file
./build/calibrate_camera_charuco 1 calibration_data/camera1_calibration.yml  # Specific camera and file
```

**Camera Selection:**
- The tool will automatically scan for all available cameras (0-9)
- Each camera will be displayed with its resolution and backend
- If multiple cameras are found, you'll be prompted to select one
- Example output:
  ```
  Scanning for available cameras...
    Camera 0: 1280x720 (AVFoundation)
    Camera 1: 1920x1080 (AVFoundation)

  Multiple cameras detected. Please select:
    [0] Camera 0
    [1] Camera 1

  Enter selection (0-1):
  ```

**Camera Calibration Process:**

1. **Position the Board**
   - Hold the printed ChArUco board in front of camera
   - Ensure good, even lighting
   - Wait for green corners to appear (board detected)

2. **Capture Frames** (Press SPACE)
   - Capture at least 15-20 frames (more is better)
   - Vary the angle: tilt left/right, up/down
   - Vary the distance: near, far, medium
   - Cover all areas of the camera frame
   - Keep board flat and stable when capturing

3. **Perform Calibration** (Press 'c')
   - Wait for calibration to complete
   - Check reprojection error:
     - < 0.5 pixels = Excellent
     - < 1.0 pixels = Very Good
     - < 2.0 pixels = Good
     - > 2.0 pixels = Consider recalibrating

**Controls:**
- `SPACE` - Capture current frame
- `c` - Start calibration
- `ESC` - Exit

### Step 3: Generate and Print Test Paper

```bash
cd aruco_markers
python3 generate_markers.py
```

This creates:
- `template.pdf` - Print this at **100% scale**
- Individual marker PNG files

**After printing:**
1. Draw a **perfect circle** in the center of the paper
   - Use a compass for best results
   - Make it reasonably large (diameter ~5-10 cm)
   - Use black marker for good contrast
2. Verify the 4 corner ArUco markers are clearly visible
   - Each marker should be ~1.8 cm × 1.8 cm

### Step 4: Test Calibration Accuracy

**Interactive camera selection:**
```bash
./build/test_calibration_accuracy
```

**Or specify camera and calibration file:**
```bash
./build/test_calibration_accuracy 0  # Use camera 0, default calibration file
./build/test_calibration_accuracy 1 calibration_data/camera1_calibration.yml  # Specific setup
```

**Testing Process:**

1. Select the same camera you calibrated (if multiple cameras available)
2. Show the test paper (with drawn circle) to camera
2. Ensure all 4 corner markers are visible
3. The tool will:
   - Detect corner ArUco markers (IDs: 0, 1, 2, 3)
   - Apply lens undistortion
   - Transform to bird's-eye view
   - Detect the circle
   - Measure two perpendicular diameters
   - Calculate ratio (should be ~1.0)

**Quality Assessment:**
- Ratio = 1.000 ± 0.005 → **Excellent!**
- Ratio = 1.000 ± 0.010 → **Very Good**
- Ratio = 1.000 ± 0.020 → **Good**
- Ratio = 1.000 ± 0.050 → **Acceptable**
- Ratio > 1.050 or < 0.950 → **Poor - Recalibrate**

**Controls:**
- `SPACE` - Toggle measurement display on/off
- `s` - Save current frame
- `ESC` - Exit

## Understanding the Results

### Calibration Output File

The `camera_calibration.yml` file contains:

```yaml
calibration_date: "2025-10-22 15:30:45"
image_width: 1280
image_height: 720
reprojection_error: 0.342
camera_matrix: !!opencv-matrix
  rows: 3
  cols: 3
  dt: d
  data: [ fx, 0, cx,
          0, fy, cy,
          0, 0, 1 ]
distortion_coefficients: !!opencv-matrix
  rows: 8
  cols: 1
  dt: d
  data: [ k1, k2, p1, p2, k3, k4, k5, k6 ]
```

**Key Parameters:**
- `fx, fy` - Focal lengths in pixels
- `cx, cy` - Principal point (optical center)
- `k1, k2, k3, k4, k5, k6` - Radial distortion coefficients
- `p1, p2` - Tangential distortion coefficients

### Circle Diameter Ratio

The ratio of perpendicular diameters indicates calibration quality:

- **Ratio = 1.0000** means the circle remains perfectly circular after transformation
- **Ratio ≠ 1.0000** indicates residual distortion or calibration error

## Troubleshooting

### Problem: ChArUco board not detected

**Solutions:**
- Improve lighting (avoid shadows, glare)
- Ensure board is printed clearly
- Move board closer to camera
- Keep board flat (don't bend or wrinkle)
- Try different angles

### Problem: Low number of corners detected

**Solutions:**
- Move board to fill more of the frame
- Ensure board is in focus
- Check print quality (should be sharp, high contrast)
- Clean camera lens

### Problem: High reprojection error (> 2.0)

**Solutions:**
- Capture more frames (30-40 instead of 15-20)
- Ensure better variety in angles and distances
- Check board is flat when capturing
- Verify printed board dimensions are correct
- Improve lighting conditions

### Problem: Circle ratio far from 1.0

**Solutions:**
- Re-run calibration with more frames
- Ensure test circle is truly circular (use compass)
- Check that all 4 corner markers are detected
- Verify test paper is flat
- Recalibrate with better frame coverage

### Problem: ArUco markers not detected in test

**Solutions:**
- Ensure markers are printed clearly
- Improve lighting (avoid shadows on markers)
- Move paper to fill camera view
- Check marker IDs are correct (0, 1, 2, 3)

## Tips for Best Results

1. **Lighting**
   - Use bright, diffuse lighting
   - Avoid shadows on the board/paper
   - No direct sunlight or glare

2. **Frame Variety**
   - Tilt: left, right, up, down
   - Distance: near, medium, far
   - Position: center, edges, corners
   - Rotation: slight rotations around center

3. **Board Quality**
   - Print at high quality (600+ DPI)
   - Use thick paper or mount on cardboard
   - Keep perfectly flat
   - Avoid creases or bends

4. **Multiple Cameras**
   - Each camera needs separate calibration
   - Use camera ID to specify which camera to calibrate:
     ```bash
     ./build/calibrate_camera_charuco 0 calibration_data/camera0_calibration.yml
     ./build/calibrate_camera_charuco 1 calibration_data/camera1_calibration.yml
     ```
   - Built-in vs external cameras usually have different distortion
   - Different resolutions need different calibrations
   - Test each camera with its own calibration file:
     ```bash
     ./build/test_calibration_accuracy 0 calibration_data/camera0_calibration.yml
     ./build/test_calibration_accuracy 1 calibration_data/camera1_calibration.yml
     ```

## Using Calibration in Main Program

Once calibrated, the main shape recognition program will use `camera_calibration.yml` to:

1. **Undistort** incoming video frames
2. **Detect** the paper with ArUco markers
3. **Transform** to bird's-eye view (correcting perspective)
4. **Detect** and classify shapes accurately
5. **Measure** shape properties precisely

## File Structure

```
DrawingShapeRecognition/
├── tools/
│   ├── 1_generate_charuco_calibration_board.py  # Generate calibration board
│   ├── 2_calibrate_camera_charuco.cpp            # Calibration tool
│   └── 3_test_calibration_accuracy.cpp           # Test tool
├── aruco_markers/
│   └── generate_markers.py                       # Generate test paper
├── calibration_data/
│   ├── camera_calibration.yml                    # Your calibration file
│   └── README.md                                  # Quick reference
└── build/
    ├── calibrate_camera_charuco                  # Calibration executable
    └── test_calibration_accuracy                 # Test executable
```

## Technical Details

### ChArUco Board Specifications

Based on your printed board:
- Grid: 7 × 5 blocks
- Square size: 2.07 cm × 2.07 cm
- Marker size: 1.4 cm × 1.4 cm (inside squares)
- Dictionary: DICT_4X4_50
- Total size: ~14.5 cm × 10.4 cm

### ArUco Test Paper Specifications

- Paper size: 21.0 cm × 29.7 cm (A4)
- Marker size: 1.8 cm × 1.8 cm
- Edge margin: 1.5 cm from paper edge
- Marker IDs: 0 (TL), 1 (TR), 2 (BR), 3 (BL)
- Drawing area: ~16 cm × 25 cm (center)

### Calibration Algorithm

The calibration uses:
- **Method:** Zhang's method with ChArUco pattern
- **Distortion model:** Rational model (8 coefficients)
- **Corner refinement:** Sub-pixel accuracy
- **Optimization:** Levenberg-Marquardt

## Next Steps

After successful calibration:

1. **Integrate into main program** - Load calibration in your shape recognition code
2. **Test with different shapes** - Draw and test various shapes
3. **Validate accuracy** - Measure known shapes to verify precision
4. **Document your setup** - Note camera model, lighting, distance

---

**Need Help?**
- Check the calibration data README: `calibration_data/README.md`
- Review source code comments in the tools
- Test with provided example images first
