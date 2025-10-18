# Camera Calibration Guide

This guide walks you through the camera calibration process to improve shape detection accuracy.

## Why Calibration?

Camera calibration corrects:
- **Lens distortion** (wide-angle lenses curve straight lines)
- **Perspective errors** (improves bird's eye view transformation)
- **Scale accuracy** (calculate real-world measurements)

**Result**: Your drawn circles will no longer appear squished or stretched!

---

## Step-by-Step Instructions

### Step 1: Generate Calibration Board

Run the Python script to create a ChArUco calibration board:

```bash
cd tools
python3 1_generate_charuco_calibration_board.py
```

**Output**:
- `charuco_calibration_board.pdf` - Print this file
- `charuco_calibration_board.png` - Reference image
- `charuco_board_config.txt` - Configuration (used by calibration program)

**IMPORTANT**:
- Print at **100% scale** (NOT fit-to-page)
- Use white A4 paper
- High quality print setting
- Verify: Each black square should be exactly 3cm x 3cm

### Step 2: Build Calibration Program

From project root:

```bash
mkdir -p cmake-build-debug
cd cmake-build-debug
cmake ..
make camera_calibration
```

**Output**: `./camera_calibration` executable

### Step 3: Run Calibration

```bash
./camera_calibration
```

**Interactive Process**:

1. **Select Camera**: Choose your camera from the list
2. **Show Board**: Hold printed ChArUco board to camera
3. **Capture Images**: Press `SPACE` when board is detected
   - Capture at least 10 images (recommended: 20)
   - Follow on-screen instructions for positions:
     - Straight (center)
     - Tilted left/right
     - Tilted up/down
     - From corners (diagonal views)
     - Close/medium/far distances
4. **Calibrate**: Press `c` when you have enough images
5. **View Results**: Before/After comparison will be shown

**Keyboard Controls**:
- `SPACE` - Capture current frame
- `c` - Start calibration (minimum 10 images)
- `r` - Reset (delete all captured images)
- `q` - Quit without calibrating

### Step 4: Check Results

After calibration completes:

**Output File**: `camera_calibration.yml` (in project root)

**Quality Metrics**:
- **< 0.5 px error**: Excellent ✅
- **< 1.0 px error**: Very Good ✅
- **< 2.0 px error**: Good 👍
- **> 2.0 px error**: Acceptable (but could be better)

If error is high (> 2.0 px):
- Try again with better lighting
- Keep board completely flat
- Capture more diverse angles
- Ensure board is printed accurately (measure squares!)

---

## Using Calibration in Your Code

Once you have `camera_calibration.yml`, load it in your shape detection program:

```cpp
// Load calibration
cv::FileStorage fs("camera_calibration.yml", cv::FileStorage::READ);
cv::Mat camera_matrix, dist_coeffs;
fs["camera_matrix"] >> camera_matrix;
fs["distortion_coefficients"] >> dist_coeffs;
fs.release();

// In your main loop:
cv::Mat frame, undistorted;
cap >> frame;

// Apply distortion correction
cv::undistort(frame, undistorted, camera_matrix, dist_coeffs);

// Now use 'undistorted' for ArUco detection and shape recognition
```

---

## Tips for Best Results

### Calibration Board:
- Use thick paper or mount on cardboard (keep flat)
- Avoid wrinkles, folds, or shadows
- Good lighting (even, no glare)

### Image Capture:
- Board should fill ~50-80% of frame
- All 4 corners must be visible
- Vary angles significantly (don't just rotate slightly)
- Include extreme angles (but keep board fully visible)

### Camera:
- Use same camera for calibration and actual detection
- If camera settings change (focus, zoom), recalibrate
- Same resolution as your actual usage

---

## Troubleshooting

### "NO BOARD DETECTED"
- Is board completely visible?
- Lighting too dark?
- Board too far from camera?
- Print quality poor (blurry markers)?

### High Reprojection Error
- Recapture with more varied angles
- Check if board is flat (not warped)
- Measure printed squares (should be 3x3 cm exactly)

### Calibration Crashes
- Need at least 10 images
- All images must have valid board detections
- Try with better lighting

---

## When to Recalibrate

You need to recalibrate if:
- ❌ Different camera used
- ❌ Camera zoom/focus changed
- ❌ Camera lens changed
- ✅ Same camera, different location: **NO** (calibration stays valid)
- ✅ Different lighting: **NO** (calibration is independent of lighting)

---

## Files Generated

```
project_root/
├── tools/
│   ├── charuco_calibration_board.pdf    ← Print this
│   ├── charuco_calibration_board.png    ← Reference
│   └── charuco_board_config.txt         ← Config
└── camera_calibration.yml                ← Use this in your code
```

---

## Next Steps

After successful calibration:

1. ✅ You have `camera_calibration.yml`
2. Load it in your shape detection code
3. Apply `cv::undistort()` to every frame before processing
4. Your circles should now appear perfectly round! 🎯

---

**Questions?** Check OpenCV documentation:
- [Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [ChArUco Board](https://docs.opencv.org/4.x/df/d4a/tutorial_charuco_detection.html)
