# 📖 Camera Calibration & Usage Guide

## Quick Decision Table

| Change | Recalibration Needed? | Action Required |
|--------|----------------------|-----------------|
| Different camera | ✅ YES | Run calibration for new camera |
| Same camera, different location | ❌ NO | Just run the program |
| Same camera, different lighting | ❌ NO | Just run the program |
| Same camera, different angle | ❌ NO | Just run the program |
| Camera zoom/focus changed | ✅ YES | Recalibrate |
| Camera settings (brightness, contrast) | ❌ NO | Just run the program |

---

## Detailed Scenarios

### Scenario 1: Using a Different Camera

**Example:** Switching from laptop camera to external webcam

**Steps:**
```bash
# 1. Calibrate for the new camera
./camera_calibration
# Select the NEW camera
# Show ChArUco board, capture 20+ images
# New camera_calibration.yml will be created

# 2. Test
./test_aruco_detection
# Select the SAME camera you calibrated for
```

**Why?** Each camera has different lens characteristics and focal lengths.

---

### Scenario 2: Same Camera, Different Location

**Example:** Moving from Classroom A to Classroom B

**Steps:**
```bash
# NOTHING! Just run:
./test_aruco_detection
```

**Why?** Calibration is camera-specific, not location-specific!

---

### Scenario 3: Different Lighting

**Example:** Moving from bright room to dark room

**Steps:**
```bash
# NOTHING! Just run:
./test_aruco_detection
```

**Why?** Calibration is lighting-independent (focuses on geometry and lens).

**NOTE:** If ArUco markers can't be detected (too dark), increase camera brightness or add light.

---

### Scenario 4: Using from Different Angles

**Example:** Holding camera at an angle instead of straight

**Steps:**
```bash
# NOTHING! Just run:
./test_aruco_detection
```

**Why?** ArUco markers handle perspective correction automatically!

**IMPORTANT:** All 4 markers must be fully visible!

---

### Scenario 5: Camera Zoom/Focus Changed

**Example:** You adjusted the camera zoom

**Steps:**
```bash
# RECALIBRATION REQUIRED!
./camera_calibration
# Select same camera
# Calibrate with new zoom setting
```

**Why?** Zoom/focus changes the focal length (fx, fy)!

---

## Multiple Camera Setup

If you use multiple cameras regularly:

### File Organization:
```bash
# Create separate calibration files:
camera_calibration_cam0.yml    # Laptop camera
camera_calibration_cam1.yml    # External webcam
camera_calibration_cam2.yml    # Phone camera

# When using, copy the correct one:
cp camera_calibration_cam1.yml camera_calibration.yml
./test_aruco_detection
```

---

## Practical Examples

### Example 1: Going to Assignment Presentation

**Situation:**
- Using laptop camera (Camera 0)
- Calibrated at home
- Going to school

**Steps:**
```bash
# 1. Bring camera_calibration.yml with you (USB, cloud)
# 2. Go to school
# 3. Run the program:
./test_aruco_detection
Camera ID: 0  # Same camera!

# IT WILL WORK! ✅
```

**No recalibration needed!** Same camera works everywhere.

---

### Example 2: Demos in Multiple Classrooms

**Situation:**
- Using external webcam
- Demos in 3 different classrooms
- Different lighting in each room

**Steps:**
```bash
# FIRST CLASSROOM (one-time setup):
./camera_calibration
Camera ID: 1  # External webcam
# camera_calibration.yml created

# OTHER CLASSROOMS:
# Do nothing! Just run:
./test_aruco_detection
Camera ID: 1

# WORKS IN ALL ROOMS! ✅
```

**No recalibration needed!** Lighting doesn't affect calibration.

---

### Example 3: You Changed Zoom

**Situation:**
- Paper appears too small
- You used camera zoom

**Steps:**
```bash
# RECALIBRATION REQUIRED!
./camera_calibration
# Calibrate with zoom enabled
# New camera_calibration.yml will be created

./test_aruco_detection
# Now uses zoomed calibration ✅
```

---

## Troubleshooting

### "ArUco markers not detected"

**Possible causes:**
1. Too dark → Add light or increase camera brightness
2. Markers too small → Move camera closer
3. Markers blurry → Clean lens, better print quality
4. Not all 4 markers visible → Adjust camera angle

**Solution:** Lighting and positioning issue, NOT calibration issue!

---

### "Circle still looks squished"

**Possible causes:**
1. Using calibration from different camera → Recalibrate
2. Camera zoom changed → Recalibrate
3. Hand-drawn circle not perfect → Normal! (~5-10% deviation is OK)

**Solution:** Check which camera was used for calibration vs. which camera is running now.

---

## Best Practices

### ✅ DO:
- Calibrate once per camera
- Keep calibration files safe (backup!)
- Name calibration files clearly (camera_calibration_cam0.yml)
- Test after calibration to verify quality

### ❌ DON'T:
- Recalibrate for different locations (unnecessary!)
- Recalibrate for different lighting (unnecessary!)
- Use calibration from Camera 0 with Camera 1
- Change zoom after calibration (requires recalibration)

---

## Summary

**You need to recalibrate ONLY when:**
1. Using a different camera
2. Camera zoom/focus changed
3. Camera lens changed (if detachable)

**You DON'T need to recalibrate for:**
1. Different location
2. Different lighting
3. Different camera angle
4. Different time of day
5. Camera brightness/contrast adjustments

**Golden Rule:** One calibration per camera setup. That's it! 🎯
