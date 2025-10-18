#!/usr/bin/env python3
"""
ChArUco Calibration Board Generator
For camera calibration - print once and use for all cameras
"""

import cv2
import numpy as np
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import cm
import os

print("=" * 70)
print("  ChArUco CAMERA CALIBRATION BOARD GENERATOR")
print("=" * 70)

# ChArUco configuration
SQUARES_X = 7  # Horizontal squares (chess board)
SQUARES_Y = 5  # Vertical squares
SQUARE_LENGTH_CM = 3.0  # Each square 3cm x 3cm
MARKER_LENGTH_CM = 2.0  # ArUco marker inside square (2cm x 2cm)

# ArUco dictionary (same as drawing template)
ARUCO_DICT = cv2.aruco.DICT_4X4_50

print(f"\nBoard Configuration:")
print(f"  Squares: {SQUARES_X} x {SQUARES_Y}")
print(f"  Square size: {SQUARE_LENGTH_CM} cm")
print(f"  Marker size: {MARKER_LENGTH_CM} cm")
print(f"  ArUco Dict: DICT_4X4_50")
print(f"  Total board size: {SQUARES_X * SQUARE_LENGTH_CM} x {SQUARES_Y * SQUARE_LENGTH_CM} cm")

# Create ChArUco board
dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
board = cv2.aruco.CharucoBoard(
    (SQUARES_X, SQUARES_Y),
    SQUARE_LENGTH_CM / 100.0,  # Convert to meters for OpenCV
    MARKER_LENGTH_CM / 100.0,
    dictionary
)

# Generate high-resolution image
print("\nGenerating high-resolution board image...")
BOARD_IMAGE_SIZE_PX = 3000  # High resolution for printing
board_image = board.generateImage((BOARD_IMAGE_SIZE_PX, BOARD_IMAGE_SIZE_PX), marginSize=50, borderBits=1)

# Save PNG (for reference)
png_filename = "charuco_calibration_board.png"
cv2.imwrite(png_filename, board_image)
print(f"  PNG saved: {png_filename} ({board_image.shape[1]}x{board_image.shape[0]} px)")

# Create PDF for printing
print("\nGenerating PDF for printing...")
pdf_filename = "charuco_calibration_board.pdf"
c = canvas.Canvas(pdf_filename, pagesize=A4)
page_width, page_height = A4

# Calculate board dimensions
board_width_cm = SQUARES_X * SQUARE_LENGTH_CM
board_height_cm = SQUARES_Y * SQUARE_LENGTH_CM

# Center on A4 page
x_offset = (21.0 - board_width_cm) / 2 * cm  # A4 width = 21cm
y_offset = (29.7 - board_height_cm) / 2 * cm  # A4 height = 29.7cm

# Draw board image
c.drawImage(
    png_filename,
    x_offset, y_offset,
    width=board_width_cm * cm,
    height=board_height_cm * cm,
    preserveAspectRatio=True
)

# Add instructions at bottom
c.setFont("Helvetica-Bold", 10)
c.drawString(2*cm, 2*cm, "CAMERA CALIBRATION BOARD - ChArUco Pattern")

c.setFont("Helvetica", 8)
instructions = [
    "Print this page at 100% scale (no fit-to-page)",
    "Use white A4 paper, high quality print",
    "Keep flat - do not fold or wrinkle",
    f"Verify: Measure one black square = {SQUARE_LENGTH_CM} cm x {SQUARE_LENGTH_CM} cm"
]

y_pos = 1.5 * cm
for instruction in instructions:
    c.drawString(2*cm, y_pos, instruction)
    y_pos -= 0.4 * cm

c.save()
print(f"  PDF saved: {pdf_filename}")

# Save board configuration (for calibration program)
config_filename = "charuco_board_config.txt"
with open(config_filename, 'w') as f:
    f.write(f"# ChArUco Board Configuration\n")
    f.write(f"# This file is used by the calibration program\n")
    f.write(f"SQUARES_X={SQUARES_X}\n")
    f.write(f"SQUARES_Y={SQUARES_Y}\n")
    f.write(f"SQUARE_LENGTH={SQUARE_LENGTH_CM / 100.0}\n")  # in meters
    f.write(f"MARKER_LENGTH={MARKER_LENGTH_CM / 100.0}\n")  # in meters
    f.write(f"ARUCO_DICT={ARUCO_DICT}\n")

print(f"  Config saved: {config_filename}")

print("\n" + "=" * 70)
print("  READY FOR PRINTING")
print("=" * 70)
print("\nNext steps:")
print("  1. Print 'charuco_calibration_board.pdf'")
print("  2. Verify printed square size (should be 3x3 cm)")
print("  3. Run calibration program: ./camera_calibration")
print("\nIMPORTANT:")
print("  - Print at 100% scale (NOT fit-to-page)")
print("  - Use thick white paper if possible")
print("  - Keep board flat (attach to cardboard if needed)")
print("=" * 70)