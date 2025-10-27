#!/usr/bin/env python3
"""
Optimized ArUco Marker Template Generator
Maximum detection accuracy and drawing area
"""

import cv2
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.pdfgen import canvas

print("🎯 Optimized ArUco Template Generator")
print("=" * 60)

# ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Configuration for maximum accuracy
MARKER_SIZE_PX = 250  # High resolution for sharp printing
QUIET_ZONE_PX = 50  # ArUco standard: ~20% of marker size
MARKER_IDS = [0, 1, 2, 3]
MARKER_NAMES = ["TOP_LEFT", "TOP_RIGHT", "BOTTOM_RIGHT", "BOTTOM_LEFT"]

print("\n📄 Generating PNG markers (high resolution)...")
for marker_id, name in zip(MARKER_IDS, MARKER_NAMES):
    # Generate marker
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, MARKER_SIZE_PX)

    # Add quiet zone (white border) - CRITICAL for detection accuracy
    marker_with_border = cv2.copyMakeBorder(
        marker_image,
        QUIET_ZONE_PX, QUIET_ZONE_PX, QUIET_ZONE_PX, QUIET_ZONE_PX,
        cv2.BORDER_CONSTANT, value=255
    )

    filename = f"marker_{marker_id}_{name}.png"
    cv2.imwrite(filename, marker_with_border)
    print(f"  ✅ {filename} ({marker_with_border.shape[0]}×{marker_with_border.shape[1]} px)")

print("\n📄 Generating optimized PDF template...")

# PDF setup
pdf_filename = "template.pdf"
c = canvas.Canvas(pdf_filename, pagesize=A4)
page_width, page_height = A4

# === OPTIMIZED PARAMETERS ===
# Marker display size in PDF (2.5cm x 2.5cm - optimal for detection)
MARKER_DISPLAY_SIZE = 2.5 * cm

# Position from edges (1.5cm - allows for cutting/pasting tolerance)
EDGE_MARGIN = 1.5 * cm

# Calculate exact positions for 4 corners
positions = {
    'TOP_LEFT': (EDGE_MARGIN, page_height - EDGE_MARGIN - MARKER_DISPLAY_SIZE),
    'TOP_RIGHT': (page_width - EDGE_MARGIN - MARKER_DISPLAY_SIZE, page_height - EDGE_MARGIN - MARKER_DISPLAY_SIZE),
    'BOTTOM_RIGHT': (page_width - EDGE_MARGIN - MARKER_DISPLAY_SIZE, EDGE_MARGIN),
    'BOTTOM_LEFT': (EDGE_MARGIN, EDGE_MARGIN)
}

marker_files = {
    'TOP_LEFT': "marker_0_TOP_LEFT.png",
    'TOP_RIGHT': "marker_1_TOP_RIGHT.png",
    'BOTTOM_RIGHT': "marker_2_BOTTOM_RIGHT.png",
    'BOTTOM_LEFT': "marker_3_BOTTOM_LEFT.png"
}

# Draw markers
for corner, (x, y) in positions.items():
    c.drawImage(
        marker_files[corner],
        x, y,
        width=MARKER_DISPLAY_SIZE,
        height=MARKER_DISPLAY_SIZE,
        preserveAspectRatio=True
    )

# Optional: Draw very light dashed border showing active drawing area
# (can be removed if not needed)
c.setStrokeColorRGB(0.9, 0.9, 0.9)  # Very light gray
c.setDash(2, 4)  # Small dashes
c.setLineWidth(0.5)

# Active area boundaries (between markers + 1cm margin)
active_x1 = EDGE_MARGIN + MARKER_DISPLAY_SIZE + 1 * cm
active_y1 = EDGE_MARGIN + MARKER_DISPLAY_SIZE + 1 * cm
active_x2 = page_width - EDGE_MARGIN - MARKER_DISPLAY_SIZE - 1 * cm
active_y2 = page_height - EDGE_MARGIN - MARKER_DISPLAY_SIZE - 1 * cm

c.rect(active_x1, active_y1, active_x2 - active_x1, active_y2 - active_y1)

# Minimal ID labels (very small, just for reference)
c.setFont("Helvetica", 6)
c.setFillColorRGB(0.7, 0.7, 0.7)  # Gray
for corner, (x, y) in positions.items():
    marker_id = corner.split('_')[0][0] + corner.split('_')[1][0]  # e.g., "TL"
    c.drawString(x + 0.1 * cm, y - 0.3 * cm, marker_id)

# Save PDF
c.save()

# Calculate and print statistics
active_width_cm = (active_x2 - active_x1) / cm
active_height_cm = (active_y2 - active_y1) / cm
active_area_cm2 = active_width_cm * active_height_cm
total_area_cm2 = (21 * 29.7)  # A4
efficiency = (active_area_cm2 / total_area_cm2) * 100

print(f"  ✅ {pdf_filename}")
print("\n" + "=" * 60)
print("📊 TEMPLATE SPECIFICATIONS")
print("=" * 60)
print(f"Page Size:           21.0 × 29.7 cm (A4)")
print(f"Marker Size:         2.5 × 2.5 cm each")
print(f"Edge Margin:         1.5 cm")
print(f"Active Drawing Area: {active_width_cm:.1f} × {active_height_cm:.1f} cm")
print(f"Active Area:         {active_area_cm2:.0f} cm² ({efficiency:.1f}% of page)")
print(f"")
print(f"Marker IDs:")
print(f"  • Top-Left:     0")
print(f"  • Top-Right:    1")
print(f"  • Bottom-Right: 2")
print(f"  • Bottom-Left:  3")
print("=" * 60)
print("\n✅ READY FOR PRINTING")
print("   → Print at 100% scale (no fit-to-page)")
print("   → Use white A4 paper")
print("   → High quality / Best setting")
print("=" * 60)
