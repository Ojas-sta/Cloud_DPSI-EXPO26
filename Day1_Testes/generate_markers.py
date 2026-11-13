#!/usr/bin/env python3
"""
ArUco & AprilTag (tag36h11) Marker & Precision Board Generator for A4 Printouts
Venue: DPSI EXPO-2026 | Platform: Unitree Go2 Pro Quadruped Docent
Supported Families:
  • ArUco:     cv2.aruco.DICT_4X4_250
  • AprilTag:  cv2.aruco.DICT_APRILTAG_36h11

Generates razor-sharp 300-DPI A4 printable sheets (2480 x 3508 px) with:
  • Exact 1:1 metric scale (160mm x 160mm fiducials)
  • 100mm (10cm) physical verification calibration check ruler
  • High-contrast alignment crosshairs and corner crop marks
  • Descriptive header, orientation indicators, and standoff distance specs
"""

import os
import cv2
import numpy as np

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
MARKERS_ARUCO_DIR = os.path.join(OUTPUT_DIR, "markers")
MARKERS_APRILTAG_DIR = os.path.join(OUTPUT_DIR, "markers_apriltag")
BOARDS_DIR = os.path.join(OUTPUT_DIR, "boards")

os.makedirs(MARKERS_ARUCO_DIR, exist_ok=True)
os.makedirs(MARKERS_APRILTAG_DIR, exist_ok=True)
os.makedirs(BOARDS_DIR, exist_ok=True)

# A4 at 300 DPI: 210mm x 297mm
DPI = 300
MM_TO_PX = DPI / 25.4  # ~11.811 px / mm
A4_W_PX = 2480
A4_H_PX = 3508

DICT_ARUCO = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
DICT_APRILTAG = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


def draw_metric_calibration_ruler(canvas: np.ndarray, x_start: int, y_pos: int, length_mm: int = 100):
    """Draws a true-to-scale 100mm (10cm) check ruler on the printout."""
    length_px = int(length_mm * MM_TO_PX)
    cv2.line(canvas, (x_start, y_pos), (x_start + length_px, y_pos), (0, 0, 0), 4)

    # Major ticks every 10mm (1cm), minor ticks every 5mm
    for mm in range(0, length_mm + 1):
        x = int(x_start + mm * MM_TO_PX)
        if mm % 10 == 0:
            cv2.line(canvas, (x, y_pos - 30), (x, y_pos + 30), (0, 0, 0), 4)
            cm = mm // 10
            cv2.putText(
                canvas, f"{cm}", (x - 12, y_pos + 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA
            )
        elif mm % 5 == 0:
            cv2.line(canvas, (x, y_pos - 18), (x, y_pos + 18), (0, 0, 0), 3)

    cv2.putText(
        canvas, "10 CM PHYSICAL CALIBRATION CHECK RULER (VERIFY WITH REAL RULER AFTER PRINTING)",
        (x_start, y_pos - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (60, 60, 60), 2, cv2.LINE_AA
    )


def draw_crop_marks(canvas: np.ndarray, x: int, y: int, size: int):
    """Draws L-shaped crop corner marks around fiducial."""
    arm = 60
    gap = 25
    # Top-Left
    cv2.line(canvas, (x - gap, y - gap), (x - gap - arm, y - gap), (0, 0, 0), 3)
    cv2.line(canvas, (x - gap, y - gap), (x - gap, y - gap - arm), (0, 0, 0), 3)
    # Top-Right
    cv2.line(canvas, (x + size + gap, y - gap), (x + size + gap + arm, y - gap), (0, 0, 0), 3)
    cv2.line(canvas, (x + size + gap, y - gap), (x + size + gap, y - gap - arm), (0, 0, 0), 3)
    # Bottom-Left
    cv2.line(canvas, (x - gap, y + size + gap), (x - gap - arm, y + size + gap), (0, 0, 0), 3)
    cv2.line(canvas, (x - gap, y + size + gap), (x - gap, y + size + gap + arm), (0, 0, 0), 3)
    # Bottom-Right
    cv2.line(canvas, (x + size + gap, y + size + gap), (x + size + gap + arm, y + size + gap), (0, 0, 0), 3)
    cv2.line(canvas, (x + size + gap, y + size + gap), (x + size + gap, y + size + gap + arm), (0, 0, 0), 3)


def generate_single_marker_a4(
    tag_id: int,
    tag_type: str,
    title: str,
    subtitle: str,
    instructions: str,
    dictionary,
    family_name: str,
    dict_code: str,
    output_dir: str,
    output_filename: str,
    marker_size_mm: float = 160.0
):
    """Generates an A4 portrait printable sheet for an individual ArUco or AprilTag marker."""
    canvas = np.full((A4_H_PX, A4_W_PX, 3), 255, dtype=np.uint8)

    # 1. Outer Border / Margin
    margin = 80
    cv2.rectangle(canvas, (margin, margin), (A4_W_PX - margin, A4_H_PX - margin), (220, 220, 220), 2)

    # 2. Header
    cv2.putText(
        canvas, f"DPSI EXPO-2026 • AUTONOMOUS DOCENT QUADRUPED • {family_name.upper()}",
        (margin + 40, margin + 110), cv2.FONT_HERSHEY_SIMPLEX, 1.05, (80, 80, 80), 3, cv2.LINE_AA
    )
    cv2.putText(
        canvas, title,
        (margin + 40, margin + 200), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (10, 10, 10), 5, cv2.LINE_AA
    )
    cv2.putText(
        canvas, subtitle,
        (margin + 40, margin + 270), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 102, 204), 3, cv2.LINE_AA
    )

    cv2.line(canvas, (margin + 40, margin + 310), (A4_W_PX - margin - 40, margin + 310), (0, 102, 204), 4)

    # 3. Orientation Arrow (for Floor Tags pointing North, for Booth tags facing Table)
    arrow_y = margin + 410
    if tag_type == "FLOOR":
        arrow_text = "▲ ▲ ▲  ALIGN ARROW FORWARD ALONG AISLE (NORTH / TOWARDS STALLS)  ▲ ▲ ▲"
    else:
        arrow_text = "▼ ▼ ▼  MOUNT LEVEL ON BOOTH DESK SKIRT FACING AISLE  ▼ ▼ ▼"

    cv2.putText(
        canvas, arrow_text,
        (margin + 80, arrow_y), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (160, 40, 20), 3, cv2.LINE_AA
    )

    # 4. Generate Fiducial Marker Image
    marker_size_px = int(marker_size_mm * MM_TO_PX)
    marker_img = cv2.aruco.generateImageMarker(dictionary, tag_id, marker_size_px)
    marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

    # Center marker on page
    cx = A4_W_PX // 2
    cy = 1750
    x1 = cx - marker_size_px // 2
    y1 = cy - marker_size_px // 2
    x2 = x1 + marker_size_px
    y2 = y1 + marker_size_px

    # Quiet Zone (White border around marker)
    quiet_px = 30
    cv2.rectangle(canvas, (x1 - quiet_px, y1 - quiet_px), (x2 + quiet_px, y2 + quiet_px), (0, 0, 0), 6)
    canvas[y1:y2, x1:x2] = marker_bgr

    # Crop marks
    draw_crop_marks(canvas, x1 - quiet_px, y1 - quiet_px, marker_size_px + 2 * quiet_px)

    # Center Crosshair Ticks
    cv2.line(canvas, (cx, y1 - quiet_px - 40), (cx, y1 - quiet_px - 8), (0, 0, 255), 4)
    cv2.line(canvas, (cx, y2 + quiet_px + 8), (cx, y2 + quiet_px + 40), (0, 0, 255), 4)
    cv2.line(canvas, (x1 - quiet_px - 40, cy), (x1 - quiet_px - 8, cy), (0, 0, 255), 4)
    cv2.line(canvas, (x2 + quiet_px + 8, cy), (x2 + quiet_px + 40, cy), (0, 0, 255), 4)

    # 5. Metadata Tag Box below marker
    tag_box_y = y2 + quiet_px + 120
    cv2.rectangle(
        canvas, (cx - 520, tag_box_y), (cx + 520, tag_box_y + 160),
        (245, 245, 245), -1
    )
    cv2.rectangle(
        canvas, (cx - 520, tag_box_y), (cx + 520, tag_box_y + 160),
        (0, 0, 0), 3
    )
    cv2.putText(
        canvas, f"{family_name.upper()} ID: {tag_id}  |  {dict_code}",
        (cx - 440, tag_box_y + 65), cv2.FONT_HERSHEY_SIMPLEX, 1.25, (0, 0, 0), 3, cv2.LINE_AA
    )
    cv2.putText(
        canvas, f"FIDUCIAL SIZE: {marker_size_mm:.0f}mm x {marker_size_mm:.0f}mm (1:1 TRUE SCALE)",
        (cx - 430, tag_box_y + 125), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (80, 80, 80), 2, cv2.LINE_AA
    )

    # 6. Usage Instructions
    inst_y = tag_box_y + 240
    for line in instructions.split("\n"):
        cv2.putText(
            canvas, line, (margin + 60, inst_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.82, (50, 50, 50), 2, cv2.LINE_AA
        )
        inst_y += 50

    # 7. Physical Calibration Check Ruler
    draw_metric_calibration_ruler(canvas, cx - int(50 * MM_TO_PX), A4_H_PX - margin - 150, length_mm=100)

    # Save PNG
    out_path = os.path.join(output_dir, output_filename)
    cv2.imwrite(out_path, canvas)
    print(f"  ✓ Generated: {output_filename} ({family_name} @ 300 DPI)")
    return out_path


def generate_precision_board_a4(
    dictionary,
    family_name: str,
    dict_code: str,
    output_filename: str
):
    """
    Generates a Multi-Marker 3x2 Precision Docking & Calibration Board (A4 Portrait)
    using cv2.aruco.GridBoard (Markers 10..15 with 45mm markers and 15mm separation).
    """
    canvas = np.full((A4_H_PX, A4_W_PX, 3), 255, dtype=np.uint8)
    margin = 80
    cv2.rectangle(canvas, (margin, margin), (A4_W_PX - margin, A4_H_PX - margin), (220, 220, 220), 2)

    # Header
    cv2.putText(
        canvas, f"DPSI EXPO-2026 • UNITREE GO2 PRECISION DOCKING TARGET • {family_name.upper()}",
        (margin + 40, margin + 110), cv2.FONT_HERSHEY_SIMPLEX, 1.05, (80, 80, 80), 3, cv2.LINE_AA
    )
    cv2.putText(
        canvas, f"MULTI-MARKER {family_name.upper()} DOCKING & CALIBRATION BOARD",
        (margin + 40, margin + 200), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (10, 10, 10), 4, cv2.LINE_AA
    )
    cv2.putText(
        canvas, f"Grid: 3 x 2 Fiducials (IDs 10..15) | Marker: 45mm | Separation: 15mm | {dict_code}",
        (margin + 40, margin + 270), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (0, 102, 204), 2, cv2.LINE_AA
    )
    cv2.line(canvas, (margin + 40, margin + 310), (A4_W_PX - margin - 40, margin + 310), (0, 102, 204), 4)

    # GridBoard generation (3 columns, 2 rows)
    board_markers_x = 3
    board_markers_y = 2
    marker_len_mm = 45.0
    marker_sep_mm = 15.0
    marker_len_px = int(marker_len_mm * MM_TO_PX)
    marker_sep_px = int(marker_sep_mm * MM_TO_PX)

    total_w = board_markers_x * marker_len_px + (board_markers_x - 1) * marker_sep_px
    total_h = board_markers_y * marker_len_px + (board_markers_y - 1) * marker_sep_px

    start_x = (A4_W_PX - total_w) // 2
    start_y = 1200

    # Draw backing plate rectangle
    pad = 50
    cv2.rectangle(
        canvas, (start_x - pad, start_y - pad), (start_x + total_w + pad, start_y + total_h + pad),
        (240, 240, 240), -1
    )
    cv2.rectangle(
        canvas, (start_x - pad, start_y - pad), (start_x + total_w + pad, start_y + total_h + pad),
        (0, 0, 0), 4
    )

    tag_id_base = 10
    for r in range(board_markers_y):
        for c in range(board_markers_x):
            tid = tag_id_base + (r * board_markers_x + c)
            mx = start_x + c * (marker_len_px + marker_sep_px)
            my = start_y + r * (marker_len_px + marker_sep_px)

            m_img = cv2.aruco.generateImageMarker(dictionary, tid, marker_len_px)
            m_bgr = cv2.cvtColor(m_img, cv2.COLOR_GRAY2BGR)
            canvas[my:my + marker_len_px, mx:mx + marker_len_px] = m_bgr
            cv2.rectangle(canvas, (mx, my), (mx + marker_len_px, my + marker_len_px), (0, 0, 0), 2)

            # Marker ID badge
            cv2.putText(
                canvas, f"#{tid}", (mx + 10, my - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (60, 60, 60), 2, cv2.LINE_AA
            )

    # Board Optical Center Crosshair
    board_cx = start_x + total_w // 2
    board_cy = start_y + total_h // 2
    ch_len = 80
    cv2.circle(canvas, (board_cx, board_cy), 25, (0, 0, 255), 3)
    cv2.line(canvas, (board_cx - ch_len, board_cy), (board_cx + ch_len, board_cy), (0, 0, 255), 3)
    cv2.line(canvas, (board_cx, board_cy - ch_len), (board_cx, board_cy + ch_len), (0, 0, 255), 3)
    cv2.putText(
        canvas, "OPTICAL DOCKING BORESIGHT CENTER",
        (board_cx - 240, board_cy + 55), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA
    )

    # Mounting Instructions
    notes_y = start_y + total_h + pad + 150
    notes = [
        f"• PRIMARY PURPOSE: Mount at Stall Docking Station to enable multi-tag {family_name} PnP pose solving.",
        "• ADVANTAGE OVER SINGLE TAG: Provides 24 coplanar corner points, eliminating yaw/pitch flip jitter.",
        "• CAMERA CALIBRATION: Can also be used with OpenCV solvePnP / calibrateCamera for optical check.",
        "• MOUNTING SPEC: Mount vertically on stall booth table apron, centered at Go2 camera height (0.35m)."
    ]
    for line in notes:
        cv2.putText(canvas, line, (margin + 60, notes_y), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (40, 40, 40), 2, cv2.LINE_AA)
        notes_y += 65

    # 10cm Calibration check ruler
    draw_metric_calibration_ruler(canvas, (A4_W_PX - int(100 * MM_TO_PX)) // 2, A4_H_PX - margin - 150, length_mm=100)

    out_path = os.path.join(BOARDS_DIR, output_filename)
    cv2.imwrite(out_path, canvas)
    print(f"  ✓ Generated: {output_filename} ({family_name} @ 300 DPI)")
    return out_path


def create_all_markers():
    print("\n" + "=" * 80)
    print("  🖨️ GENERATING ALL PRINTABLE A4 FIDUCIALS FOR DAY 1 TESTING (ARUCO & APRILTAG)")
    print("=" * 80 + "\n")

    # Specifications
    floor_specs = [
        (100, "FLOOR MARKER #100", "Concourse Entrance / Cold-Start Localization Waypoint",
         "• Position: Place flat on aisle floor at the entrance (Start Line).\n"
         "• Orientation: Arrow pointing forward along corridor (North).\n"
         "• Action: Robot snaps cold-start position and resets odometry drift to 0.00m."),

        (101, "FLOOR MARKER #101", "Stall P01 Staging & Right Turn Decision Waypoint",
         "• Position: Place flat on aisle floor exactly 2.00m North of Marker 100.\n"
         "• Association: Staging node for Stall 1 (Stepwise AI), which sits to the right.\n"
         "• Action: Robot detects tag, verifies hop 1, and pivots -90° towards Stall 1."),

        (102, "FLOOR MARKER #102", "Concourse Arterial Waypoint (Mid-Corridor)",
         "• Position: Place flat on aisle floor 2.00m North of Marker 101.\n"
         "• Action: Corridor arterial guidance waypoint. Maintains center aisle tracking."),

        (103, "FLOOR MARKER #103", "Stall P02 Staging & Right Turn Decision Waypoint",
         "• Position: Place flat on aisle floor 2.00m North of Marker 102.\n"
         "• Association: Staging node for Stall 2 (Move Smarter: ADAS) on the right.\n"
         "• Action: Robot executes right turn to acquire Stall 2 apron fiducial."),

        (104, "FLOOR MARKER #104", "Concourse Terminus / Tour Complete Waypoint",
         "• Position: Place flat on aisle floor 2.00m North of Marker 103.\n"
         "• Action: Triggers Grand Finale salute, final bow posture, and exhibition completion.")
    ]

    booth_specs = [
        (1, "STALL MARKER #1: P01 (STEPWISE AI)", "Biomechanical Gait Analysis — Capstone Project Apron Tag",
         "• Mounting: Tape to front skirt of booth desk facing the main aisle.\n"
         "• Height: Center marker 0.35m above floor (aligned with Go2 front camera).\n"
         "• Standoff: Robot visual-servos to exactly 0.95m standoff distance.\n"
         "• Sequence: 180° presenter pivot -> +8° eye-contact pitch -> Speech narration."),

        (2, "STALL MARKER #2: P02 (MOVE SMARTER: ADAS)", "AI Autonomous Driver Assistance — Capstone Project Apron Tag",
         "• Mounting: Tape to front skirt of Booth P02 table facing the main aisle.\n"
         "• Height: Center marker 0.35m above floor.\n"
         "• Standoff: Robot docks at 0.95m, performs 180° pivot, and delivers ADAS docent speech."),

        (3, "STALL MARKER #3: P03 (ECO-DRIVE)", "Electric Vehicle Energy Optimization — Apron Tag",
         "• Mounting: Tape to front skirt of Booth P03 table.\n"
         "• Usage: Optional third stall for multi-stall tour scaling."),

        (4, "STALL MARKER #4: P04 (SMART SENSORS)", "Industrial IoT & Fleet Telemetry — Apron Tag",
         "• Mounting: Tape to front skirt of Booth P04 table.\n"
         "• Usage: Optional fourth stall for full auditorium loop test.")
    ]

    # 1. Generate ArUco Pack
    print("  --- 1. Generating ArUco (DICT_4X4_250) Pack ---")
    for fid, title, sub, inst in floor_specs:
        generate_single_marker_a4(
            tag_id=fid, tag_type="FLOOR", title=title, subtitle=sub, instructions=inst,
            dictionary=DICT_ARUCO, family_name="ArUco", dict_code="DICT_4X4_250",
            output_dir=MARKERS_ARUCO_DIR, output_filename=f"floor_marker_{fid}_a4.png"
        )
    for bid, title, sub, inst in booth_specs:
        generate_single_marker_a4(
            tag_id=bid, tag_type="BOOTH", title=title, subtitle=sub, instructions=inst,
            dictionary=DICT_ARUCO, family_name="ArUco", dict_code="DICT_4X4_250",
            output_dir=MARKERS_ARUCO_DIR, output_filename=f"stall_marker_{bid}_a4.png"
        )
    generate_precision_board_a4(
        dictionary=DICT_ARUCO, family_name="ArUco", dict_code="DICT_4X4_250",
        output_filename="aruco_precision_docking_board_a4.png"
    )

    # 2. Generate AprilTag Pack
    print("\n  --- 2. Generating AprilTag (DICT_APRILTAG_36h11) Pack ---")
    for fid, title, sub, inst in floor_specs:
        generate_single_marker_a4(
            tag_id=fid, tag_type="FLOOR", title=f"APRILTAG: {title}", subtitle=sub, instructions=inst,
            dictionary=DICT_APRILTAG, family_name="AprilTag", dict_code="DICT_APRILTAG_36h11",
            output_dir=MARKERS_APRILTAG_DIR, output_filename=f"floor_marker_{fid}_apriltag_a4.png"
        )
    for bid, title, sub, inst in booth_specs:
        generate_single_marker_a4(
            tag_id=bid, tag_type="BOOTH", title=f"APRILTAG: {title}", subtitle=sub, instructions=inst,
            dictionary=DICT_APRILTAG, family_name="AprilTag", dict_code="DICT_APRILTAG_36h11",
            output_dir=MARKERS_APRILTAG_DIR, output_filename=f"stall_marker_{bid}_apriltag_a4.png"
        )
    generate_precision_board_a4(
        dictionary=DICT_APRILTAG, family_name="AprilTag", dict_code="DICT_APRILTAG_36h11",
        output_filename="apriltag_precision_docking_board_a4.png"
    )

    print("\n" + "=" * 80)
    print("  🎉 BOTH ARUCO & APRILTAG PACKS GENERATED SUCCESSFULLY IN Day1_Testes/")
    print("=" * 80 + "\n")


if __name__ == '__main__':
    create_all_markers()
