# Day 1 Field Testing Kit & Runbook: Dual ArUco & AprilTag Setup

Welcome to the **Day 1 Testing Kit** for the Unitree Go2 Pro Quadruped Docent at the DPSI EXPO-2026. This package contains everything needed to conduct live field tests, compare **ArUco (`DICT_4X4_250`)** vs. **AprilTag (`DICT_APRILTAG_36h11`)** side-by-side using the `--apriltag` flag, and run all 4 engineering milestones end-to-end.

---

## 📁 Package Contents

```
Day1_Testes/
├── README.md                                 # Master field guide & test instructions (this document)
├── setup.md                                  # Network, hardware, IP, camera, and CLI reference
├── print_all_markers.html                    # 1-Click Master Printable HTML document (ArUco Pack)
├── print_all_apriltags.html                  # 1-Click Master Printable HTML document (AprilTag Pack)
├── generate_markers.py                       # Python script to regenerate both 300-DPI A4 packs
├── compare_aruco_vs_apriltag.py              # Quantitative benchmark & live camera jitter evaluator
│
├── markers/                                  # ArUco (DICT_4X4_250) A4 PNG Sheets (2480 x 3508 px)
│   ├── floor_marker_100_a4.png               # Floor Tag 100: Entrance / Cold-Start Localization
│   ├── floor_marker_101_a4.png               # Floor Tag 101: Stall 1 Staging & Right Turn Waypoint
│   ├── floor_marker_102_a4.png               # Floor Tag 102: Concourse Arterial Waypoint
│   ├── floor_marker_103_a4.png               # Floor Tag 103: Stall 2 Staging & Right Turn Waypoint
│   ├── floor_marker_104_a4.png               # Floor Tag 104: Concourse Terminus / Tour Complete
│   ├── stall_marker_1_a4.png                 # Stall Tag 1: Stepwise AI (Biomechanical Gait Analysis)
│   ├── stall_marker_2_a4.png                 # Stall Tag 2: Move Smarter: ADAS (Autonomous Driving)
│   ├── stall_marker_3_a4.png                 # Stall Tag 3: Eco-Drive (EV Energy Optimization)
│   └── stall_marker_4_a4.png                 # Stall Tag 4: Smart Sensors (Industrial IoT Telemetry)
│
├── markers_apriltag/                         # AprilTag (DICT_APRILTAG_36h11) A4 PNG Sheets (2480 x 3508 px)
│   ├── floor_marker_100_apriltag_a4.png      # AprilTag Floor Tag 100
│   ├── floor_marker_101_apriltag_a4.png      # AprilTag Floor Tag 101
│   ├── floor_marker_102_apriltag_a4.png      # AprilTag Floor Tag 102
│   ├── floor_marker_103_apriltag_a4.png      # AprilTag Floor Tag 103
│   ├── floor_marker_104_apriltag_a4.png      # AprilTag Floor Tag 104
│   ├── stall_marker_1_apriltag_a4.png        # AprilTag Stall Tag 1: Stepwise AI
│   ├── stall_marker_2_apriltag_a4.png        # AprilTag Stall Tag 2: Move Smarter: ADAS
│   ├── stall_marker_3_apriltag_a4.png        # AprilTag Stall Tag 3: Eco-Drive
│   └── stall_marker_4_apriltag_a4.png        # AprilTag Stall Tag 4: Smart Sensors
│
└── boards/
    ├── aruco_precision_docking_board_a4.png  # ArUco 3x2 multi-marker precision docking board
    └── apriltag_precision_docking_board_a4.png # AprilTag 3x2 multi-marker precision docking board
```

---

## 🖨️ Printing Instructions (A4 Standard)

### ArUco Pack
1. Open [`Day1_Testes/print_all_markers.html`](file:///Users/roopalisingh/.gemini/antigravity/scratch/cloud-argus-docent/Day1_Testes/print_all_markers.html) in Chrome/Safari.
2. Click **"Print All Sheets"** (`Cmd + P`).

### AprilTag Pack
1. Open [`Day1_Testes/print_all_apriltags.html`](file:///Users/roopalisingh/.gemini/antigravity/scratch/cloud-argus-docent/Day1_Testes/print_all_apriltags.html) in Chrome/Safari.
2. Click **"Print All Sheets"** (`Cmd + P`).

### Print Dialog Settings
- **Destination**: Color or Black & White Printer (or Save as PDF).
- **Paper Size**: **A4** ($210 \times 297\,\text{mm}$).
- **Scale**: **100% (Actual Size)** *(Do NOT choose "Fit to Printable Area")*.
- **Margins**: **None** (or Minimal).
- **Verification**: Check the printed **10 cm physical check ruler** at the bottom of each page with a real ruler to ensure exact 1:1 metric scale.

---

## 🔬 Live ArUco vs. AprilTag Accuracy Benchmark

Before starting the full mission, run the quantitative benchmark on your laptop or with the physical Go2 camera:

### Synthetic Simulation Benchmark
```bash
python3 Day1_Testes/compare_aruco_vs_apriltag.py
```
- Tests detection rates, reprojection RMSE, and range jitter across $1.0\text{--}4.5\,\text{m}$ distance.

### Live Camera Benchmark (Physical Markers)
Tape one ArUco and one AprilTag marker side-by-side and point your camera:
```bash
python3 Day1_Testes/compare_aruco_vs_apriltag.py --camera 0 --frames 60
```
- Outputs real-time side-by-side detection rates, mean distance, pose jitter ($\sigma_x, \sigma_y, \sigma_z$ in mm), and heading jitter ($\sigma_{\text{yaw}}$ in deg).

---

## 🚦 Testing Sequence: The 4 Sequential Milestones

### Step 0: Voice-Guided Power-On Self-Test (POST) & Board Calibration
Before running any milestones, execute the voice-guided POST suite to sequentially verify all 7 subsystems:
```bash
# Offline Emulation (Mac Dry-Run)
python3 Day1_Testes/post_diagnostics.py --emulate --serve --port 8088

# On Physical Go2 Robot (USB Camera)
python3 Day1_Testes/post_diagnostics.py --real --camera 0 --serve --port 8088

# Or via main.py flag
python3 unitree_direct_sdk/main.py --post --serve --port 8088
```
- **What it tests**: Audio/TTS, CycloneDDS/IP telemetry & battery %, +8.0° docent pitch tilt & yaw nod, 360° LiDAR 0.60m E-stop, interactive ArUco board lock, interactive AprilTag board lock & jitter comparison, and exports `Day1_Testes/last_post_report.json`.
- **Operator action**: Listen to spoken prompts; hold up the printed ArUco and AprilTag boards at 1.0m when prompted.

---

All milestones seamlessly support both fiducial families using the `--apriltag` flag.

### Milestone 1: Teleoperated Discovery Walk
- **ArUco**:
  ```bash
  python3 unitree_direct_sdk/main.py --milestone 1 --serve --port 8088
  ```
- **AprilTag**:
  ```bash
  python3 unitree_direct_sdk/main.py --milestone 1 --apriltag --serve --port 8088
  ```
- **Action**: Drive past Marker 100, past 101, turn right towards Stall 1, and press **`M`** (or click **Save Graph** on `http://localhost:8088`).

---

### Milestone 2: Autonomous Hop + 5-Step Docent Sequence
- **ArUco**:
  ```bash
  python3 unitree_direct_sdk/main.py --milestone 2 --pacing off --serve --port 8088
  ```
- **AprilTag**:
  ```bash
  python3 unitree_direct_sdk/main.py --milestone 2 --apriltag --pacing off --serve --port 8088
  ```
- **Action**: Robot snaps cold-start position on Marker 100, autonomous drives to 101, turns right to Stall 1, visual-servos to exact **$0.95\,\text{m}$ standoff**, performs the **$180^\circ$ Presenter Pivot**, locks `BALANCE_STAND` with **$+8.0^\circ$ eye-contact tilt**, delivers speech narration, and resets pitch to $0.0^\circ$.

---

### Milestone 3: Real-Time Human / Judge Speed Pacing
- **ArUco**:
  ```bash
  python3 unitree_direct_sdk/main.py --milestone 3 --pacing on --serve --port 8088
  ```
- **AprilTag**:
  ```bash
  python3 unitree_direct_sdk/main.py --milestone 3 --apriltag --pacing on --serve --port 8088
  ```
- **Action**: Walk alongside the robot's right flank.
  - As you walk: Go2 matches walking speed ($0.35\,\text{m/s}$).
  - As you stop / lag behind ($>0.30\,\text{m}$): Go2 smoothly halts robot (`ESCORT: WAITING FOR JUDGES`).
  - As you catch up: Go2 smoothly accelerates back to cruising speed.

---

### Milestone 4: Multi-Stall Scale-Up & Table Clearance Re-Entry
- **ArUco**:
  ```bash
  python3 unitree_direct_sdk/main.py --milestone 4 --preset scaled_5m_2s --pacing on --serve --port 8088
  ```
- **AprilTag**:
  ```bash
  python3 unitree_direct_sdk/main.py --milestone 4 --apriltag --preset scaled_5m_2s --pacing on --serve --port 8088
  ```
- **Action**: Autonomous docent tour across Stall 1 and Stall 2, executing a **$0.65\,\text{m}$ table clearance forward step** before pivoting North to prevent rear-leg clipping on booth tables.

*(Add `--real` to any command when connected to physical Go2 hardware)*.
