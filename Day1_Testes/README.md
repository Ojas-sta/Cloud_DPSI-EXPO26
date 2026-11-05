# Day 1 Field Testing Kit & Runbook

Welcome to the **Day 1 Testing Kit** for the Unitree Go2 Pro Quadruped Docent at the DPSI EXPO-2026. This package contains everything needed to conduct live field tests from arrival to complete multi-stall autonomous docent presentation.

---

## 📁 Package Contents

```
Day1_Testes/
├── README.md                                 # Master field guide & test instructions (this document)
├── setup.md                                  # Network, hardware, IP, and pre-flight configuration
├── print_all_markers.html                    # 1-Click Master Printable HTML document (all 10 sheets)
├── generate_markers.py                       # Python script to regenerate 300-DPI A4 sheets
│
├── markers/                                  # Individual 300-DPI Printable A4 PNG Sheets
│   ├── floor_marker_100_a4.png               # Floor Tag 100: Entrance / Cold-Start Localization
│   ├── floor_marker_101_a4.png               # Floor Tag 101: Stall 1 Staging & Right Turn Waypoint
│   ├── floor_marker_102_a4.png               # Floor Tag 102: Concourse Arterial Waypoint
│   ├── floor_marker_103_a4.png               # Floor Tag 103: Stall 2 Staging & Right Turn Waypoint
│   ├── floor_marker_104_a4.png               # Floor Tag 104: Concourse Terminus / Tour Complete
│   ├── stall_marker_1_a4.png                 # Stall Tag 1: Stepwise AI (Biomechanical Gait Analysis)
│   ├── stall_marker_2_a4.png                 # Stall Tag 2: Move Smarter (ADAS Autonomous Driving)
│   ├── stall_marker_3_a4.png                 # Stall Tag 3: Eco-Drive (EV Energy Optimization)
│   └── stall_marker_4_a4.png                 # Stall Tag 4: Smart Sensors (IoT Telemetry)
│
└── boards/
    └── aruco_precision_docking_board_a4.png  # Multi-marker 3x2 precision docking & calibration board
```

---

## 🖨️ Printing Instructions (A4 Standard)

### Quickest Method (One-Click Print):
1. Open [`Day1_Testes/print_all_markers.html`](file:///Users/roopalisingh/.gemini/antigravity/scratch/cloud-argus-docent/Day1_Testes/print_all_markers.html) in Google Chrome or Safari.
2. Click the blue **"Print All Sheets"** button (or press `Cmd + P`).
3. Set your print dialog:
   - **Destination**: Color or Black & White Printer (or Save as PDF)
   - **Paper Size**: **A4** ($210 \times 297\,\text{mm}$)
   - **Scale**: **100% (Actual Size)** *(Do NOT use "Fit to Printable Area")*
   - **Margins**: **None** (or Minimal)
4. **Verification**: Check the printed **10 cm physical check ruler** at the bottom of the page with a real ruler to ensure exact 1:1 metric scale.

---

## 🚦 Testing Sequence: The 4 Sequential Milestones

### Milestone 1: Teleoperated Discovery Walk
- **Setup**: Place **Floor Tag 100** at start, **Floor Tag 101** $2.0\,\text{m}$ ahead, and **Stall Tag 1** on the table to the right.
- **Run**:
  ```bash
  python3 unitree_direct_sdk/main.py --milestone 1 --serve --port 8088
  ```
- **Action**: Teleop the robot past 100, past 101, turn right towards Stall 1, and press **`M`** (or click **Save Graph** on `http://localhost:8088`).
- **Result**: Generates and saves `config/discovered_aruco_graph.yaml`.

---

### Milestone 2: Autonomous Hop + 5-Step Docent Sequence
- **Run**:
  ```bash
  python3 unitree_direct_sdk/main.py --milestone 2 --pacing off --serve --port 8088
  ```
- **Action**: Robot snaps cold-start position on Marker 100, autonomous drives to 101, turns right to Stall 1, visual-servos to exact **$0.95\,\text{m}$ standoff**, performs the **$180^\circ$ Presenter Pivot**, locks `BALANCE_STAND` with **$+8.0^\circ$ eye-contact tilt**, delivers speech narration, and resets pitch to $0.0^\circ$.

---

### Milestone 3: Real-Time Human / Judge Speed Pacing
- **Run**:
  ```bash
  python3 unitree_direct_sdk/main.py --milestone 3 --pacing on --serve --port 8088
  ```
- **Action**: Walk alongside the robot's right flank.
  - As you walk: Go2 matches walking speed ($0.35\,\text{m/s}$).
  - As you stop / lag behind ($>0.30\,\text{m}$): Go2 anti-jerk rate limiter smoothly halts robot with zero knee torque bobbing (`ESCORT: WAITING FOR JUDGES`).
  - As you catch up: Go2 smoothly accelerates back to cruising speed.

---

### Milestone 4: Multi-Stall Scale-Up & Table Clearance Re-Entry
- **Setup**: Tape Floor Markers `100`..`104` along the aisle. Mount Stall 1 at 101 and Stall 2 at 103.
- **Run**:
  ```bash
  python3 unitree_direct_sdk/main.py --milestone 4 --preset scaled_5m_2s --pacing on --serve --port 8088
  ```
- **Action**: Autonomous docent tour across Stall 1 and Stall 2, executing a **$0.65\,\text{m}$ table clearance forward step** before pivoting North to prevent rear-leg clipping on booth tables.

---

## 🛑 Emergency Protocols
1. **Software Brake**: Press **`Space`** on the teleop keyboard or tap the red **`EMERGENCY BRAKE`** button on the Web Cockpit.
2. **0.40s Watchdog**: Releasing teleop keys automatically decays motion to zero in 0.40 seconds.
3. **Physical Controller**: Keep the Go2 handheld remote set to standby; press `L2 + B` for instant manual override.
