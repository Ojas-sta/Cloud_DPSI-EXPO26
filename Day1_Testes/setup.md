# Day 1 Hardware & Environment Setup Guide

This guide details the complete hardware, network, software, and physical layout setup required for live Unitree Go2 quadruped testing at the DPSI EXPO-2026 venue.

---

## 1. Physical Layout & Marker Placement

### Corridor & Aisle Setup
Tape the printed **A4 Floor Markers** flat onto the floor along the concourse centerline, spaced approximately **2.00 meters** apart:

```
[Start Line]
     │
     ▼
[Floor Tag 100]  ── 2.0m ──► [Floor Tag 101] ── 2.0m ──► [Floor Tag 102] ── 2.0m ──► [Floor Tag 103] ── 2.0m ──► [Floor Tag 104]
(Cold-Start Entry)                 │ (Turn -90°)                                         │ (Turn -90°)                (Terminus Salute)
                                   ▼                                                     ▼
                             [Stall Tag 1]                                         [Stall Tag 2]
                             (Stepwise AI)                                         (ADAS Smarter)
```

### Mounting Specifications
1. **Floor Markers (IDs 100..104)**:
   - Tape flat to the floor with non-marking gaffer or masking tape along the white quiet-zone margins.
   - Align the printed **`▲ NORTH`** arrow pointing directly forward along the concourse travel direction.
2. **Booth Markers (IDs 1..4)**:
   - Mount vertically to the front skirt/edge of the project demonstration table facing the aisle.
   - Recommended mounting height: **$0.35\,\text{m}$ (35 cm)** above the floor, matching the Go2 camera lens optical axis.
3. **Precision Docking Board**:
   - Mount at Stall 1 or Stall 2 if performing multi-fiducial calibration or high-precision docking evaluations.

---

## 2. Laptop & Network Configuration

The Unitree Go2 internal computer runs on subnet `192.168.123.0/24`. The robot's primary Motion CPU sits at `192.168.123.161`.

### Option A: Wired Ethernet (Recommended for Zero Latency)
1. Connect an Ethernet cable between your laptop and the Go2 RJ45 port (inside the battery compartment or rear payload connector).
2. Configure your laptop's Ethernet adapter with a static IP:
   - **IP Address**: `192.168.123.222`
   - **Subnet Mask**: `255.255.255.0`
   - **Router / Gateway**: *(leave blank or `192.168.123.1`)*
3. Verify connection:
   ```bash
   ping -c 3 192.168.123.161
   ```

### Option B: Go2 Wi-Fi Hotspot
1. Connect laptop Wi-Fi to the robot's network:
   - **SSID**: `Unitree_Go2_XXXX`
   - **Password**: *(as printed on the underside of your Go2 battery)*
2. Check connectivity:
   ```bash
   ping -c 3 192.168.123.161
   ```

---

## 3. Python Environment & Dependencies

The direct SDK runs pure Python 3 without requiring ROS 2 or MuJoCo:

```bash
cd /Users/roopalisingh/.gemini/antigravity/scratch/cloud-argus-docent

# Required packages
pip install opencv-python numpy networkx pyyaml
```

If connecting to physical Unitree Go2 SDK2 libraries:
```bash
# Verify cyclonedds or unitree_sdk2py is available if using physical sport client
python3 -c "import cv2; print('OpenCV:', cv2.__version__)"
```

---

## 4. Camera Verification

Identify your camera index:
```bash
# Test external USB camera (usually index 0, 1, or 2)
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
print('Camera 0 available:', ret, 'Shape:', frame.shape if ret else 'None')
cap.release()
"
```
- If using an external USB camera mounted on Go2: pass `--camera 0` (or `--camera 1`).
- If running in virtual emulation mode: omit `--camera` (the system automatically generates photorealistic synthetic ArUco frames).

---

## 5. Pre-Flight Diagnostic Test Suite & Voice-Guided POST

### Automated Unit Test Suite
```bash
python3 unitree_direct_sdk/test_milestones_end_to_end.py
```

### Voice-Guided Power-On Self-Test (POST) & Interactive Calibration
Run the 7-stage interactive diagnostic before putting the Go2 on the aisle:
```bash
# In Emulation Mode (Offline Mac)
python3 Day1_Testes/post_diagnostics.py --emulate --serve --port 8088

# On Physical Robot (Live USB Camera)
python3 Day1_Testes/post_diagnostics.py --real --camera 0 --serve --port 8088
```

Expected output:
```
================================================================================
  🎉 ALL 4 ENGINEERING MILESTONES VERIFIED WITH 100% SUCCESS!
================================================================================
```

---

## 6. Execution Command Quick Reference

### ArUco Mode (Default)
| Mission Milestone | CLI Command |
|---|---|
| **M1: Teleop Discovery** | `python3 unitree_direct_sdk/main.py --milestone 1 --serve --port 8088` |
| **M2: Autonomous Docking** | `python3 unitree_direct_sdk/main.py --milestone 2 --pacing off --serve --port 8088` |
| **M3: Judge Escort Pacing** | `python3 unitree_direct_sdk/main.py --milestone 3 --pacing on --serve --port 8088` |
| **M4: Scaled Multi-Stall** | `python3 unitree_direct_sdk/main.py --milestone 4 --preset scaled_5m_2s --serve --port 8088` |

### AprilTag Mode (`--apriltag`)
| Mission Milestone | CLI Command |
|---|---|
| **M1: Teleop Discovery** | `python3 unitree_direct_sdk/main.py --milestone 1 --apriltag --serve --port 8088` |
| **M2: Autonomous Docking** | `python3 unitree_direct_sdk/main.py --milestone 2 --apriltag --pacing off --serve --port 8088` |
| **M3: Judge Escort Pacing** | `python3 unitree_direct_sdk/main.py --milestone 3 --apriltag --pacing on --serve --port 8088` |
| **M4: Scaled Multi-Stall** | `python3 unitree_direct_sdk/main.py --milestone 4 --apriltag --preset scaled_5m_2s --serve --port 8088` |

*(Add `--real` to any command when connected to physical Go2 hardware)*.

---

## 7. Fiducial Accuracy Comparison Benchmark

To run side-by-side detection, pose jitter, and reprojection error comparisons:

```bash
# Simulation mode
python3 Day1_Testes/compare_aruco_vs_apriltag.py

# Live physical camera mode
python3 Day1_Testes/compare_aruco_vs_apriltag.py --camera 0 --frames 60
```
