# Unitree Go2 Edu: Pre-Flight Power-On Self-Test (POST) Report

- **Execution Time:** 2026-11-13 08:24:08
- **System Verdict:** `PASS (MISSION READY)`
- **Hardware Target:** `Physical Unitree Go2 (en11)`

### Subsystem Diagnostic Table

| Subsystem | Status | Details | Metric |
| :--- | :---: | :--- | :--- |
| **CycloneDDS / SportClient** | `PASS` | Connected to 192.168.123.161 on en11 | `RTT < 2ms` |
| **BMS Battery Health** | `PASS` | Battery state-of-charge nominal for multi-stall tour | `SOC: 94.0%, Voltage: 28.40V, Current: -3.8A` |
| **Motor Joint Actuators** | `PASS` | All 12 joint actuators clear, temperatures within safe operating range | `Max Joint Temp: 36.5°C, Posture: BALANCE_STAND, Lockout: SAFETY_CLEAR` |
| **4D LiDAR L1 Subsystem** | `PASS` | Point cloud stream active with low jitter | `22.5 Hz (Threshold: > 15.0 Hz)` |
| **120° Fisheye Camera** | `PASS` | Video stream active, frame dimensions verified | `1920x1080 @ 21.0 FPS` |
| **Audio Output & Assets** | `PASS` | All 31+ stall pitch audio guides verified on disk | `Player: afplay, FishAudio v3: 32/32, Fallback: 32/32` |
| **Topological Graph & Config** | `PASS` | Master stall coordinates and graph topologies loaded | `Configs: 3/3, 120° Calib NPZ: Present` |
