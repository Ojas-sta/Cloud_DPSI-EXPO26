# Unitree Go2 Pro: Power-On Self-Test (POST) Diagnostic Report

**Timestamp**: 2026-11-05T17:15:07Z | **Status**: ALL SYSTEMS GO (READY FOR DISCOVERY)

| Subsystem Stage | Status | Key Metric / Verification Details |
|---|---|---|
| **Audio & TTS Voice Synthesis** | 🟢 PASS | `{"backend": "macOS say (Samantha)", "latency_sec": 6.88, "speaker_volume": "Nominal", "web_subtitle_broadcast": true}` |
| **Network & Telemetry Link** | 🟢 PASS | `{"target_ip": "192.168.123.161", "network_link": "Connected", "battery_soc_pct": 94.0, "battery_voltage_v": 28.4, "imu_status": "Locked (Zero Drift)", "mode": "Direct Emulation (Zero-Clipping)"}` |
| **Posture & Pitch Micro-Actuation** | 🟢 PASS | `{"stand_height_m": 0.354, "eye_contact_pitch_deg": 8.0, "neutral_pitch_deg": 0.0, "joint_compliance": "Verified (Nominal)"}` |
| **360° LiDAR & Safety Bubble** | 🟢 PASS | `{"corridor_clearance_m": {"left": 1.4, "right": 1.6}, "centering_bias_vy": -0.02, "e_stop_envelope_m": 0.6, "e_stop_trigger_tested": true, "anti_jerk_slewing": "0.8 m/s\u00b2 Decel / 0.5 m/s\u00b2 Accel Verified"}` |
| **ArUco Optical Board Tracking** | 🟢 PASS | `{"locked_tag_id": 101, "measured_distance_m": 2.012, "reprojection_rmse_px": 0.002, "dictionary": "cv2.aruco.DICT_4X4_250", "solver": "SOLVEPNP_IPPE_SQUARE"}` |
| **AprilTag Optical Board Tracking** | 🟢 PASS | `{"locked_tag_id": 101, "measured_distance_m": 2.009, "reprojection_rmse_px": 0.004, "aruco_rmse_baseline_px": 0.002, "recommended_winner": "ArUco (DICT_4X4_250)", "subpixel_gain_pct": 0.0}` |

---
*Generated automatically by Day1_Testes/post_diagnostics.py*
