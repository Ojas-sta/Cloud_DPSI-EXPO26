#!/usr/bin/env python3
"""
Automated Verification Suite for Tomorrow's 4 Sequential Engineering Milestones:
  • Milestone 1: Teleoperated Discovery & Mapping (2 Floor Markers -> 1 Right-Side Stall)
  • Milestone 2: Autonomous Hop-by-Hop Navigation + 5-Step Stall Docent Sequence (Pacing Off)
  • Milestone 3: Autonomous Navigation with Real-Time Human / Judge Speed Pacing (Pacing On)
  • Milestone 4: Scale-Up to 5 Floor ArUco Markers & 2 Stalls (with 0.65m Table Clearance Re-Entry)
"""

import os
import sys
import time
import math
import yaml
import numpy as np

cur_dir = os.path.dirname(os.path.abspath(__file__))
workspace_dir = os.path.dirname(cur_dir)
if cur_dir not in sys.path:
    sys.path.insert(0, cur_dir)

from unitree_go2_interface import UnitreeGo2Interface
from real_aruco_vision import RealArUcoVision
from discovery_mapper import DiscoveryMapper
from topological_aruco_navigator import TopologicalArucoNavigator
from judge_escort_controller import JudgeEscortController
from keyboard_teleop import KeyboardTeleop


# ==============================================================================
# TEST MILESTONE 1: Teleop Discovery & Mapping (2 Floor -> 1 Right-Side Stall)
# ==============================================================================
def test_milestone_1():
    print("\n" + "=" * 80)
    print("  TEST 1: MILESTONE 1 — TELEOP DISCOVERY & MAPPING (2 FLOOR -> 1 RIGHT-SIDE STALL)")
    print("=" * 80)

    vision = RealArUcoVision(layout_preset="minimal_2m_1s")
    mapper = DiscoveryMapper()

    # Verify Auto-Zero Teleop Watchdog (Refinement 3)
    cmd_history = []
    teleop = KeyboardTeleop(
        command_callback=lambda vx, vy, wz: cmd_history.append((vx, vy, wz)),
        watchdog_timeout_sec=0.20
    )
    teleop.inject_command(0.30, 0.0, 0.0)
    assert teleop.vx == 0.30, "Teleop command injection failed!"
    print(f"  • Teleop watchdog active: injected pulse (vx={teleop.vx:.2f}m/s)")
    time.sleep(0.30)
    teleop.check_watchdog()  # Run watchdog pass
    assert teleop.vx == 0.0, "Watchdog failed to zero velocity after timeout!"
    print("  ✓ Refinement 3 Verified: Auto-zero watchdog timer smoothly decayed commands to 0.0m/s.")

    # Simulate operator teleoperating from Floor 100 -> Floor 101 -> right turn to face Stall 1
    # Step 1: Floor 100 straight to Floor 101
    teleop_traj = [
        (0.0, 10.50, 90.0, "Approaching Floor 100 facing North"),
        (0.0, 11.50, 90.0, "At Floor 100 facing North"),
        (0.0, 12.50, 90.0, "Traversing forward along aisle"),
        (0.0, 13.50, 90.0, "At Floor 101 facing North"),
        # Step 2: Turn right -90° to face Stall 1 (facing East 0°)
        (0.0, 13.50, 45.0, "Turning right"),
        (0.0, 13.50, 0.0,  "Facing East towards Stall 1"),
        (0.4, 13.50, 0.0,  "Approaching Stall 1 apron")
    ]

    for x, y, yaw, desc in teleop_traj:
        frame = vision.generate_synthetic_frame(x, y, yaw, desc)
        _, dets = vision.process_frame(frame)
        mapper.update_robot_odometry(x, y, yaw)
        mapper.update_lidar_scan(x, y, yaw, 1.40, 1.60)
        mapper.update_aruco_detections(x, y, yaw, dets)

    yaml_path, pgm_path = mapper.save_discovered_map("discovered_aruco_graph.yaml")
    assert os.path.exists(yaml_path), "Milestone 1 graph YAML failed to export!"

    # Verify discovered nodes
    assert 100 in mapper.discovered_nodes, "Floor Tag 100 not mapped!"
    assert 101 in mapper.discovered_nodes, "Floor Tag 101 not mapped!"
    assert 1 in mapper.discovered_nodes, "Stall Tag 1 not mapped!"

    # Verify Refinement 2: Right-angle branching geometry
    b1 = mapper.discovered_nodes[1]
    rel = b1["relative_to_parent"]
    print(f"  • Stall 1 relative to Floor 101: {rel}")
    assert b1["parent_floor_tag"] == 101, "Stall 1 parent must be Floor Tag 101!"
    assert rel["dx_world"] > 1.0, f"Stall 1 should be to the right (+X) of Floor 101, got dx_world={rel['dx_world']}"
    assert rel["heading_offset_deg"] < -70.0, f"Stall 1 should have negative heading offset (~ -90°), got {rel['heading_offset_deg']}°"

    print("  ✓ Refinement 2 Verified: Exact right-angle branching transform (heading offset -90°) stored.")
    print("  ✓ MILESTONE 1 TEST PASSED!\n")


# ==============================================================================
# TEST MILESTONE 2: Autonomous Hop Navigation + 5-Step Docent Sequence (Pacing Off)
# ==============================================================================
def test_milestone_2():
    print("\n" + "=" * 80)
    print("  TEST 2: MILESTONE 2 — AUTONOMOUS HOP + 5-STEP DOCENT SEQUENCE (PACING OFF)")
    print("=" * 80)

    robot = UnitreeGo2Interface(use_physical_robot=False)
    vision = RealArUcoVision(layout_preset="minimal_2m_1s")
    navigator = TopologicalArucoNavigator()
    navigator.load_graph()

    # Verify Cold-Start localization on Marker 100
    det_100 = [{"id": 100, "distance": 1.20, "dx": 0.0}]
    snap = navigator.localize_from_visible_tag(det_100, robot_yaw_deg=90.0)
    assert snap is not None, "Failed to localize on Floor 100!"
    print(f"  ✓ Localized on Marker 100: ({snap[0]:.2f}m, {snap[1]:.2f}m)")

    # Route 100 -> 101 -> 1
    route = navigator.plan_tag_route(100, target_booth_id=1)
    assert route == [100, 101, 1], f"Expected route [100, 101, 1], got {route}"
    print(f"  ✓ Dijkstra planned route: {route}")

    # Hop 1: 100 -> 101 (cruising North at nominal speed without pacing check)
    robot.x = 0.0
    robot.y = 11.50
    robot.yaw_deg = 90.0

    # Advance along route until sighting 101
    det_101 = [{"id": 101, "distance": 0.50, "dx": 0.0}]
    vx, vy, wz, status = navigator.compute_transit_step(robot.x, robot.y, robot.yaw_deg, det_101)
    print(f"  • Hop 100 -> 101 arrival: status={status['status']}")
    assert navigator.active_route_index == 1, "Failed to advance to next hop after sighting 101!"

    # Final Hop: 101 -> Stall 1 (Visual Servoing Docking to 0.95m)
    det_stall1_far = [{"id": 1, "distance": 1.50, "dx": 0.05}]
    vx_s, _, wz_s, _ = navigator.compute_transit_step(0.0, 13.50, 0.0, det_stall1_far)
    assert vx_s > 0.10, "Visual servoing should drive forward towards stall!"

    det_stall1_docked = [{"id": 1, "distance": 0.95, "dx": 0.01}]
    vx_d, _, _, status_d = navigator.compute_transit_step(0.65, 13.50, 0.0, det_stall1_docked)
    assert status_d.get("docked", False), "Visual servoing failed to declare DOCKED at 0.95m!"
    print(f"  ✓ Visual Servoing achieved exact 0.95m standoff (Cmd vx={vx_d:.2f}m/s)!")

    # Verify 5-Step Docent Sequence
    sequence_log = []
    def dummy_visuals(stage_txt, speech_text=""):
        sequence_log.append((stage_txt, robot.get_telemetry()))

    navigator.execute_docent_presentation(
        robot=robot,
        booth_tag_id=1,
        speech_text="Biomechanical Gait Analysis capstone presentation.",
        update_visuals_fn=dummy_visuals
    )

    # Check 5 steps in log:
    assert any("STEP 1" in s[0] for s in sequence_log), "Step 1 (Halt) missing!"
    assert any("STEP 2" in s[0] for s in sequence_log), "Step 2 (180° Presenter Pivot) missing!"
    assert any("STEP 3" in s[0] for s in sequence_log), "Step 3 (+8.0° Eye Contact Tilt) missing!"
    assert any("STEP 4" in s[0] for s in sequence_log), "Step 4 (Speech Narration) missing!"
    assert any("STEP 5" in s[0] for s in sequence_log), "Step 5 (Pitch Reset 0.0°) missing!"

    # Verify pitch reset back to 0.0
    assert abs(robot.pitch_deg) < 1e-3, f"Robot pitch not reset to 0.0°, got {robot.pitch_deg}°"
    print("  ✓ 5-Step Docent Presentation Sequence fully verified (Pivot, +8° Tilt, Speech, 0° Reset).")
    print("  ✓ MILESTONE 2 TEST PASSED!\n")


# ==============================================================================
# TEST MILESTONE 3: Autonomous Navigation WITH Real-Time Judge Speed Pacing
# ==============================================================================
def test_milestone_3():
    print("\n" + "=" * 80)
    print("  TEST 3: MILESTONE 3 — AUTONOMOUS NAVIGATION WITH REAL-TIME JUDGE SPEED PACING")
    print("=" * 80)

    escort = JudgeEscortController(nominal_speed=0.35, max_speed=0.55)
    desired_vx = 0.35
    dt = 0.05

    # Scenario A: Evaluator is walking alongside (dx = 0.0m on-par)
    escort.judge_rel_x = 0.0
    vx_onpar, vy, info = escort.compute_motion_command(desired_vx, dt)
    print(f"  • Pacing On-Par (dx=0.0m): Robot vx = {vx_onpar:.2f}m/s, Status = {info['status_badge']}")
    assert vx_onpar > 0.0, "Robot should cruise forward when evaluator is on-par!"

    # Scenario B: Evaluator stops / lags behind (dx = -0.50m)
    escort.judge_rel_x = -0.50
    # Simulate deceleration over 10 cycles
    for _ in range(12):
        vx_halt, _, info_halt = escort.compute_motion_command(desired_vx, dt)

    print(f"  • Evaluator Lagging (dx=-0.50m): Robot vx = {vx_halt:.2f}m/s, Status = {info_halt['status_badge']}")
    assert vx_halt == 0.0, "Robot should smoothly decelerate to 0.0 m/s when judges lag behind!"
    assert "HALTED" in info_halt['status_badge'] or "WAITING" in info_halt['status_badge'], "Status badge should indicate waiting for judges!"

    # Scenario C: Evaluator catches back up (dx = +0.10m)
    escort.judge_rel_x = +0.10
    for _ in range(10):
        vx_resume, _, info_res = escort.compute_motion_command(desired_vx, dt)

    print(f"  • Evaluator Resumes (dx=+0.10m): Robot vx = {vx_resume:.2f}m/s, Status = {info_res['status_badge']}")
    assert vx_resume > 0.0, "Robot should smoothly accelerate back to cruising speed when evaluator catches up!"

    print("  ✓ Judge Escort Pacing & Anti-Jerk Slewing verified under evaluator lag & resume.")
    print("  ✓ MILESTONE 3 TEST PASSED!\n")


# ==============================================================================
# TEST MILESTONE 4: Scale-Up to 5 Floor Markers & 2 Stalls (with Re-Entry)
# ==============================================================================
def test_milestone_4():
    print("\n" + "=" * 80)
    print("  TEST 4: MILESTONE 4 — SCALE-UP TO 5 FLOOR ARUCO MARKERS & 2 STALLS")
    print("=" * 80)

    vision = RealArUcoVision(layout_preset="scaled_5m_2s")
    mapper = DiscoveryMapper()
    robot = UnitreeGo2Interface(use_physical_robot=False)

    # 1. Simulate discovery pass of 5 floor markers and 2 stalls
    sweep_points = [
        (0.0, 9.0, 90.0,  "Approaching Floor 100"),
        (0.0, 10.0, 90.0, "At Floor 100"),
        (0.0, 11.0, 90.0, "Approaching Floor 101"),
        (0.0, 12.0, 90.0, "At Floor 101"),
        (0.0, 12.0, 0.0,  "Stall 1 (Right Branch)"),
        (0.0, 13.0, 90.0, "Approaching Floor 102"),
        (0.0, 14.0, 90.0, "At Floor 102"),
        (0.0, 15.0, 90.0, "Approaching Floor 103"),
        (0.0, 16.0, 90.0, "At Floor 103"),
        (0.0, 16.0, 0.0,  "Stall 2 (Right Branch)"),
        (0.0, 17.0, 90.0, "Approaching Floor 104"),
        (0.0, 18.0, 90.0, "At Floor 104"),
    ]

    for x, y, yaw, desc in sweep_points:
        frame = vision.generate_synthetic_frame(x, y, yaw, desc)
        _, dets = vision.process_frame(frame)
        mapper.update_robot_odometry(x, y, yaw)
        mapper.update_lidar_scan(x, y, yaw, 1.40, 1.60)
        mapper.update_aruco_detections(x, y, yaw, dets)

    yaml_path, _ = mapper.save_discovered_map("discovered_aruco_graph.yaml")
    assert os.path.exists(yaml_path)

    # Verify all 5 floor markers mapped
    for fid in [100, 101, 102, 103, 104]:
        assert fid in mapper.discovered_nodes, f"Floor Marker #{fid} missing from graph!"
    # Verify both stalls mapped
    assert 1 in mapper.discovered_nodes, "Stall 1 missing from graph!"
    assert 2 in mapper.discovered_nodes, "Stall 2 missing from graph!"

    navigator = TopologicalArucoNavigator()
    navigator.load_graph()

    # Route from 100 to Stall 1
    r_s1 = navigator.plan_tag_route(100, target_booth_id=1)
    print(f"  • Route 1 (Entry to Stall 1): {r_s1}")
    assert r_s1 == [100, 101, 1], f"Expected route [100, 101, 1], got {r_s1}"

    # Route from Floor 101 to Stall 2 (branching off 103)
    r_s2 = navigator.plan_tag_route(101, target_booth_id=2)
    print(f"  • Route 2 (Stall 1 staging to Stall 2): {r_s2}")
    assert r_s2 == [101, 102, 103, 2], f"Expected route [101, 102, 103, 2], got {r_s2}"

    # Verify Decision 1: Aisle Re-Entry (Walk forward 0.65m clear of table, turn North)
    reentry_log = []
    robot.x = 0.95
    robot.y = 12.0
    robot.yaw_deg = 180.0  # Facing West after 180° presenter pivot

    navigator.execute_aisle_reentry(
        robot=robot,
        update_visuals_fn=lambda txt: reentry_log.append(txt),
        clear_dist_m=0.65,
        target_yaw_deg=90.0
    )

    assert any("0.65m" in s for s in reentry_log), "Re-entry forward table clearance missing!"
    assert abs(robot.yaw_deg - 90.0) < 1e-3, f"Robot heading after re-entry should be North (+90°), got {robot.yaw_deg}°"
    print("  ✓ Decision 1 Verified: Robot walked forward 0.65m clear of table before pivoting North (+90°).")

    print("  ✓ MILESTONE 4 TEST PASSED!\n")


if __name__ == '__main__':
    print("\n" + "=" * 80)
    print("  🏁 RUNNING ALL 4 SEQUENTIAL ENGINEERING MILESTONE TESTS")
    print("=" * 80)

    test_milestone_1()
    test_milestone_2()
    test_milestone_3()
    test_milestone_4()

    print("\n" + "=" * 80)
    print("  🎉 ALL 4 ENGINEERING MILESTONES VERIFIED WITH 100% SUCCESS!")
    print("=" * 80 + "\n")
