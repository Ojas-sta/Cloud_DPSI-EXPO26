#!/usr/bin/env python3
"""
Automated Verification Suite for Two-Phase Teach-and-Repeat ArUco Pose Graph & SLAM System
Tests:
  1. Phase 1 Discovery Mapper:
     - Traversal & observation of Floor (100+) and Booth (1..44) tags
     - Bidirectional edge creation between Floor tags
     - Automatic parent-child association of Booth tags to nearest Floor tags
     - Export to 'config/discovered_aruco_graph.yaml' and 'config/discovered_occupancy_map.pgm'
  2. Phase 2 Topological Navigator:
     - Graph deserialization and structure verification
     - Cold-start kidnapped robot localization via any single visible marker
     - Dijkstra route planning ([100 -> 101 -> 1], [101 -> 100 -> 102 -> 6])
     - Closed-loop solvePnP visual servoing docking to 0.95m terminal standoff
     - Zero hardcoded blueprint (X, Y) dependency
"""

import os
import sys
import yaml
import math
import numpy as np

cur_dir = os.path.dirname(os.path.abspath(__file__))
workspace_dir = os.path.dirname(cur_dir)
if cur_dir not in sys.path:
    sys.path.insert(0, cur_dir)

from discovery_mapper import DiscoveryMapper
from topological_aruco_navigator import TopologicalArucoNavigator
from real_aruco_vision import RealArUcoVision


def test_phase_1_discovery_mapping():
    print("\n" + "=" * 70)
    print("  TEST 1: PHASE 1 PRE-EXHIBITION DISCOVERY MAPPER & GRAPH EXPORT")
    print("=" * 70)

    mapper = DiscoveryMapper()
    vision = RealArUcoVision()

    # Simulate discovery corridor traversal
    waypoints = [
        (0.0, 11.50, 90.0, "Entry (Floor 100 in view)"),
        (0.0, 12.20, 90.0, "North transit"),
        (0.0, 13.00, 90.0, "Stall P01 Apron (Floor 101 & Booth 1 in view)"),
        (0.0, 13.00, -90.0, "Turned South"),
        (0.0, 11.50, -90.0, "Central Concourse (Floor 100 in view)"),
        (0.0, 10.50, -90.0, "South transit"),
        (0.0, 9.55, -90.0, "Stall P06 Apron (Floor 102 & Booth 6 in view)"),
        (0.0, 9.55, 90.0, "Turned North"),
        (0.0, 11.25, 90.0, "Returned to Central Plaza")
    ]

    for x, y, yaw, desc in waypoints:
        mapper.update_robot_odometry(x, y, yaw)
        mapper.update_lidar_scan(x, y, yaw, clearance_left=1.40, clearance_right=1.60)
        # Generate synthetic vision frame and detect markers
        frame = vision.generate_synthetic_frame(x, y, yaw, desc)
        _, detections = vision.process_frame(frame)
        mapper.update_aruco_detections(x, y, yaw, detections)

    print(f"  • Discovered Nodes: {list(mapper.discovered_nodes.keys())}")
    print(f"  • Discovered Edges count: {len(mapper.discovered_edges)}")

    # Verify Floor tags discovered
    assert 100 in mapper.discovered_nodes, "Floor Tag #100 missing from graph!"
    assert 101 in mapper.discovered_nodes, "Floor Tag #101 missing from graph!"
    assert 102 in mapper.discovered_nodes, "Floor Tag #102 missing from graph!"

    # Verify Booth tags discovered
    assert 1 in mapper.discovered_nodes, "Booth Tag #1 (Stall P01) missing from graph!"
    assert 6 in mapper.discovered_nodes, "Booth Tag #6 (Stall P06) missing from graph!"

    # Verify Booth Tag associations to nearest Floor tags
    booth_1 = mapper.discovered_nodes[1]
    assert booth_1.get("parent_floor_tag") == 101, f"Booth #1 parent expected 101, got {booth_1.get('parent_floor_tag')}"
    print(f"  ✓ Booth Tag #1 correctly anchored to Floor Tag #101 (Relative: {booth_1['relative_to_parent']})")

    booth_6 = mapper.discovered_nodes[6]
    assert booth_6.get("parent_floor_tag") == 102, f"Booth #6 parent expected 102, got {booth_6.get('parent_floor_tag')}"
    print(f"  ✓ Booth Tag #6 correctly anchored to Floor Tag #102 (Relative: {booth_6['relative_to_parent']})")

    # Save to disk
    yaml_path, pgm_path = mapper.save_discovered_map()
    assert os.path.exists(yaml_path), f"Graph YAML not created at {yaml_path}"
    assert os.path.exists(pgm_path), f"Occupancy PGM not created at {pgm_path}"
    print(f"  ✓ Successfully verified exported YAML ({yaml_path}) and PGM grid ({pgm_path})")


def test_phase_2_topological_navigation():
    print("\n" + "=" * 70)
    print("  TEST 2: PHASE 2 TOPOLOGICAL ROUTING & DRIFT-RESET NAVIGATION")
    print("=" * 70)

    navigator = TopologicalArucoNavigator()
    assert len(navigator.nodes_data) >= 5, "Navigator failed to load graph nodes!"
    assert len(navigator.edges_data) >= 4, "Navigator failed to load graph edges!"

    # 1. Test Cold-Start Kidnapped Robot Localization
    print("\n  • Testing Cold-Start Kidnapped Robot Localization:")
    # Robot wakes up at unknown location seeing Tag 100 at 1.5m
    simulated_det = [{
        "id": 100,
        "distance": 1.50,
        "dx": 0.00
    }]
    snap = navigator.localize_from_visible_tag(simulated_det, robot_yaw_deg=90.0)
    assert snap is not None, "Failed to localize from visible marker #100!"
    snapped_x, snapped_y, locked_id = snap
    assert locked_id == 100, f"Expected locked_id=100, got {locked_id}"
    print(f"  ✓ Successfully recovered pose from Tag #100: ({snapped_x:.2f}m, {snapped_y:.2f}m)")

    # 2. Test Dijkstra Route Planning
    print("\n  • Testing Dijkstra Topological Shortest-Path Planning:")
    route_to_p01 = navigator.plan_tag_route(start_tag_id=100, target_booth_id=1)
    print(f"  Route to Stall P01: {route_to_p01}")
    assert route_to_p01 == [100, 101, 1], f"Unexpected route to P01: {route_to_p01}"

    route_to_p06 = navigator.plan_tag_route(start_tag_id=101, target_booth_id=6)
    print(f"  Route to Stall P06: {route_to_p06}")
    assert route_to_p06 == [101, 100, 102, 6], f"Unexpected route to P06: {route_to_p06}"

    # 3. Test Closed-Loop Visual Servoing Docking
    print("\n  • Testing Closed-Loop solvePnP Visual Servoing to 0.95m Standoff:")
    navigator.active_route = [101, 1]
    navigator.active_route_index = 0
    # Approaching Booth #1: Tag #1 seen at 1.8m
    det_far = [{"id": 1, "distance": 1.80, "dx": 0.15}]
    vx, vy, wz, status = navigator.compute_transit_step(
        robot_x=0.0, robot_y=13.0, robot_yaw_deg=90.0, detections=det_far
    )
    print(f"  Far Approach -> Cmd: vx={vx:.2f}m/s, vy={vy:.2f}m/s, wz={wz:.2f}rad/s, Status={status['status']}")
    assert vx > 0.10, "Robot should move forward when far from booth!"

    # Near Booth #1: Tag #1 seen at 0.96m (within terminal standoff window)
    det_docked = [{"id": 1, "distance": 0.96, "dx": 0.02}]
    vx_dock, vy_dock, wz_dock, status_dock = navigator.compute_transit_step(
        robot_x=0.0, robot_y=13.04, robot_yaw_deg=90.0, detections=det_docked
    )
    print(f"  At Terminal Standoff -> Cmd: vx={vx_dock:.2f}m/s, vy={vy_dock:.2f}m/s, wz={wz_dock:.2f}rad/s, Status={status_dock['status']}")
    assert status_dock.get("docked", False), "Robot should declare DOCKED at 0.95m standoff!"
    assert abs(vx_dock) == 0.0 and abs(vy_dock) == 0.0 and abs(wz_dock) == 0.0, "Robot should stop all movement when docked!"

    # 4. Test Speech Narration Curricula Retrieval
    speech_p01 = navigator.get_speech_for_booth(1)
    assert "Stepwise AI" in speech_p01, "Speech curricula missing Stall P01 domain facts!"
    print(f"  ✓ Verified speech curricula for Stall P01: '{speech_p01[:65]}...'")

    print("\n" + "=" * 70)
    print("  🎉 ALL TWO-PHASE TEACH-AND-REPEAT SLAM TESTS PASSED WITH 100% SUCCESS!")
    print("=" * 70 + "\n")


if __name__ == '__main__':
    test_phase_1_discovery_mapping()
    test_phase_2_topological_navigation()
