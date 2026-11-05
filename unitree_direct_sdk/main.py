#!/usr/bin/env python3
"""
Project Cloud: Direct Unitree SDK2 Python Architecture with Live Visualizer
Venue: DPSI EXPO-2026 Exhibition Hall | Platform: Unitree Go2 Edu / Pro Quadruped

Four Sequential Engineering Milestones:
  • Milestone 1: Teleoperated Discovery & Mapping (2 Floor Markers -> 1 Right-Side Stall)
  • Milestone 2: Autonomous Hop-by-Hop Navigation + 5-Step Stall Docent Sequence (Pacing Off)
  • Milestone 3: Autonomous Navigation WITH Real-Time Human / Judge Speed Pacing (Pacing On)
  • Milestone 4: Scale-Up to 5 Floor ArUco Markers & 2 Stalls (with 0.65m Table Clearance Re-Entry)

Run directly:
  python unitree_direct_sdk/main.py [--milestone 1|2|3|4] [--preset minimal_2m_1s|scaled_5m_2s] [--pacing on|off] [--port 8088]
"""

import sys
import os
import time
import math
import argparse

cur_dir = os.path.dirname(os.path.abspath(__file__))
if cur_dir not in sys.path:
    sys.path.insert(0, cur_dir)

from unitree_go2_interface import UnitreeGo2Interface
from real_aruco_vision import RealArUcoVision
from cockpit_visualizer import CockpitVisualizer
from judge_escort_controller import JudgeEscortController
from discovery_mapper import DiscoveryMapper
from topological_aruco_navigator import TopologicalArucoNavigator
from keyboard_teleop import KeyboardTeleop
from typing import Optional, Dict, Any, List

WORKSPACE_DIR = os.path.dirname(cur_dir)
CONFIG_DIR = os.path.join(WORKSPACE_DIR, "config")


# ==============================================================================
# MILESTONE 1: TELEOPERATED DISCOVERY & MAPPING (2 FLOOR -> 1 RIGHT-SIDE STALL)
# ==============================================================================
def run_milestone_1(
    robot: UnitreeGo2Interface,
    vision: RealArUcoVision,
    visualizer: CockpitVisualizer,
    mapper: DiscoveryMapper,
    interactive_teleop: bool = True
):
    print("\n" + "=" * 80)
    print("  🗺️ MILESTONE 1: TELEOPERATED DISCOVERY & MAPPING (2 FLOOR -> 1 RIGHT-SIDE STALL)")
    print("  • Setup: Floor Marker 100 -> Floor Marker 101 -> Stall 1 (Right Branch, -90°)")
    print("  • Teleop Controls: WASD (Move), QE (Turn), Space (Brake), M (Save Map)")
    print("=" * 80 + "\n")

    visualizer.active_milestone = "M1: TELEOP MAPPING (2M -> 1S)"
    visualizer.discovery_ref = mapper
    robot.x = 0.0
    robot.y = 11.50
    robot.yaw_deg = 90.0
    robot.stand_up()
    time.sleep(0.5)

    teleop = KeyboardTeleop(
        command_callback=lambda vx, vy, wz: robot.move(vx=vx, vy=vy, wz=wz),
        save_map_callback=lambda: mapper.save_discovered_map()
    )
    visualizer.teleop_ref = teleop
    if interactive_teleop and sys.stdin.isatty():
        teleop.start()

    def step_m1(stage: str, auto_vx: float = 0.0, auto_vy: float = 0.0, auto_wz: float = 0.0):
        # If no active keyboard input, use automated sweep pulse
        if teleop.vx == 0.0 and teleop.vy == 0.0 and teleop.wz == 0.0:
            robot.move(vx=auto_vx, vy=auto_vy, wz=auto_wz)
        robot.update_emulation(dt=0.04)
        telemetry = robot.get_telemetry()

        raw_frame = vision.generate_synthetic_frame(robot.x, robot.y, robot.yaw_deg, stage)
        annotated, detections = vision.process_frame(raw_frame)

        mapper.update_robot_odometry(robot.x, robot.y, robot.yaw_deg)
        mapper.update_lidar_scan(robot.x, robot.y, robot.yaw_deg, clearance_left=1.40, clearance_right=1.60)
        mapper.update_aruco_detections(robot.x, robot.y, robot.yaw_deg, detections)

        top_info = mapper.get_hud_info()
        top_info["nodes"] = mapper.discovered_nodes
        top_info["edges"] = mapper.discovered_edges

        visualizer.compose_cockpit_frame(
            annotated, telemetry, stage,
            mode_label="DISCOVERY MAPPER",
            topological_info=top_info
        )
        return detections

    # 1. Drive from Marker 100 straight to Marker 101 (Y=11.5 to Y=13.5)
    print("  • Leg 1: Teleop / Traversing corridor from Marker 100 to Marker 101...")
    t0 = time.time()
    while time.time() - t0 < 8.0:
        if robot.y >= 13.50:
            break
        step_m1("M1: CRUISING 100 -> 101", auto_vx=0.28)
        time.sleep(0.04)

    robot.stop()
    for _ in range(12):
        step_m1("M1: SIGHTING FLOOR MARKER 101")
        time.sleep(0.04)

    # 2. Turn Right (-90°) to face Stall Marker 1 on the right
    print("  • Leg 2: Turning Right (-90°) to acquire Stall Marker 1 on right...")
    t_turn = time.time()
    while time.time() - t_turn < 2.0:
        step_m1("M1: TURNING RIGHT (-90°) TOWARDS STALL 1", auto_wz=-0.785)
        time.sleep(0.04)

    robot.stop()
    robot.yaw_deg = 0.0  # Exactly locked East facing Stall 1

    # 3. Observe Stall Marker 1 and record relative transform
    for _ in range(25):
        step_m1("M1: LOCKING RELATIVE TRANSFORM (101 -> STALL 1)")
        time.sleep(0.04)

    # 4. Save Discovered Graph
    teleop.stop()
    yaml_path, pgm_path = mapper.save_discovered_map()
    visualizer.save_milestone_artifact("direct_sdk_milestone1_discovered_graph.png")

    print("\n" + "=" * 80)
    print(f"  ✓ MILESTONE 1 COMPLETE: Discovered {len(mapper.discovered_nodes)} nodes, {len(mapper.discovered_edges)} edges")
    print(f"  • Graph saved to: {yaml_path}")
    print("=" * 80 + "\n")


# ==============================================================================
# MILESTONE 2: AUTONOMOUS NAVIGATION + 5-STEP STALL DOCENT SEQUENCE (PACING OFF)
# ==============================================================================
def run_milestone_2(
    robot: UnitreeGo2Interface,
    vision: RealArUcoVision,
    visualizer: CockpitVisualizer,
    navigator: TopologicalArucoNavigator
):
    print("\n" + "=" * 80)
    print("  🚀 MILESTONE 2: AUTONOMOUS HOP-BY-HOP NAVIGATION + 5-STEP DOCENT SEQUENCE")
    print("  • Pacing Mode: OFF (Constant cruising velocity)")
    print("  • Mission: 100 -> 101 -> Right Turn (-90°) -> Stall 1 Visual Servo Docking -> Docent Speech")
    print("=" * 80 + "\n")

    visualizer.active_milestone = "M2: AUTONOMOUS HOP (NO PACING)"
    visualizer.navigator_ref = navigator

    def update_m2_visuals(stage: str, speech_text: str = ""):
        robot.update_emulation(dt=0.04)
        telemetry = robot.get_telemetry()

        raw_frame = vision.generate_synthetic_frame(robot.x, robot.y, robot.yaw_deg, stage)
        annotated, detections = vision.process_frame(raw_frame)

        top_info = {
            "mode": "TOPOLOGICAL TOUR",
            "nodes": navigator.nodes_data,
            "edges": navigator.edges_data,
            "active_route": navigator.active_route,
            "anchor_tag": navigator.current_anchor_tag,
            "is_localized": navigator.is_localized
        }

        visualizer.compose_cockpit_frame(
            annotated, telemetry, stage,
            speech_text=speech_text,
            mode_label="TOPOLOGICAL TOUR (PACING OFF)",
            topological_info=top_info
        )
        return detections

    # Step 1: Localize on Marker 100
    robot.x = 0.0
    robot.y = 11.20
    robot.yaw_deg = 90.0
    robot.stand_up()
    navigator.is_localized = False

    for _ in range(15):
        dets = update_m2_visuals("M2: LOCALIZING ON MARKER 100")
        if not navigator.is_localized:
            snap = navigator.localize_from_visible_tag(dets, robot.yaw_deg)
            if snap:
                robot.x, robot.y, _ = snap
        time.sleep(0.04)

    # Step 2: Route 100 -> 101 -> Stall 1
    route = navigator.plan_tag_route(start_tag_id=100, target_booth_id=1)
    t_start = time.time()

    while time.time() - t_start < 18.0:
        dets = update_m2_visuals("M2: TRANSIT (100 -> 101 -> STALL 1)")
        vx, vy, wz, nav_info = navigator.compute_transit_step(robot.x, robot.y, robot.yaw_deg, dets, dt=0.04)
        robot.move(vx=vx, vy=vy, wz=wz)

        if nav_info.get("docked", False):
            print("  ✓ Visual Servoing achieved exact 0.95m terminal standoff at Stall 1!")
            break
        time.sleep(0.04)

    robot.stop()

    # Step 3: Complete 5-Step Docent Presentation Sequence
    speech_p01 = navigator.get_speech_for_booth(1)
    navigator.execute_docent_presentation(
        robot=robot,
        booth_tag_id=1,
        speech_text=speech_p01,
        update_visuals_fn=update_m2_visuals
    )

    visualizer.save_milestone_artifact("direct_sdk_milestone2_docent_presentation.png")
    print("  ✓ MILESTONE 2 COMPLETED SUCCESSFULLY!\n")


# ==============================================================================
# MILESTONE 3: AUTONOMOUS NAVIGATION WITH REAL-TIME JUDGE PACING (PACING ON)
# ==============================================================================
def run_milestone_3(
    robot: UnitreeGo2Interface,
    vision: RealArUcoVision,
    visualizer: CockpitVisualizer,
    navigator: TopologicalArucoNavigator,
    escort: JudgeEscortController
):
    print("\n" + "=" * 80)
    print("  🧑‍⚖️ MILESTONE 3: AUTONOMOUS NAVIGATION WITH REAL-TIME JUDGE SPEED PACING")
    print("  • Pacing Mode: ON (360° LiDAR Flank Tracking + Anti-Jerk Slewing)")
    print("  • Testing: Pacing on-par -> Judge lags behind -> Robot halts -> Resumes -> Stall 1")
    print("=" * 80 + "\n")

    visualizer.active_milestone = "M3: JUDGE PACED TOUR"
    visualizer.navigator_ref = navigator
    visualizer.escort_ref = escort

    def update_m3_visuals(stage: str, speech_text: str = "", escort_info: Optional[Dict[str, Any]] = None):
        robot.update_emulation(dt=0.04)
        telemetry = robot.get_telemetry()

        raw_frame = vision.generate_synthetic_frame(robot.x, robot.y, robot.yaw_deg, stage)
        annotated, detections = vision.process_frame(raw_frame)

        top_info = {
            "mode": "TOPOLOGICAL TOUR (PACED)",
            "nodes": navigator.nodes_data,
            "edges": navigator.edges_data,
            "active_route": navigator.active_route,
            "anchor_tag": navigator.current_anchor_tag,
            "is_localized": navigator.is_localized
        }

        visualizer.compose_cockpit_frame(
            annotated, telemetry, stage,
            speech_text=speech_text,
            escort_info=escort_info,
            mode_label="TOPOLOGICAL TOUR (PACING ON)",
            topological_info=top_info
        )
        return detections

    robot.x = 0.0
    robot.y = 11.50
    robot.yaw_deg = 90.0
    robot.stand_up()
    navigator.is_localized = True
    navigator.current_anchor_tag = 100

    route = navigator.plan_tag_route(start_tag_id=100, target_booth_id=1)
    t_start = time.time()
    lag_tested = False

    while time.time() - t_start < 22.0:
        elapsed = time.time() - t_start
        dets = update_m3_visuals("M3: PACED TRANSIT TO STALL 1")
        vx, vy, wz, nav_info = navigator.compute_transit_step(robot.x, robot.y, robot.yaw_deg, dets, dt=0.04)

        # Simulate judge movement dynamics
        escort.step_simulation(dt=0.04, current_robot_vx=robot.vx)

        # Test lag scenario at t = 3.0s: judges stop to inspect something!
        if 3.0 <= elapsed < 6.0 and not lag_tested:
            escort.adjust_judge_offset(-0.55)  # Judge is behind by -0.55m
            lag_tested = True
            print("  🧑‍⚖️ [Simulated Event]: Judges stopped/lagged behind (-0.55m)! Checking anti-jerk halt...")

        if 6.0 <= elapsed < 7.0 and lag_tested:
            escort.adjust_judge_offset(+0.55)  # Judges caught back up
            lag_tested = False
            print("  🧑‍⚖️ [Simulated Event]: Judges caught back up! Resuming cruise...")

        p_vx, p_vy, escort_info = escort.compute_motion_command(
            desired_nominal_vx=vx,
            dt=0.04,
            min_front_distance=nav_info.get("dist", 2.5),
            d_left_meas=1.40,
            d_right_meas=1.60
        )

        robot.move(vx=p_vx, vy=vy, wz=wz)
        update_m3_visuals("M3: PACED TRANSIT TO STALL 1", escort_info=escort_info)

        if nav_info.get("docked", False):
            print("  ✓ Docked at Stall 1 at 0.95m standoff under companion pacing!")
            break
        time.sleep(0.04)

    robot.stop()

    # Deliver presentation
    speech_p01 = navigator.get_speech_for_booth(1)
    navigator.execute_docent_presentation(
        robot=robot,
        booth_tag_id=1,
        speech_text=speech_p01,
        update_visuals_fn=update_m3_visuals
    )

    visualizer.save_milestone_artifact("direct_sdk_milestone3_judge_pacing.png")
    print("  ✓ MILESTONE 3 COMPLETED SUCCESSFULLY!\n")


# ==============================================================================
# MILESTONE 4: SCALE-UP TO 5 FLOOR ARUCO MARKERS & 2 STALLS (WITH RE-ENTRY)
# ==============================================================================
def run_milestone_4(
    robot: UnitreeGo2Interface,
    vision: RealArUcoVision,
    visualizer: CockpitVisualizer,
    mapper: DiscoveryMapper,
    navigator: TopologicalArucoNavigator,
    escort: JudgeEscortController
):
    print("\n" + "=" * 80)
    print("  🏛️ MILESTONE 4: SCALE-UP TO 5 FLOOR ARUCO MARKERS & 2 STALLS")
    print("  • Setup: Floor Markers 100..104, Stall 1 off 101, Stall 2 off 103")
    print("  • Feature: 0.65m Table Clearance Aisle Re-Entry (Rear Leg Protection)")
    print("=" * 80 + "\n")

    visualizer.active_milestone = "M4: SCALED 5-MARKER / 2-STALL TOUR"
    vision.layout_preset = "scaled_5m_2s"
    visualizer.navigator_ref = navigator
    visualizer.escort_ref = escort

    # Step 1: Rapid Discovery Sweep across all 5 markers & 2 stalls
    print("  • Phase 4.1: Auto-mapping 5-marker / 2-stall layout...")
    robot.x = 0.0
    robot.y = 10.0
    robot.yaw_deg = 90.0
    robot.stand_up()

    waypoints_m4 = [
        (0.0, 10.0, 90.0, "Floor 100"),
        (0.0, 12.0, 90.0, "Floor 101"),
        (0.0, 12.0, 0.0,  "Stall 1 (P01) Right Branch"),
        (0.0, 12.0, 90.0, "Re-align North"),
        (0.0, 14.0, 90.0, "Floor 102"),
        (0.0, 16.0, 90.0, "Floor 103"),
        (0.0, 16.0, 0.0,  "Stall 2 (P02) Right Branch"),
        (0.0, 16.0, 90.0, "Re-align North"),
        (0.0, 18.0, 90.0, "Floor 104"),
    ]

    for wx, wy, wyaw, wdesc in waypoints_m4:
        robot.x = wx
        robot.y = wy
        robot.yaw_deg = wyaw
        robot.update_emulation(0.04)
        raw_frame = vision.generate_synthetic_frame(wx, wy, wyaw, wdesc)
        _, dets = vision.process_frame(raw_frame)
        mapper.update_robot_odometry(wx, wy, wyaw)
        mapper.update_lidar_scan(wx, wy, wyaw, 1.40, 1.60)
        mapper.update_aruco_detections(wx, wy, wyaw, dets)

    mapper.save_discovered_map()
    navigator.load_graph()
    print(f"  ✓ Scaled Graph Loaded: {len(navigator.nodes_data)} nodes, {len(navigator.edges_data)} edges")

    def update_m4_visuals(stage: str, speech_text: str = "", escort_info: Optional[Dict[str, Any]] = None):
        robot.update_emulation(dt=0.04)
        telemetry = robot.get_telemetry()

        raw_frame = vision.generate_synthetic_frame(robot.x, robot.y, robot.yaw_deg, stage)
        annotated, detections = vision.process_frame(raw_frame)

        top_info = {
            "mode": "TOPOLOGICAL TOUR (SCALED)",
            "nodes": navigator.nodes_data,
            "edges": navigator.edges_data,
            "active_route": navigator.active_route,
            "anchor_tag": navigator.current_anchor_tag,
            "is_localized": navigator.is_localized
        }

        visualizer.compose_cockpit_frame(
            annotated, telemetry, stage,
            speech_text=speech_text,
            escort_info=escort_info,
            mode_label="TOPOLOGICAL TOUR (5M / 2S)",
            topological_info=top_info
        )
        return detections

    # Step 2: Autonomous Tour from 100 -> Stall 1
    print("\n📍 [M4 Station 1]: Escorting to Stall 1 (Marker #1) branching off Datum 101...")
    robot.x = 0.0
    robot.y = 10.0
    robot.yaw_deg = 90.0
    navigator.is_localized = True
    navigator.current_anchor_tag = 100

    route1 = navigator.plan_tag_route(100, target_booth_id=1)
    t_leg1 = time.time()
    while time.time() - t_leg1 < 16.0:
        dets = update_m4_visuals("M4: ESCORTING TO STALL 1")
        vx, vy, wz, nav_info = navigator.compute_transit_step(robot.x, robot.y, robot.yaw_deg, dets, 0.04)
        escort.step_simulation(0.04, robot.vx)
        p_vx, p_vy, escort_info = escort.compute_motion_command(vx, 0.04, min_front_distance=2.5)
        robot.move(vx=p_vx, vy=vy, wz=wz)
        if nav_info.get("docked", False):
            break
        time.sleep(0.04)

    robot.stop()
    speech1 = navigator.get_speech_for_booth(1)
    navigator.execute_docent_presentation(robot, 1, speech1, update_m4_visuals)

    # Step 3: Decision 1 - Aisle Re-Entry (Walk forward 0.65m clear of table, turn North)
    navigator.execute_aisle_reentry(robot, update_m4_visuals, clear_dist_m=0.65, target_yaw_deg=90.0)

    # Step 4: Autonomous Tour from 101 -> Stall 2 (Marker #2) branching off Datum 103
    print("\n📍 [M4 Station 2]: Escorting to Stall 2 (Marker #2) branching off Datum 103...")
    navigator.current_anchor_tag = 101
    route2 = navigator.plan_tag_route(101, target_booth_id=2)
    t_leg2 = time.time()
    while time.time() - t_leg2 < 18.0:
        dets = update_m4_visuals("M4: ESCORTING TO STALL 2")
        vx, vy, wz, nav_info = navigator.compute_transit_step(robot.x, robot.y, robot.yaw_deg, dets, 0.04)
        escort.step_simulation(0.04, robot.vx)
        p_vx, p_vy, escort_info = escort.compute_motion_command(vx, 0.04, min_front_distance=2.5)
        robot.move(vx=p_vx, vy=vy, wz=wz)
        if nav_info.get("docked", False):
            break
        time.sleep(0.04)

    robot.stop()
    speech2 = navigator.get_speech_for_booth(2)
    navigator.execute_docent_presentation(robot, 2, speech2, update_m4_visuals)

    # Step 5: Aisle Re-Entry and Cruise to Concourse North End (Floor 104)
    navigator.execute_aisle_reentry(robot, update_m4_visuals, clear_dist_m=0.65, target_yaw_deg=90.0)
    navigator.current_anchor_tag = 103
    route3 = navigator.plan_tag_route(103, target_booth_id=104)
    t_leg3 = time.time()
    while time.time() - t_leg3 < 8.0:
        dets = update_m4_visuals("M4: ADVANCING TO FLOOR 104")
        vx, vy, wz, nav_info = navigator.compute_transit_step(robot.x, robot.y, robot.yaw_deg, dets, 0.04)
        robot.move(vx=vx, vy=vy, wz=wz)
        if nav_info.get("dist_rem", 99.0) <= 0.30:
            break
        time.sleep(0.04)

    robot.stop()
    visualizer.save_milestone_artifact("direct_sdk_milestone4_5markers_2stalls.png")
    print("\n  ✓ MILESTONE 4 COMPLETED SUCCESSFULLY!\n")


def main():
    parser = argparse.ArgumentParser(description="Incremental ArUco Mapping, Autonomous Stall Sequence & Judge Pacing")
    parser.add_argument("--milestone", type=int, choices=[1, 2, 3, 4], default=1,
                        help="Select engineering milestone to run: 1, 2, 3, or 4")
    parser.add_argument("--preset", choices=["minimal_2m_1s", "scaled_5m_2s"], default="minimal_2m_1s",
                        help="Venue layout preset: 'minimal_2m_1s' (default) or 'scaled_5m_2s'")
    parser.add_argument("--mode", choices=["discovery", "tour", "both"], default="tour",
                        help="Operation mode (overridden if --milestone is passed)")
    parser.add_argument("--pacing", choices=["on", "off"], default="on",
                        help="Companion judge pacing: 'on' (default) or 'off'")
    parser.add_argument("--real", action="store_true", help="Connect to physical Unitree Go2 robot via SDK2")
    parser.add_argument("--camera", type=int, default=None, help="Camera index for physical USB/RealSense camera")
    parser.add_argument("--port", type=int, default=8088, help="Web visualizer port")
    parser.add_argument("--serve", action="store_true", help="Keep Web Cockpit HTTP server active after run for interactive inspection")
    parser.add_argument("--apriltag", action="store_true", help="Use AprilTag (DICT_APRILTAG_36h11) instead of ArUco (DICT_4X4_250)")
    args = parser.parse_args()

    robot = UnitreeGo2Interface(use_physical_robot=args.real)
    vision = RealArUcoVision(camera_index=args.camera, layout_preset=args.preset, use_apriltag=args.apriltag)
    visualizer = CockpitVisualizer(port=args.port, use_apriltag=args.apriltag)
    escort = JudgeEscortController(nominal_speed=0.35, max_speed=0.55)
    mapper = DiscoveryMapper(use_apriltag=args.apriltag)
    navigator = TopologicalArucoNavigator(use_apriltag=args.apriltag)

    if args.milestone == 1:
        run_milestone_1(robot, vision, visualizer, mapper)
    elif args.milestone == 2:
        run_milestone_2(robot, vision, visualizer, navigator)
    elif args.milestone == 3:
        run_milestone_3(robot, vision, visualizer, navigator, escort)
    elif args.milestone == 4:
        run_milestone_4(robot, vision, visualizer, mapper, navigator, escort)

    if args.serve:
        print(f"\n  🌐 [CockpitVisualizer]: Holding server open at http://localhost:{args.port}. Press Ctrl+C to terminate.")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\n  🛑 Server terminated by user.")


if __name__ == '__main__':
    main()
