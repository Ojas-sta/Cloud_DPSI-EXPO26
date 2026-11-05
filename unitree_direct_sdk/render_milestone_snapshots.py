#!/usr/bin/env python3
"""
Renders high-resolution visual snapshot artifacts for Milestones 1, 2, 3, and 4
and saves them directly into the conversation artifact directory.
"""

import os
import cv2
import numpy as np
import math
from real_aruco_vision import RealArUcoVision
from discovery_mapper import DiscoveryMapper
from topological_aruco_navigator import TopologicalArucoNavigator
from judge_escort_controller import JudgeEscortController
from cockpit_visualizer import CockpitVisualizer

ARTIFACT_DIR = "/Users/roopalisingh/.gemini/antigravity/brain/ab9c06fe-2bf2-446d-becc-2e4354fd5ec7"
os.makedirs(ARTIFACT_DIR, exist_ok=True)


def create_milestone1_snapshot():
    """Milestone 1: Teleoperated Discovery & Mapping."""
    vision = RealArUcoVision(layout_preset="minimal_2m_1s")
    mapper = DiscoveryMapper()

    # Approached Stall 1
    frame = vision.generate_synthetic_frame(0.0, 13.50, 0.0, "Milestone 1: Facing Stall 1 (Teleop Discovery)")
    _, dets = vision.process_frame(frame)
    mapper.update_robot_odometry(0.0, 13.50, 0.0)
    mapper.update_aruco_detections(0.0, 13.50, 0.0, dets)

    viz = CockpitVisualizer(port=8089, active_milestone="M1")
    telemetry = {
        "x": 0.0, "y": 13.50, "yaw_deg": 0.0, "posture": "TELEOP_WALK",
        "vx": 0.25, "vy": 0.0, "wz": 0.0, "battery_pct": 94, "pitch_deg": 0.0
    }
    hud = viz.compose_cockpit_frame(
        camera_annotated=frame,
        telemetry=telemetry,
        active_mission_stage="MILESTONE 1: TELEOP DISCOVERY & MAPPING",
        escort_info={"status_badge": "M1: DISCOVERY (0.40s Watchdog)", "judge_rel_x": 0.0},
        mode_label="DISCOVERY MAPPER",
        topological_info={"nodes": mapper.discovered_nodes, "edges": mapper.discovered_edges}
    )
    cv2.imwrite(os.path.join(ARTIFACT_DIR, "direct_sdk_milestone1_discovered_graph.png"), hud)
    print("  ✓ Saved direct_sdk_milestone1_discovered_graph.png")


def create_milestone2_snapshot():
    """Milestone 2: Autonomous Hop + 5-Step Docent Sequence."""
    vision = RealArUcoVision(layout_preset="minimal_2m_1s")
    frame = vision.generate_synthetic_frame(0.65, 13.50, -90.0, "Milestone 2: Stall #1 Presentation (Audience View)")

    viz = CockpitVisualizer(port=8090, active_milestone="M2")
    telemetry = {
        "x": 0.65, "y": 13.50, "yaw_deg": -90.0, "posture": "BALANCE_STAND",
        "vx": 0.0, "vy": 0.0, "wz": 0.0, "battery_pct": 91, "pitch_deg": 8.0
    }
    hud = viz.compose_cockpit_frame(
        camera_annotated=frame,
        telemetry=telemetry,
        active_mission_stage="MILESTONE 2: STALL #1 DOCENT PRESENTATION (+8.0° EYE CONTACT)",
        speech_text="Biomechanical Gait Analysis capstone presentation with Unitree Go2 docent.",
        escort_info={"status_badge": "PACING: OFF (Docked Presentation)", "judge_rel_x": 0.0},
        mode_label="TOPOLOGICAL TOUR"
    )
    cv2.imwrite(os.path.join(ARTIFACT_DIR, "direct_sdk_milestone2_docent_presentation.png"), hud)
    print("  ✓ Saved direct_sdk_milestone2_docent_presentation.png")


def create_milestone3_snapshot():
    """Milestone 3: Autonomous Navigation with Judge Speed Pacing."""
    vision = RealArUcoVision(layout_preset="minimal_2m_1s")
    frame = vision.generate_synthetic_frame(0.0, 12.00, 90.0, "Milestone 3: Concourse Transit with Judge Companion Pacing")

    viz = CockpitVisualizer(port=8091, active_milestone="M3")
    telemetry = {
        "x": 0.0, "y": 12.00, "yaw_deg": 90.0, "posture": "STAND_WALK",
        "vx": 0.35, "vy": 0.0, "wz": 0.0, "battery_pct": 89, "pitch_deg": 0.0
    }
    escort_info = {
        "status_badge": "ESCORT: ON PAR (Δx: +0.05m | Smooth Slewing)",
        "judge_rel_x": 0.05,
        "judge_rel_y": 0.90,
        "judge_tracked": True,
        "raw_vx": 0.35,
        "slewed_vx": 0.35,
        "flank_clearance": 0.90
    }
    hud = viz.compose_cockpit_frame(
        camera_annotated=frame,
        telemetry=telemetry,
        active_mission_stage="MILESTONE 3: ARUCO HOP WITH JUDGE PACING",
        escort_info=escort_info,
        mode_label="TOPOLOGICAL TOUR"
    )
    cv2.imwrite(os.path.join(ARTIFACT_DIR, "direct_sdk_milestone3_judge_pacing.png"), hud)
    print("  ✓ Saved direct_sdk_milestone3_judge_pacing.png")


def create_milestone4_snapshot():
    """Milestone 4: Scaled 5 Floor Markers & 2 Stalls + Aisle Re-Entry."""
    vision = RealArUcoVision(layout_preset="scaled_5m_2s")
    mapper = DiscoveryMapper()

    for x, y, yaw in [(0.0, 9.0, 90.0), (0.0, 11.0, 90.0), (0.0, 13.0, 90.0), (0.0, 15.0, 90.0), (0.0, 17.0, 90.0)]:
        f = vision.generate_synthetic_frame(x, y, yaw, "Milestone 4: Scaled Floor Layout Sweep")
        _, dets = vision.process_frame(f)
        mapper.update_robot_odometry(x, y, yaw)
        mapper.update_aruco_detections(x, y, yaw, dets)

    frame = vision.generate_synthetic_frame(0.0, 14.0, 90.0, "Milestone 4: Concourse Arterial Track (Aisle Re-Entry to Stall #2)")
    viz = CockpitVisualizer(port=8092, active_milestone="M4")
    telemetry = {
        "x": 0.0, "y": 14.0, "yaw_deg": 90.0, "posture": "STAND_WALK",
        "vx": 0.35, "vy": 0.0, "wz": 0.0, "battery_pct": 86, "pitch_deg": 0.0
    }
    hud = viz.compose_cockpit_frame(
        camera_annotated=frame,
        telemetry=telemetry,
        active_mission_stage="MILESTONE 4: RE-ENTRY CLEAR (0.65m Forward -> North Turn)",
        escort_info={"status_badge": "M4: MULTI-STALL SCALE-UP (5 Tags / 2 Stalls)", "judge_rel_x": 0.0, "judge_tracked": False},
        topological_info={"nodes": mapper.discovered_nodes, "edges": mapper.discovered_edges, "active_route": [101, 102, 103, 2]},
        mode_label="TOPOLOGICAL TOUR"
    )
    cv2.imwrite(os.path.join(ARTIFACT_DIR, "direct_sdk_milestone4_5markers_2stalls.png"), hud)
    print("  ✓ Saved direct_sdk_milestone4_5markers_2stalls.png")


if __name__ == '__main__':
    create_milestone1_snapshot()
    create_milestone2_snapshot()
    create_milestone3_snapshot()
    create_milestone4_snapshot()
