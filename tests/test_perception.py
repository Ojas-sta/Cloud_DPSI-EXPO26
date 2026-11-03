"""
Project Cloud: Perception Test Suite
Validates ArUco 6DOF marker pose estimation and floor spill hazard detection.
"""

import sys
import os
import pytest
import numpy as np

repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src_dir = os.path.join(repo_dir, "ros2_ws", "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from cloud_perception import ArucoTrackerNode, FloorSpillDetectorNode

def test_aruco_tracker_detection_and_pnp():
    tracker = ArucoTrackerNode(marker_size_m=0.16)
    # Generate frame with marker ID 10 at 1.8m distance
    synthetic_frame = tracker.generate_synthetic_marker_frame(
        marker_id=10, distance_m=1.80, lateral_offset_m=0.0
    )

    detections = tracker.process_frame(synthetic_frame)
    assert len(detections) > 0, "Failed to detect synthetic ArUco marker!"

    d = detections[0]
    assert d["marker_id"] == 10
    # Tolerance within 0.20m on synthetic projection
    assert abs(d["distance_m"] - 1.80) < 0.20
    assert abs(d["lateral_offset_m"]) < 0.05
    assert d["pose"] is not None

def test_aruco_tracker_offset_heading():
    tracker = ArucoTrackerNode(marker_size_m=0.16)
    # Marker shifted laterally to the right by +0.15m
    synthetic_frame = tracker.generate_synthetic_marker_frame(
        marker_id=12, distance_m=1.80, lateral_offset_m=0.15
    )

    detections = tracker.process_frame(synthetic_frame)
    assert len(detections) > 0
    d = detections[0]
    assert d["marker_id"] == 12
    assert d["lateral_offset_m"] > 0.08  # Confirms positive lateral detection

def test_floor_spill_detector_positive():
    detector = FloorSpillDetectorNode(confidence_threshold=0.80, min_area_sq_m=0.05)
    bgr, depth = detector.generate_synthetic_spill_frame(with_spill=True)

    alert = detector.process_rgbd(bgr, depth)
    assert alert is not None, "Failed to detect liquid spill puddle!"
    assert alert.confidence >= 0.80
    assert alert.estimated_area_sq_m >= 0.05
    assert alert.severity == "CRITICAL"
    assert len(alert.image_base64_jpeg) > 200

def test_floor_spill_detector_clean_floor_negative():
    detector = FloorSpillDetectorNode(confidence_threshold=0.80, min_area_sq_m=0.05)
    bgr, depth = detector.generate_synthetic_spill_frame(with_spill=False)

    alert = detector.process_rgbd(bgr, depth)
    assert alert is None, "False positive detected on clean concrete floor!"
