"""
Project Cloud: Closed-Loop Terminal Precision Standoff Docking Unit Tests
Verifies:
  1. RANSAC 3D Plane Fitting & Normal Vector Extraction Accuracy
  2. Multi-Modal Optical + LiDAR Point Cloud Fusion
  3. 3-DOF Holonomic PD Control Law Saturation & Slew Acceleration Limiting
  4. Slip-Gated Traction Scaling on Slick Floors (F_z < 10 N)
  5. 10-Cycle Convergence Latch & Presentation Posture (+8 deg eye contact)
  6. Automatic Target Standoff Geometry (0.95m for stalls, 0.50m for stage dock 99)
  7. Occlusion Timeout & Fault Recovery
"""

import os
import sys
import time
import math
import pytest
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import String, Int32, Bool, Float32MultiArray

repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src_dir = os.path.join(repo_dir, "ros2_ws", "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)
if repo_dir not in sys.path:
    sys.path.insert(0, repo_dir)

from cloud_navigation.terminal_docking_servo import (
    TerminalDockingServoNode,
    RansacPlaneFitter,
    DockingState,
)


@pytest.fixture(scope="module")
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_ransac_plane_fitting_synthetic():
    """Validates RANSAC plane fitting on a noisy synthetic backboard with outliers."""
    fitter = RansacPlaneFitter(distance_threshold=0.02, max_iterations=100, min_inliers=30)

    # Generate synthetic plane at Z = 1.25m with slight yaw (+3 degrees)
    # in camera optical frame: plane equation ~ -sin(yaw)*X - cos(yaw)*Z + 1.25 = 0
    np.random.seed(123)
    n_inliers = 150
    x_in = np.random.uniform(-0.4, 0.4, n_inliers)
    y_in = np.random.uniform(-0.3, 0.3, n_inliers)
    # Vertical backboard with small tilt
    z_in = 1.25 + 0.05 * x_in + np.random.normal(0, 0.005, n_inliers)
    inliers = np.column_stack([x_in, y_in, z_in])

    # Add 40 random outliers
    n_outliers = 40
    outliers = np.random.uniform(-1.0, 1.0, (n_outliers, 3))
    outliers[:, 2] = np.random.uniform(0.5, 3.0, n_outliers)

    cloud = np.vstack([inliers, outliers])
    fit_res = fitter.fit(cloud)

    assert fit_res is not None, "RANSAC plane fitting must succeed on synthetic backboard"
    normal, d_perp, coeffs, inlier_count = fit_res

    # Plane normal should point toward the camera [0, 0, -1] approximately
    assert normal[2] < -0.95, f"Normal Z must point toward camera, got {normal}"
    assert abs(d_perp - 1.25) < 0.02, f"Orthogonal standoff {d_perp:.3f}m must be close to 1.25m"
    assert inlier_count >= 130, f"Expected >= 130 inliers, got {inlier_count}"


def test_frustum_point_extraction(rclpy_init):
    """Verifies 3D bounding frustum cropping around optical marker ray."""
    node = TerminalDockingServoNode("test_docking_frustum")
    np.random.seed(42)

    # Cloud spread across [-2, 2] in X, Y, Z
    cloud = np.random.uniform(-2.0, 2.0, (500, 3))
    # Place known points inside frustum centered at (0.2, -0.1, 1.5)
    target_pts = np.array([
        [0.22, -0.08, 1.52],
        [0.19, -0.12, 1.48],
        [0.25, -0.05, 1.55]
    ])
    full_cloud = np.vstack([cloud, target_pts])

    cropped = node.extract_frustum_points(
        full_cloud, center_x=0.20, center_y=-0.10, center_z=1.50,
        half_x=0.30, half_y=0.30, half_z=0.30
    )
    assert len(cropped) >= 3, "Cropped points must include target points"
    for pt in cropped:
        assert abs(pt[0] - 0.20) <= 0.30
        assert abs(pt[1] - (-0.10)) <= 0.30
        assert abs(pt[2] - 1.50) <= 0.30


def test_target_standoff_geometry_assignment(rclpy_init):
    """Verifies default standoff assignment for standard stalls (0.95m) vs Stage Dock (0.50m)."""
    node = TerminalDockingServoNode("test_docking_geometry")

    # 1. Booth stall 15 (AprilTag 24)
    node.start_docking(target_apriltag_id=24, label="STALL_15")
    assert node.target_id == 24
    assert node.target_standoff_m == 0.95, f"Expected 0.95m standoff for stall, got {node.target_standoff_m}"

    # 2. Stage Recharge Dock (AprilTag 99)
    node.start_docking(target_apriltag_id=99, label="STAGE_DOCK_99")
    assert node.target_id == 99
    assert node.target_standoff_m == 0.50, f"Expected 0.50m standoff for dock 99, got {node.target_standoff_m}"


def test_holonomic_pd_clamping_and_acceleration_limiting(rclpy_init):
    """Verifies velocity saturation envelopes and acceleration limits."""
    node = TerminalDockingServoNode("test_docking_pd")
    node.start_docking(target_apriltag_id=10, standoff_m=0.95)

    # Simulate large positive errors (far away, offset left, tilted right)
    node.current_tag_id = 10
    node.current_tag_pose = {"x": 0.80, "y": 0.0, "z": 3.00, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    node.last_tag_seen_time = time.time()

    # Step 1: should be limited by acceleration slew limit from 0
    res1 = node.control_step()
    assert res1["state"] == "SERVOING"
    assert res1["vx"] <= node.max_vx_fwd
    assert res1["vx"] > 0.0
    assert abs(res1["vy"]) <= node.max_vy
    assert abs(res1["wz"]) <= node.max_wz

    # Run multiple steps to let velocities reach peak clamped bounds
    for _ in range(25):
        time.sleep(0.02)
        res = node.control_step()

    assert res["vx"] <= node.max_vx_fwd + 1e-4
    assert res["vx"] >= node.max_vx_rev - 1e-4
    assert abs(res["vy"]) <= node.max_vy + 1e-4
    assert abs(res["wz"]) <= node.max_wz + 1e-4


def test_slip_gated_traction_scaling(rclpy_init):
    """Verifies that low foot normal forces scale down commanded velocity on slick floors."""
    node = TerminalDockingServoNode("test_docking_slip")
    node.start_docking(target_apriltag_id=12, standoff_m=0.95)
    node.current_tag_id = 12
    node.current_tag_pose = {"x": 0.0, "y": 0.0, "z": 2.00, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    node.last_tag_seen_time = time.time()

    # Normal traction (F_z = 18N)
    node.latest_foot_forces = [18.0, 18.0, 18.0, 18.0]
    res_normal = node.control_step()
    vx_normal = res_normal["vx"]

    # Low traction / slip condition (F_z = 5N on front feet)
    node.latest_foot_forces = [5.0, 5.0, 18.0, 18.0]
    res_slip = node.control_step()
    vx_slip = res_slip["vx"]

    # Slipped velocity should be scaled down relative to nominal
    assert vx_slip < vx_normal or vx_slip <= (5.0 / 18.0) * node.max_vx_fwd + 0.05


def test_convergence_latch_and_presentation_posture(rclpy_init):
    """Verifies that sustaining 10 cycles within tolerances triggers presentation pitch."""
    node = TerminalDockingServoNode("test_docking_latch")
    node.start_docking(target_apriltag_id=14, standoff_m=0.95)
    node.current_tag_id = 14
    # Pose within tolerances: |e_x| = |0.96 - 0.95| = 0.01m <= 0.03m, e_y = -0.01m, e_th ~ 0.01 rad
    node.current_tag_pose = {"x": 0.01, "y": 0.0, "z": 0.96, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    node.last_tag_seen_time = time.time()

    # Step through 9 cycles: should be SETTLING, not yet PRESENTATION_LOCKED
    for cycle in range(1, 10):
        time.sleep(0.01)
        res = node.control_step()
        assert res["state"] == "SETTLING", f"Cycle {cycle} should be SETTLING, got {res['state']}"
        assert res["settled_cycles"] == cycle

    # 10th cycle: triggers latch!
    time.sleep(0.01)
    res_10 = node.control_step()
    assert res_10["state"] == DockingState.PRESENTATION_LOCKED.value
    assert res_10["vx"] == 0.0
    assert res_10["vy"] == 0.0
    assert res_10["wz"] == 0.0
    assert node.state == DockingState.PRESENTATION_LOCKED
    assert node.presentation_pitch_rad == -0.14 # +8 deg nose-up eye contact


def test_occlusion_timeout_abort(rclpy_init):
    """Verifies abort triggers if optical tracking is lost for > 3.0 seconds."""
    node = TerminalDockingServoNode("test_docking_occlusion")
    node.start_docking(target_apriltag_id=15, standoff_m=0.95)
    node.current_tag_id = 15
    node.current_tag_pose = {"x": 0.0, "y": 0.0, "z": 1.80, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    node.last_tag_seen_time = time.time() - 3.5 # 3.5 seconds ago

    res = node.control_step()
    assert res["state"] == DockingState.ABORTED.value
    assert node.state == DockingState.ABORTED
    assert res["vx"] == 0.0
    assert res["vy"] == 0.0
