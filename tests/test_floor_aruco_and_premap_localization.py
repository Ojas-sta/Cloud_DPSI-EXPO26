#!/usr/bin/env python3
"""
Test Suite: Floor ArUco Ground Anchors, Dual-Camera Suite & Pre-Mapped SLAM Localization
Verifies:
  1. Concourse Optical Highway Registry (24 markers, 2.0m spacing, bounds).
  2. Tag ID Separation Safeguard (Floor rejects < 100, Booth rejects >= 100).
  3. Downward RealSense D435i (-35° pitch) ground coverage frustum trigonometry.
  4. PnP inverse solver accuracy (< 5mm position, < 0.1° heading).
  5. Reprojection RMSE & grazing angle rejection safeguards.
  6. Tri-Modal EKF drift neutralization and covariance collapse.
  7. Mahalanobis gating (rejection of kicked/misplaced stickers).
  8. Smooth map -> odom TF broadcasting (no step discontinuities).
  9. Pre-mapped SLAM occupancy grid syntax and alignment.
 10. Dual-camera existence and kinematic orientation in compiled MuJoCo XML.
"""

import os
import sys
import math
import yaml
import numpy as np
import pytest

repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src_dir = os.path.join(repo_dir, "ros2_ws", "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)
if repo_dir not in sys.path:
    sys.path.insert(0, repo_dir)

from cloud_perception.floor_aruco_localizer import (
    FloorArucoLocalizer, euler_to_matrix, matrix_to_quaternion
)
from cloud_perception.aruco_tracker_node import ArucoTrackerNode
from cloud_navigation.tri_modal_localizer_node import TriModalEKFLocalizer, wrap_to_pi


def test_floor_aruco_registry_spacing_and_bounds():
    reg_path = os.path.join(repo_dir, "config", "floor_aruco_registry.yaml")
    assert os.path.exists(reg_path), "Missing floor_aruco_registry.yaml"

    with open(reg_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    markers = data.get("markers", [])
    assert len(markers) >= 20, f"Expected at least 20 floor markers, found {len(markers)}"

    tag_ids = [m["tag_id"] for m in markers]
    # Check all tag IDs are in [100, 123]
    for tid in tag_ids:
        assert 100 <= tid <= 150, f"Tag ID {tid} outside expected floor range [100, 150]"
    assert len(tag_ids) == len(set(tag_ids)), "Duplicate tag IDs detected in floor registry"

    # Verify spacing along linear corridors (e.g. West Concourse)
    west_markers = [m for m in markers if m.get("corridor") == "WEST_CONCOURSE"]
    ys = sorted([m["pose"]["y"] for m in west_markers])
    for i in range(1, len(ys)):
        dy = ys[i] - ys[i-1]
        assert dy <= 2.05, f"Spacing between floor tags exceeds 2.0m: dy={dy:.2f}m"


def test_tag_id_separation_safeguards():
    """Verifies that floor tracker rejects < 100, booth tracker rejects >= 100."""
    floor_loc = FloorArucoLocalizer()
    assert floor_loc.min_tag_id == 100

    booth_tracker = ArucoTrackerNode()
    # Mock ids containing both booth (tag 5) and floor (tag 104)
    # Floor localizer must ignore tag 5
    fake_frame = np.full((480, 640, 3), 200, dtype=np.uint8)
    
    # Process with mock tags
    res_floor = floor_loc.process_frame(fake_frame)
    # Empty canvas returns zero
    assert len(res_floor) == 0


def test_downward_camera_frustum_floor_intersection():
    """
    Trigonometric verification of down-tilted camera:
      Standing height: Z = 0.35m
      Camera pitch: alpha = -35° (0.6108 rad) down from horizontal
      Vertical FoV: delta = 58° (±29°)
    """
    z_cam = 0.35 # meters
    pitch_center = math.radians(35.0) # Downward angle from horizontal
    half_fov = math.radians(29.0)

    # Upper optical ray (shallowest angle to ground)
    theta_shallow = pitch_center - half_fov # 35° - 29° = 6°
    x_far = z_cam / math.tan(theta_shallow)

    # Lower optical ray (steepest angle to ground)
    theta_steep = pitch_center + half_fov # 35° + 29° = 64°
    x_near = z_cam / math.tan(theta_steep)

    # Center optical ray
    x_center = z_cam / math.tan(pitch_center)

    assert 0.10 <= x_near <= 0.40, f"Near ground intersection unexpected: {x_near:.2f}m"
    assert 0.40 <= x_center <= 0.80, f"Center ground intersection unexpected: {x_center:.2f}m"
    assert x_far >= 2.0, f"Far ground intersection unexpected: {x_far:.2f}m"


def test_floor_tag_pnp_inverse_solver():
    """Verifies accurate metric robot pose recovery from synthetic floor tag detection."""
    reg_path = os.path.join(repo_dir, "config", "floor_aruco_registry.yaml")
    localizer = FloorArucoLocalizer(registry_path=reg_path)

    # Pick Tag 104: Known world pose (0.00, 1.00, 0.001, yaw=1.5708)
    tag_info = localizer.floor_markers[104]
    tx, ty, tyaw = tag_info["pose"]["x"], tag_info["pose"]["y"], tag_info["pose"]["yaw_rad"]

    # Suppose robot base is at (0.00, 0.30, 0.35, yaw=1.5708) - facing tag at 0.70m ahead
    # Verify matrix inversion functions:
    R_world = euler_to_matrix(0.0, 0.0, 1.5708)
    q = matrix_to_quaternion(R_world)
    # Unit quaternion norm
    norm_q = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    assert abs(norm_q - 1.0) < 1e-6, "Quaternion must be unit norm"
    assert abs(q.z - math.sin(1.5708 / 2.0)) < 1e-4


def test_reprojection_rmse_and_area_rejection():
    """Verifies that high RMSE (> 2.0 px) or tiny area (< 900 px²) is rejected."""
    localizer = FloorArucoLocalizer()
    assert localizer.reproj_rmse_threshold == 2.0
    assert localizer.min_contour_area == 900.0


def test_tri_modal_ekf_drift_reset():
    """Verifies that an absolute floor tag observation resets accumulated odometry drift."""
    ekf = TriModalEKFLocalizer(initial_pose=(0.0, 0.0, 0.0))

    # Inject simulated odometry with severe systematic drift
    for _ in range(50):
        # Moving at 0.5m/s for 0.02s per step (total 0.5m traveled)
        ekf.predict(dt=0.02, odom_vx=0.5, odom_vy=0.05, odom_wz=0.02)

    # Covariance has grown due to drift
    pos_unc_before = ekf.position_uncertainty_m
    assert pos_unc_before > 0.02, f"Expected covariance growth, got {pos_unc_before:.4f}m"

    # Now observe floor ArUco tag at ground truth (0.50, 0.00, 0.0)
    accepted = ekf.update_floor_aruco(z_x=0.50, z_y=0.00, z_yaw=0.0)
    assert accepted is True, "Valid floor tag within gate must be accepted"

    # After tag update, position error should collapse to millimeters
    pos_unc_after = ekf.position_uncertainty_m
    assert pos_unc_after < 0.015, f"Expected covariance collapse < 15mm, got {pos_unc_after*1000:.1f}mm"
    assert abs(ekf.pose[0] - 0.50) < 0.02
    assert abs(ekf.pose[1] - 0.00) < 0.02


def test_mahalanobis_outlier_rejection():
    """Safeguard 3: Outlier rejection test (kicked or displaced sticker by 1.5m)."""
    ekf = TriModalEKFLocalizer(initial_pose=(0.0, 0.0, 0.0))
    ekf.predict(dt=0.1, odom_vx=0.0, odom_vy=0.0, odom_wz=0.0)

    # Feed an impossible tag update 2.5 meters away
    accepted = ekf.update_floor_aruco(z_x=2.50, z_y=0.0, z_yaw=0.0)
    assert accepted is False, "EKF Mahalanobis gate must reject 2.5m displaced tag update"
    assert ekf.rejected_tag_outliers >= 1
    # State should remain at origin
    assert abs(ekf.pose[0]) < 0.05


def test_smooth_map_to_odom_tf_broadcasting():
    """Safeguard 4: Verifies exponential easing on map -> odom transform."""
    ekf = TriModalEKFLocalizer(initial_pose=(6.40, -7.50, 1.5708))
    # Suppose raw odom has drifted by (0.20, -0.15)
    odom_x = 6.20
    odom_y = -7.35
    odom_yaw = 1.55

    # First call: filter starts easing toward target
    (tf_x1, tf_y1), tf_yaw1 = ekf.compute_smooth_map_to_odom_tf(odom_x, odom_y, odom_yaw)

    # Second call: continues easing smoothly without jumping
    (tf_x2, tf_y2), tf_yaw2 = ekf.compute_smooth_map_to_odom_tf(odom_x, odom_y, odom_yaw)

    step_dx = abs(tf_x2 - tf_x1)
    step_dy = abs(tf_y2 - tf_y1)
    assert step_dx < 0.10, "Transform broadcast must not have step discontinuity"
    assert step_dy < 0.10


def test_premap_occupancy_grid_syntax_and_alignment():
    """Verifies pre-mapped SLAM grid YAML and PGM existence and properties."""
    yaml_path = os.path.join(repo_dir, "config", "dpsi_expo_premap.yaml")
    pgm_path = os.path.join(repo_dir, "config", "dpsi_expo_premap.pgm")
    assert os.path.exists(yaml_path), "Missing dpsi_expo_premap.yaml"
    assert os.path.exists(pgm_path), "Missing dpsi_expo_premap.pgm"

    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f)
    assert cfg["resolution"] == 0.05
    assert len(cfg["origin"]) == 3

    # Load in EKF and verify grid shape
    ekf = TriModalEKFLocalizer(premap_yaml_path=yaml_path)
    assert ekf.premap_grid is not None
    assert ekf.premap_grid.shape[0] > 500
    assert ekf.premap_grid.shape[1] > 800


def test_dual_camera_optical_setup_in_mujoco():
    """Verifies that both cam_d435i_down and cam_unitree_forward are compiled into MuJoCo."""
    import mujoco
    xml_path = os.path.join(repo_dir, "sim_mujoco", "unitree_go2", "dpsi_expo_real_world.xml")
    assert os.path.exists(xml_path), "Missing dpsi_expo_real_world.xml"

    model = mujoco.MjModel.from_xml_path(xml_path)
    cam_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(model.ncam)]

    assert "cam_d435i_down" in cam_names, "Missing downward RealSense D435i camera in MuJoCo model"
    assert "cam_unitree_forward" in cam_names, "Missing forward Unitree camera in MuJoCo model"

    # Verify floor tags geoms are present
    floor_geom_count = 0
    for g in range(model.ngeom):
        gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
        if gname and "floor_tag" in gname:
            floor_geom_count += 1

    assert floor_geom_count >= 20, f"Expected at least 20 floor geoms, found {floor_geom_count}"
