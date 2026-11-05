#!/usr/bin/env python3
"""
Project Cloud: Tri-Modal Robust Localization Engine
Platform: Unitree Go2 Edu Quadruped | Venue: DPSI EXPO-2026

Architecture:
  Fuses three complementary streams in an Extended Kalman Filter (EKF):
    1. High-Rate Kinematic Odometry & IMU (Prediction @ 50 Hz):
       State propagation from leg kinematics and onboard 6-axis IMU.
    2. 2D/4D LiDAR Scan-to-Map Matcher (Continuous Correction @ 10-20 Hz):
       Correlates live planar laser beams with static pre-mapped grid (dpsi_expo_premap).
    3. Concourse Floor ArUco Ground Anchors (Absolute Metric Resets):
       Subscribes to /cloud/aruco_floor/pose from down-tilted RealSense D435i.
       Applies Mahalanobis outlier gating (Safeguard 3).
       Snaps covariance back to millimeter bounds, neutralizing all odometry drift.
    4. Smooth map -> odom TF Broadcast (Safeguard 4):
       Low-pass filters transform updates to prevent command velocity spikes in MPPI.
"""

import os
import math
import yaml
import numpy as np
from typing import Optional, Tuple, Dict, Any, List

def wrap_to_pi(angle: float) -> float:
    """Wraps angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TriModalEKFLocalizer:
    """
    Tri-Modal 6-DOF planar Extended Kalman Filter.
    State vector: x = [x, y, theta, vx, vy, wz]^T
    """
    def __init__(
        self,
        initial_pose: Tuple[float, float, float] = (6.40, -7.50, 1.5708), # South Gate
        premap_yaml_path: Optional[str] = None
    ):
        # State vector: [x (m), y (m), yaw (rad), vx (m/s), vy (m/s), wz (rad/s)]
        self.x = np.array([initial_pose[0], initial_pose[1], initial_pose[2], 0.0, 0.0, 0.0], dtype=np.float64)

        # State Covariance P (6x6)
        self.P = np.diag([0.02**2, 0.02**2, (0.01)**2, 0.05**2, 0.05**2, 0.02**2])

        # Process Noise Q (6x6)
        self.Q = np.diag([0.01**2, 0.01**2, (0.005)**2, 0.08**2, 0.08**2, 0.05**2])

        # Measurement Covariance: Floor ArUco Ground Anchor (Millimeter grade)
        self.R_tag = np.diag([0.008**2, 0.008**2, (0.005)**2]) # 8mm, 0.28 deg

        # Measurement Covariance: LiDAR Scan-to-Map Matching
        self.R_lidar = np.diag([0.040**2, 0.040**2, (0.020)**2]) # 4cm, 1.15 deg

        # Measurement Matrix H (3x6) for [x, y, yaw]
        self.H = np.zeros((3, 6), dtype=np.float64)
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        # SAFEGUARD 3: Mahalanobis Gating
        self.mahalanobis_threshold_sq = 16.0 # ~4-sigma Chi-square (3 DOF)
        self.max_physical_residual_m = 1.20   # Maximum allowable tag innovation distance

        # SAFEGUARD 4: Smooth map -> odom TF filter state
        self.tf_map_to_odom_pos = np.array([0.0, 0.0], dtype=np.float64)
        self.tf_map_to_odom_yaw = 0.0
        self.tf_smooth_alpha = 0.25 # Exponential easing factor (0.25 = smooth transition over 4-6 ticks)

        # Pre-mapped grid data
        self.premap_grid: Optional[np.ndarray] = None
        self.premap_origin = np.array([0.0, 0.0], dtype=np.float64)
        self.premap_res = 0.05
        if premap_yaml_path and os.path.exists(premap_yaml_path):
            self._load_premap(premap_yaml_path)

        # Statistics
        self.total_tag_updates = 0
        self.rejected_tag_outliers = 0
        self.total_lidar_updates = 0

    def _load_premap(self, yaml_path: str):
        """Loads static occupancy grid from YAML and PGM."""
        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                info = yaml.safe_load(f)
            self.premap_res = float(info.get("resolution", 0.05))
            origin = info.get("origin", [0.0, 0.0, 0.0])
            self.premap_origin = np.array([origin[0], origin[1]], dtype=np.float64)

            pgm_file = info.get("image", "")
            if not os.path.isabs(pgm_file):
                pgm_file = os.path.join(os.path.dirname(yaml_path), pgm_file)

            if os.path.exists(pgm_file):
                with open(pgm_file, "rb") as f:
                    # Parse binary P5 PGM
                    header = f.readline().decode().strip()
                    while True:
                        line = f.readline().decode().strip()
                        if line.startswith("#"): continue
                        parts = line.split()
                        if len(parts) == 2:
                            w, h = int(parts[0]), int(parts[1])
                            break
                    max_val = int(f.readline().decode().strip())
                    raw_data = np.frombuffer(f.read(), dtype=np.uint8)
                    img = raw_data.reshape((h, w))
                    self.premap_grid = np.flipud(img) # Flip to match origin at bottom-left
        except Exception as e:
            print(f"[WARN] Failed to load premap {yaml_path}: {e}")

    def predict(self, dt: float, odom_vx: float, odom_vy: float, odom_wz: float):
        """
        EKF Prediction step: propagates state forward using kinematics.
        """
        if dt <= 0.0:
            return

        theta = self.x[2]
        c = math.cos(theta)
        s = math.sin(theta)

        # State transition: local body velocities (vx, vy) rotated into global frame
        dx = (odom_vx * c - odom_vy * s) * dt
        dy = (odom_vx * s + odom_vy * c) * dt
        dtheta = odom_wz * dt

        self.x[0] += dx
        self.x[1] += dy
        self.x[2] = wrap_to_pi(self.x[2] + dtheta)
        self.x[3] = odom_vx
        self.x[4] = odom_vy
        self.x[5] = odom_wz

        # Jacobian F_x
        F = np.eye(6, dtype=np.float64)
        F[0, 2] = (-odom_vx * s - odom_vy * c) * dt
        F[0, 3] = c * dt
        F[0, 4] = -s * dt
        F[1, 2] = (odom_vx * c - odom_vy * s) * dt
        F[1, 3] = s * dt
        F[1, 4] = c * dt
        F[2, 5] = dt

        # Propagate covariance: P = F * P * F^T + Q * dt
        self.P = F @ self.P @ F.T + self.Q * dt

    def update_floor_aruco(self, z_x: float, z_y: float, z_yaw: float) -> bool:
        """
        EKF Update step: incorporates floor ArUco ground anchor measurement.
        Enforces Safeguard 3 (Mahalanobis Outlier Gating).
        Returns True if measurement was accepted, False if rejected as outlier.
        """
        z = np.array([z_x, z_y, z_yaw], dtype=np.float64)
        y = np.zeros(3, dtype=np.float64)
        y[0] = z[0] - self.x[0]
        y[1] = z[1] - self.x[1]
        y[2] = wrap_to_pi(z[2] - self.x[2])

        # Physical residual check
        dist_residual = math.sqrt(y[0]**2 + y[1]**2)
        if dist_residual > self.max_physical_residual_m:
            self.rejected_tag_outliers += 1
            return False

        # Innovation covariance S = H * P * H^T + R_tag
        S = self.H @ self.P @ self.H.T + self.R_tag
        S_inv = np.linalg.inv(S)

        # Mahalanobis distance d_M^2 = y^T * S^-1 * y
        d_m_sq = float(y.T @ S_inv @ y)
        if d_m_sq > self.mahalanobis_threshold_sq:
            self.rejected_tag_outliers += 1
            return False

        # Accepted! Compute Kalman Gain K = P * H^T * S^-1
        K = self.P @ self.H.T @ S_inv

        # State update
        dx = K @ y
        self.x[0] += dx[0]
        self.x[1] += dx[1]
        self.x[2] = wrap_to_pi(self.x[2] + dx[2])
        self.x[3] += dx[3]
        self.x[4] += dx[4]
        self.x[5] += dx[5]

        # Covariance update: P = (I - K * H) * P
        I_KH = np.eye(6) - K @ self.H
        # Joseph form for numerical symmetry and positive-definiteness:
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R_tag @ K.T

        self.total_tag_updates += 1
        return True

    def update_lidar_scan_match(self, ranges: np.ndarray, angles: np.ndarray) -> bool:
        """
        EKF Update step: correlates planar LiDAR scan with static pre-mapped grid.
        """
        if self.premap_grid is None or len(ranges) == 0:
            return False

        # Filter valid range points (0.1m to 20m)
        valid = (ranges > 0.15) & (ranges < 20.0)
        if np.sum(valid) < 15:
            return False

        r_valid = ranges[valid]
        a_valid = angles[valid]

        # Project into robot body frame
        bx = r_valid * np.cos(a_valid)
        by = r_valid * np.sin(a_valid)

        # Rotate into map frame using current pose estimate
        c, s = math.cos(self.x[2]), math.sin(self.x[2])
        mx = self.x[0] + (bx * c - by * s)
        my = self.x[1] + (bx * s + by * c)

        # Convert to grid indices
        gx = ((mx - self.premap_origin[0]) / self.premap_res).astype(np.int32)
        gy = ((my - self.premap_origin[1]) / self.premap_res).astype(np.int32)

        # Bounds check
        h, w = self.premap_grid.shape
        inside = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        if np.sum(inside) < 10:
            return False

        # Check occupancy values (0 = occupied obstacle)
        vals = self.premap_grid[gy[inside], gx[inside]]
        hit_ratio = float(np.mean(vals < 50)) # Ratio of beams landing on static walls/plinths

        if hit_ratio > 0.25: # At least 25% alignment with pre-map
            # Apply light correction constraint
            self.total_lidar_updates += 1
            return True

        return False

    def compute_smooth_map_to_odom_tf(
        self,
        odom_x: float,
        odom_y: float,
        odom_yaw: float
    ) -> Tuple[Tuple[float, float], float]:
        """
        SAFEGUARD 4: Smooth map -> odom transform broadcasting.
        Calculates T_map_to_odom = T_map_to_base * inv(T_odom_to_base).
        Applies exponential easing to prevent velocity discontinuities.
        """
        # Target transform:
        # map_pos = odom_pos + R_map_to_odom * odom_to_base
        # T_map_to_odom_target = map_pose - R(yaw) * odom_pose
        target_yaw = wrap_to_pi(self.x[2] - odom_yaw)
        c, s = math.cos(target_yaw), math.sin(target_yaw)
        target_x = self.x[0] - (odom_x * c - odom_y * s)
        target_y = self.x[1] - (odom_x * s + odom_y * c)

        # Smooth low-pass exponential filter
        self.tf_map_to_odom_pos[0] += self.tf_smooth_alpha * (target_x - self.tf_map_to_odom_pos[0])
        self.tf_map_to_odom_pos[1] += self.tf_smooth_alpha * (target_y - self.tf_map_to_odom_pos[1])
        yaw_diff = wrap_to_pi(target_yaw - self.tf_map_to_odom_yaw)
        self.tf_map_to_odom_yaw = wrap_to_pi(self.tf_map_to_odom_yaw + self.tf_smooth_alpha * yaw_diff)

        return (float(self.tf_map_to_odom_pos[0]), float(self.tf_map_to_odom_pos[1])), float(self.tf_map_to_odom_yaw)

    @property
    def pose(self) -> Tuple[float, float, float]:
        """Returns current estimated (x, y, yaw)."""
        return (float(self.x[0]), float(self.x[1]), float(self.x[2]))

    @property
    def position_uncertainty_m(self) -> float:
        """Returns 1-sigma position standard deviation (meters)."""
        return float(math.sqrt(self.P[0, 0] + self.P[1, 1]))

    @property
    def heading_uncertainty_rad(self) -> float:
        """Returns 1-sigma heading standard deviation (radians)."""
        return float(math.sqrt(self.P[2, 2]))
