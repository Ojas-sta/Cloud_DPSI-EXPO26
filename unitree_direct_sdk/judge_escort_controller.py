#!/usr/bin/env python3
"""
Project Cloud: 360° LiDAR Judge Escort Companion Pacing & Side-Edge Centering
Incorporating 5 Production Safeguards:
  1. Longitudinal Velocity Slewing (Anti-Jerk Filter: a_accel <= 0.5 m/s², a_decel <= 0.8 m/s²)
  2. Multi-Target Clustering & Evaluator Association (Euclidean clustering + leg filter + EMA + coasting)
  3. Lateral Centering Deadband (±0.10m deadband, max 2.0m corridor clamp, |vy| <= 0.15 m/s)
  4. Flank Switching Hysteresis (1.5-second sustained latch before switching LEFT <-> RIGHT)
  5. Interactive Simulation Hotkeys ([, ], p, \\) for dynamic evaluator testing
"""

import time
import math
import numpy as np
from typing import List, Tuple, Dict, Any, Optional


class JudgeEscortController:
    def __init__(
        self,
        nominal_speed: float = 0.35,
        max_speed: float = 0.55,
        accel_max: float = 0.50,  # m/s^2
        decel_max: float = 0.80,  # m/s^2
        coast_timeout: float = 0.80,  # seconds to coast if track temporarily occluded
        flank_latch_duration: float = 1.50,  # seconds to switch flanks
        deadband_lateral: float = 0.10,  # meters
        max_eval_corridor_dist: float = 2.00,  # meters
        max_lateral_vel: float = 0.15  # m/s
    ):
        self.nominal_speed = nominal_speed
        self.max_speed = max_speed
        self.accel_max = accel_max
        self.decel_max = decel_max
        self.coast_timeout = coast_timeout
        self.flank_latch_duration = flank_latch_duration
        self.deadband_lateral = deadband_lateral
        self.max_eval_corridor_dist = max_eval_corridor_dist
        self.max_lateral_vel = max_lateral_vel

        # Active Locomotion State
        self.current_vx = 0.0
        self.current_vy = 0.0

        # Judge Tracker State
        self.active_flank = "LEFT"  # "LEFT" or "RIGHT"
        self.candidate_flank = "LEFT"
        self.candidate_flank_start_time = time.time()
        
        # Tracked Judge Position in Robot Body Frame (X=Forward, Y=Left)
        self.judge_tracked = True
        self.judge_rel_x = 0.05  # Initial offset: on-par (+5cm)
        self.judge_rel_y = 0.90  # On left flank (0.90m away)
        self.judge_velocity = nominal_speed
        self.judge_is_walking = True
        self.last_detection_time = time.time()
        self.ema_alpha = 0.25  # Exponential moving average filter weight

        # Concourse Clearance
        self.d_left = 1.40
        self.d_right = 1.60

        # Diagnostics & Status Badge
        self.status_badge = "ESCORT: ON PAR"
        self.status_color = (0, 255, 120)  # Green

    # --------------------------------------------------------------------------
    # SAFEGUARD 2: Multi-Target Clustering & Leg Association
    # --------------------------------------------------------------------------
    def cluster_and_filter_legs(self, raw_lidar_points_2d: np.ndarray) -> List[Dict[str, float]]:
        """
        Clusters raw 2D LiDAR returns using Euclidean binning (eps=0.20m, min_samples=4)
        and filters by human leg/torso geometry (width between 0.15m and 0.70m).
        """
        if raw_lidar_points_2d is None or len(raw_lidar_points_2d) < 4:
            return []

        clusters = []
        pts = np.array(raw_lidar_points_2d)
        visited = np.zeros(len(pts), dtype=bool)
        eps = 0.20
        min_pts = 4

        for i in range(len(pts)):
            if visited[i]:
                continue
            queue = [i]
            visited[i] = True
            cluster_indices = []

            while queue:
                curr = queue.pop(0)
                cluster_indices.append(curr)
                diffs = pts - pts[curr]
                dists = np.hypot(diffs[:, 0], diffs[:, 1])
                neighbors = np.where(dists <= eps)[0]

                for n_idx in neighbors:
                    if not visited[n_idx]:
                        visited[n_idx] = True
                        queue.append(n_idx)

            if len(cluster_indices) >= min_pts:
                c_arr = pts[cluster_indices]
                width = float(np.hypot(
                    np.max(c_arr[:, 0]) - np.min(c_arr[:, 0]),
                    np.max(c_arr[:, 1]) - np.min(c_arr[:, 1])
                ))
                # Realistic human leg / dual-leg stance width
                if 0.15 <= width <= 0.70:
                    centroid = np.mean(c_arr, axis=0)
                    clusters.append({
                        "x": float(centroid[0]),
                        "y": float(centroid[1]),
                        "width": float(width),
                        "num_points": len(cluster_indices)
                    })

        return clusters

    def update_sensor_lidar(self, lidar_points_body_frame: Optional[np.ndarray], now: float):
        """Processes real 360° LiDAR cloud and updates judge track."""
        if lidar_points_body_frame is None:
            return

        clusters = self.cluster_and_filter_legs(lidar_points_body_frame)

        left_candidates = [c for c in clusters if 0.55 <= c["y"] <= 1.45 and -1.4 <= c["x"] <= 1.2]
        right_candidates = [c for c in clusters if -1.45 <= c["y"] <= -0.55 and -1.4 <= c["x"] <= 1.2]

        # Determine detection per flank
        best_left = min(left_candidates, key=lambda c: abs(c["x"] - self.judge_rel_x), default=None)
        best_right = min(right_candidates, key=lambda c: abs(c["x"] - self.judge_rel_x), default=None)

        # SAFEGUARD 4: Flank Switching Hysteresis (1.5-second latch)
        if self.active_flank == "LEFT":
            if best_left is not None:
                self._update_track(best_left["x"], best_left["y"], now)
                self.candidate_flank = "LEFT"
            elif best_right is not None:
                if self.candidate_flank != "RIGHT":
                    self.candidate_flank = "RIGHT"
                    self.candidate_flank_start_time = now
                elif now - self.candidate_flank_start_time >= self.flank_latch_duration:
                    self.active_flank = "RIGHT"
                    self._update_track(best_right["x"], best_right["y"], now)
            else:
                self._handle_occlusion(now)
        else:  # RIGHT FLANK ACTIVE
            if best_right is not None:
                self._update_track(best_right["x"], best_right["y"], now)
                self.candidate_flank = "RIGHT"
            elif best_left is not None:
                if self.candidate_flank != "LEFT":
                    self.candidate_flank = "LEFT"
                    self.candidate_flank_start_time = now
                elif now - self.candidate_flank_start_time >= self.flank_latch_duration:
                    self.active_flank = "LEFT"
                    self._update_track(best_left["x"], best_left["y"], now)
            else:
                self._handle_occlusion(now)

    def _update_track(self, meas_x: float, meas_y: float, now: float):
        dt = max(0.01, now - self.last_detection_time)
        # 1D EMA Filtering
        self.judge_rel_x = (1.0 - self.ema_alpha) * self.judge_rel_x + self.ema_alpha * meas_x
        self.judge_rel_y = (1.0 - self.ema_alpha) * self.judge_rel_y + self.ema_alpha * meas_y
        self.last_detection_time = now
        self.judge_tracked = True

    def _handle_occlusion(self, now: float):
        elapsed = now - self.last_detection_time
        if elapsed > self.coast_timeout:
            self.judge_tracked = False

    # --------------------------------------------------------------------------
    # SAFEGUARD 5: Interactive Simulation Hotkeys Hook
    # --------------------------------------------------------------------------
    def adjust_judge_offset(self, delta_x: float):
        """[ / ] hotkeys: adjust relative longitudinal distance."""
        self.judge_rel_x += delta_x
        self.last_detection_time = time.time()
        self.judge_tracked = True
        print(f"  🧑‍⚖️ [Judge Manual Hotkey]: Offset delta={delta_x:+.2f}m -> rel_x={self.judge_rel_x:+.2f}m")

    def toggle_judge_walking(self):
        """'p' hotkey: pause/resume judge walking."""
        self.judge_is_walking = not self.judge_is_walking
        self.last_detection_time = time.time()
        print(f"  🧑‍⚖️ [Judge Manual Hotkey]: Judge walking state = {self.judge_is_walking}")

    def toggle_judge_flank(self):
        """'\\' hotkey: switch active flank between LEFT and RIGHT."""
        self.active_flank = "RIGHT" if self.active_flank == "LEFT" else "LEFT"
        self.judge_rel_y = -0.90 if self.active_flank == "RIGHT" else 0.90
        self.last_detection_time = time.time()
        print(f"  🧑‍⚖️ [Judge Manual Hotkey]: Active flank toggled -> {self.active_flank}")

    def step_simulation(self, dt: float, current_robot_vx: float):
        """Updates simulated judge position in emulation mode."""
        if not self.judge_is_walking:
            # Judge is standing still! Relative offset decreases as robot moves forward
            self.judge_rel_x -= current_robot_vx * dt
        else:
            # Judge is walking at nominal human pace (~0.38 m/s)
            judge_walking_speed = 0.38
            # Relative speed determines dx drift
            rel_speed = judge_walking_speed - current_robot_vx
            self.judge_rel_x += rel_speed * dt
            # Natural human pacing equilibrium: slight soft spring back to 0.0m
            self.judge_rel_x += (0.00 - self.judge_rel_x) * 0.10 * dt

        self.last_detection_time = time.time()
        self.judge_tracked = True

    # --------------------------------------------------------------------------
    # SAFEGUARDS 1 & 3: Longitudinal Slewing & Lateral Deadband Centering
    # --------------------------------------------------------------------------
    def compute_motion_command(
        self,
        desired_nominal_vx: float,
        dt: float,
        min_front_distance: float = 3.0,
        d_left_meas: float = 1.40,
        d_right_meas: float = 1.60
    ) -> Tuple[float, float, Dict[str, Any]]:
        """
        Computes rate-limited longitudinal pacing (vx) and deadbanded centering (vy).
        """
        self.d_left = d_left_meas
        self.d_right = d_right_meas

        # 1. Evaluate Target Forward Velocity from Evaluator Offset
        if not self.judge_tracked:
            # Track lost for > 0.8s: Fall back to gentle autonomous cruise
            target_vx = min(0.28, desired_nominal_vx)
            self.status_badge = "ESCORT: SEARCHING (COASTING)"
            self.status_color = (0, 200, 255)  # Cyan
        else:
            dx = self.judge_rel_x  # Positive = judge ahead; Negative = judge behind

            # SAFEGUARD 1: Continuous cubic/linear ramp approaching cutoff (-0.30m)
            # No hard step discontinuity!
            if dx <= -0.45:
                # Judges are far behind or stopped: full gentle halt
                target_vx = 0.0
                self.status_badge = "ESCORT: WAITING FOR JUDGES (HALTED)"
                self.status_color = (0, 165, 255)  # Amber
            elif dx < -0.15:
                # Approaching stop: smooth cubic blend from nominal down to 0.0
                # Normalized ratio from 0.0 (at -0.45m) to 1.0 (at -0.15m)
                ratio = (dx - (-0.45)) / 0.30
                smooth_factor = ratio * ratio * (3.0 - 2.0 * ratio)  # Smoothstep
                target_vx = desired_nominal_vx * smooth_factor
                self.status_badge = f"ESCORT: SLOWING FOR JUDGES (Δx: {dx:+.2f}m)"
                self.status_color = (0, 215, 255)  # Gold
            elif abs(dx) <= 0.15:
                # Judges on-par within ±15cm: match nominal walking speed
                target_vx = desired_nominal_vx
                self.status_badge = f"ESCORT: ON PAR (Δx: {dx:+.2f}m)"
                self.status_color = (0, 255, 120)  # Green
            else:
                # Judges walking ahead (+0.15m to +0.80m): catch up proportionally
                catchup_factor = min(1.45, 1.0 + (dx - 0.15) * 0.90)
                target_vx = min(self.max_speed, desired_nominal_vx * catchup_factor)
                self.status_badge = f"ESCORT: PACING BRISK (Δx: {dx:+.2f}m)"
                self.status_color = (0, 255, 200)  # Cyan-Green

        # Absolute Frontal Collision Halt Buffer (< 0.60m)
        if min_front_distance <= 0.60:
            target_vx = 0.0
            self.status_badge = "SAFETY: FRONT OBSTACLE STOP"
            self.status_color = (0, 0, 255)  # Red

        # SAFEGUARD 1 (Cont): Apply Anti-Jerk Rate Limiter (Slewing Filter)
        # a_accel <= 0.50 m/s^2, a_decel <= 0.80 m/s^2
        max_delta_up = self.accel_max * dt
        max_delta_down = self.decel_max * dt

        if target_vx > self.current_vx:
            self.current_vx = min(target_vx, self.current_vx + max_delta_up)
        else:
            self.current_vx = max(target_vx, self.current_vx - max_delta_down)

        # ----------------------------------------------------------------------
        # SAFEGUARD 3: Lateral Centering Deadband & Corridor Clamping
        # ----------------------------------------------------------------------
        target_vy = 0.0
        # If open corridor junction, min clearance exceeds 2.0m -> disable centering
        if min(self.d_left, self.d_right) <= self.max_eval_corridor_dist:
            # Clamp effective distances
            d_l_clamped = min(self.max_eval_corridor_dist, self.d_left)
            d_r_clamped = min(self.max_eval_corridor_dist, self.d_right)
            diff = d_l_clamped - d_r_clamped

            # ±0.10m deadband eliminates chatter oscillation
            if abs(diff) > self.deadband_lateral:
                # If too close to left (diff < 0), steer right (vy < 0)
                # If too close to right (diff > 0), steer left (vy > 0)
                centering_gain = 0.25
                target_vy = math.copysign(
                    min(self.max_lateral_vel, (abs(diff) - self.deadband_lateral) * centering_gain),
                    diff
                )

        # Slew rate limit on lateral velocity
        vy_slew = 0.40 * dt
        if target_vy > self.current_vy:
            self.current_vy = min(target_vy, self.current_vy + vy_slew)
        else:
            self.current_vy = max(target_vy, self.current_vy - vy_slew)

        info = {
            "judge_tracked": self.judge_tracked,
            "flank": self.active_flank,
            "judge_rel_x": self.judge_rel_x,
            "judge_rel_y": self.judge_rel_y,
            "vx": self.current_vx,
            "vy": self.current_vy,
            "status_badge": self.status_badge,
            "status_color": self.status_color,
            "d_left": self.d_left,
            "d_right": self.d_right
        }
        return self.current_vx, self.current_vy, info
