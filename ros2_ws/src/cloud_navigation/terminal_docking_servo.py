#!/usr/bin/env python3
"""
Project Cloud: Closed-Loop Terminal Precision Standoff Docking Subsystem
Platform: Unitree Go2 Edu Quadruped | Venue: DPSI EXPO-2026 (40+ Stalls & Dock)

Combines:
  1. Forward-facing Intel RealSense D435i optical AprilTag tag36h11 tracking (IDs 10-49, 99).
  2. Forward Unitree 4D LiDAR L1 planar normal extraction via RANSAC 3D plane fitting.
  3. Closed-loop 3-DOF holonomic PD velocity control on /cmd_vel_safe to achieve:
     - 0.95m +/- 0.03m terminal standoff (0.50m +/- 0.02m for stage dock 99).
     - Centered lateral alignment (+/- 0.03m).
     - Heading perpendicular to booth banner (+/- 0.03 rad / 1.7 deg).
  4. Automatic stance lock (BALANCE_STAND) and eye-contact presentation tilt
     (pitch = -0.14 rad / +8.0 deg) upon sustained 10-cycle convergence.
  5. Slip gating via foot force telemetry to prevent skidding on slick floors.
"""

import os
import sys
import time
import math
from typing import Optional, Tuple, Dict, Any, List
from enum import Enum
import numpy as np

# Optional ROS 2 Jazzy imports with graceful standalone fallback
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, Vector3, PoseStamped, Point, Quaternion
    from std_msgs.msg import String, Int32, Bool, Float32MultiArray, Header
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    class Node:
        def __init__(self, name: str):
            self._name = name
        def get_logger(self):
            import logging
            return logging.getLogger(self._name)
        def create_subscription(self, *args, **kwargs):
            return None
        def create_publisher(self, *args, **kwargs):
            return None
        def create_timer(self, *args, **kwargs):
            return None


class DockingState(Enum):
    IDLE = "IDLE"
    ACQUIRING = "ACQUIRING"
    SERVOING = "SERVOING"
    SETTLING = "SETTLING"
    PRESENTATION_LOCKED = "PRESENTATION_LOCKED"
    ABORTED = "ABORTED"


class RansacPlaneFitter:
    """
    Robust 3D RANSAC plane estimator for planar surface normal extraction
    and orthogonal standoff distance calculation.
    """
    def __init__(self, distance_threshold: float = 0.02, max_iterations: int = 80, min_inliers: int = 20):
        self.distance_threshold = distance_threshold
        self.max_iterations = max_iterations
        self.min_inliers = min_inliers

    def fit(self, points: np.ndarray) -> Optional[Tuple[np.ndarray, float, np.ndarray, int]]:
        """
        Fits plane a*x + b*y + c*z + d = 0 to points (N x 3).
        Returns:
            (normal, d_perp, plane_coeffs, inlier_count) or None if insufficient points.
            normal: unit vector [a, b, c] pointing outward from the plane toward sensor.
            d_perp: perpendicular distance from origin to plane.
            plane_coeffs: [a, b, c, d] where a^2+b^2+c^2 = 1.
        """
        if points is None or len(points) < self.min_inliers:
            return None

        n_pts = points.shape[0]
        best_inliers = 0
        best_coeffs = None

        # Seeded deterministic iterations for reproducible control cycles
        rng = np.random.RandomState(42)

        for _ in range(self.max_iterations):
            idx = rng.choice(n_pts, 3, replace=False)
            p1, p2, p3 = points[idx]

            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal = normal / norm

            d = -np.dot(normal, p1)

            # Distance of all points to plane: |ax + by + cz + d|
            distances = np.abs(np.dot(points, normal) + d)
            inliers = np.count_nonzero(distances <= self.distance_threshold)

            if inliers > best_inliers:
                best_inliers = inliers
                best_coeffs = np.array([normal[0], normal[1], normal[2], d])

        if best_coeffs is None or best_inliers < self.min_inliers:
            return None

        # Refine plane using all inliers via SVD
        distances = np.abs(np.dot(points, best_coeffs[:3]) + best_coeffs[3])
        inlier_mask = distances <= self.distance_threshold
        inlier_points = points[inlier_mask]

        centroid = np.mean(inlier_points, axis=0)
        centered = inlier_points - centroid
        _, _, vh = np.linalg.svd(centered)
        refined_normal = vh[2, :]
        refined_normal = refined_normal / np.linalg.norm(refined_normal)
        refined_d = -np.dot(refined_normal, centroid)

        # Ensure normal vector points toward the sensor origin (0, 0, 0)
        # If normal points away (dot with -centroid > 0), flip normal
        if np.dot(refined_normal, centroid) > 0:
            refined_normal = -refined_normal
            refined_d = -refined_d

        d_perp = abs(refined_d)
        plane_coeffs = np.array([refined_normal[0], refined_normal[1], refined_normal[2], refined_d])
        return refined_normal, d_perp, plane_coeffs, int(best_inliers)


class TerminalDockingServoNode(Node):
    """
    ROS 2 Closed-Loop Terminal Precision Standoff Docking Subsystem.
    """
    def __init__(self, node_name: str = "terminal_docking_servo"):
        super().__init__(node_name)

        # Control Parameters
        self.default_standoff_m = 0.95
        self.stage_dock_standoff_m = 0.50
        self.presentation_pitch_rad = -0.14 # +8.0 deg nose-up eye contact

        # PD Controller Gains
        self.kp_x = 0.85
        self.kd_x = 0.12
        self.kp_y = 0.90
        self.kd_y = 0.15
        self.kp_th = 1.20
        self.kd_th = 0.20

        # Saturation & Slew Envelopes
        self.max_vx_fwd = 0.22
        self.max_vx_rev = -0.18
        self.max_vy = 0.15
        self.max_wz = 0.35
        self.max_accel_x = 0.40 # m/s^2 (traction control on terrazzo)
        self.max_accel_y = 0.35
        self.max_accel_wz = 0.80 # rad/s^2

        # Convergence criteria
        self.tol_x = 0.03       # +/- 3 cm longitudinal standoff
        self.tol_y = 0.03       # +/- 3 cm lateral center
        self.tol_th = 0.03      # +/- 1.7 deg yaw heading
        self.required_settled_cycles = 10 # 10 cycles @ 20 Hz = 0.5 sec

        # State Variables
        self.state = DockingState.IDLE
        self.target_id: Optional[int] = None
        self.target_standoff_m: float = self.default_standoff_m
        self.target_label: str = ""

        self.last_ex: float = 0.0
        self.last_ey: float = 0.0
        self.last_eth: float = 0.0
        self.last_cmd_vx: float = 0.0
        self.last_cmd_vy: float = 0.0
        self.last_cmd_wz: float = 0.0
        self.last_update_time: float = time.time()
        self.last_tag_seen_time: float = 0.0
        self.settled_counter: int = 0

        # Sensor readings cache
        self.current_tag_id: Optional[int] = None
        self.current_tag_pose: Optional[Dict[str, float]] = None # x, y, z in camera optical frame
        self.latest_cloud_points: Optional[np.ndarray] = None
        self.latest_foot_forces: List[float] = [18.0, 18.0, 18.0, 18.0]

        self.plane_fitter = RansacPlaneFitter(distance_threshold=0.02, max_iterations=60, min_inliers=20)

        # Publishers
        if HAS_ROS2:
            self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel_safe", 10)
            self.pub_posture = self.create_publisher(String, "/go2_posture_cmd", 10)
            self.pub_euler = self.create_publisher(Vector3, "/go2_body_euler", 10)
            self.pub_arrived = self.create_publisher(Bool, "/cloud/docking/arrived", 10)
            self.pub_status = self.create_publisher(String, "/cloud/docking/status", 10)

            # Subscriptions
            self.sub_tag_pose = self.create_subscription(
                PoseStamped, "/camera/apriltag_pose", self._on_tag_pose, 10
            )
            self.sub_tag_id = self.create_subscription(
                Int32, "/camera/apriltag_id", self._on_tag_id, 10
            )
            self.sub_cloud = self.create_subscription(
                PointCloud2, "/cloud/lidar/pointcloud2", self._on_point_cloud, 10
            )
            self.sub_foot_forces = self.create_subscription(
                Float32MultiArray, "/cloud/telemetry/foot_forces", self._on_foot_forces, 10
            )
            self.sub_target_stall = self.create_subscription(
                String, "/cloud/docking/target_stall", self._on_target_stall_cmd, 10
            )

            # Control loop timer (20 Hz)
            self.timer = self.create_timer(0.05, self.control_step)

    # --------------------------------------------------------------------------
    # Public Mission & Target Commands
    # --------------------------------------------------------------------------
    def start_docking(self, target_apriltag_id: int, standoff_m: Optional[float] = None, label: str = ""):
        """
        Arms terminal precision standoff docking for a specific AprilTag ID.
        """
        self.target_id = target_apriltag_id
        self.target_label = label or f"Marker_{target_apriltag_id}"

        if standoff_m is not None:
            self.target_standoff_m = standoff_m
        elif target_apriltag_id == 99:
            self.target_standoff_m = self.stage_dock_standoff_m
        else:
            self.target_standoff_m = self.default_standoff_m

        self.settled_counter = 0
        self.last_ex = 0.0
        self.last_ey = 0.0
        self.last_eth = 0.0
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_wz = 0.0
        self.last_update_time = time.time()
        self.state = DockingState.ACQUIRING

        if HAS_ROS2:
            self.get_logger().info(
                f"[TerminalDockingServo] Armed for {self.target_label} (Tag {self.target_id}) "
                f"at {self.target_standoff_m:.2f}m standoff."
            )
            self._publish_status(f"ACQUIRING_TAG_{self.target_id}")

    def abort(self, reason: str = "Commanded abort"):
        """Aborts active docking and stops robot motion."""
        self.state = DockingState.ABORTED
        self._publish_cmd_vel(0.0, 0.0, 0.0)
        if HAS_ROS2:
            self.get_logger().warn(f"[TerminalDockingServo] Docking aborted: {reason}")
            self._publish_status(f"ABORTED: {reason}")

    # --------------------------------------------------------------------------
    # Sensor Callbacks & Data Parsing
    # --------------------------------------------------------------------------
    def _on_target_stall_cmd(self, msg: String):
        """Allows activating docking via stall ID string (e.g. 'STALL_01' or '99')."""
        cmd_str = msg.data.strip().upper()
        if cmd_str.startswith("STALL_"):
            try:
                num = int(cmd_str.split("_")[1])
                tag_id = 9 + num # Stalls 01-40 -> Tags 10-49
                self.start_docking(tag_id, standoff_m=self.default_standoff_m, label=cmd_str)
            except Exception as e:
                if HAS_ROS2:
                    self.get_logger().error(f"Failed parsing stall command {cmd_str}: {e}")
        elif cmd_str in ("DOCK_99", "STAGE_DOCK", "99"):
            self.start_docking(99, standoff_m=self.stage_dock_standoff_m, label="STAGE_DOCK_99")

    def _on_tag_id(self, msg: Int32):
        self.current_tag_id = int(msg.data)

    def _on_tag_pose(self, msg: PoseStamped):
        """Caches camera optical frame coordinates: X=right, Y=down, Z=forward."""
        self.last_tag_seen_time = time.time()
        self.current_tag_pose = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "qx": msg.pose.orientation.x,
            "qy": msg.pose.orientation.y,
            "qz": msg.pose.orientation.z,
            "qw": msg.pose.orientation.w,
        }

    def _on_foot_forces(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.latest_foot_forces = list(msg.data[:4])

    def _on_point_cloud(self, msg: PointCloud2):
        """Converts ROS 2 PointCloud2 to Nx3 numpy array (XYZ)."""
        if not HAS_ROS2:
            return
        try:
            pts_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            pts_list = [p for p in pts_gen]
            if pts_list:
                self.latest_cloud_points = np.array(pts_list, dtype=np.float32)
        except Exception:
            pass

    # --------------------------------------------------------------------------
    # Multi-Modal Target State Estimation
    # --------------------------------------------------------------------------
    def extract_frustum_points(
        self,
        cloud: np.ndarray,
        center_x: float,
        center_y: float,
        center_z: float,
        half_x: float = 0.35,
        half_y: float = 0.35,
        half_z: float = 0.40,
    ) -> np.ndarray:
        """
        Extracts 3D LiDAR points in a bounding box centered at the optical tag position.
        In camera optical frame: X is lateral, Y is vertical, Z is forward depth.
        In base_link frame: X is forward depth, Y is lateral, Z is vertical.
        Handles both optical and base coordinate conventions cleanly.
        """
        if cloud is None or len(cloud) == 0:
            return np.empty((0, 3))

        mask = (
            (np.abs(cloud[:, 0] - center_x) <= half_x) &
            (np.abs(cloud[:, 1] - center_y) <= half_y) &
            (np.abs(cloud[:, 2] - center_z) <= half_z)
        )
        return cloud[mask]

    def estimate_relative_booth_pose(self) -> Optional[Tuple[float, float, float, str]]:
        """
        Fuses AprilTag optical observation with 4D LiDAR planar backboard fit.
        Returns:
            (d_forward, y_lateral, yaw_misalign_rad, mode_str) or None if target unavailable.
            - d_forward: measured distance along robot forward axis (meters).
            - y_lateral: lateral offset relative to booth center (meters, positive = robot is left of booth).
            - yaw_misalign_rad: robot yaw relative to booth banner normal (rad, positive = robot turned left).
        """
        if self.target_id is None:
            return None

        # Check tag visibility and ID match
        if self.current_tag_id != self.target_id or self.current_tag_pose is None:
            return None

        tag_x = self.current_tag_pose["x"]
        tag_y = self.current_tag_pose["y"]
        tag_z = self.current_tag_pose["z"]

        # Optical default
        d_optical = tag_z
        # In camera optical frame: +X is right. In robot base_link: +Y is left.
        # Robot lateral offset relative to marker: if marker is to the right (+X_cam),
        # the robot is to the left (-Y_lat relative to marker center).
        y_optical = -tag_x
        yaw_optical = math.atan2(tag_x, max(0.01, tag_z))

        # Check for LiDAR planar refinement
        lidar_success = False
        d_fused = d_optical
        yaw_fused = yaw_optical
        mode = "OPTICAL_ONLY"

        if self.latest_cloud_points is not None and len(self.latest_cloud_points) >= 25:
            # Crop points around optical marker in camera/sensor frame
            frustum_pts = self.extract_frustum_points(self.latest_cloud_points, tag_x, tag_y, tag_z)
            if len(frustum_pts) >= 20:
                fit_res = self.plane_fitter.fit(frustum_pts)
                if fit_res is not None:
                    normal, d_perp, _, inliers = fit_res
                    # In camera optical frame: normal of vertical backboard points back toward camera [0, 0, -1]
                    # Check that plane is roughly vertical (abs(normal_y) < 0.35)
                    if abs(normal[1]) < 0.35:
                        d_fused = d_perp
                        # Heading from normal: normal_x / normal_z
                        yaw_fused = math.atan2(normal[0], -normal[2])
                        lidar_success = True
                        mode = f"LIDAR_RANSAC_FUSED({inliers}pts)"

        return d_fused, y_optical, yaw_fused, mode

    # --------------------------------------------------------------------------
    # Deterministic Closed-Loop Control Law (20 Hz)
    # --------------------------------------------------------------------------
    def control_step(self) -> Dict[str, Any]:
        """
        Executes one control cycle. Returns telemetry dictionary.
        """
        now = time.time()
        dt = max(0.01, min(0.20, now - self.last_update_time))
        self.last_update_time = now

        result = {
            "state": self.state.value,
            "target_id": self.target_id,
            "vx": 0.0,
            "vy": 0.0,
            "wz": 0.0,
            "settled_cycles": self.settled_counter,
            "ex": 0.0,
            "ey": 0.0,
            "eth": 0.0,
            "mode": "IDLE"
        }

        if self.state in (DockingState.IDLE, DockingState.PRESENTATION_LOCKED, DockingState.ABORTED):
            self.latest_telemetry = result
            return result

        # Check occlusion timeout
        if (now - self.last_tag_seen_time) > 3.0 and self.last_tag_seen_time > 0:
            self.abort("AprilTag tracking lost for > 3.0s")
            result["state"] = self.state.value
            self.latest_telemetry = result
            return result

        estimate = self.estimate_relative_booth_pose()
        if estimate is None:
            # Target not acquired yet in camera FOV
            self.state = DockingState.ACQUIRING
            self._publish_cmd_vel(0.0, 0.0, 0.0)
            result["state"] = "ACQUIRING"
            self.latest_telemetry = result
            return result

        d_meas, y_meas, th_meas, mode = estimate
        result["mode"] = mode

        # Transition to SERVOING
        if self.state == DockingState.ACQUIRING:
            self.state = DockingState.SERVOING
            if HAS_ROS2:
                self.get_logger().info(f"[TerminalDockingServo] Target locked via {mode}. Servoing active.")

        # Compute Errors
        e_x = d_meas - self.target_standoff_m   # Longitudinal error (+ = robot too far away)
        e_y = y_meas                           # Lateral error (+ = robot too far left)
        e_th = th_meas                         # Yaw error (+ = robot angled left)

        de_x = (e_x - self.last_ex) / dt
        de_y = (e_y - self.last_ey) / dt
        de_th = (e_th - self.last_eth) / dt

        self.last_ex = e_x
        self.last_ey = e_y
        self.last_eth = e_th

        result["ex"] = round(float(e_x), 4)
        result["ey"] = round(float(e_y), 4)
        result["eth"] = round(float(e_th), 4)

        # Holonomic PD Control Law
        raw_vx = self.kp_x * e_x + self.kd_x * de_x
        raw_vy = self.kp_y * e_y + self.kd_y * de_y
        raw_wz = -(self.kp_th * e_th + self.kd_th * de_th) # counteract yaw misalignment

        # Clamp raw velocities
        target_vx = max(self.max_vx_rev, min(self.max_vx_fwd, raw_vx))
        target_vy = max(-self.max_vy, min(self.max_vy, raw_vy))
        target_wz = max(-self.max_wz, min(self.max_wz, raw_wz))

        # Slip Gating: In quadruped locomotion, 2 legs are in swing (F_z ~ 0N).
        # Check total ground reaction force. If total normal force drops below 35 N, scale target.
        total_fz = sum(self.latest_foot_forces) if self.latest_foot_forces else 60.0
        if total_fz < 35.0:
            slip_scale = max(0.2, total_fz / 60.0)
            target_vx *= slip_scale
            target_vy *= slip_scale
            target_wz *= slip_scale

        # Apply Slew / Acceleration Limits toward target
        max_dvx = self.max_accel_x * dt
        max_dvy = self.max_accel_y * dt
        max_dwz = self.max_accel_wz * dt

        cmd_vx = max(self.last_cmd_vx - max_dvx, min(self.last_cmd_vx + max_dvx, target_vx))
        cmd_vy = max(self.last_cmd_vy - max_dvy, min(self.last_cmd_vy + max_dvy, target_vy))
        cmd_wz = max(self.last_cmd_wz - max_dwz, min(self.last_cmd_wz + max_dwz, target_wz))

        self.last_cmd_vx = cmd_vx
        self.last_cmd_vy = cmd_vy
        self.last_cmd_wz = cmd_wz

        # Convergence & Settling Check
        within_tolerance = (
            abs(e_x) <= self.tol_x and
            abs(e_y) <= self.tol_y and
            abs(e_th) <= self.tol_th
        )

        if within_tolerance:
            self.settled_counter += 1
            self.state = DockingState.SETTLING
        else:
            self.settled_counter = max(0, self.settled_counter - 1)
            self.state = DockingState.SERVOING

        result["settled_cycles"] = self.settled_counter

        # Latch Condition: Sustained 10 cycles (0.5s) within +/- 3cm and +/- 1.7 deg
        if self.settled_counter >= self.required_settled_cycles:
            self._execute_presentation_latch(d_meas, y_meas, th_meas)
            result["state"] = DockingState.PRESENTATION_LOCKED.value
            result["vx"] = 0.0
            result["vy"] = 0.0
            result["wz"] = 0.0
            return result

        # Publish motion command
        self._publish_cmd_vel(cmd_vx, cmd_vy, cmd_wz)
        result["vx"] = round(cmd_vx, 3)
        result["vy"] = round(cmd_vy, 3)
        result["wz"] = round(cmd_wz, 3)
        result["state"] = self.state.value
        self.latest_telemetry = result
        return result

    def _execute_presentation_latch(self, final_dist: float, final_lat: float, final_yaw: float):
        """
        Halts robot motion, locks balance posture, tilts chassis +8 deg nose-up,
        and broadcasts docking arrival signal.
        """
        self.state = DockingState.PRESENTATION_LOCKED
        self._publish_cmd_vel(0.0, 0.0, 0.0)

        if HAS_ROS2:
            # 1. Lock stance kinematics
            msg_posture = String(data="BALANCE_STAND")
            self.pub_posture.publish(msg_posture)

            # 2. Command +8.0 deg eye contact nose-up pitch (-0.14 rad)
            msg_euler = Vector3(x=0.0, y=self.presentation_pitch_rad, z=0.0)
            self.pub_euler.publish(msg_euler)

            # 3. Publish arrival signal
            msg_arrived = Bool(data=True)
            self.pub_arrived.publish(msg_arrived)

            self.get_logger().info(
                f"[TerminalDockingServo] ARRIVED & LOCKED at {self.target_label}! "
                f"Dist: {final_dist:.3f}m (target {self.target_standoff_m:.2f}m), "
                f"Offset: {final_lat:.3f}m, Heading: {math.degrees(final_yaw):.1f}deg. "
                f"Posture: BALANCE_STAND, Pitch: +8.0 deg eye contact."
            )
            self._publish_status(f"LOCKED_{self.target_label}")

    # --------------------------------------------------------------------------
    # Publisher Helpers
    # --------------------------------------------------------------------------
    def _publish_cmd_vel(self, vx: float, vy: float, wz: float):
        if HAS_ROS2:
            t = Twist()
            t.linear.x = float(vx)
            t.linear.y = float(vy)
            t.angular.z = float(wz)
            self.pub_cmd_vel.publish(t)

    def _publish_status(self, text: str):
        if HAS_ROS2:
            s = String(data=text)
            self.pub_status.publish(s)


def main(args=None):
    if not HAS_ROS2:
        print("[ERROR] ROS 2 Jazzy is required to run TerminalDockingServoNode executable.")
        sys.exit(1)
    rclpy.init(args=args)
    node = TerminalDockingServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.abort("Keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
