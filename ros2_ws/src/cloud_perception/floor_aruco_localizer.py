#!/usr/bin/env python3
"""
Project Cloud: Floor ArUco Optical Ground Highway Localizer Node
Platform: Unitree Go2 Edu Quadruped | Venue: DPSI EXPO-2026

Role:
  Processes down-tilted RealSense D435i camera frames (-35° pitch).
  Detects 180mm floor ArUco ground anchors (IDs 100 to 123).
  Enforces strict safeguards:
    1. Tag ID Separation: Reject any ID < 100 (reserved for booth aprons).
    2. Motion Blur & Grazing Angle Reject: Discard if area < 900 px² or reprojection RMSE > 2.0 px.
    3. Solves metric PnP and inverts transforms to yield absolute base pose in 'map' frame.
    4. Publishes /cloud/aruco_floor/pose with millimeter measurement covariance.
"""

import os
import math
import yaml
import numpy as np
import cv2
from typing import Optional, Dict, Any, List, Tuple

try:
    from cloud_core.bus import CloudNode
    from cloud_core.types import PoseStamped, Pose, Point, Quaternion, TimeStamp
except ImportError:
    # Standalone mock types for direct unit test import
    class TimeStamp:
        @staticmethod
        def now():
            import time
            return time.time()

    class Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = x, y, z

    class Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x, self.y, self.z, self.w = x, y, z, w

    class Pose:
        def __init__(self, position=None, orientation=None):
            self.position = position or Point()
            self.orientation = orientation or Quaternion()

    class PoseStamped:
        def __init__(self, stamp=None, frame_id="map", pose=None):
            self.stamp = stamp or TimeStamp.now()
            self.frame_id = frame_id
            self.pose = pose or Pose()

    class CloudNode:
        def __init__(self, name=""):
            self.name = name
        def create_publisher(self, msg_type, topic, qos=10):
            class MockPub:
                def __init__(self, t): self.topic = t
                def publish(self, msg): pass
            return MockPub(topic)


def euler_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Euler ZYX to 3x3 rotation matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return R_z @ R_y @ R_x


def matrix_to_quaternion(R: np.ndarray) -> Quaternion:
    """Converts 3x3 rotation matrix to unit quaternion."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return Quaternion(x=x, y=y, z=z, w=w)


class FloorArucoLocalizer(CloudNode):
    """
    Subscribes to downward camera (/cloud/camera/depth_down/image_raw).
    Detects floor markers, solves PnP, and publishes absolute base_link poses.
    """
    def __init__(
        self,
        node_name: str = "floor_aruco_localizer",
        registry_path: Optional[str] = None,
        marker_size_m: float = 0.18,
        camera_pitch_rad: float = -0.6108, # -35 deg downward
        camera_pos_body: Tuple[float, float, float] = (0.30, 0.0, 0.02)
    ):
        super().__init__(node_name)
        self.marker_size = marker_size_m
        self.min_tag_id = 100 # SAFEGUARD 1: Strict lower bound
        self.max_tag_id = 123
        self.reproj_rmse_threshold = 2.0 # SAFEGUARD 2: Max allowable reprojection error (px)
        self.min_contour_area = 900.0    # SAFEGUARD 2: Grazing rejection (30x30 px)

        # Downward RealSense D435i Intrinsic Matrix (640x480)
        self.camera_matrix = np.array([
            [385.0,   0.0, 320.0],
            [  0.0, 385.0, 240.0],
            [  0.0,   0.0,   1.0]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # 3D corners of square floor marker in tag frame (flat on floor Z=0)
        half = self.marker_size / 2.0
        self.obj_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0]
        ], dtype=np.float64)

        # Extrinsic Transform: base_link -> camera optical frame
        # Optical convention: X right, Y down, Z forward
        # Chassis convention: X forward, Y left, Z up
        # Base pitch -35° about Y_chassis
        R_pitch = euler_to_matrix(0.0, camera_pitch_rad, 0.0)
        # Permutation from chassis to optical frame:
        # X_opt = -Y_chassis, Y_opt = -Z_chassis, Z_opt = +X_chassis
        R_chassis_to_opt = np.array([
            [ 0.0, -1.0,  0.0],
            [ 0.0,  0.0, -1.0],
            [ 1.0,  0.0,  0.0]
        ])
        self.R_base_to_cam = R_chassis_to_opt @ R_pitch
        self.t_base_to_cam = np.array(camera_pos_body, dtype=np.float64).reshape((3, 1))

        # 4x4 Homogeneous T_base_to_cam
        self.T_base_to_cam = np.eye(4)
        self.T_base_to_cam[0:3, 0:3] = self.R_base_to_cam
        self.T_base_to_cam[0:3, 3:] = self.t_base_to_cam

        # ArUco Dictionary Setup
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)
        else:
            self.detector_params = cv2.aruco.DetectorParameters_create()
            self.detector = None

        # Load floor marker database
        self.floor_markers: Dict[int, Dict[str, Any]] = {}
        if registry_path and os.path.exists(registry_path):
            with open(registry_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
                for item in data.get("markers", []):
                    tid = item["tag_id"]
                    self.floor_markers[tid] = item

        # Publishers
        self.pub_floor_pose = self.create_publisher(PoseStamped, "/cloud/aruco_floor/pose")
        self.last_tag_id: Optional[int] = None
        self.last_robot_pose: Optional[PoseStamped] = None

    def process_frame(self, image_bgr: np.ndarray) -> List[Dict[str, Any]]:
        """
        Processes camera image from down-tilted RealSense D435i.
        Returns list of valid floor tag observations with calculated robot global pose.
        """
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.detector_params
            )

        results = []
        if ids is None or len(ids) == 0:
            return results

        for i, raw_id in enumerate(ids.flatten()):
            tag_id = int(raw_id)

            # SAFEGUARD 1: Reject booth markers (IDs < 100) or out-of-range tags
            if tag_id < self.min_tag_id or tag_id > self.max_tag_id:
                continue

            img_corners = corners[i].reshape((4, 2))

            # SAFEGUARD 2A: Grazing angle / tiny area reject
            contour_area = float(cv2.contourArea(img_corners.astype(np.float32)))
            if contour_area < self.min_contour_area:
                continue

            # Solve PnP: Marker -> Camera optical frame
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points,
                img_corners,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            if not success:
                continue

            # SAFEGUARD 2B: Reprojection RMSE validation
            proj_pts, _ = cv2.projectPoints(
                self.obj_points, rvec, tvec, self.camera_matrix, self.dist_coeffs
            )
            rmse = float(np.sqrt(np.mean((img_corners - proj_pts.reshape((4, 2))) ** 2)))
            if rmse > self.reproj_rmse_threshold:
                continue

            # Convert rvec to 3x3 rotation matrix R_cam_to_tag
            R_cam_to_tag, _ = cv2.Rodrigues(rvec)

            # 4x4 Homogeneous T_cam_to_tag
            T_cam_to_tag = np.eye(4)
            T_cam_to_tag[0:3, 0:3] = R_cam_to_tag
            T_cam_to_tag[0:3, 3:] = tvec

            # Look up ground truth tag pose in 'map' frame
            if tag_id not in self.floor_markers:
                # Still record measurement relative to camera
                results.append({
                    "tag_id": tag_id,
                    "rmse_px": rmse,
                    "area_px": contour_area,
                    "distance_m": float(np.linalg.norm(tvec)),
                    "world_pose": None
                })
                continue

            tag_info = self.floor_markers[tag_id]
            tx = tag_info["pose"]["x"]
            ty = tag_info["pose"]["y"]
            tz = tag_info["pose"]["z"]
            tyaw = tag_info["pose"]["yaw_rad"]

            # Tag in world: T_world_to_tag
            R_world_to_tag = euler_to_matrix(0.0, 0.0, tyaw)
            T_world_to_tag = np.eye(4)
            T_world_to_tag[0:3, 0:3] = R_world_to_tag
            T_world_to_tag[0:3, 3] = [tx, ty, tz]

            # Invert: T_world_to_cam = T_world_to_tag * inv(T_cam_to_tag)
            T_tag_to_cam = np.linalg.inv(T_cam_to_tag)
            T_world_to_cam = T_world_to_tag @ T_tag_to_cam

            # Invert: T_world_to_base = T_world_to_cam * inv(T_base_to_cam)
            T_cam_to_base = np.linalg.inv(self.T_base_to_cam)
            T_world_to_base = T_world_to_cam @ T_cam_to_base

            robot_x = float(T_world_to_base[0, 3])
            robot_y = float(T_world_to_base[1, 3])
            robot_z = float(T_world_to_base[2, 3])
            q_base = matrix_to_quaternion(T_world_to_base[0:3, 0:3])

            pose_msg = PoseStamped(
                stamp=TimeStamp.now(),
                frame_id="map",
                pose=Pose(
                    position=Point(x=robot_x, y=robot_y, z=robot_z),
                    orientation=q_base
                )
            )

            res = {
                "tag_id": tag_id,
                "rmse_px": rmse,
                "area_px": contour_area,
                "distance_m": float(np.linalg.norm(tvec)),
                "world_pose": pose_msg,
                "robot_xy": (robot_x, robot_y)
            }
            results.append(res)

            self.last_tag_id = tag_id
            self.last_robot_pose = pose_msg
            self.pub_floor_pose.publish(pose_msg)

        return results
