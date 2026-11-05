"""
Project Cloud: 6DOF ArUco Optical Tracker Node
Detects 160mm ArUco markers on exhibit booths and tool cabinets using cv2.aruco.
Solves Perspective-n-Point (PnP) for precise 6DOF camera-to-landmark pose.
Reference: PROJECT.md Section 7.2
"""

import math
import numpy as np
import cv2
from typing import Optional, Tuple, Dict, Any, List

from cloud_core.bus import CloudNode
from cloud_core.types import PoseStamped, Pose, Point, Quaternion, TimeStamp

class ArucoTrackerNode(CloudNode):
    """
    ArUco tracking node running at 30 FPS.
    Detects DICT_5X5_250 markers and publishes 6DOF PoseStamped.
    """
    def __init__(
        self,
        node_name: str = "aruco_tracker_node",
        marker_size_m: float = 0.16,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None
    ):
        super().__init__(node_name)
        self.marker_size = marker_size_m

        # Default RealSense D435i Intrinsic Calibration (640x480)
        if camera_matrix is None:
            self.camera_matrix = np.array([
                [385.0,   0.0, 320.0],
                [  0.0, 385.0, 240.0],
                [  0.0,   0.0,   1.0]
            ], dtype=np.float64)
        else:
            self.camera_matrix = camera_matrix

        if dist_coeffs is None:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        else:
            self.dist_coeffs = dist_coeffs

        # Define 3D corner coordinates of marker in marker coordinate frame
        half = self.marker_size / 2.0
        self.obj_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0]
        ], dtype=np.float64)

        # ArUco Dictionary Setup
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)
        else:
            self.detector_params = cv2.aruco.DetectorParameters_create()
            self.detector = None

        # Publishers
        self.pub_pose = self.create_publisher(PoseStamped, "/cloud/detected_marker_pose")
        self.pub_marker_id = self.create_publisher(int, "/cloud/detected_marker_id")

        self.last_detected_id: Optional[int] = None
        self.last_pose: Optional[PoseStamped] = None

    def process_frame(self, image_bgr: np.ndarray) -> List[Dict[str, Any]]:
        """
        Processes a single BGR camera frame.
        Returns list of detected marker detections with 6DOF pose.
        """
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.detector_params
            )

        detections = []
        if ids is None or len(ids) == 0:
            return detections

        for i, marker_id in enumerate(ids.flatten()):
            # SAFEGUARD 1: Reject floor markers (IDs >= 100) - booth tracker strictly handles IDs 1-44
            if int(marker_id) >= 100:
                continue

            img_corners = corners[i].reshape((4, 2))
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points,
                img_corners,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success:
                # Convert rotation vector to quaternion
                R, _ = cv2.Rodrigues(rvec)
                q = self._rotation_matrix_to_quaternion(R)

                # Camera optical frame: X right, Y down, Z forward
                pos = Point(
                    x=float(tvec[0][0]),
                    y=float(tvec[1][0]),
                    z=float(tvec[2][0])
                )

                pose_stamped = PoseStamped(
                    stamp=TimeStamp.now(),
                    frame_id="camera_color_optical_frame",
                    pose=Pose(position=pos, orientation=q)
                )

                detection = {
                    "marker_id": int(marker_id),
                    "pose": pose_stamped,
                    "distance_m": float(np.linalg.norm(tvec)),
                    "lateral_offset_m": float(tvec[0][0]),
                    "corners": img_corners
                }
                detections.append(detection)

                # Publish detection
                self.last_detected_id = int(marker_id)
                self.last_pose = pose_stamped
                self.pub_marker_id.publish(int(marker_id))
                self.pub_pose.publish(pose_stamped)

        return detections

    @staticmethod
    def _rotation_matrix_to_quaternion(R: np.ndarray) -> Quaternion:
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

    def generate_synthetic_marker_frame(
        self,
        marker_id: int = 10,
        distance_m: float = 1.8,
        lateral_offset_m: float = 0.0,
        frame_size: Tuple[int, int] = (640, 480)
    ) -> np.ndarray:
        """
        Generates a synthetic camera image with rendered ArUco marker.
        Enables 100% deterministic camera testing without hardware.
        """
        img = np.full((frame_size[1], frame_size[0], 3), 230, dtype=np.uint8)

        # Draw ArUco marker
        marker_img = cv2.aruco.generateImageMarker(self.dictionary, marker_id, 200)

        # Perspective projection onto synthetic canvas
        fx = self.camera_matrix[0, 0]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        proj_size = int((self.marker_size / distance_m) * fx)
        proj_cx = int(cx + (lateral_offset_m / distance_m) * fx)
        proj_cy = int(cy)

        x1 = max(0, proj_cx - proj_size // 2)
        y1 = max(0, proj_cy - proj_size // 2)
        x2 = min(frame_size[0], proj_cx + proj_size // 2)
        y2 = min(frame_size[1], proj_cy + proj_size // 2)

        actual_w = x2 - x1
        actual_h = y2 - y1

        if actual_w > 10 and actual_h > 10:
            resized_marker = cv2.resize(marker_img, (actual_w, actual_h), interpolation=cv2.INTER_NEAREST)
            resized_marker_bgr = cv2.cvtColor(resized_marker, cv2.COLOR_GRAY2BGR)
            img[y1:y2, x1:x2] = resized_marker_bgr

        return img
