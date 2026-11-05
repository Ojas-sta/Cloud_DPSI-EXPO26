#!/usr/bin/env python3
"""
Real OpenCV Computer Vision Pipeline for ArUco Marker Detection & Pose Estimation
Features:
  • Genuine cv2.aruco detection on live camera frames
  • Fallback synthetic frame renderer displaying real ArUco fiducials in 3D perspective
  • Computes real distance, lateral offset, and orientation
  • Renders high-contrast bounding boxes, 3D axes, and HUD telemetry badges
"""

import cv2
import numpy as np
import math
import time
from typing import List, Tuple, Dict, Any, Optional


class RealArUcoVision:
    def __init__(
        self,
        camera_index: Optional[int] = None,
        marker_size_meters: float = 0.20,
        layout_preset: str = "minimal_2m_1s",
        use_apriltag: bool = False
    ):
        self.marker_size = marker_size_meters
        self.camera_index = camera_index
        self.layout_preset = layout_preset
        self.use_apriltag = use_apriltag
        self.cap = None

        # Camera intrinsic matrix (standard 720p FOV ~69 deg)
        self.width = 1280
        self.height = 720
        fx = 900.0
        fy = 900.0
        cx = self.width / 2.0
        cy = self.height / 2.0
        self.camera_matrix = np.array([
            [fx,  0, cx],
            [ 0, fy, cy],
            [ 0,  0,  1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        # Initialize detector dictionary: ArUco (DICT_4X4_250) or AprilTag (DICT_APRILTAG_36h11)
        if self.use_apriltag:
            self.fiducial_family_name = "AprilTag (36h11)"
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            self.aruco_params = cv2.aruco.DetectorParameters()
            # AprilTag-tuned parameters: high precision corner refinement
            if hasattr(cv2.aruco, 'CORNER_REFINE_APRILTAG'):
                self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
            elif hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
                self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            print("  🏷️ [RealArUcoVision]: Configured for AprilTag (tag36h11) with subpixel refinement.")
        else:
            self.fiducial_family_name = "ArUco (4X4_250)"
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
            self.aruco_params = cv2.aruco.DetectorParameters()
            if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
                self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # In OpenCV 4.7+, use ArucoDetector; fallback to cv2.aruco.detectMarkers if older
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.detector = None

        # Try connecting to real physical camera if requested
        if camera_index is not None:
            self._init_camera(camera_index)

    def _init_camera(self, idx: int):
        print(f"  📷 [RealArUcoVision]: Attempting connection to physical camera {idx}...")
        self.cap = cv2.VideoCapture(idx)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            print(f"  ✓ [RealArUcoVision]: Physical camera {idx} connected (1280x720).")
        else:
            print(f"  ⚠️ [RealArUcoVision]: Camera {idx} unavailable. Using synthetic visual renderer.")
            self.cap = None

    def generate_synthetic_frame(self, robot_x: float, robot_y: float, robot_yaw_deg: float, target_stall: str) -> np.ndarray:
        """Generates a realistic 3D perspective camera frame containing real ArUco markers."""
        frame = np.full((self.height, self.width, 3), (28, 24, 20), dtype=np.uint8)

        # Floor concourse perspective
        horizon_y = int(self.height * 0.42)
        cv2.rectangle(frame, (0, horizon_y), (self.width, self.height), (45, 38, 32), -1)
        # Perspective floor lines
        for offset_x in range(-600, 1800, 150):
            pt_top = (int(self.width / 2 + offset_x * 0.15), horizon_y)
            pt_bot = (int(self.width / 2 + offset_x * 1.6), self.height)
            cv2.line(frame, pt_top, pt_bot, (65, 55, 48), 1, cv2.LINE_AA)

        # 1. Determine which booth tags (IDs 1..44) are in view based on preset
        if self.layout_preset == "scaled_5m_2s":
            booths = [
                {"id": 1, "stall": "P01", "name": "Stepwise AI", "x": 1.60, "y": 12.00},
                {"id": 2, "stall": "P02", "name": "Move Smarter: ADAS", "x": 1.60, "y": 16.00},
            ]
            floor_tags = [
                {"id": 100, "name": "Floor Tag 100", "x": 0.0, "y": 10.00},
                {"id": 101, "name": "Floor Tag 101", "x": 0.0, "y": 12.00},
                {"id": 102, "name": "Floor Tag 102", "x": 0.0, "y": 14.00},
                {"id": 103, "name": "Floor Tag 103", "x": 0.0, "y": 16.00},
                {"id": 104, "name": "Floor Tag 104", "x": 0.0, "y": 18.00},
            ]
        else:  # default "minimal_2m_1s"
            booths = [
                {"id": 1, "stall": "P01", "name": "Stepwise AI", "x": 1.60, "y": 13.50},
            ]
            floor_tags = [
                {"id": 100, "name": "Floor Tag 100", "x": 0.0, "y": 11.50},
                {"id": 101, "name": "Floor Tag 101", "x": 0.0, "y": 13.50},
            ]

        yaw_rad = math.radians(robot_yaw_deg)
        cos_theta = math.cos(yaw_rad)
        sin_theta = math.sin(yaw_rad)

        for b in booths:
            dx = b["x"] - robot_x
            dy = b["y"] - robot_y
            forward_dist = dx * cos_theta + dy * sin_theta
            lateral_dist = dx * sin_theta - dy * cos_theta

            if 0.4 <= forward_dist <= 4.0 and abs(lateral_dist / max(0.1, forward_dist)) <= 1.2:
                # Render booth desk & real ArUco tag
                scale = 450.0 / forward_dist
                center_x = int(self.width / 2.0 + (lateral_dist * scale))
                center_y = int(horizon_y + 120.0 * (1.0 / forward_dist))
                tag_w = int(self.marker_size * scale)

                # Booth Desk surface
                desk_w = int(tag_w * 3.5)
                desk_h = int(tag_w * 1.8)
                cv2.rectangle(
                    frame,
                    (center_x - desk_w // 2, center_y + tag_w // 2),
                    (center_x + desk_w // 2, center_y + tag_w // 2 + desk_h),
                    (70, 80, 100),
                    -1
                )
                cv2.rectangle(
                    frame,
                    (center_x - desk_w // 2, center_y + tag_w // 2),
                    (center_x + desk_w // 2, center_y + tag_w // 2 + desk_h),
                    (20, 200, 230),
                    2
                )

                # Draw real, mathematically valid ArUco marker image
                marker_img = cv2.aruco.generateImageMarker(self.aruco_dict, b["id"], tag_w)
                marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

                x1 = center_x - tag_w // 2
                y1 = center_y - tag_w // 2
                x2 = x1 + tag_w
                y2 = y1 + tag_w

                if 0 <= x1 and x2 < self.width and 0 <= y1 and y2 < self.height:
                    cv2.rectangle(frame, (x1 - 8, y1 - 8), (x2 + 8, y2 + 8), (255, 255, 255), -1)
                    frame[y1:y2, x1:x2] = marker_bgr

                # Stall Banner on Desk
                banner_txt = f"STALL {b['stall']}: {b['name']}"
                cv2.putText(
                    frame, banner_txt, (center_x - desk_w // 2 + 10, center_y + tag_w // 2 + 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA
                )

        # 2. Determine which floor tags (IDs 100+) are in view
        for ft in floor_tags:
            dx = ft["x"] - robot_x
            dy = ft["y"] - robot_y
            forward_dist = dx * cos_theta + dy * sin_theta
            lateral_dist = dx * sin_theta - dy * cos_theta

            if 0.35 <= forward_dist <= 3.2 and abs(lateral_dist / max(0.1, forward_dist)) <= 1.4:
                scale = 420.0 / forward_dist
                center_x = int(self.width / 2.0 + (lateral_dist * scale))
                center_y = int(horizon_y + 230.0 * (1.0 / forward_dist))
                tag_w = int(self.marker_size * 1.1 * scale)

                x1 = center_x - tag_w // 2
                y1 = center_y - tag_w // 2
                x2 = x1 + tag_w
                y2 = y1 + tag_w

                if 0 <= x1 and x2 < self.width and 0 <= y1 and y2 < self.height:
                    # Circular floor decal background
                    cv2.circle(frame, (center_x, center_y), int(tag_w * 0.75), (0, 180, 255), 2)
                    cv2.rectangle(frame, (x1 - 6, y1 - 6), (x2 + 6, y2 + 6), (255, 255, 255), -1)
                    marker_img = cv2.aruco.generateImageMarker(self.aruco_dict, ft["id"], tag_w)
                    marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
                    frame[y1:y2, x1:x2] = marker_bgr

                    # Floor Tag ID Label
                    cv2.putText(
                        frame, f"FLOOR TAG #{ft['id']}", (center_x - 55, y2 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 220, 255), 1, cv2.LINE_AA
                    )

        return frame

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, List[Dict[str, Any]]]:
        """Runs OpenCV ArUco detection, draws 3D annotations, and returns detected tags."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        annotated = frame.copy()
        detections = []

        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and len(ids) > 0:
            # Draw detected marker outlines
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids, (0, 255, 0))

            # Estimate 3D pose using solvePnP
            half_s = self.marker_size / 2.0
            obj_points = np.array([
                [-half_s,  half_s, 0.0],
                [ half_s,  half_s, 0.0],
                [ half_s, -half_s, 0.0],
                [-half_s, -half_s, 0.0]
            ], dtype=np.float32)

            for i, tag_id in enumerate(ids.flatten()):
                img_pts = corners[i][0].astype(np.float32)
                # Refinement 1: Planar square ArUco solver (SOLVEPNP_IPPE_SQUARE with SOLVEPNP_ITERATIVE fallback)
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, img_pts, self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if not success or np.isnan(tvec).any():
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, img_pts, self.camera_matrix, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_ITERATIVE
                    )

                if success and not np.isnan(tvec).any():
                    # Draw 3D coordinate axes on marker (X=Red, Y=Green, Z=Blue)
                    cv2.drawFrameAxes(annotated, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.12, 3)

                    # Compute distance and lateral offset
                    dist = float(np.linalg.norm(tvec))
                    dx = float(tvec[0][0])
                    dz = float(tvec[2][0])

                    # Overlay high-contrast HUD badge above tag
                    top_left = tuple(img_pts[0].astype(int))
                    badge_pos = (top_left[0] - 20, max(30, top_left[1] - 20))
                    tag_prefix = "APRILTAG" if self.use_apriltag else "ARUCO"
                    hud_text = f"{tag_prefix} #{tag_id} | Dist: {dist:.2f}m | Lat: {dx:+.2f}m"

                    (tw, th), _ = cv2.getTextSize(hud_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
                    cv2.rectangle(
                        annotated,
                        (badge_pos[0] - 6, badge_pos[1] - th - 6),
                        (badge_pos[0] + tw + 6, badge_pos[1] + 6),
                        (10, 180, 240),
                        -1
                    )
                    cv2.putText(
                        annotated, hud_text, badge_pos,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (10, 10, 10), 2, cv2.LINE_AA
                    )

                    detections.append({
                        "id": int(tag_id),
                        "distance": dist,
                        "dx": dx,
                        "dz": dz,
                        "rvec": rvec,
                        "tvec": tvec
                    })

        return annotated, detections
