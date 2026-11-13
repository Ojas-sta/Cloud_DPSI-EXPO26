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
        use_builtin_cam: bool = False,
        marker_size_meters: float = 0.20,
        layout_preset: str = "minimal_2m_1s",
        use_apriltag: bool = False
    ):
        self.marker_size = marker_size_meters
        self.camera_index = camera_index
        self.use_builtin_cam = use_builtin_cam
        self.layout_preset = layout_preset
        self.use_apriltag = use_apriltag
        self.cap = None
        self.video_client = None

        # Camera intrinsic matrix:
        # Synthetic renderer uses fx=450 matching perspective scale; physical camera uses calibrated D435i
        self.width = 1920
        self.height = 1080
        fx = 450.0
        fy = 450.0
        cx = 960.0
        cy = 540.0
        self.camera_matrix = np.array([
            [fx,  0, cx],
            [ 0, fy, cy],
            [ 0,  0,  1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        # Check for calibrated 120° fisheye intrinsics (Kannala-Brandt model)
        self.is_fisheye_calibrated = False
        self.fisheye_D = None
        import os
        calib_npz = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "go2_fisheye_120_calib.npz")
        if os.path.exists(calib_npz):
            try:
                data = np.load(calib_npz)
                self.camera_matrix = data["K"].astype(np.float32)
                self.fisheye_D = data["D"].astype(np.float32)
                dim = data.get("DIM", np.array([1280, 720]))
                self.width, self.height = int(dim[0]), int(dim[1])
                self.is_fisheye_calibrated = True
                print(f"  🎯 [RealArUcoVision]: Loaded 120° Fisheye Calibration from {os.path.basename(calib_npz)} (RMS: {float(data.get('rms', 0.0)):.4f}px).")
            except Exception as e:
                print(f"  ⚠️ [RealArUcoVision]: Could not load fisheye calib ({e}).")

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
        if self.use_builtin_cam:
            self._init_go2_builtin_camera()
        elif camera_index is not None:
            self._init_camera(camera_index)

    def _init_go2_builtin_camera(self):
        """Initializes connection to Unitree Go2 front-mounted chassis camera via SDK2 VideoClient."""
        try:
            print("  📹 [RealArUcoVision]: Initializing Go2 built-in front camera via SDK2 VideoClient...")
            from unitree_sdk2py.go2.video.video_client import VideoClient
            self.video_client = VideoClient()
            self.video_client.SetTimeout(3.0)
            self.video_client.Init()
            # Test frame capture
            code, data = self.video_client.GetImageSample()
            if code == 0 and len(data) > 0:
                test_img = cv2.imdecode(np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR)
                if test_img is not None:
                    h, w = test_img.shape[:2]
                    self.width, self.height = w, h
                    # Builtin wide-angle camera matrix approximation (1280x720 or 1920x1080)
                    fx = w * 0.65
                    fy = w * 0.65
                    cx = w / 2.0
                    cy = h / 2.0
                    self.camera_matrix = np.array([
                        [fx,  0, cx],
                        [ 0, fy, cy],
                        [ 0,  0,  1]
                    ], dtype=np.float32)
                    print(f"  ✓ [RealArUcoVision]: Go2 Built-in Front Camera CONNECTED ({w}x{h}, fx={fx:.1f})!")
            else:
                print(f"  ⚠️ [RealArUcoVision]: Go2 VideoClient responded with code {code}. Make sure DDS is initialized.")
        except Exception as e:
            print(f"  ⚠️ [RealArUcoVision]: Could not initialize Go2 VideoClient ({e}). Using synthetic fallback.")

    def _init_camera(self, idx: int):
        print(f"  📷 [RealArUcoVision]: Attempting connection to physical camera {idx}...")
        self.cap = cv2.VideoCapture(idx)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            # Apply calibrated RealSense D435i intrinsics for real hardware
            fx = 1642.26
            fy = 1642.26
            cx = 999.63
            cy = 490.62
            self.camera_matrix = np.array([
                [fx,  0, cx],
                [ 0, fy, cy],
                [ 0,  0,  1]
            ], dtype=np.float32)
            print(f"  ✓ [RealArUcoVision]: Physical camera {idx} connected ({actual_w}x{actual_h}) with calibrated matrix (fx={fx}).")
        else:
            print(f"  ⚠️ [RealArUcoVision]: Camera {idx} unavailable. Using synthetic visual renderer.")
            self.cap = None

    def read_frame(self, fallback_x: float = 0.0, fallback_y: float = 0.0, fallback_yaw: float = 0.0, stage: str = "") -> np.ndarray:
        """Reads a real frame from hardware (Go2 built-in camera or USB/RealSense); otherwise falls back to synthetic generation."""
        if self.use_builtin_cam and self.video_client is not None:
            try:
                code, data = self.video_client.GetImageSample()
                if code == 0 and len(data) > 0:
                    img = cv2.imdecode(np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR)
                    if img is not None and img.size > 0:
                        return img
            except Exception:
                pass

        if self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None and frame.size > 0:
                return frame
        return self.generate_synthetic_frame(fallback_x, fallback_y, fallback_yaw, stage)

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
        if self.layout_preset == "dpsi_expo_31stalls":
            import os, yaml
            cfg_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "dpsi_expo_31stalls.yaml")
            booths = []
            floor_tags = [
                {"id": 100, "name": "Floor SW Entrance", "x": 1.80, "y": 1.00},
                {"id": 101, "name": "Floor West Aisle Low", "x": 1.80, "y": 5.50},
                {"id": 102, "name": "Floor West Crosswalk", "x": 1.80, "y": 9.50},
                {"id": 103, "name": "Floor West Aisle High", "x": 1.80, "y": 13.50},
                {"id": 104, "name": "Floor NW Junction", "x": 1.80, "y": 17.20},
                {"id": 105, "name": "Floor North Concourse Mid", "x": 9.50, "y": 17.20},
                {"id": 106, "name": "Floor North Center", "x": 16.00, "y": 17.20},
                {"id": 107, "name": "Floor North East", "x": 23.20, "y": 17.20},
                {"id": 108, "name": "Floor East Aisle High", "x": 23.20, "y": 13.50},
                {"id": 109, "name": "Floor East Crosswalk", "x": 23.20, "y": 9.50},
                {"id": 110, "name": "Floor East Aisle Low", "x": 23.20, "y": 5.50},
                {"id": 111, "name": "Floor Mid Aisle High", "x": 14.80, "y": 13.50},
                {"id": 112, "name": "Floor Mid Crosswalk", "x": 14.80, "y": 9.50},
                {"id": 113, "name": "Floor Mid Aisle Low", "x": 14.80, "y": 5.50},
            ]
            if os.path.exists(cfg_path):
                with open(cfg_path, "r") as f:
                    cfg_data = yaml.safe_load(f)
                for s in cfg_data.get("stalls", []):
                    booths.append({
                        "id": s["tag_id"],
                        "stall": s["id"],
                        "name": s["title"],
                        "x": s["booth_pose"]["x"],
                        "y": s["booth_pose"]["y"]
                    })
        elif self.layout_preset == "scaled_36s":
            import os, yaml
            cfg_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "scaled_36s.yaml")
            booths = []
            floor_tags = []
            if os.path.exists(cfg_path):
                with open(cfg_path, "r") as f:
                    cfg_data = yaml.safe_load(f)
                for s in cfg_data.get("stalls", []):
                    booths.append({
                        "id": s["tag_id"],
                        "stall": s["id"],
                        "name": s["title"],
                        "x": s["booth_pose"]["x"],
                        "y": s["booth_pose"]["y"]
                    })
                for c in cfg_data.get("cohorts", []):
                    for ft_id in c.get("floor_tags", []):
                        idx_rel = ft_id - 100
                        floor_tags.append({
                            "id": ft_id,
                            "name": f"Floor Tag {ft_id}",
                            "x": 0.0 if idx_rel < 9 else (4.0 + (idx_rel - 9) * 2.0 if idx_rel < 18 else (12.0 if idx_rel < 27 else 20.0)),
                            "y": 3.0 + (idx_rel % 9) * 2.0
                        })
        elif self.layout_preset == "scaled_5m_2s":
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
                    pad = max(12, int(tag_w * 0.15))
                    cv2.rectangle(frame, (max(0, x1 - pad), max(0, y1 - pad)), (min(self.width - 1, x2 + pad), min(self.height - 1, y2 + pad)), (255, 255, 255), -1)
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
                    pad = max(12, int(tag_w * 0.15))
                    # Circular floor decal background
                    cv2.circle(frame, (center_x, center_y), int(tag_w * 0.75 + pad), (0, 180, 255), 2)
                    cv2.rectangle(frame, (max(0, x1 - pad), max(0, y1 - pad)), (min(self.width - 1, x2 + pad), min(self.height - 1, y2 + pad)), (255, 255, 255), -1)
                    marker_img = cv2.aruco.generateImageMarker(self.aruco_dict, ft["id"], tag_w)
                    marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
                    frame[y1:y2, x1:x2] = marker_bgr

                    # Floor Tag ID Label
                    cv2.putText(
                        frame, f"FLOOR TAG #{ft['id']}", (center_x - 55, min(self.height - 10, y2 + pad + 20)),
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

                # 120° Fisheye Kannala-Brandt correction
                if self.is_fisheye_calibrated and self.fisheye_D is not None:
                    # Undistort 2D corners into rectified optical plane before solvePnP
                    pts_undist = cv2.fisheye.undistortPoints(
                        img_pts.reshape(1, -1, 2).astype(np.float64),
                        self.camera_matrix.astype(np.float64),
                        self.fisheye_D.astype(np.float64),
                        P=self.camera_matrix.astype(np.float64)
                    ).reshape(-1, 2).astype(np.float32)
                    solve_pts = pts_undist
                    solve_dist = np.zeros((4, 1), dtype=np.float32)
                else:
                    solve_pts = img_pts
                    solve_dist = self.dist_coeffs

                # Refinement 1: Planar square solver (SOLVEPNP_IPPE_SQUARE with SOLVEPNP_ITERATIVE fallback)
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, solve_pts, self.camera_matrix, solve_dist,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if not success or np.isnan(tvec).any():
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, solve_pts, self.camera_matrix, solve_dist,
                        flags=cv2.SOLVEPNP_ITERATIVE
                    )

                if success and not np.isnan(tvec).any():
                    # Draw 3D coordinate axes on marker (X=Red, Y=Green, Z=Blue)
                    cv2.drawFrameAxes(annotated, self.camera_matrix, solve_dist, rvec, tvec, 0.12, 3)

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
                        "tvec": tvec,
                        "corners": corners[i]
                    })

        return annotated, detections
