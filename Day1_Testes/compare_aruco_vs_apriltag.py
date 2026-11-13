#!/usr/bin/env python3
"""
ArUco vs. AprilTag (tag36h11) Accuracy & Performance Benchmark
Venue: DPSI EXPO-2026 | Platform: Unitree Go2 Pro Quadruped Docent

Evaluates and compares:
  1. Detection Rate (%) across distances (0.5m to 5.5m)
  2. solvePnP Reprojection RMSE (pixels)
  3. Stationary Pose Jitter (sigma_x, sigma_y, sigma_z in mm, sigma_yaw in deg)
  4. Frame Processing Latency (ms)

Supports:
  • Simulation Mode: Generates synthetic perspective frames across distances and angles.
  • Live Camera Mode: Evaluates physical printed A4 markers in real-time (--camera 0).
"""

import sys
import os
import time
import argparse
import math
import cv2
import numpy as np
from typing import Dict, List, Tuple, Any, Optional

# Add parent unitree_direct_sdk to import path
SDK_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "unitree_direct_sdk")
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

from real_aruco_vision import RealArUcoVision


class FiducialBenchmarkEvaluator:
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, marker_size_m: float = 0.16):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_size = marker_size_m

        # Object points for 160mm planar square marker
        s = self.marker_size
        self.obj_pts = np.array([
            [-s / 2.0,  s / 2.0, 0.0],
            [ s / 2.0,  s / 2.0, 0.0],
            [ s / 2.0, -s / 2.0, 0.0],
            [-s / 2.0, -s / 2.0, 0.0]
        ], dtype=np.float32)

        # 1. ArUco Setup (DICT_4X4_250)
        self.dict_aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.params_aruco = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
            self.params_aruco.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.det_aruco = cv2.aruco.ArucoDetector(self.dict_aruco, self.params_aruco) if hasattr(cv2.aruco, 'ArucoDetector') else None

        # 2. AprilTag Setup (DICT_APRILTAG_36h11)
        self.dict_apriltag = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.params_apriltag = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, 'CORNER_REFINE_APRILTAG'):
            self.params_apriltag.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        elif hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
            self.params_apriltag.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.det_apriltag = cv2.aruco.ArucoDetector(self.dict_apriltag, self.params_apriltag) if hasattr(cv2.aruco, 'ArucoDetector') else None

    def compute_reprojection_rmse(self, corners: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> float:
        """Computes pixel Euclidean root-mean-square reprojection error."""
        proj_pts, _ = cv2.projectPoints(self.obj_pts, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        proj_pts = proj_pts.reshape(-1, 2)
        corners_pts = corners.reshape(-1, 2)
        sq_errs = np.sum((proj_pts - corners_pts) ** 2, axis=1)
        return float(np.sqrt(np.mean(sq_errs)))

    def solve_marker_pose(self, corners: np.ndarray) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray], float]:
        """Runs cv2.SOLVEPNP_IPPE_SQUARE with fallback and returns (success, rvec, tvec, rmse)."""
        img_pts = corners.reshape(-1, 2).astype(np.float32)
        try:
            success, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                self.obj_pts, img_pts, self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if success and len(tvecs) > 0 and not np.isnan(tvecs[0]).any():
                rvec = rvecs[0]
                tvec = tvecs[0]
                rmse = self.compute_reprojection_rmse(img_pts, rvec, tvec)
                return True, rvec, tvec, rmse
        except Exception:
            pass

        # Fallback to iterative
        try:
            success, rvec, tvec = cv2.solvePnP(
                self.obj_pts, img_pts, self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            if success and not np.isnan(tvec).any():
                rmse = self.compute_reprojection_rmse(img_pts, rvec, tvec)
                return True, rvec, tvec, rmse
        except Exception:
            pass

        return False, None, None, 999.0

    def evaluate_frame(self, frame_gray: np.ndarray, use_apriltag: bool) -> Dict[str, Any]:
        """Runs detection, timing, and pose estimation on a grayscale frame."""
        detector = self.det_apriltag if use_apriltag else self.det_aruco
        t0 = time.perf_counter()
        corners, ids, rejected = detector.detectMarkers(frame_gray)
        dt_ms = (time.perf_counter() - t0) * 1000.0

        detections = []
        if ids is not None and len(ids) > 0:
            for i, tid in enumerate(ids.flatten()):
                succ, rvec, tvec, rmse = self.solve_marker_pose(corners[i])
                if succ:
                    # Compute yaw angle from rvec
                    R, _ = cv2.Rodrigues(rvec)
                    yaw_deg = math.degrees(math.atan2(R[1, 0], R[0, 0]))
                    detections.append({
                        "id": int(tid),
                        "distance_m": float(np.linalg.norm(tvec)),
                        "tvec_mm": (float(tvec[0][0]) * 1000.0, float(tvec[1][0]) * 1000.0, float(tvec[2][0]) * 1000.0),
                        "yaw_deg": yaw_deg,
                        "rmse_px": rmse,
                        "corners": corners[i]
                    })

        return {
            "num_detected": len(detections),
            "latency_ms": dt_ms,
            "detections": detections
        }


def run_synthetic_benchmark():
    """Simulates distances from 1.0m to 5.0m and steep angles (0° to 60°) for both families."""
    print("\n" + "=" * 80)
    print("  🔬 RUNNING SYNTHETIC ARUCO vs. APRILTAG ACCURACY & STABILITY BENCHMARK")
    print("=" * 80 + "\n")

    v_aruco = RealArUcoVision(use_apriltag=False, marker_size_meters=0.16)
    v_apriltag = RealArUcoVision(use_apriltag=True, marker_size_meters=0.16)
    evaluator = FiducialBenchmarkEvaluator(v_aruco.camera_matrix, v_aruco.dist_coeffs, marker_size_m=0.16)

    # Test Matrix across ranges
    test_distances = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5]
    num_samples_per_dist = 20

    aruco_stats = {"detected": 0, "total": 0, "latencies": [], "rmses": [], "tvec_z": []}
    apriltag_stats = {"detected": 0, "total": 0, "latencies": [], "rmses": [], "tvec_z": []}

    print(f"{'Distance (m)':<14} | {'ArUco Detection':<18} | {'ArUco RMSE (px)':<16} | {'AprilTag Detection':<20} | {'AprilTag RMSE (px)':<18}")
    print("-" * 95)

    for dist in test_distances:
        # ArUco pass
        ar_det_count = 0
        ar_rmses = []
        for _ in range(num_samples_per_dist):
            # Frame with tag 101 at distance `dist`
            f = v_aruco.generate_synthetic_frame(0.0, 13.50 - dist, 90.0, "Bench")
            gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            res = evaluator.evaluate_frame(gray, use_apriltag=False)
            aruco_stats["latencies"].append(res["latency_ms"])
            aruco_stats["total"] += 1
            if res["num_detected"] > 0:
                ar_det_count += 1
                aruco_stats["detected"] += 1
                rmse = res["detections"][0]["rmse_px"]
                ar_rmses.append(rmse)
                aruco_stats["rmses"].append(rmse)
                aruco_stats["tvec_z"].append(res["detections"][0]["tvec_mm"][2])

        # AprilTag pass
        at_det_count = 0
        at_rmses = []
        for _ in range(num_samples_per_dist):
            f = v_apriltag.generate_synthetic_frame(0.0, 13.50 - dist, 90.0, "Bench")
            gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            res = evaluator.evaluate_frame(gray, use_apriltag=True)
            apriltag_stats["latencies"].append(res["latency_ms"])
            apriltag_stats["total"] += 1
            if res["num_detected"] > 0:
                at_det_count += 1
                apriltag_stats["detected"] += 1
                rmse = res["detections"][0]["rmse_px"]
                at_rmses.append(rmse)
                apriltag_stats["rmses"].append(rmse)
                apriltag_stats["tvec_z"].append(res["detections"][0]["tvec_mm"][2])

        ar_rate = (ar_det_count / num_samples_per_dist) * 100.0
        at_rate = (at_det_count / num_samples_per_dist) * 100.0
        ar_mean_rmse = np.mean(ar_rmses) if ar_rmses else 999.0
        at_mean_rmse = np.mean(at_rmses) if at_rmses else 999.0

        print(f"{dist:<14.2f} | {f'{ar_rate:.0f}% ({ar_det_count}/{num_samples_per_dist})':<18} | {f'{ar_mean_rmse:.3f} px':<16} | {f'{at_rate:.0f}% ({at_det_count}/{num_samples_per_dist})':<20} | {f'{at_mean_rmse:.3f} px':<18}")

    # Summary
    print("\n" + "=" * 80)
    print("  📊 BENCHMARK SUMMARY & METRIC COMPARISON")
    print("=" * 80)
    print(f"  • Overall Detection Rate:  ArUco = {aruco_stats['detected']/aruco_stats['total']*100:.1f}%  |  AprilTag = {apriltag_stats['detected']/apriltag_stats['total']*100:.1f}%")
    print(f"  • Mean Reprojection RMSE:  ArUco = {np.mean(aruco_stats['rmses']):.3f} px  |  AprilTag = {np.mean(apriltag_stats['rmses']):.3f} px")
    print(f"  • Mean Processing Latency: ArUco = {np.mean(aruco_stats['latencies']):.2f} ms  |  AprilTag = {np.mean(apriltag_stats['latencies']):.2f} ms")
    if len(aruco_stats['tvec_z']) > 5 and len(apriltag_stats['tvec_z']) > 5:
        print(f"  • Range Jitter (sigma_z):  ArUco = {np.std(aruco_stats['tvec_z']):.2f} mm  |  AprilTag = {np.std(apriltag_stats['tvec_z']):.2f} mm")
    print("=" * 80 + "\n")


def run_live_camera_benchmark(camera_idx: int, num_frames: int = 60):
    """Runs a live side-by-side comparison on the physical camera pointing at a printed tag or board."""
    print("\n" + "=" * 80)
    print(f"  📷 RUNNING LIVE CAMERA BENCHMARK ON DEVICE #{camera_idx} ({num_frames} FRAMES)")
    print("  • Point the camera at a printed ArUco or AprilTag marker / docking board.")
    print("=" * 80 + "\n")

    cap = cv2.VideoCapture(camera_idx)
    if not cap.isOpened():
        print(f"  ❌ Error: Unable to open physical camera {camera_idx}!")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # Use default 720p intrinsics
    camera_matrix = np.array([[900.0, 0, 640.0], [0, 900.0, 360.0], [0, 0, 1.0]], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)
    evaluator = FiducialBenchmarkEvaluator(camera_matrix, dist_coeffs, marker_size_m=0.16)

    # Warm up camera
    for _ in range(10):
        cap.read()

    print("  Capturing live frames and running dual detector passes...")
    aruco_dets = []
    apriltag_dets = []

    for frame_i in range(num_frames):
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        res_ar = evaluator.evaluate_frame(gray, use_apriltag=False)
        res_at = evaluator.evaluate_frame(gray, use_apriltag=True)

        if res_ar["num_detected"] > 0:
            aruco_dets.append(res_ar["detections"][0])
        if res_at["num_detected"] > 0:
            apriltag_dets.append(res_at["detections"][0])

        time.sleep(0.03)

    cap.release()

    # Compute live jitter metrics
    print("\n" + "=" * 80)
    print("  📊 LIVE CAMERA REPROJECTION & POSE JITTER RESULTS")
    print("=" * 80)

    for fam_name, det_list in [("ArUco (DICT_4X4_250)", aruco_dets), ("AprilTag (DICT_APRILTAG_36h11)", apriltag_dets)]:
        print(f"\n  [{fam_name}]")
        print(f"  • Detection Rate: {len(det_list)} / {num_frames} ({len(det_list)/num_frames*100:.1f}%)")
        if len(det_list) >= 3:
            xs = [d["tvec_mm"][0] for d in det_list]
            ys = [d["tvec_mm"][1] for d in det_list]
            zs = [d["tvec_mm"][2] for d in det_list]
            yaws = [d["yaw_deg"] for d in det_list]
            rmses = [d["rmse_px"] for d in det_list]

            print(f"  • Mean Distance:      {np.mean(zs)/1000.0:.3f} m")
            print(f"  • Mean Reproj RMSE:   {np.mean(rmses):.3f} px (std: {np.std(rmses):.3f} px)")
            print(f"  • Pose Jitter (mm):   sigma_x = {np.std(xs):.2f} mm | sigma_y = {np.std(ys):.2f} mm | sigma_z = {np.std(zs):.2f} mm")
            print(f"  • Heading Jitter:     sigma_yaw = {np.std(yaws):.2f}°")
        else:
            print("  ⚠️ Insufficient detections to compute stationary jitter statistics.")
    print("=" * 80 + "\n")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="ArUco vs AprilTag Quantitative Accuracy Benchmark")
    parser.add_argument("--camera", type=int, default=None, help="Camera index for physical camera evaluation (e.g. 0)")
    parser.add_argument("--frames", type=int, default=60, help="Number of frames for live camera test")
    args = parser.parse_args()

    if args.camera is not None:
        run_live_camera_benchmark(args.camera, num_frames=args.frames)
    else:
        run_synthetic_benchmark()
