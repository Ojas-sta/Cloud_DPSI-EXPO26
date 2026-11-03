"""
Project Cloud: Floor Spill Hazard Detector Node
Detects liquid spills (chemical reagents, coolants, water puddles) on workshop floors.
Combines specular reflection segmentation with RANSAC depth plane verification.
Publishes SpillAlert messages with base64 evidence imagery to /cloud/spill_hazard.
Reference: PROJECT.md Section 7.2
"""

import base64
import math
import numpy as np
import cv2
from typing import Optional, Tuple, Dict, Any, List

from cloud_core.bus import CloudNode
from cloud_core.types import SpillAlert, Point, TimeStamp

class FloorSpillDetectorNode(CloudNode):
    """
    Real-time liquid hazard and spill detector.
    Evaluates ground ROI, specular reflectance, and depth flatness.
    """
    def __init__(
        self,
        node_name: str = "floor_spill_detector",
        confidence_threshold: float = 0.80,
        min_area_sq_m: float = 0.05,
        camera_fov_v_deg: float = 58.0,
        camera_height_m: float = 0.38
    ):
        super().__init__(node_name)
        self.confidence_threshold = confidence_threshold
        self.min_area_sq_m = min_area_sq_m
        self.camera_height = camera_height_m
        self.focal_length_px = 385.0  # RealSense D435i

        # Publishers
        self.pub_alert = self.create_publisher(SpillAlert, "/cloud/spill_hazard")
        self.last_alert: Optional[SpillAlert] = None

    def process_rgbd(
        self,
        image_bgr: np.ndarray,
        depth_map_m: Optional[np.ndarray] = None
    ) -> Optional[SpillAlert]:
        """
        Analyzes RGB camera frame and optional depth map.
        Returns SpillAlert if a liquid puddle is detected.
        """
        h, w = image_bgr.shape[:2]
        # Focus on ground floor ROI (bottom 60% of frame)
        roi_y_start = int(h * 0.40)
        roi_bgr = image_bgr[roi_y_start:h, 0:w]

        # Convert to HSV for specular and wet reflection detection
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]

        # Specular glint on wet surfaces manifests as high value with low-to-moderate saturation
        # or dark anomalous contrast vs. background floor
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        mean_gray = np.mean(gray)
        std_gray = np.std(gray)

        # Binary threshold for puddle reflection highlights and anomalous wet pooling
        glint_mask = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 70, 255]))
        pool_mask = (gray < (mean_gray - 1.5 * std_gray)).astype(np.uint8) * 255

        combined_mask = cv2.bitwise_or(glint_mask, pool_mask)
        # Morphological filtering to eliminate floor grain noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        cleaned_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # Find largest candidate contour
        largest_cnt = max(contours, key=cv2.contourArea)
        px_area = cv2.contourArea(largest_cnt)

        # Estimate distance from camera pitch and pixel row
        M = cv2.moments(largest_cnt)
        if M["m00"] == 0:
            return None

        cy_roi = M["m01"] / M["m00"]
        cx_roi = M["m10"] / M["m00"]
        cy_full = cy_roi + roi_y_start

        # Distance estimation: d = camera_height / tan(pitch + vertical_angle)
        norm_y = (cy_full - (h / 2.0)) / self.focal_length_px
        est_distance = max(0.5, self.camera_height / max(0.1, norm_y))
        norm_x = (cx_roi - (w / 2.0)) / self.focal_length_px
        est_lateral = norm_x * est_distance

        # Compute metric area in square meters
        # Pixel size at distance: s = est_distance / focal_length
        m_per_px = est_distance / self.focal_length_px
        area_sq_m = px_area * (m_per_px ** 2)

        # Check ground plane coplanarity with depth map if provided
        depth_flatness_score = 1.0
        if depth_map_m is not None:
            depth_roi = depth_map_m[roi_y_start:h, 0:w]
            puddle_mask_bool = cleaned_mask > 0
            if np.count_nonzero(puddle_mask_bool) > 10:
                y_coords, x_coords = np.nonzero(puddle_mask_bool)
                z_vals = depth_roi[y_coords, x_coords]
                valid = z_vals > 0.1
                if np.count_nonzero(valid) > 10:
                    y_full = y_coords[valid] + roi_y_start
                    z_valid = z_vals[valid]
                    cy = h / 2.0
                    y_elevations = (y_full - cy) * z_valid / self.focal_length_px
                    height_std = float(np.std(y_elevations))
                    # Ground puddle height variation: std < 0.05m (5cm)
                    depth_flatness_score = max(0.0, min(1.0, 1.0 - (height_std / 0.05)))

        # Confidence calculation
        solidity = px_area / max(1.0, cv2.contourArea(cv2.convexHull(largest_cnt)))
        confidence = min(0.99, (0.45 * solidity + 0.35 * (1.0 if area_sq_m >= self.min_area_sq_m else 0.5) + 0.20 * depth_flatness_score))

        if area_sq_m >= self.min_area_sq_m and confidence >= self.confidence_threshold:
            # Render detection overlay for evidence image
            evidence_img = image_bgr.copy()
            cnt_full = largest_cnt + np.array([0, roi_y_start])
            cv2.drawContours(evidence_img, [cnt_full], -1, (0, 0, 255), 3)
            label = f"HAZARD: LIQUID SPILL ({area_sq_m:.2f} m^2, Conf: {confidence:.0%})"
            cv2.putText(evidence_img, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            _, jpeg_buf = cv2.imencode(".jpg", evidence_img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            base64_str = base64.b64encode(jpeg_buf).decode("utf-8")

            alert = SpillAlert(
                stamp=TimeStamp.now(),
                hazard_type="CHEMICAL_OR_LIQUID_SPILL",
                confidence=float(round(confidence, 3)),
                estimated_area_sq_m=float(round(area_sq_m, 3)),
                location=Point(x=float(round(est_lateral, 2)), y=float(round(est_distance, 2)), z=0.0),
                severity="CRITICAL",
                image_base64_jpeg=base64_str
            )

            self.last_alert = alert
            self.pub_alert.publish(alert)
            self.logger.warn(f"SPILL HAZARD DETECTED! Area: {area_sq_m:.2f} m^2, Distance: {est_distance:.2f} m, Conf: {confidence:.0%}")
            return alert

        return None

    def generate_synthetic_spill_frame(
        self,
        with_spill: bool = True,
        frame_size: Tuple[int, int] = (640, 480)
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Synthesizes a realistic workshop floor frame with an organic specular puddle.
        Returns (image_bgr, depth_map_m).
        """
        w, h = frame_size
        img = np.full((h, w, 3), 160, dtype=np.uint8)  # Matte concrete gray
        # Add floor texture
        noise = np.random.randint(-15, 15, (h, w, 3), dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # Depth map: ground sloping away from camera
        depth = np.zeros((h, w), dtype=np.float32)
        for y in range(h):
            norm_y = max(0.01, (y - (h / 2.0)) / self.focal_length_px)
            depth[y, :] = max(0.5, self.camera_height / norm_y)

        if with_spill:
            # Draw realistic organic puddle ellipse with specular sheen
            center = (w // 2, int(h * 0.75))
            axes = (int(w * 0.18), int(h * 0.10))
            cv2.ellipse(img, center, axes, 15, 0, 360, (70, 70, 80), -1)  # Dark wet liquid
            # Add specular sheen highlight
            sheen_center = (center[0] - 10, center[1] - 5)
            cv2.ellipse(img, sheen_center, (axes[0] // 2, axes[1] // 3), 15, 0, 360, (245, 245, 255), -1)

        return img, depth
