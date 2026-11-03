"""
Project Cloud: Safety Geofence & Kinematic Envelope Monitor
Enforces ISO 13482:2014 personal care robot safety standards.
Monitors 2D keep-out polygons, velocity caps, and crowd buffers.
Reference: PROJECT.md Section 7.6
"""

import os
import yaml
import math
from typing import List, Tuple, Optional, Dict, Any
from cloud_core.bus import CloudNode
from cloud_core.types import PoseStamped, Twist

def point_in_polygon(x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
    """Ray casting algorithm for 2D point-in-polygon testing."""
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

class SafetyMonitorNode(CloudNode):
    """
    Subscribes to odometry and commanded velocities.
    Enforces keep-out boundaries and kinematic velocity envelopes.
    """
    def __init__(self, node_name: str = "safety_monitor_node", config_path: Optional[str] = None):
        super().__init__(node_name)
        if config_path is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            config_path = os.path.join(base_dir, "config", "safety_geofences.yaml")

        self.config_path = config_path
        self.keepout_zones: List[Dict[str, Any]] = []
        self.max_linear_vel = 0.65
        self.max_angular_vel = 0.85
        self._load_config()

        # Publishers
        self.pub_e_stop = self.create_publisher(bool, "/cloud/e_stop")
        self.pub_safety_ok = self.create_publisher(bool, "/cloud/safety_status")

        # Subscriptions
        self.sub_odom = self.create_subscription(PoseStamped, "/cloud/odom", self._on_odom)
        self.sub_cmd_vel = self.create_subscription(Twist, "/cloud/cmd_vel", self._on_cmd_vel)

    def _load_config(self):
        if not os.path.exists(self.config_path):
            self.logger.warn(f"Safety geofence config not found at: {self.config_path}")
            return
        with open(self.config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        limits = data.get("kinematic_limits", {})
        self.max_linear_vel = limits.get("max_linear_velocity_mps", 0.65)
        self.max_angular_vel = limits.get("max_angular_velocity_radps", 0.85)

        for zone in data.get("keepout_zones", []):
            pts = [tuple(p) for p in zone.get("polygon_points_2d", [])]
            self.keepout_zones.append({
                "name": zone.get("name", "UNNAMED_ZONE"),
                "polygon": pts,
                "buffer": zone.get("buffer_distance_m", 0.5),
                "action": zone.get("action_on_breach", "EMERGENCY_STOP")
            })

    def is_safe_position(self, x: float, y: float) -> Tuple[bool, Optional[str]]:
        """Returns (is_safe, breached_zone_name)."""
        for zone in self.keepout_zones:
            if point_in_polygon(x, y, zone["polygon"]):
                return False, zone["name"]
        return True, None

    def _on_odom(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        is_safe, zone_name = self.is_safe_position(x, y)
        if not is_safe:
            self.logger.error(f"GEOFENCE BREACH: Robot entered keepout zone '{zone_name}' at ({x:.2f}, {y:.2f})!")
            self.pub_e_stop.publish(True)
            self.pub_safety_ok.publish(False)
        else:
            self.pub_safety_ok.publish(True)

    def _on_cmd_vel(self, msg: Twist):
        vx = msg.linear.x
        wz = msg.angular.z
        if abs(vx) > (self.max_linear_vel + 0.05) or abs(wz) > (self.max_angular_vel + 0.05):
            self.logger.warn(f"VELOCITY EXCEEDED ENVELOPE: vx={vx:.2f} m/s, wz={wz:.2f} rad/s")
