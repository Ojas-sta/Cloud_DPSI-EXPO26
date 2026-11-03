"""
Project Cloud: Optical Breadcrumb Follower Node (Visual Servoing)
Implements dual PID visual servoing to lock onto ArUco markers,
align lateral camera heading, and maintain canonical 1.80m standoff distance.
Reference: PROJECT.md Section 7.2
"""

import math
import time
from typing import Optional

from cloud_core.bus import CloudNode
from cloud_core.types import Twist, Vector3, PoseStamped

class OpticalFollowerNode(CloudNode):
    """
    Visual servoing follower that computes cmd_vel from ArUco marker pose.
    """
    def __init__(
        self,
        node_name: str = "optical_follower_node",
        target_standoff_m: float = 1.80,
        kp_z: float = 0.50,
        kd_z: float = 0.10,
        kp_x: float = 0.85,
        kd_x: float = 0.15
    ):
        super().__init__(node_name)
        self.target_standoff = target_standoff_m
        self.kp_z = kp_z
        self.kd_z = kd_z
        self.kp_x = kp_x
        self.kd_x = kd_x

        self.enabled: bool = False
        self.target_marker_id: Optional[int] = None
        self.last_ez: float = 0.0
        self.last_ex: float = 0.0
        self.last_time: float = time.time()
        self.settled_cycles: int = 0
        self.arrival_declared: bool = False

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, "/cloud/cmd_vel")
        self.pub_arrived = self.create_publisher(bool, "/cloud/navigation_arrived")

        # Subscriptions
        self.sub_pose = self.create_subscription(
            PoseStamped, "/cloud/detected_marker_pose", self._on_marker_pose
        )
        self.sub_id = self.create_subscription(
            int, "/cloud/detected_marker_id", self._on_marker_id
        )

    def set_target_marker(self, marker_id: int):
        self.target_marker_id = marker_id
        self.settled_cycles = 0
        self.arrival_declared = False
        self.enabled = True
        self.logger.info(f"Optical follower targeted marker ID {marker_id} at {self.target_standoff}m standoff.")

    def stop(self):
        self.enabled = False
        self.pub_cmd_vel.publish(Twist())

    def _on_marker_id(self, marker_id: int):
        self.current_visible_id = marker_id

    def _on_marker_pose(self, msg: PoseStamped):
        if not self.enabled:
            return
        if self.target_marker_id is not None and getattr(self, "current_visible_id", None) != self.target_marker_id:
            return

        now = time.time()
        dt = max(0.001, now - self.last_time)
        self.last_time = now

        # In camera optical frame:
        # X: lateral displacement (meters, positive = marker is to robot's right)
        # Z: longitudinal distance (meters, forward)
        z_dist = msg.pose.position.z
        x_lat = msg.pose.position.x

        e_z = z_dist - self.target_standoff
        e_x = x_lat

        de_z = (e_z - self.last_ez) / dt
        de_x = (e_x - self.last_ex) / dt

        self.last_ez = e_z
        self.last_ex = e_x

        # Check arrival condition
        if abs(e_z) < 0.08 and abs(e_x) < 0.06:
            self.settled_cycles += 1
            if self.settled_cycles >= 4 and not self.arrival_declared:
                self.arrival_declared = True
                self.logger.info(f"Standoff achieved! Dist: {z_dist:.2f}m, Offset: {x_lat:.3f}m. Marking ARRIVED.")
                self.pub_cmd_vel.publish(Twist())
                self.pub_arrived.publish(True)
                return
        else:
            self.settled_cycles = 0
            self.arrival_declared = False

        # Longitudinal velocity control (vx)
        vx = self.kp_z * e_z + self.kd_z * de_z
        vx = max(-0.25, min(0.40, vx))

        # Angular yaw velocity control (wz)
        # If marker is to the right (+x), robot must turn right (-wz in ROS convention)
        wz = -(self.kp_x * e_x + self.kd_x * de_x)
        wz = max(-0.80, min(0.80, wz))

        cmd = Twist(
            linear=Vector3(x=vx, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=wz)
        )
        self.pub_cmd_vel.publish(cmd)
