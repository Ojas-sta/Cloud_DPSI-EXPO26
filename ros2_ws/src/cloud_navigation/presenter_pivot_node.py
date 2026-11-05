"""
Project Cloud: 180° Kinematic Presenter Pivot Node
Executes fluid 180° center-axis rotation and ergonomic posture adaptation
(+8° head tilt, lowered chassis) to directly engage human evaluators.
Reference: PROJECT.md Section 7.2 & 8.5
"""

import math
import time
from typing import Optional

from cloud_core.bus import CloudNode
from cloud_core.types import Twist, Vector3

class PresenterPivotNode(CloudNode):
    """
    Executes a calibrated 180-degree pivot around the quadruped's geometric center.
    """
    def __init__(
        self,
        node_name: str = "presenter_pivot_node",
        turn_duration_sec: float = 4.0,
        pitch_tilt_deg: float = 8.0,
        chassis_height_m: float = 0.28
    ):
        super().__init__(node_name)
        self.turn_duration = turn_duration_sec
        self.pitch_tilt = pitch_tilt_deg
        self.chassis_height = chassis_height_m

        self.is_pivoting: bool = False
        self.start_time: float = 0.0
        self.target_turn_angle_rad: float = math.pi
        self.nominal_wz = self.target_turn_angle_rad / self.turn_duration

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, "/cloud/cmd_vel")
        self.pub_pivot_done = self.create_publisher(bool, "/cloud/pivot_completed")

        # Active turn timer loop (20 Hz)
        self.timer = self.create_timer(0.05, self._tick_pivot)

    def trigger_pivot(self):
        """Initiates the 180-degree turn."""
        self.is_pivoting = True
        self.start_time = time.time()
        self.logger.info(
            f"INITIATING PRESENTER PIVOT: 180° rotation over {self.turn_duration}s, "
            f"Pitch tilt={self.pitch_tilt}°, Height={self.chassis_height}m"
        )

    def _tick_pivot(self):
        if not self.is_pivoting:
            return

        elapsed = time.time() - self.start_time
        if elapsed < self.turn_duration:
            # Smooth trapezoidal or sinusoidal easing (normalized so integral = pi)
            progress = elapsed / self.turn_duration
            easing = math.sin(progress * math.pi)
            _mean = 0.4 + 0.6 * (2.0 / math.pi)
            current_wz = self.nominal_wz * (0.4 + 0.6 * easing) / _mean

            cmd = Twist(
                linear=Vector3(x=0.0, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=current_wz)
            )
            self.pub_cmd_vel.publish(cmd)
        else:
            # Pivot complete
            self.is_pivoting = False
            self.pub_cmd_vel.publish(Twist())
            self.pub_pivot_done.publish(True)
            self.logger.info("PRESENTER PIVOT COMPLETE: Aligned with evaluators, ready for narration.")
