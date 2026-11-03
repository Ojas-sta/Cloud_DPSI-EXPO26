"""
Project Cloud: Software-in-the-Loop (SIL) Mock Quadruped Hardware Bridge
Simulates Unitree Go2 Edu kinematics, telemetry, odometry, battery,
and tactile console button for headless local testing and development.
"""

import math
import time
from typing import Optional
from cloud_core.bus import CloudNode
from cloud_core.types import (
    Twist,
    PoseStamped,
    Point,
    Quaternion,
    RobotStateReport,
    LEDCommand,
    TimeStamp
)

def euler_to_quaternion(yaw: float, pitch: float = 0.0, roll: float = 0.0) -> Quaternion:
    """Converts Euler angles in radians to unit quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return Quaternion(x=x, y=y, z=z, w=w)


class MockGo2Robot(CloudNode):
    """
    High-fidelity SIL simulation of the Unitree Go2 quadruped.
    Runs at 20 Hz, integrating velocity commands and broadcasting state reports.
    """
    def __init__(self, node_name: str = "mock_go2_robot", dt: float = 0.05):
        super().__init__(node_name)
        self.dt = dt

        # State Variables
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0
        self.pitch: float = 0.0
        self.roll: float = 0.0
        self.body_height: float = 0.32  # meters

        self.cmd_linear_x: float = 0.0
        self.cmd_linear_y: float = 0.0
        self.cmd_angular_z: float = 0.0

        self.battery_pct: float = 98.0
        self.battery_voltage: float = 25.2
        self.active_persona: str = "THE_DOCENT"
        self.fsm_state: str = "STATE_IDLE_STANDBY"
        self.active_landmark: str = "NODE_00"
        self.emergency_stop: bool = False
        self.last_led_cmd: Optional[LEDCommand] = None

        # Publishers
        self.pub_odom = self.create_publisher(PoseStamped, "/cloud/odom")
        self.pub_state = self.create_publisher(RobotStateReport, "/cloud/robot_state")
        self.pub_button = self.create_publisher(bool, "/cloud/console_button")

        # Subscriptions
        self.sub_cmd_vel = self.create_subscription(Twist, "/cloud/cmd_vel", self._on_cmd_vel)
        self.sub_led = self.create_subscription(LEDCommand, "/cloud/led_cmd", self._on_led_cmd)

        # Physics & Telemetry Timer Loop (20 Hz)
        self.timer = self.create_timer(self.dt, self._update_physics)
        self.logger.info("Mock Go2 Quadruped SIL initialized at 20 Hz.")

    def _on_cmd_vel(self, msg: Twist):
        if self.emergency_stop:
            self.cmd_linear_x = 0.0
            self.cmd_linear_y = 0.0
            self.cmd_angular_z = 0.0
            return
        # Clip max velocity to safe limits (0.6 m/s linear, 1.2 rad/s angular)
        self.cmd_linear_x = max(-0.4, min(0.6, msg.linear.x))
        self.cmd_linear_y = max(-0.3, min(0.3, msg.linear.y))
        self.cmd_angular_z = max(-1.2, min(1.2, msg.angular.z))

    def _on_led_cmd(self, msg: LEDCommand):
        self.last_led_cmd = msg
        self.logger.debug(f"LED State Changed: Mode={msg.mode}, RGB=({msg.red},{msg.green},{msg.blue})")

    def press_console_button(self):
        """Simulates physical green button press on the rear console."""
        self.logger.info("TACTILE BUTTON PRESSED: Emitting push-to-proceed event.")
        self.pub_button.publish(True)

    def trigger_e_stop(self, active: bool = True):
        self.emergency_stop = active
        if active:
            self.cmd_linear_x = 0.0
            self.cmd_linear_y = 0.0
            self.cmd_angular_z = 0.0
            self.logger.warn("EMERGENCY STOP TRIGGERED: Quadruped frozen.")
        else:
            self.logger.info("Emergency stop cleared.")

    def set_posture(self, pitch_deg: float, body_height_m: float):
        """Adjusts quadruped stance (e.g. for presenter pitch and eye contact)."""
        self.pitch = math.radians(pitch_deg)
        self.body_height = body_height_m
        self.logger.info(f"Posture adapted: Pitch={pitch_deg:.1f} deg, Height={body_height_m:.2f} m")

    def _update_physics(self):
        # Kinematic integration
        delta_yaw = self.cmd_angular_z * self.dt
        self.yaw = (self.yaw + delta_yaw) % (2.0 * math.pi)

        # Transform local body velocities to world frame
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        vx_world = self.cmd_linear_x * cos_y - self.cmd_linear_y * sin_y
        vy_world = self.cmd_linear_x * sin_y + self.cmd_linear_y * cos_y

        self.x += vx_world * self.dt
        self.y += vy_world * self.dt

        # Battery decay simulation (slow idle, faster when moving)
        motion_factor = abs(self.cmd_linear_x) + abs(self.cmd_angular_z)
        drain = (0.0005 + motion_factor * 0.002) * self.dt
        self.battery_pct = max(0.0, self.battery_pct - drain)
        self.battery_voltage = 20.0 + (self.battery_pct / 100.0) * 5.2

        # Publish Odometry
        q = euler_to_quaternion(self.yaw, self.pitch, self.roll)
        odom = PoseStamped(
            stamp=TimeStamp.now(),
            frame_id="odom",
            pose=None
        )
        from cloud_core.types import Pose
        odom.pose = Pose(
            position=Point(x=self.x, y=self.y, z=self.body_height),
            orientation=q
        )
        self.pub_odom.publish(odom)

        # Publish Full Robot State
        report = RobotStateReport(
            stamp=TimeStamp.now(),
            active_persona=self.active_persona,
            fsm_state=self.fsm_state,
            active_landmark_id=self.active_landmark,
            battery_percentage=round(self.battery_pct, 2),
            battery_voltage=round(self.battery_voltage, 2),
            current_pose=odom.pose,
            emergency_stop_active=self.emergency_stop,
            push_button_ready=True
        )
        self.pub_state.publish(report)
