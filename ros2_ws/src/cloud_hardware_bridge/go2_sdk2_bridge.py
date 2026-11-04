#!/usr/bin/env python3
"""
Project Cloud: Production Unitree SDK2 Hardware Bridge
Platform: Unitree Go2 Edu Quadruped (CycloneDDS RPC)
Subscribes to:
  - /cmd_vel_safe (geometry_msgs/Twist)
  - /collision_monitor_state (nav2_msgs/CollisionMonitorState or std_msgs/Int32)
  - /go2_posture_cmd (std_msgs/String: STAND_UP, BALANCE_STAND, STAND_DOWN, DAMP)
  - /go2_body_euler (geometry_msgs/Vector3: roll, pitch, yaw)
Features:
  - Auto-resets presentation Euler tilt on transit when non-zero cmd_vel arrives
  - Deadband filtering against stepping jitter (|vx|<0.02, |vy|<0.02, |wz|<0.03)
  - E-Stop low-CG bracing on 0.65m perimeter breach
  - Automatic SIL fallback if CycloneDDS physical interface / unitree_sdk2py is unlinked
"""

import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String, Int32

# Attempt Unitree SDK2 Python bindings
HAS_UNITREE_SDK2 = False
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    HAS_UNITREE_SDK2 = True
except (ImportError, Exception):
    HAS_UNITREE_SDK2 = False

# Attempt nav2_msgs for CollisionMonitorState
try:
    from nav2_msgs.msg import CollisionMonitorState
    HAS_COLLISION_MSG = True
except ImportError:
    HAS_COLLISION_MSG = False


class Go2Sdk2HardwareBridge(Node):
    """
    ROS 2 to Unitree SDK2 SportClient Hardware Bridge.
    Executes a 50 Hz deterministic RPC loop.
    """
    def __init__(self, node_name: str = "go2_sdk2_hardware_bridge"):
        super().__init__(node_name)

        self.declare_parameter("network_interface", "eth0")
        self.declare_parameter("cmd_vel_timeout_sec", 0.25)
        self.declare_parameter("force_mock", False)

        net_iface = self.get_parameter("network_interface").value
        self.cmd_timeout = self.get_parameter("cmd_vel_timeout_sec").value
        force_mock = self.get_parameter("force_mock").value

        self.is_mock_mode = force_mock or (not HAS_UNITREE_SDK2)
        self.sport_client = None

        if not self.is_mock_mode:
            try:
                ChannelFactoryInitialize(0, net_iface)
                self.sport_client = SportClient(enableLease=False)
                self.sport_client.SetTimeout(5.0)
                self.sport_client.Init()
                self.sport_client.AutoRecoverySet(True)
                self.get_logger().info(f"Go2 SDK2 SportClient Bridge online via CycloneDDS on {net_iface}.")
            except Exception as e:
                self.get_logger().warn(f"Failed to bind physical interface '{net_iface}': {e}. Operating in SIL fallback mode.")
                self.is_mock_mode = True
        else:
            self.get_logger().info("Go2 Hardware Bridge active in Software-in-the-Loop (SIL) mock mode.")

        # State Variables
        self.is_estopped = False
        self.in_locomotion_mode = True
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.current_yaw = 0.0
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vyaw = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Subscriptions
        self.create_subscription(Twist, "/cmd_vel_safe", self.cmd_vel_cb, 10)
        self.create_subscription(String, "/go2_posture_cmd", self.posture_cmd_cb, 10)
        self.create_subscription(Vector3, "/go2_body_euler", self.euler_cmd_cb, 10)

        if HAS_COLLISION_MSG:
            self.create_subscription(
                CollisionMonitorState, "/collision_monitor_state", self.collision_monitor_cb, 10
            )
        else:
            self.create_subscription(
                Int32, "/collision_monitor_state", self.collision_int_cb, 10
            )

        # 50 Hz deterministic RPC command dispatch timer
        self.timer = self.create_timer(0.02, self.control_loop)

    def collision_monitor_cb(self, msg):
        # Action type 1 == STOP (0.65m intrusion breached)
        is_stop = getattr(msg, "action_type", 0) == 1
        self._handle_collision_state(is_stop)

    def collision_int_cb(self, msg: Int32):
        is_stop = msg.data == 1
        self._handle_collision_state(is_stop)

    def _handle_collision_state(self, is_stop: bool):
        if is_stop and not self.is_estopped:
            self.is_estopped = True
            self.get_logger().warn("E-STOP (<0.65m intrusion)! Executing low-CG brace.")
            if self.sport_client:
                self.sport_client.StopMove()
                self.sport_client.BalanceStand()
                self.sport_client.Euler(0.0, 0.0, 0.0)
        elif not is_stop and self.is_estopped:
            self.is_estopped = False
            self.get_logger().info("Safety perimeter clear. Resuming nominal operation.")

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        vx = float(max(-0.30, min(0.75, msg.linear.x)))
        vy = float(max(-0.35, min(0.35, msg.linear.y)))
        vyaw = float(max(-1.00, min(1.00, msg.angular.z)))

        is_moving = abs(vx) >= 0.02 or abs(vy) >= 0.02 or abs(vyaw) >= 0.03

        # AUTO-RESET EULER TILT ON TRANSIT:
        # If in BALANCE_STAND with non-zero presentation pitch, reset to neutral Euler(0,0,0)
        # and re-enable locomotion mode so Go2 immediately walks smoothly without dragging.
        if is_moving and (not self.in_locomotion_mode or abs(self.current_pitch) > 0.01):
            if abs(self.current_pitch) > 0.01 or abs(self.current_roll) > 0.01:
                self.get_logger().info("Transit initiated: Auto-resetting presentation Euler tilt to (0,0,0).")
                self.current_pitch = 0.0
                self.current_roll = 0.0
                self.current_yaw = 0.0
                if self.sport_client:
                    self.sport_client.Euler(0.0, 0.0, 0.0)
            self.in_locomotion_mode = True

        self.target_vx = vx
        self.target_vy = vy
        self.target_vyaw = vyaw

    def posture_cmd_cb(self, msg: String):
        cmd = msg.data.upper().strip()
        self.get_logger().info(f"Received Go2 posture command: {cmd}")
        if cmd == "STAND_UP":
            if self.sport_client:
                self.sport_client.StandUp()
                time.sleep(1.0)
                self.sport_client.BalanceStand()
            self.in_locomotion_mode = True
        elif cmd == "BALANCE_STAND":
            if self.sport_client:
                self.sport_client.StopMove()
                self.sport_client.BalanceStand()
            self.in_locomotion_mode = False
        elif cmd == "STAND_DOWN":
            self.in_locomotion_mode = False
            if self.sport_client:
                self.sport_client.StandDown()
        elif cmd == "DAMP":
            self.in_locomotion_mode = False
            if self.sport_client:
                self.sport_client.Damp()

    def euler_cmd_cb(self, msg: Vector3):
        """Commands torso roll, pitch, yaw (e.g. pitch = -0.14 rad for +8 deg eye contact)."""
        if not self.is_estopped and not self.in_locomotion_mode:
            self.current_roll = float(msg.x)
            self.current_pitch = float(msg.y)
            self.current_yaw = float(msg.z)
            if self.sport_client:
                self.sport_client.Euler(self.current_roll, self.current_pitch, self.current_yaw)

    def control_loop(self):
        if self.is_estopped or not self.in_locomotion_mode:
            return

        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            if self.sport_client:
                self.sport_client.StopMove()
            return

        # Deadband check against in-place micro-stepping jitter
        if (
            abs(self.target_vx) < 0.02
            and abs(self.target_vy) < 0.02
            and abs(self.target_vyaw) < 0.03
        ):
            if self.sport_client:
                self.sport_client.StopMove()
        else:
            if self.sport_client:
                self.sport_client.Move(self.target_vx, self.target_vy, self.target_vyaw)


def main(args=None):
    rclpy.init(args=args)
    node = Go2Sdk2HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
