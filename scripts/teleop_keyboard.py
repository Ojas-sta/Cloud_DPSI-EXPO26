#!/usr/bin/env python3
"""
Project Cloud: Interactive Keyboard Teleoperation & Manual Override (Plan B)
Platform: Unitree Go2 Edu Quadruped | Venue: DPSI EXPO-2026

Controls:
  W / S : Drive forward / backward (linear.x)
  A / D : Strafe left / right (linear.y)
  Q / E : Turn left / right (angular.z)
  Up / Down Arrow : Pitch nose up / down (eye-contact angle)
  B : Stance lock (BALANCE_STAND)
  P : Prone / Sit (SIT)
  L : Locomotion stand (STAND)
  Spacebar : Emergency Stop (zero all velocities)
  CTRL-C / X : Exit teleoperation
"""

import sys
import select
import termios
import tty
import time
import os

cur_dir = os.path.dirname(os.path.abspath(__file__))
base_dir = os.path.dirname(cur_dir)
src_dir = os.path.join(base_dir, "ros2_ws", "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String


BANNER = """
================================================================================
  🎮 PROJECT CLOUD: INTERACTIVE KEYBOARD TELEOPERATION (PLAN B)
================================================================================
  Locomotion:                     Posture & Pitch:
    [W] Forward    (+0.40 m/s)      [B] Balance Stand (Pose Mode)
    [S] Reverse    (-0.40 m/s)      [L] Normal Stand (Walk Mode)
    [A] Strafe L   (+0.30 m/s)      [P] Prone / Sit
    [D] Strafe R   (-0.30 m/s)      [↑] Pitch Nose Up (+0.04 rad)
    [Q] Yaw Left   (+0.60 rad/s)    [↓] Pitch Nose Down (-0.04 rad)
    [E] Yaw Right  (-0.60 rad/s)    [0] Reset Pitch (0.0 rad)

  Safety:
    [SPACE] EMERGENCY STOP (Zero velocity & stance lock)
    [X]     Exit Teleoperation
================================================================================
"""


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('cloud_keyboard_teleop')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.pub_posture = self.create_publisher(String, '/go2_posture_cmd', 10)
        self.pub_euler = self.create_publisher(Vector3, '/go2_body_euler', 10)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.pitch = 0.0

        self.lin_step = 0.35
        self.strafe_step = 0.25
        self.ang_step = 0.55
        self.pitch_step = 0.04

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = float(self.vx)
        msg.linear.y = float(self.vy)
        msg.angular.z = float(self.wz)
        self.pub_cmd_vel.publish(msg)

    def publish_posture(self, posture: str):
        msg = String()
        msg.data = posture
        self.pub_posture.publish(msg)

    def publish_pitch(self):
        v = Vector3()
        v.x = 0.0
        v.y = float(self.pitch)
        v.z = 0.0
        self.pub_euler.publish(v)

    def stop(self):
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.publish_cmd()


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # Escape sequence for arrow keys
            extra = sys.stdin.read(2)
            key += extra
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardTeleopNode()

    print(BANNER)
    print("  Status: Teleop Node active on /cmd_vel_safe. Press keys to drive...")

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.05)
            if not key:
                continue

            if key in ('w', 'W'):
                node.vx = min(0.60, node.vx + node.lin_step)
            elif key in ('s', 'S'):
                node.vx = max(-0.40, node.vx - node.lin_step)
            elif key in ('a', 'A'):
                node.vy = min(0.40, node.vy + node.strafe_step)
            elif key in ('d', 'D'):
                node.vy = max(-0.40, node.vy - node.strafe_step)
            elif key in ('q', 'Q'):
                node.wz = min(1.0, node.wz + node.ang_step)
            elif key in ('e', 'E'):
                node.wz = max(-1.0, node.wz - node.ang_step)
            elif key == ' ':
                node.stop()
                print("  🛑 [ESTOP] Velocity zeroed.")
            elif key in ('\x1b[A', 'k'):  # Up arrow
                node.pitch = max(-0.35, node.pitch - node.pitch_step)
                node.publish_pitch()
                print(f"  📐 Pitch tilt: {math.degrees(-node.pitch):+.1f}°")
            elif key in ('\x1b[B', 'j'):  # Down arrow
                node.pitch = min(0.35, node.pitch + node.pitch_step)
                node.publish_pitch()
                print(f"  📐 Pitch tilt: {math.degrees(-node.pitch):+.1f}°")
            elif key == '0':
                node.pitch = 0.0
                node.publish_pitch()
                print("  📐 Pitch reset to 0.0°")
            elif key in ('b', 'B'):
                node.publish_posture("BALANCE_STAND")
                print("  🦿 Posture set: BALANCE_STAND")
            elif key in ('l', 'L'):
                node.publish_posture("STAND")
                print("  🦿 Posture set: STAND (Walk)")
            elif key in ('p', 'P'):
                node.publish_posture("SIT")
                print("  🦿 Posture set: SIT (Prone)")
            elif key in ('x', 'X', '\x03'):  # Exit
                break

            node.publish_cmd()
            print(f"  🏎️  cmd_vel: vx={node.vx:+.2f} m/s, vy={node.vy:+.2f} m/s, wz={node.wz:+.2f} rad/s")

    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n  Teleoperation session ended safely.")


if __name__ == '__main__':
    main()
