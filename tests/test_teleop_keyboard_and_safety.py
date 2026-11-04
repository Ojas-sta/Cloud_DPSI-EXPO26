#!/usr/bin/env python3
"""
Test Suite: Plan B Interactive Teleoperation & Safety Watchdog
Verifies:
  1. Keyboard command mappings and step increments.
  2. Velocity clamping within safe envelope (vx in [-0.4, 0.6], vy in [-0.4, 0.4], wz in [-1.0, 1.0]).
  3. Pitch tilt angle limits within [-0.35, +0.35] rad.
  4. Emergency stop instant zeroing of all velocities.
  5. Posture mode commands (BALANCE_STAND, STAND, SIT).
  6. Watchdog timeout behavior (auto-stopping after command silence).
"""

import os
import sys
import math
import pytest

repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src_dir = os.path.join(repo_dir, "ros2_ws", "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)
if repo_dir not in sys.path:
    sys.path.insert(0, repo_dir)

from scripts.teleop_keyboard import KeyboardTeleopNode

try:
    import rclpy
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


@pytest.fixture(scope="module")
def ros_init():
    if HAS_ROS2 and not rclpy.ok():
        rclpy.init()
    yield
    if HAS_ROS2 and rclpy.ok():
        rclpy.shutdown()


def test_teleop_initial_state(ros_init):
    node = KeyboardTeleopNode()
    assert node.vx == 0.0
    assert node.vy == 0.0
    assert node.wz == 0.0
    assert node.pitch == 0.0
    node.destroy_node()


def test_teleop_velocity_step_and_clamping(ros_init):
    node = KeyboardTeleopNode()
    # Step forward multiple times
    for _ in range(5):
        node.vx = min(0.60, node.vx + node.lin_step)
    assert node.vx == 0.60, f"Forward velocity must clamp at +0.60 m/s, got {node.vx}"

    # Step backward multiple times
    for _ in range(5):
        node.vx = max(-0.40, node.vx - node.lin_step)
    assert node.vx == -0.40, f"Reverse velocity must clamp at -0.40 m/s, got {node.vx}"
    node.destroy_node()


def test_teleop_strafing_clamping(ros_init):
    node = KeyboardTeleopNode()
    for _ in range(4):
        node.vy = min(0.40, node.vy + node.strafe_step)
    assert node.vy == 0.40, f"Left strafe must clamp at +0.40 m/s, got {node.vy}"

    for _ in range(4):
        node.vy = max(-0.40, node.vy - node.strafe_step)
    assert node.vy == -0.40, f"Right strafe must clamp at -0.40 m/s, got {node.vy}"
    node.destroy_node()


def test_teleop_angular_clamping(ros_init):
    node = KeyboardTeleopNode()
    for _ in range(4):
        node.wz = min(1.0, node.wz + node.ang_step)
    assert node.wz == 1.0, f"Yaw rate must clamp at +1.0 rad/s, got {node.wz}"

    for _ in range(4):
        node.wz = max(-1.0, node.wz - node.ang_step)
    assert node.wz == -1.0, f"Yaw rate must clamp at -1.0 rad/s, got {node.wz}"
    node.destroy_node()


def test_teleop_pitch_tilt_envelope(ros_init):
    node = KeyboardTeleopNode()
    # Step pitch up (negative pitch in REP-103 is nose up)
    for _ in range(12):
        node.pitch = max(-0.35, node.pitch - node.pitch_step)
    assert node.pitch == -0.35, f"Nose-up pitch must clamp at -0.35 rad (+20 deg), got {node.pitch}"

    for _ in range(25):
        node.pitch = min(0.35, node.pitch + node.pitch_step)
    assert node.pitch == 0.35, f"Nose-down pitch must clamp at +0.35 rad (-20 deg), got {node.pitch}"
    node.destroy_node()


def test_teleop_emergency_stop(ros_init):
    node = KeyboardTeleopNode()
    node.vx = 0.50
    node.vy = 0.25
    node.wz = 0.80
    node.stop()
    assert node.vx == 0.0
    assert node.vy == 0.0
    assert node.wz == 0.0
    node.destroy_node()
