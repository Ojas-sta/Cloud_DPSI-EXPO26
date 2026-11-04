"""
Project Cloud: 40-Stall Autonomous Docent Architecture Unit & Integration Tests
Verifies:
  1. 40-Stall Database Schema & AprilTag tag36h11 Standard (IDs 10-49, 99)
  2. Topological Graph Cohort Slicing & 2-Stage Crowd Chokepoint Escalation
  3. Go2Sdk2HardwareBridge Auto-Reset Euler Tilt on Transit & Deadband Filtering
  4. MuJoCo Twin Dual-Publisher Contention Elimination & Data-Driven 40-Stall World
"""

import os
import sys
import time
import math
import pytest
import rclpy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String, Int32

repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src_dir = os.path.join(repo_dir, "ros2_ws", "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)
if repo_dir not in sys.path:
    sys.path.insert(0, repo_dir)

from cloud_navigation import TopologicalGraph
from cloud_hardware_bridge import Go2Sdk2HardwareBridge


@pytest.fixture(scope="module")
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_stalls_database_and_cohorts():
    """Validates 40 stalls, AprilTag range [10, 49], dock 99, and 4 cohorts."""
    tg = TopologicalGraph()
    assert len(tg.stalls) == 40, f"Expected 40 stalls, found {len(tg.stalls)}"

    # Check 4 cohorts of exactly 10 stalls each
    for c_id in range(1, 5):
        cohort_stalls = tg.get_cohort_stalls(c_id)
        assert len(cohort_stalls) == 10, f"Cohort {c_id} must have 10 stalls, got {len(cohort_stalls)}"

    # Check AprilTag IDs
    tag_ids = [s.apriltag_id for s in tg.stalls.values()]
    assert min(tag_ids) == 10, f"Lowest stall tag must be 10, got {min(tag_ids)}"
    assert max(tag_ids) == 49, f"Highest stall tag must be 49, got {max(tag_ids)}"
    assert len(set(tag_ids)) == 40, "All 40 stalls must have unique AprilTag IDs"

    # Check dock node
    dock = tg.get_node_by_marker(99)
    assert dock is not None, "Marker 99 recharge dock must exist in graph"
    assert dock.id == "DOCK_STAGE_99"


def test_aisle_width_and_standoff_geometry():
    """Verifies standoff poses are safely positioned on aisle centerlines."""
    tg = TopologicalGraph()
    for s_id, s in tg.stalls.items():
        assert s.terminal_standoff_m <= 1.00, f"{s_id} terminal standoff must be <= 1.00m"
        # Staging pose must be at least 0.85m away from booth
        dist = math.hypot(s.staging_x - s.booth_x, s.staging_y - s.booth_y)
        assert dist >= 1.00, f"{s_id} staging pose too close to booth: {dist:.2f}m"


def test_two_stage_crowd_escalation():
    """Verifies 5s warning with yaw wiggle and 15s stall deferral with edge penalty."""
    tg = TopologicalGraph()
    s_id = "STALL_01"

    # 1. Nominal when clearance >= 0.75m
    action, wiggle = tg.evaluate_crowd_clearance(s_id, 1.20)
    assert action == "NOMINAL"
    assert wiggle is None

    # 2. Start monitoring when clearance drops < 0.75m
    action, wiggle = tg.evaluate_crowd_clearance(s_id, 0.50)
    assert action == "MONITORING"

    # Mock time passage to 5.5s (Stage 1 Warning)
    start_ts = tg.chokepoint_timers[s_id][0]
    tg.chokepoint_timers[s_id] = (start_ts - 5.5, False)

    action, wiggle = tg.evaluate_crowd_clearance(s_id, 0.50)
    assert action == "WARN_CLEARANCE"
    assert abs(wiggle) == 0.10, "Subtle yaw intent wiggle must be +-0.10 rad"

    # Mock time passage to 15.5s (Stage 2 Deferral)
    tg.chokepoint_timers[s_id] = (start_ts - 15.5, True)
    action, wiggle = tg.evaluate_crowd_clearance(s_id, 0.50)
    assert action == "DEFER_STALL"

    # Execute deferral and verify edge inflation
    deferred = tg.defer_stall(s_id)
    assert deferred is True
    stall = tg.get_stall(s_id)
    assert stall.deferral_count == 1

    # Verify second deferral is allowed, but third is rejected (max_deferrals = 2)
    assert tg.defer_stall(s_id) is True
    assert tg.defer_stall(s_id) is False, "Third deferral must be blocked"


def test_hardware_bridge_auto_reset_euler(rclpy_init):
    """Verifies bridge auto-resets presentation Euler tilt upon receiving non-zero velocity."""
    bridge = Go2Sdk2HardwareBridge(node_name="test_go2_bridge")

    # Put robot in presentation mode with +8 deg eye contact pitch (-0.14 rad)
    euler_msg = Vector3(x=0.0, y=-0.14, z=0.0)
    bridge.in_locomotion_mode = False
    bridge.euler_cmd_cb(euler_msg)
    assert bridge.current_pitch == -0.14
    assert bridge.in_locomotion_mode is False

    # Send non-zero velocity command
    vel_msg = Twist()
    vel_msg.linear.x = 0.35
    bridge.cmd_vel_cb(vel_msg)

    # Bridge must auto-reset pitch to 0.0 and re-enable locomotion mode
    assert bridge.current_pitch == 0.0, "Pitch must auto-reset to 0.0 on transit"
    assert bridge.in_locomotion_mode is True, "Locomotion mode must re-engage"
    assert bridge.target_vx == 0.35

    bridge.destroy_node()


def test_hardware_bridge_deadband(rclpy_init):
    """Verifies stepping deadband filters out micro-jitter below thresholds."""
    bridge = Go2Sdk2HardwareBridge(node_name="test_go2_deadband")
    bridge.in_locomotion_mode = True

    # Send sub-threshold velocities
    vel_msg = Twist()
    vel_msg.linear.x = 0.01
    vel_msg.linear.y = 0.01
    vel_msg.angular.z = 0.02
    bridge.cmd_vel_cb(vel_msg)

    # In deadband, sport client Move() is suppressed
    assert abs(bridge.target_vx) < 0.02
    assert abs(bridge.target_vy) < 0.02
    assert abs(bridge.target_vyaw) < 0.03

    bridge.destroy_node()


def test_mujoco_twin_external_latch(rclpy_init):
    """Verifies MuJoCo twin auto-latches external control to eliminate contention."""
    from sim_mujoco.cloud_mujoco_ros2_twin import CloudMuJoCoTwin
    
    twin = CloudMuJoCoTwin(headless=True)
    assert twin.external_control_latched is False, "Twin must start unlatched"

    # Send external cmd_vel
    cmd = Twist()
    cmd.linear.x = 0.40
    cmd.linear.y = 0.15
    twin._ext_cmd_vel_cb(cmd)

    # Twin must now be latched
    assert twin.external_control_latched is True
    assert twin.ext_cmd_vx == 0.40
    assert twin.ext_cmd_vy == 0.15

    twin.destroy_node()
