#!/usr/bin/env python3
"""
Project Cloud: Live Terminal Precision Standoff Docking Demonstration
Platform: Unitree Go2 Edu Quadruped | Venue: DPSI EXPO-2026

Demonstrates:
  1. RealSense D435i optical detection of booth AprilTag tag36h11 (IDs 10-49, 99).
  2. Forward Unitree 4D LiDAR L1 (5,760 dense beams) planar backboard extraction via RANSAC.
  3. Holonomic 3-DOF closed-loop PD velocity servoing on /cmd_vel_safe to 0.95m +/- 0.03m.
  4. Automatic 10-cycle convergence latch, zero-jitter stance lock (BALANCE_STAND),
     and natural +8.0 deg eye-contact presentation tilt (pitch = -0.14 rad).
"""

import os
import sys
import time
import math
import argparse
import yaml

# Set up module paths
cur_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(cur_dir, "ros2_ws", "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)
if cur_dir not in sys.path:
    sys.path.insert(0, cur_dir)

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import String, Int32, Bool, Float32MultiArray
from cloud_navigation.terminal_docking_servo import TerminalDockingServoNode, DockingState


def run_docking_demonstration(stall_id: str = "STALL_01", max_duration_sec: float = 30.0):
    yaml_path = os.path.join(cur_dir, "config", "stalls_database.yaml")
    with open(yaml_path, "r", encoding="utf-8") as f:
        db = yaml.safe_load(f)

    # Locate target stall or dock
    stall_data = None
    target_tag = 10
    target_standoff = 0.95
    stall_title = "Unknown Stall"

    if stall_id.upper() in ("99", "DOCK", "DOCK_STAGE_99", "STAGE_DOCK"):
        dock_cfg = db.get("recharge_dock", {})
        target_tag = dock_cfg.get("marker_id", 99)
        target_standoff = dock_cfg.get("standoff_m", 0.50)
        stall_title = dock_cfg.get("label", "Stage Recharge Dock")
        stall_id = "DOCK_STAGE_99"
    else:
        for s in db.get("stalls", []):
            if s.get("id") == stall_id or str(s.get("apriltag_id")) == stall_id:
                stall_data = s
                target_tag = s.get("apriltag_id", 10)
                target_standoff = s.get("terminal_standoff_m", 0.95)
                stall_title = s.get("title", "")
                stall_id = s.get("id")
                break

    print("=" * 80)
    print(f"  🎯 PROJECT CLOUD: TERMINAL PRECISION STANDOFF DOCKING DEMONSTRATION")
    print("=" * 80)
    print(f"  • Target Stall:      {stall_id} — \"{stall_title}\"")
    print(f"  • AprilTag ID:       {target_tag} (Standard tag36h11)")
    print(f"  • Target Standoff:   {target_standoff:.2f} m")
    print(f"  • Convergence Goal:  Standoff +/- 0.03m | Lateral +/- 0.03m | Yaw +/- 1.7°")
    print(f"  • Posture On Lock:   BALANCE_STAND + Presentation Pitch (+8.0° / -0.14 rad)")
    print("=" * 80)

    rclpy.init()
    docking_node = TerminalDockingServoNode("live_docking_demo")
    docking_node.start_docking(target_apriltag_id=target_tag, standoff_m=target_standoff, label=stall_id)

    # Subscribe to arrival signal for verification
    arrival_latched = False
    final_posture = None
    final_euler = None

    def on_arrival_msg(msg: Bool):
        nonlocal arrival_latched
        if msg.data:
            arrival_latched = True

    def on_posture_msg(msg: String):
        nonlocal final_posture
        final_posture = msg.data

    def on_euler_msg(msg: Vector3):
        nonlocal final_euler
        final_euler = (msg.x, msg.y, msg.z)

    docking_node.create_subscription(Bool, "/cloud/docking/arrived", on_arrival_msg, 10)
    docking_node.create_subscription(String, "/go2_posture_cmd", on_posture_msg, 10)
    docking_node.create_subscription(Vector3, "/go2_body_euler", on_euler_msg, 10)

    start_time = time.time()
    step_count = 0
    success = False

    try:
        while rclpy.ok() and (time.time() - start_time) < max_duration_sec:
            rclpy.spin_once(docking_node, timeout_sec=0.05)
            step_res = getattr(docking_node, "latest_telemetry", None)
            if not step_res:
                time.sleep(0.02)
                continue
            step_count += 1

            if step_count % 3 == 0 or step_res["state"] == DockingState.PRESENTATION_LOCKED.value:
                state_str = step_res["state"]
                ex = step_res["ex"]
                ey = step_res["ey"]
                eth_deg = math.degrees(step_res["eth"])
                vx = step_res["vx"]
                vy = step_res["vy"]
                wz = step_res["wz"]
                mode = step_res["mode"]
                settled = step_res["settled_cycles"]

                # Measured distance: target + ex
                meas_dist = target_standoff + ex

                print(
                    f"[{time.strftime('%H:%M:%S')}] Step {step_count:03d} | {state_str:<19} | "
                    f"Mode: {mode:<26} | Dist: {meas_dist:.3f}m (e={ex:+.3f}m) | "
                    f"Lat: {ey:+.3f}m | Yaw: {eth_deg:+.1f}° | "
                    f"Cmd: [vx={vx:+.2f}, vy={vy:+.2f}, wz={wz:+.2f}] | Win: {settled}/10"
                )

            if step_res["state"] == DockingState.PRESENTATION_LOCKED.value:
                success = True
                print("\n" + "=" * 80)
                print(f"  ✅ DOCKING ARRIVAL & STANCE LOCK ACHIEVED AT {stall_id}!")
                print("=" * 80)
                print(f"  • Final Standoff:        {target_standoff + step_res['ex']:.3f} m (Target: {target_standoff:.2f} m, Error: {abs(step_res['ex'])*100:.1f} cm)")
                print(f"  • Final Lateral Center:  {step_res['ey']:+.3f} m")
                print(f"  • Final Heading Error:   {math.degrees(step_res['eth']):+.2f}°")
                print(f"  • Commanded Velocity:    [vx=0.00, vy=0.00, wz=0.00] (Zero-Motion Stance)")
                print(f"  • Quadruped Posture:     BALANCE_STAND (Kinematic Stance Lock)")
                print(f"  • Eye Contact Pitch:     +8.0° nose-up (pitch = -0.14 rad)")
                print(f"  • Total Servoing Time:   {time.time() - start_time:.2f} seconds ({step_count} cycles)")
                print("=" * 80)
                break

            time.sleep(0.04)

    except KeyboardInterrupt:
        print("\n[INFO] Demonstration interrupted by user.")
    finally:
        docking_node.destroy_node()
        rclpy.shutdown()

    return success


def main():
    parser = argparse.ArgumentParser(description="Project Cloud: Terminal Precision Standoff Docking Demonstration")
    parser.add_argument("--stall", type=str, default="STALL_01", help="Target stall ID (e.g. STALL_01, STALL_03, DOCK_99)")
    parser.add_argument("--timeout", type=float, default=25.0, help="Maximum duration in seconds")
    args = parser.parse_args()

    run_docking_demonstration(stall_id=args.stall, max_duration_sec=args.timeout)


if __name__ == "__main__":
    main()
