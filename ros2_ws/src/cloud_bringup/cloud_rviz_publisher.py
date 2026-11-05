"""
Project Cloud: High-Fidelity Quadruped Kinematics & Exhibition Mission Simulator
Platform: Unitree Go2 Edu Quadruped
Environment: ROS 2 Jazzy (macOS Pixi Robostack)

Features:
- Analytical 12-DOF Quadruped Inverse Kinematics (IK)
- 2-Beat Diagonal Trot Gait Generator (Cycloidal Swing / Ground Stance)
- Smooth Waypoint Navigation with Proportional Turning & Velocity Regulation
- Automatic Management of robot_state_publisher for Flawless TF Tree Resolution
- 30 Hz Telemetry & Interactive Exhibition Environmental Markers
"""

import sys
import os
import math
import time
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy
    )

def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def leg_inverse_kinematics(x_foot, y_foot, z_foot, l_thigh=0.213, l_calf=0.213):
    """
    Computes analytical 3-DOF joint angles (hip_roll, thigh_pitch, calf_pitch)
    for a single Go2 quadruped leg in hip-centered coordinates.
    """
    # 1. Hip Abduction / Roll
    hip = math.atan2(y_foot, -z_foot)
    
    # 2. Sagittal reach distance (hypotenuse in x-z plane)
    d = math.hypot(x_foot, z_foot)
    max_reach = (l_thigh + l_calf) * 0.985
    min_reach = abs(l_thigh - l_calf) * 1.05
    d = max(min_reach, min(max_reach, d))
    
    # 3. Knee / Calf Pitch (Law of Cosines)
    cos_beta = (l_thigh**2 + l_calf**2 - d**2) / (2.0 * l_thigh * l_calf)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    # Go2 convention: straight leg is 0, backward bend is negative
    calf = -(math.pi - beta)
    
    # 4. Thigh Pitch
    alpha = math.atan2(x_foot, -z_foot)
    cos_gamma = (l_thigh**2 + d**2 - l_calf**2) / (2.0 * l_thigh * d)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))
    gamma = math.acos(cos_gamma)
    thigh = alpha + gamma
    
    return hip, thigh, calf

class CloudRVizPublisher(Node):
    def __init__(self):
        super().__init__('cloud_rviz_publisher')

        repo_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        self.urdf_path = os.path.join(repo_dir, "ros2_ws", "src", "go2_description", "urdf", "go2_rviz.urdf")

        if not os.path.exists(self.urdf_path):
            self.get_logger().error(f"URDF missing at: {self.urdf_path}")
            urdf_content = ""
        else:
            with open(self.urdf_path, "r", encoding="utf-8") as f:
                urdf_content = f.read()

        # 1. Publish Robot Description with Transient Local QoS (latched)
        qos_latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub_description = self.create_publisher(String, '/robot_description', qos_latched)
        desc_msg = String()
        desc_msg.data = urdf_content
        self.pub_description.publish(desc_msg)
        self.get_logger().info("Published official Unitree Go2 URDF to /robot_description.")

        # Ensure robot_state_publisher is active to guarantee full TF tree resolution
        self._ensure_robot_state_publisher()

        # 2. Publishers for Joint States, Markers, Path, and TF
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/cloud/exhibition_markers', 10)
        self.pub_path = self.create_publisher(Path, '/cloud/odom_path', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Joint Names from official Unitree Go2 URDF
        self.joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]

        # Trajectory Path Container
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        # Robot Physical Dimensions & Stance Reference
        self.nominal_height = 0.345
        self.stance_h = 0.330
        self.step_clearance = 0.055
        self.max_step_len = 0.160

        # State Variables
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = self.nominal_height
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.gait_phase = 0.0

        # Mission Tour Scenes (Waypoints)
        self.scenes = [
            {"id": 1, "name": "SCENE 1: VIP GREETING", "fsm": "STATE_DOCENT_GREET", "x": 0.0, "y": 0.0, "yaw": 0.0, "hold": 6.0},
            {"id": 2, "name": "SCENE 2: ARGUS PATROL", "fsm": "STATE_ARGUS_PATROL", "x": 3.0, "y": 0.0, "yaw": 0.0, "hold": 4.5},
            {"id": 3, "name": "SCENE 3: SPILL HAZARD ALERT", "fsm": "STATE_HAZARD_ALERT", "x": 5.2, "y": 0.0, "yaw": 0.0, "hold": 5.5},
            {"id": 4, "name": "SCENE 4: ARUCO DOCKING", "fsm": "STATE_DOCENT_CRUISE", "x": 10.2, "y": 0.0, "yaw": 0.0, "hold": 5.0},
            {"id": 5, "name": "SCENE 5: 180° PRESENTER PIVOT", "fsm": "STATE_DOCENT_NARRATE", "x": 10.2, "y": 0.0, "yaw": math.pi, "hold": 6.0},
            {"id": 6, "name": "SCENE 6: STAGE & PRONE DOCK", "fsm": "STATE_PRONE_REST", "x": 0.0, "y": 0.0, "yaw": 0.0, "hold": 6.0}
        ]

        self.scene_idx = 0
        self.stage_timer = 0.0
        self.is_holding = True
        self.last_update = time.time()

        # Run 30 Hz Telemetry & Kinematics Loop
        self.timer = self.create_timer(1.0 / 30.0, self._update_loop)
        self.get_logger().info("Cloud Go2 Quadruped Gait Controller running at 30 Hz.")

    def _ensure_robot_state_publisher(self):
        """Spawns robot_state_publisher if not already active."""
        try:
            # Check if rsp is already active via pgrep
            chk = subprocess.run(["pgrep", "-f", "robot_state_publisher"], stdout=subprocess.PIPE, text=True)
            if not chk.stdout.strip():
                self.get_logger().info("Launching background robot_state_publisher for TF...")
                pixi_setup = "/Users/roopalisingh/robostack/.pixi/envs/jazzy/setup.zsh"
                cmd = f"source {pixi_setup} && exec ros2 run robot_state_publisher robot_state_publisher {self.urdf_path}"
                self.rsp_proc = subprocess.Popen(
                    ["zsh", "-c", cmd],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            else:
                self.get_logger().info("Existing robot_state_publisher detected.")
                self.rsp_proc = None
        except Exception as e:
            self.get_logger().warn(f"Could not auto-spawn robot_state_publisher: {e}")
            self.rsp_proc = None

    def _update_loop(self):
        now_time = time.time()
        dt = max(0.001, min(0.1, now_time - self.last_update))
        self.last_update = now_time

        scene = self.scenes[self.scene_idx]
        dx = scene["x"] - self.pos_x
        dy = scene["y"] - self.pos_y
        dist_to_target = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx) if dist_to_target > 0.25 else scene["yaw"]
        yaw_err = wrap_to_pi(target_yaw - self.yaw)

        # -------------------------------------------------------------
        # 1. High-Level Navigation & Behavior FSM
        # -------------------------------------------------------------
        linear_vel = 0.0
        yaw_rate = 0.0
        body_pitch_target = 0.0
        body_z_target = self.nominal_height

        if self.is_holding:
            self.stage_timer += dt
            # Special gestures while holding at stations
            if scene["fsm"] == "STATE_DOCENT_GREET":
                # VIP Greeting: Friendly head nod
                body_pitch_target = 0.12 * math.sin(self.stage_timer * 2.5)
            elif scene["fsm"] == "STATE_HAZARD_ALERT":
                # Spill Hazard: Bracing stance
                body_pitch_target = -0.08
                body_z_target = self.nominal_height - 0.03
            elif scene["fsm"] == "STATE_DOCENT_NARRATE":
                # Presenter mode: Head tilted +8° to address standing visitors
                body_pitch_target = 0.14
            elif scene["fsm"] == "STATE_PRONE_REST":
                # Prone Dock: Lower smoothly into prone fold
                fold_progress = min(1.0, self.stage_timer / 2.5)
                body_z_target = self.nominal_height - (self.nominal_height - 0.13) * fold_progress

            if self.stage_timer >= scene["hold"]:
                # Transition to next scene
                self.scene_idx = (self.scene_idx + 1) % len(self.scenes)
                self.is_holding = False
                self.stage_timer = 0.0
                next_s = self.scenes[self.scene_idx]
                self.get_logger().info(f"Navigating to {next_s['name']} ({next_s['fsm']})")
        else:
            # Traveling towards scene waypoint
            if dist_to_target > 0.12:
                # Turn towards target if large angular error
                if abs(yaw_err) > 0.35:
                    yaw_rate = math.copysign(min(1.2, abs(yaw_err) * 1.8), yaw_err)
                    linear_vel = 0.15  # Slow crawl while turning
                else:
                    yaw_rate = yaw_err * 1.4
                    linear_vel = min(0.65, max(0.25, dist_to_target * 0.75))
            else:
                # Arrived at position, align final orientation
                final_yaw_err = wrap_to_pi(scene["yaw"] - self.yaw)
                if abs(final_yaw_err) > 0.08:
                    yaw_rate = math.copysign(min(1.0, abs(final_yaw_err) * 1.8), final_yaw_err)
                    linear_vel = 0.0
                else:
                    # Fully arrived and aligned
                    self.is_holding = True
                    self.stage_timer = 0.0
                    self.get_logger().info(f"Station reached: {scene['name']}")

        # Integrate motion
        self.yaw += yaw_rate * dt
        self.yaw = wrap_to_pi(self.yaw)
        self.pos_x += linear_vel * math.cos(self.yaw) * dt
        self.pos_y += linear_vel * math.sin(self.yaw) * dt

        # Smooth pitch and height adjustments
        self.pitch += (body_pitch_target - self.pitch) * min(1.0, dt * 5.0)
        self.pos_z += (body_z_target - self.pos_z) * min(1.0, dt * 4.0)

        # -------------------------------------------------------------
        # 2. Quadruped Trot Gait Kinematics
        # -------------------------------------------------------------
        is_walking = (linear_vel > 0.05) or (abs(yaw_rate) > 0.25)
        step_len = min(self.max_step_len, linear_vel * 0.32) if is_walking else 0.0

        if is_walking:
            # Advance gait phase proportionally to motion
            gait_freq = 2.0 * math.pi * (1.4 + linear_vel * 0.8 + abs(yaw_rate) * 0.5)
            self.gait_phase = (self.gait_phase + gait_freq * dt) % (2.0 * math.pi)
            # Body bobbing & roll oscillation during trot
            body_bob = 0.006 * math.cos(2.0 * self.gait_phase)
            self.roll = 0.012 * math.sin(self.gait_phase)
        else:
            self.gait_phase = 0.0
            # Subtle resting breath
            body_bob = 0.003 * math.sin(now_time * 2.0)
            self.roll = 0.0

        current_base_z = self.pos_z + body_bob

        # Compute Foot Trajectories for all 4 legs
        # Diagonal Pair A: Front-Left (FL) and Rear-Right (RR)
        phase_A = self.gait_phase
        # Diagonal Pair B: Front-Right (FR) and Rear-Left (RL) -> 180° offset
        phase_B = (self.gait_phase + math.pi) % (2.0 * math.pi)

        # Yaw steering differential on step lengths
        turn_diff = (yaw_rate * 0.28) * 0.15
        step_left = step_len - turn_diff
        step_right = step_len + turn_diff

        def calc_foot_trajectory(phase, s_len):
            if not is_walking or abs(s_len) < 0.01:
                return 0.0, 0.0, -self.stance_h
            if phase < math.pi:
                # Stance Phase (Foot planted, pushes backward relative to hip)
                s = phase / math.pi
                xf = (s_len / 2.0) * math.cos(phase)
                zf = -self.stance_h
            else:
                # Swing Phase (Foot lifts in smooth parabolic clearance arc)
                s = (phase - math.pi) / math.pi
                xf = (s_len / 2.0) * math.cos(phase)
                zf = -self.stance_h + self.step_clearance * math.sin(math.pi * s)
            return xf, 0.0, zf

        fl_x, fl_y, fl_z = calc_foot_trajectory(phase_A, step_left)
        rr_x, rr_y, rr_z = calc_foot_trajectory(phase_A, step_right)
        fr_x, fr_y, fr_z = calc_foot_trajectory(phase_B, step_right)
        rl_x, rl_y, rl_z = calc_foot_trajectory(phase_B, step_left)

        # Handle Prone Fold posture override when docked
        if self.pos_z < 0.20:
            fold_ratio = min(1.0, max(0.0, (0.20 - self.pos_z) / 0.07))
            fl_joint = (0.0, 0.68 + fold_ratio * 0.52, -1.37 - fold_ratio * 1.03)
            fr_joint = (0.0, 0.68 + fold_ratio * 0.52, -1.37 - fold_ratio * 1.03)
            rl_joint = (0.0, 0.68 + fold_ratio * 0.52, -1.37 - fold_ratio * 1.03)
            rr_joint = (0.0, 0.68 + fold_ratio * 0.52, -1.37 - fold_ratio * 1.03)
        else:
            # Standard IK computation
            fl_joint = leg_inverse_kinematics(fl_x, fl_y, fl_z)
            fr_joint = leg_inverse_kinematics(fr_x, fr_y, fr_z)
            rl_joint = leg_inverse_kinematics(rl_x, rl_y, rl_z)
            rr_joint = leg_inverse_kinematics(rr_x, rr_y, rr_z)

        now_msg = self.get_clock().now().to_msg()

        # -------------------------------------------------------------
        # 3. Publish Dynamic TF (odom -> base_link)
        # -------------------------------------------------------------
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pos_x
        t.transform.translation.y = self.pos_y
        t.transform.translation.z = current_base_z
        t.transform.rotation = euler_to_quaternion(self.yaw, self.pitch, self.roll)
        self.tf_broadcaster.sendTransform(t)

        # -------------------------------------------------------------
        # 4. Publish 12-DOF Joint States
        # -------------------------------------------------------------
        js = JointState()
        js.header.stamp = now_msg
        js.name = self.joint_names
        js.position = [
            fl_joint[0], fl_joint[1], fl_joint[2],
            fr_joint[0], fr_joint[1], fr_joint[2],
            rl_joint[0], rl_joint[1], rl_joint[2],
            rr_joint[0], rr_joint[1], rr_joint[2]
        ]
        self.pub_joint_states.publish(js)

        # -------------------------------------------------------------
        # 5. Publish Odometry Trajectory Path
        # -------------------------------------------------------------
        if (len(self.path_msg.poses) == 0 or
            math.hypot(self.pos_x - self.path_msg.poses[-1].pose.position.x,
                       self.pos_y - self.path_msg.poses[-1].pose.position.y) > 0.12):
            pose = PoseStamped()
            pose.header.stamp = now_msg
            pose.header.frame_id = 'odom'
            pose.pose.position.x = self.pos_x
            pose.pose.position.y = self.pos_y
            pose.pose.position.z = 0.02
            self.path_msg.poses.append(pose)
            if len(self.path_msg.poses) > 200:
                self.path_msg.poses.pop(0)
            self.pub_path.publish(self.path_msg)

        # -------------------------------------------------------------
        # 6. Publish Environmental Exhibition Markers
        # -------------------------------------------------------------
        self._publish_markers(now_msg, scene)

    def _publish_markers(self, now, scene):
        markers = MarkerArray()

        # Marker 1: Floating FSM State Badge above Robot
        m_state = Marker()
        m_state.header.stamp = now
        m_state.header.frame_id = 'odom'
        m_state.ns = 'cloud_fsm'
        m_state.id = 0
        m_state.type = Marker.TEXT_VIEW_FACING
        m_state.action = Marker.ADD
        m_state.pose.position.x = self.pos_x
        m_state.pose.position.y = self.pos_y
        m_state.pose.position.z = self.pos_z + 0.45
        m_state.scale.z = 0.18
        if "HAZARD" in scene["fsm"]:
            m_state.color.r = 1.0; m_state.color.g = 0.2; m_state.color.b = 0.2; m_state.color.a = 1.0
        elif "ARGUS" in scene["fsm"]:
            m_state.color.r = 0.2; m_state.color.g = 0.8; m_state.color.b = 1.0; m_state.color.a = 1.0
        else:
            m_state.color.r = 0.4; m_state.color.g = 0.95; m_state.color.b = 0.5; m_state.color.a = 1.0
        m_state.text = f"[{scene['fsm']}]\n{scene['name']}"
        markers.markers.append(m_state)

        # Marker 2: Booth 1 Table
        m_booth = Marker()
        m_booth.header.stamp = now
        m_booth.header.frame_id = 'odom'
        m_booth.ns = 'exhibition_environment'
        m_booth.id = 1
        m_booth.type = Marker.CUBE
        m_booth.action = Marker.ADD
        m_booth.pose.position.x = 12.0
        m_booth.pose.position.y = 0.0
        m_booth.pose.position.z = 0.45
        m_booth.scale.x = 1.2; m_booth.scale.y = 2.0; m_booth.scale.z = 0.9
        m_booth.color.r = 0.12; m_booth.color.g = 0.18; m_booth.color.b = 0.28; m_booth.color.a = 0.95
        markers.markers.append(m_booth)

        # Marker 3: ArUco Marker Board (ID 10)
        m_aruco = Marker()
        m_aruco.header.stamp = now
        m_aruco.header.frame_id = 'odom'
        m_aruco.ns = 'exhibition_environment'
        m_aruco.id = 2
        m_aruco.type = Marker.CUBE
        m_aruco.action = Marker.ADD
        m_aruco.pose.position.x = 11.39
        m_aruco.pose.position.y = 0.0
        m_aruco.pose.position.z = 0.65
        m_aruco.scale.x = 0.02; m_aruco.scale.y = 0.35; m_aruco.scale.z = 0.35
        m_aruco.color.r = 0.95; m_aruco.color.g = 0.95; m_aruco.color.b = 0.95; m_aruco.color.a = 1.0
        markers.markers.append(m_aruco)

        # Marker 4: Chemical Spill Hazard Puddle
        m_spill = Marker()
        m_spill.header.stamp = now
        m_spill.header.frame_id = 'odom'
        m_spill.ns = 'exhibition_environment'
        m_spill.id = 3
        m_spill.type = Marker.CYLINDER
        m_spill.action = Marker.ADD
        m_spill.pose.position.x = 6.0
        m_spill.pose.position.y = 0.0
        m_spill.pose.position.z = 0.005
        m_spill.scale.x = 0.55; m_spill.scale.y = 0.55; m_spill.scale.z = 0.01
        m_spill.color.r = 0.1; m_spill.color.g = 0.45; m_spill.color.b = 0.85; m_spill.color.a = 0.85
        markers.markers.append(m_spill)

        # Marker 5: Central Stage Platform
        m_stage = Marker()
        m_stage.header.stamp = now
        m_stage.header.frame_id = 'odom'
        m_stage.ns = 'exhibition_environment'
        m_stage.id = 4
        m_stage.type = Marker.CYLINDER
        m_stage.action = Marker.ADD
        m_stage.pose.position.x = 0.0
        m_stage.pose.position.y = 0.0
        m_stage.pose.position.z = 0.02
        m_stage.scale.x = 3.5; m_stage.scale.y = 3.5; m_stage.scale.z = 0.04
        m_stage.color.r = 0.15; m_stage.color.g = 0.12; m_stage.color.b = 0.25; m_stage.color.a = 0.9
        markers.markers.append(m_stage)

        self.pub_markers.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = CloudRVizPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
