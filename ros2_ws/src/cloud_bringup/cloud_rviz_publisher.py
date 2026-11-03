"""
Project Cloud: Master RViz2 Live Tour & Official Go2 Simulation Publisher
Publishes official Unitree Go2 URDF, dynamic 12-DOF joint states, TF transforms,
environmental exhibition markers, and odometry trajectory paths for RViz2.
"""

import sys
import os
import math
import time

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
        w=cr * cp * cy + sr * cp * sy
    )

class CloudRVizPublisher(Node):
    def __init__(self):
        super().__init__('cloud_rviz_publisher')

        repo_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        urdf_path = os.path.join(repo_dir, "ros2_ws", "src", "go2_description", "urdf", "go2_rviz.urdf")

        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDF missing at: {urdf_path}")
            urdf_content = ""
        else:
            with open(urdf_path, "r", encoding="utf-8") as f:
                urdf_content = f.read()

        # 1. Publish Robot Description with Transient Local QoS (latching)
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

        # 2. Publishers for RViz Displays
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

        # Mission Animation Waypoints (The 6 Canonical Scenes)
        self.scenes = [
            {"name": "SCENE 1: VIP GREETING", "fsm": "STATE_DOCENT_GREET", "x": 0.0, "y": 0.0, "z": 0.32, "yaw": 0.0, "pitch": 0.0, "duration": 5.0},
            {"name": "SCENE 2: ARGUS QUERY", "fsm": "STATE_ARGUS_PATROL", "x": 2.5, "y": 0.0, "z": 0.32, "yaw": 0.0, "pitch": 0.0, "duration": 4.0},
            {"name": "SCENE 3: SPILL HAZARD E-STOP", "fsm": "STATE_HAZARD_ALERT", "x": 5.2, "y": 0.0, "z": 0.32, "yaw": 0.0, "pitch": 0.0, "duration": 5.0},
            {"name": "SCENE 4: ARUCO STANDOFF", "fsm": "STATE_DOCENT_CRUISE", "x": 10.2, "y": 0.0, "z": 0.32, "yaw": 0.0, "pitch": 0.0, "duration": 5.0},
            {"name": "SCENE 5: 180° PRESENTER PIVOT", "fsm": "STATE_DOCENT_NARRATE", "x": 10.2, "y": 0.0, "z": 0.28, "yaw": math.pi, "pitch": 0.14, "duration": 6.0},
            {"name": "SCENE 6: STAGE & PRONE DOCK", "fsm": "STATE_PRONE_REST", "x": -4.0, "y": -4.0, "z": 0.14, "yaw": -2.35, "pitch": 0.0, "duration": 5.0}
        ]

        self.current_scene_idx = 0
        self.scene_start_time = time.time()
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_z = 0.32
        self.cur_yaw = 0.0
        self.cur_pitch = 0.0

        # Run 30 Hz Telemetry Loop
        self.timer = self.create_timer(1.0 / 30.0, self._update_telemetry)
        self.get_logger().info("Cloud RViz Publisher running at 30 Hz.")

    def _update_telemetry(self):
        now_time = time.time()
        scene_data = self.scenes[self.current_scene_idx]
        elapsed = now_time - self.scene_start_time

        if elapsed > scene_data["duration"]:
            self.current_scene_idx = (self.current_scene_idx + 1) % len(self.scenes)
            self.scene_start_time = now_time
            scene_data = self.scenes[self.current_scene_idx]
            self.get_logger().info(f"Transitioned to {scene_data['name']} ({scene_data['fsm']})")

        # Smooth Interpolation towards Target Scene Pose
        lerp = 0.08
        self.cur_x += (scene_data["x"] - self.cur_x) * lerp
        self.cur_y += (scene_data["y"] - self.cur_y) * lerp
        self.cur_z += (scene_data["z"] - self.cur_z) * lerp
        self.cur_pitch += (scene_data["pitch"] - self.cur_pitch) * lerp

        d_yaw = scene_data["yaw"] - self.cur_yaw
        while d_yaw < -math.pi: d_yaw += 2 * math.pi
        while d_yaw > math.pi: d_yaw -= 2 * math.pi
        self.cur_yaw += d_yaw * lerp

        now = self.get_clock().now().to_msg()

        # 1. Publish /tf Transform (odom -> base)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base'
        t.transform.translation.x = self.cur_x
        t.transform.translation.y = self.cur_y
        t.transform.translation.z = self.cur_z
        t.transform.rotation = euler_to_quaternion(self.cur_yaw, self.cur_pitch, 0.0)
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish 12-DOF Joint States
        js = JointState()
        js.header.stamp = now
        js.name = self.joint_names

        is_moving = math.hypot(scene_data["x"] - self.cur_x, scene_data["y"] - self.cur_y) > 0.1
        trot_phase = math.sin(now_time * 8.0) if is_moving else 0.0

        if self.cur_z < 0.20:
            # Prone fold posture
            js.position = [
                0.0, 1.2, -2.4,   # FL
                0.0, 1.2, -2.4,   # FR
                0.0, 1.2, -2.4,   # RL
                0.0, 1.2, -2.4    # RR
            ]
        else:
            # Active trot / stance
            js.position = [
                0.0, 0.6 + trot_phase * 0.3, -1.3 + abs(trot_phase) * 0.2,   # FL
                0.0, 0.6 - trot_phase * 0.3, -1.3 + abs(trot_phase) * 0.2,   # FR
                0.0, 0.6 - trot_phase * 0.3, -1.3 + abs(trot_phase) * 0.2,   # RL
                0.0, 0.6 + trot_phase * 0.3, -1.3 + abs(trot_phase) * 0.2    # RR
            ]
        self.pub_joint_states.publish(js)

        # 3. Publish Path Breadcrumb
        if len(self.path_msg.poses) == 0 or math.hypot(self.cur_x - self.path_msg.poses[-1].pose.position.x, self.cur_y - self.path_msg.poses[-1].pose.position.y) > 0.15:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'odom'
            pose.pose.position.x = self.cur_x
            pose.pose.position.y = self.cur_y
            pose.pose.position.z = 0.02
            self.path_msg.poses.append(pose)
            if len(self.path_msg.poses) > 150:
                self.path_msg.poses.pop(0)
            self.pub_path.publish(self.path_msg)

        # 4. Publish 3D Environmental Exhibition Markers
        self._publish_environment_markers(now, scene_data)

    def _publish_environment_markers(self, now, scene_data):
        markers = MarkerArray()

        # Marker 1: Floating FSM State Text above Quadruped
        text_marker = Marker()
        text_marker.header.stamp = now
        text_marker.header.frame_id = 'base'
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.z = 0.55
        text_marker.scale.z = 0.18
        text_marker.color.a = 1.0
        if "HAZARD" in scene_data["fsm"]:
            text_marker.color.r = 1.0; text_marker.color.g = 0.15; text_marker.color.b = 0.15
        elif "DOCENT" in scene_data["fsm"]:
            text_marker.color.r = 0.45; text_marker.color.g = 0.25; text_marker.color.b = 0.95
        else:
            text_marker.color.r = 0.0; text_marker.color.g = 0.8; text_marker.color.b = 1.0
        text_marker.text = f"{scene_data['fsm']}\n{scene_data['name']}"
        markers.markers.append(text_marker)

        # Marker 2: Booth 1 Table (at x=12.0, y=0.0)
        table = Marker()
        table.header.stamp = now
        table.header.frame_id = 'odom'
        table.id = 10
        table.type = Marker.CUBE
        table.action = Marker.ADD
        table.pose.position.x = 12.0
        table.pose.position.y = 0.0
        table.pose.position.z = 0.45
        table.scale.x = 1.2
        table.scale.y = 2.0
        table.scale.z = 0.9
        table.color.r = 0.12; table.color.g = 0.18; table.color.b = 0.28; table.color.a = 0.95
        markers.markers.append(table)

        # Marker 3: ArUco Marker ID 10 Plate (at x=11.39, y=0.0, z=1.15)
        aruco = Marker()
        aruco.header.stamp = now
        aruco.header.frame_id = 'odom'
        aruco.id = 11
        aruco.type = Marker.CUBE
        aruco.action = Marker.ADD
        aruco.pose.position.x = 11.39
        aruco.pose.position.y = 0.0
        aruco.pose.position.z = 1.15
        aruco.scale.x = 0.02
        aruco.scale.y = 0.16
        aruco.scale.z = 0.16
        aruco.color.r = 1.0; aruco.color.g = 1.0; aruco.color.b = 1.0; aruco.color.a = 1.0
        markers.markers.append(aruco)

        # Marker 4: Chemical Spill Hazard Puddle (at x=6.0, y=0.0)
        spill = Marker()
        spill.header.stamp = now
        spill.header.frame_id = 'odom'
        spill.id = 20
        spill.type = Marker.CYLINDER
        spill.action = Marker.ADD
        spill.pose.position.x = 6.0
        spill.pose.position.y = 0.0
        spill.pose.position.z = 0.005
        spill.scale.x = 0.70
        spill.scale.y = 0.70
        spill.scale.z = 0.01
        spill.color.r = 0.05; spill.color.g = 0.35; spill.color.b = 0.85; spill.color.a = 0.85
        markers.markers.append(spill)

        # Marker 5: Central Stage Platform (at x=0.0, y=8.5)
        stage = Marker()
        stage.header.stamp = now
        stage.header.frame_id = 'odom'
        stage.id = 30
        stage.type = Marker.CYLINDER
        stage.action = Marker.ADD
        stage.pose.position.x = 0.0
        stage.pose.position.y = 8.5
        stage.pose.position.z = 0.05
        stage.scale.x = 3.5
        stage.scale.y = 3.5
        stage.scale.z = 0.10
        stage.color.r = 0.15; stage.color.g = 0.12; stage.color.b = 0.35; stage.color.a = 0.95
        markers.markers.append(stage)

        self.pub_markers.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = CloudRVizPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
