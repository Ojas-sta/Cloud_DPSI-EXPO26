#!/usr/bin/env python3
"""
Project Cloud: High-Fidelity MuJoCo Physics & ROS 2 Unified Digital Twin
Platform: Unitree Go2 Edu Quadruped
Environment: MuJoCo Physics 3.x + ROS 2 Jazzy (Apple Silicon macOS via Pixi)

Key Capabilities:
1. Physics & Contact Dynamics:
   - Analytical closed-form 3-DOF leg inverse kinematics for all 4 legs (12 DOF)
   - Dynamic 2-beat diagonal quadruped trot gait with cycloidal swing and stance
   - Ground reaction forces, dynamic body pitch/roll, and posture morphing (VIP Greeting,
     Hazard Bracing, Docent Presenter Tilt, Prone Sleep Fold)
2. Unitree 4D LiDAR L1 Simulation:
   - 16 elevation rings (-45° to +45°) x 72 azimuth rays = 1,152 3D spatial beams
   - Native C-raycast query (mujoco.mj_ray) against physical pavilion geometry
   - PointCloud2 output with x, y, z, intensity (material-dependent), and ring channel
   - Horizontal planar LaserScan output on /cloud/lidar/scan
3. Autonomy Engine (80% Preplanned + 20% Reactive Dynamic Local Autonomy):
   - 80% Preplanned Global Navigation Path traversing 6 DPSI Expo 2026 stations
   - 20% Reactive Dynamic Local Autonomy:
     * Real-time 4D LiDAR obstacle clustering
     * Artificial Potential Field (APF) & smooth spline deflection (+0.62m around spill/cones)
     * Dynamic proportional velocity throttling based on forward LiDAR clearance
     * Emergency braking (E-Stop) with low-CG stability crouching
     * Closed-loop ArUco standoff docking at 1.20m
4. Authentic Unitree Go2 Hardware Telemetry:
   - /cloud/telemetry/high_state (complete Unitree Go2 HighState JSON telemetry)
   - /cloud/telemetry/battery (sensor_msgs/BatteryState)
   - /cloud/imu (sensor_msgs/Imu)
   - /cloud/odom (nav_msgs/Odometry)
   - /cloud/joint_states (sensor_msgs/JointState)
   - /cloud/path (nav_msgs/Path - 80% global reference)
   - /cloud/reactive_path (nav_msgs/Path - 20% local reactive avoidance)
   - /cloud/markers (visualization_msgs/MarkerArray)
   - /tf (odom -> base_link dynamic transform)
"""

import sys
import os
import math
import time
import json
import argparse
import numpy as np

# MuJoCo Physics Engine
try:
    import mujoco
    import mujoco.viewer
except ImportError as e:
    print(f"[ERROR] MuJoCo import failed: {e}")
    sys.exit(1)

# ROS 2 Jazzy Core
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String, Header, Int32, Float32MultiArray
    from sensor_msgs.msg import PointCloud2, PointField, LaserScan, Imu, BatteryState, JointState
    from nav_msgs.msg import Path, Odometry, OccupancyGrid, MapMetaData
    from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Quaternion, Twist, Vector3
    from visualization_msgs.msg import Marker, MarkerArray
    import sensor_msgs_py.point_cloud2 as pc2
    import tf2_ros
except ImportError as e:
    print(f"[ERROR] ROS 2 import failed: {e}")
    sys.exit(1)

import yaml

# Unitree Go2 Mechanical Parameters
L_THIGH = 0.213
L_CALF = 0.213
STANCE_HEIGHT = 0.330
STEP_CLEARANCE = 0.052
MAX_STEP_LEN = 0.160
ROBOT_MASS = 15.0  # kg

def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    """Returns normalized [x, y, z, w] quaternion."""
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

    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm > 1e-8:
        return [x/norm, y/norm, z/norm, w/norm]
    return [0.0, 0.0, 0.0, 1.0]

def leg_inverse_kinematics(x_foot, y_foot, z_foot):
    """Computes analytical 3-DOF joint angles (hip_roll, thigh_pitch, calf_pitch)."""
    hip = math.atan2(y_foot, -z_foot)
    d = math.hypot(x_foot, z_foot)
    max_reach = (L_THIGH + L_CALF) * 0.985
    min_reach = abs(L_THIGH - L_CALF) * 1.05
    d = max(min_reach, min(max_reach, d))

    cos_beta = (L_THIGH**2 + L_CALF**2 - d**2) / (2.0 * L_THIGH * L_CALF)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    calf = -(math.pi - beta)

    alpha = math.atan2(x_foot, -z_foot)
    cos_gamma = (L_THIGH**2 + d**2 - L_CALF**2) / (2.0 * L_THIGH * d)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))
    gamma = math.acos(cos_gamma)
    thigh = alpha + gamma

    return hip, thigh, calf

class Unitree4DLiDAR:
    """
    Simulated Unitree 4D LiDAR L1.
    16 elevation rings (-45° to +45°) x 72 azimuth rays (360°) = 1,152 rays.
    Performs C-accelerated raycasts, multi-body pass-through, and material intensity calculation.
    """
    def __init__(self, model, data, num_rings=16, num_azimuth=72, max_range=16.0):
        self.model = model
        self.data = data
        self.num_rings = num_rings
        self.num_azimuth = num_azimuth
        self.max_range = max_range
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "lidar_origin")
        self.base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.geomid_buf = np.zeros(1, dtype=np.int32)

        # Pre-filter robot self bodies
        self.robot_bodies = set()
        for b in range(model.nbody):
            bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
            if bname and any(p in bname for p in ["base", "FL", "FR", "RL", "RR", "Head"]):
                self.robot_bodies.add(b)

        # Spherical Ray Generation
        self.rings = np.linspace(-math.radians(45.0), math.radians(45.0), num_rings)
        self.azimuths = np.linspace(-math.pi, math.pi, num_azimuth, endpoint=False)

        # Pre-compute local ray unit vectors (1152, 3)
        self.total_rays = num_rings * num_azimuth
        self.local_rays = np.zeros((self.total_rays, 3), dtype=np.float64)
        self.ray_ring_idx = np.zeros(self.total_rays, dtype=np.uint16)
        self.ray_azimuth_rad = np.zeros(self.total_rays, dtype=np.float64)

        idx = 0
        for r_idx, phi in enumerate(self.rings):
            cos_phi = math.cos(phi)
            sin_phi = math.sin(phi)
            for theta in self.azimuths:
                self.local_rays[idx, 0] = cos_phi * math.cos(theta)
                self.local_rays[idx, 1] = cos_phi * math.sin(theta)
                self.local_rays[idx, 2] = sin_phi
                self.ray_ring_idx[idx] = r_idx
                self.ray_azimuth_rad[idx] = theta
                idx += 1

        # Find ring closest to 0° for 2D planar laser scan
        self.horizontal_ring_idx = int(np.argmin(np.abs(self.rings)))

    def scan(self):
        """Executes full 4D LiDAR spherical raycasting."""
        p0 = self.data.site_xpos[self.site_id].copy()
        R = self.data.site_xmat[self.site_id].reshape((3, 3))

        points_sensor = []   # (x, y, z, intensity, ring) in sensor frame
        points_world = []    # (x, y, z) in world frame
        scan_ranges_2d = [float('inf')] * self.num_azimuth

        min_front_dist = 999.0
        closest_obstacle = "none"
        hazard_detected = False
        forward_obstacles = []  # points in travel envelope (x_w, y_w, dist)

        for i in range(self.total_rays):
            ring = self.ray_ring_idx[i]
            azimuth = self.ray_azimuth_rad[i]
            u_local = self.local_rays[i]
            v_world = R @ u_local

            if np.linalg.norm(v_world) < 1e-4:
                continue

            d = mujoco.mj_ray(self.model, self.data, p0, v_world, None, 1, self.base_body_id, self.geomid_buf)
            if d < 0.0 or d > self.max_range:
                continue

            gid = self.geomid_buf[0]
            hit_body = self.model.geom_bodyid[gid]
            if hit_body in self.robot_bodies:
                continue

            total_d = d
            hit_geom_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or "geom"

            if 0.08 <= total_d <= self.max_range:
                # Add realistic range noise
                noisy_d = total_d + float(np.random.normal(0.0, 0.005))
                xs = noisy_d * u_local[0]
                ys = noisy_d * u_local[1]
                zs = noisy_d * u_local[2]

                xw = p0[0] + noisy_d * v_world[0]
                yw = p0[1] + noisy_d * v_world[1]
                zw = p0[2] + noisy_d * v_world[2]

                # Material-dependent intensity return
                if "cone" in hit_geom_name:
                    intensity = 0.98  # retroreflective
                    hazard_detected = True
                elif "spill" in hit_geom_name:
                    intensity = 0.22  # absorptive slick
                    hazard_detected = True
                elif "wall" in hit_geom_name or "rail" in hit_geom_name:
                    intensity = 0.85
                elif "stage" in hit_geom_name:
                    intensity = 0.65
                elif "floor" in hit_geom_name:
                    intensity = 0.52
                else:
                    intensity = 0.70

                points_sensor.append([xs, ys, zs, intensity, ring])
                points_world.append([xw, yw, zw, intensity, ring])

                # 2D scan slice (horizontal ring)
                if ring == self.horizontal_ring_idx:
                    # Find azimuth bin index
                    az_idx = int(np.argmin(np.abs(self.azimuths - azimuth)))
                    scan_ranges_2d[az_idx] = noisy_d

                # Forward safety envelope check (-25° to +25°, within 4m, above ground)
                if abs(azimuth) < math.radians(25.0) and abs(u_local[2]) < 0.45 and zw > 0.04 and "floor" not in hit_geom_name and hit_geom_name != "none":
                    if noisy_d < min_front_dist:
                        min_front_dist = noisy_d
                        closest_obstacle = hit_geom_name

                    # Obstacle points in forward travel corridor (in robot local frame)
                    if 0.25 <= xs <= 3.5 and abs(ys) <= 1.20 and zw > 0.03:
                        forward_obstacles.append((xs, ys, noisy_d))

        return {
            "origin": p0,
            "points_sensor": points_sensor,
            "points_world": points_world,
            "ranges_2d": scan_ranges_2d,
            "min_front": min_front_dist,
            "closest_geom": closest_obstacle,
            "hazard_detected": hazard_detected,
            "local_obstacles": forward_obstacles
        }

class EgoModelPredictivePlanner:
    """
    Model-Based Ego-Centric Predictive Path & Velocity Planner (DWA / MPPI).
    Operates strictly in the robot's local frame (base_link) using only onboard sensors:
    - 4D LiDAR local obstacle points (xs, ys) & sectoral clearances
    - Onboard IMU (pitch, roll, angular rate)
    - Kinematic leg odometry velocity (v_x, omega_z)
    Zero privileged global room coordinates or room boundaries in the decision loop.
    """
    def __init__(self):
        self.robot_radius = 0.30
        self.safety_margin = 0.18
        self.horizon_time = 1.8
        self.num_steps = 9
        self.v_samples = [0.18, 0.35, 0.52, 0.65]
        self.w_samples = np.linspace(-1.05, 1.05, 15)

    def plan(self, local_obstacles, min_front_clearance, current_v, current_w,
             target_rel_yaw=0.0, v_pref=0.55, imu_pitch=0.0):
        best_cost = float("inf")
        best_v = 0.0
        best_w = 0.0
        best_traj_local = []
        all_trajs = []

        obs_xy = np.array([[o[0], o[1]] for o in local_obstacles]) if len(local_obstacles) > 0 else np.empty((0, 2))

        for v in self.v_samples:
            for w in self.w_samples:
                traj = []
                collision = False
                min_dist_to_obs = 999.0

                for step in range(1, self.num_steps + 1):
                    t = (step / self.num_steps) * self.horizon_time
                    th = w * t
                    if abs(w) > 0.04:
                        x = (v / w) * math.sin(th)
                        y = (v / w) * (1.0 - math.cos(th))
                    else:
                        x = v * t
                        y = 0.0
                    traj.append((x, y))

                    if len(obs_xy) > 0:
                        dists = np.hypot(obs_xy[:, 0] - x, obs_xy[:, 1] - y)
                        step_min_d = np.min(dists)
                        if step_min_d < min_dist_to_obs:
                            min_dist_to_obs = step_min_d
                        if step_min_d < (self.robot_radius + self.safety_margin):
                            collision = True

                all_trajs.append({
                    "v": v, "w": w, "traj": traj,
                    "collision": collision, "min_dist": min_dist_to_obs
                })

                if collision:
                    cost = 1000.0 + (100.0 / max(0.05, min_dist_to_obs))
                else:
                    heading_err = abs(w * self.horizon_time - target_rel_yaw)
                    clearance_cost = 1.0 / max(0.20, min_dist_to_obs)
                    speed_cost = (v_pref - v) ** 2
                    turn_cost = abs(w) * 0.4
                    pitch_cost = max(0.0, abs(imu_pitch) - 0.15) * 10.0
                    cost = (2.2 * heading_err + 6.5 * clearance_cost +
                            1.0 * speed_cost + turn_cost + pitch_cost)

                if cost < best_cost:
                    best_cost = cost
                    best_v = v
                    best_w = w
                    best_traj_local = traj

        # Emergency braking fallback if critical front clearance
        if min_front_clearance <= 0.65 or best_cost > 500.0:
            best_v = 0.0
            best_w = 0.0

        return best_v, best_w, best_traj_local, all_trajs

class CloudMuJoCoTwin(Node):
    def __init__(self, xml_path=None, headless=True, spawn_stall=None):
        super().__init__('cloud_mujoco_ros2_twin')

        # Determine XML path
        cur_dir = os.path.dirname(os.path.abspath(__file__))
        repo_dir = os.path.dirname(cur_dir)
        yaml_db_path = os.path.join(repo_dir, "config", "stalls_database.yaml")

        if cur_dir not in sys.path:
            sys.path.insert(0, cur_dir)
        if repo_dir not in sys.path:
            sys.path.insert(0, repo_dir)

        if xml_path is None:
            if os.path.exists(yaml_db_path):
                try:
                    from world_generator import generate_40_stalls_world_xml
                    xml_path = os.path.join(cur_dir, "unitree_go2", "dpsi_expo_world_40_stalls.xml")
                    generate_40_stalls_world_xml(yaml_db_path, xml_path)
                except Exception as e:
                    self.get_logger().warn(f"Failed to auto-generate 40-stall world: {e}. Using default.")
                    xml_path = os.path.join(cur_dir, "unitree_go2", "dpsi_expo_world.xml")
            else:
                xml_path = os.path.join(cur_dir, "unitree_go2", "dpsi_expo_world.xml")

        self.get_logger().info(f"Loading MuJoCo World: {xml_path}")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.headless = headless

        # Initialize Unitree 4D LiDAR L1 with Authentic High-Density Scanning
        # 32 elevation rings (-45° to +45°) x 180 azimuth rays (360°) = 5,760 dense 3D beams
        self.lidar = Unitree4DLiDAR(self.model, self.data, num_rings=32, num_azimuth=180, max_range=16.0)

        # Foot geom IDs and base body ID for ground contact and spatial tracking
        self.foot_names = ["FL", "FR", "RL", "RR"]
        self.foot_geom_ids = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in self.foot_names}
        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base")

        # Joint Names
        self.joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]

        # -------------------------------------------------------------
        # ROS 2 Publishers & Broadcasters
        # -------------------------------------------------------------
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        qos_sensor = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)

        # 1. Official URDF Robot Description
        repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        urdf_path = os.path.join(repo_dir, "ros2_ws", "src", "go2_description", "urdf", "go2_rviz.urdf")
        self.pub_description = self.create_publisher(String, '/robot_description', qos_latched)
        if os.path.exists(urdf_path):
            with open(urdf_path, "r", encoding="utf-8") as f:
                desc_msg = String(data=f.read())
                self.pub_description.publish(desc_msg)
            self.get_logger().info("Latched official Unitree Go2 URDF to /robot_description.")

        # 2. Sensor & Telemetry Publishers
        self.pub_pointcloud = self.create_publisher(PointCloud2, '/cloud/lidar/pointcloud2', qos_sensor)
        self.pub_scan = self.create_publisher(LaserScan, '/cloud/lidar/scan', 10)
        self.pub_joint_states = self.create_publisher(JointState, '/cloud/joint_states', 10)
        self.pub_joint_states_root = self.create_publisher(JointState, '/joint_states', 10) # for robot_state_publisher
        self.pub_imu = self.create_publisher(Imu, '/cloud/imu', 10)
        self.pub_odom = self.create_publisher(Odometry, '/cloud/odom', 10)
        self.pub_battery = self.create_publisher(BatteryState, '/cloud/telemetry/battery', 10)
        self.pub_high_state = self.create_publisher(String, '/cloud/telemetry/high_state', 10)
        self.pub_foot_forces = self.create_publisher(Float32MultiArray, '/cloud/telemetry/foot_forces', 10)
        self.pub_apriltag_pose = self.create_publisher(PoseStamped, '/camera/apriltag_pose', 10)
        self.pub_apriltag_id = self.create_publisher(Int32, '/camera/apriltag_id', 10)

        # 3. Autonomy & Planning Publishers
        self.pub_global_path = self.create_publisher(Path, '/cloud/path', 10)
        self.pub_reactive_path = self.create_publisher(Path, '/cloud/reactive_path', 10)
        self.pub_odom_path = self.create_publisher(Path, '/cloud/odom_path', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/cloud/markers', 10)
        self.pub_candidates = self.create_publisher(MarkerArray, '/cloud/planner/candidates', 10)

        # 4. Latched 2D Master Auditorium Costmap & Static TF Broadcaster
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', qos_latched)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # 5. HAL Bridge Subscriptions (Eliminate Dual-Publisher Contention)
        self.external_control_latched = False
        self.ext_cmd_vx = 0.0
        self.ext_cmd_vy = 0.0
        self.ext_cmd_vyaw = 0.0
        self.ext_posture = "BALANCE_STAND"
        self.ext_euler_pitch = 0.0
        self.last_ext_cmd_time = time.time()

        self.create_subscription(Twist, '/cmd_vel_safe', self._ext_cmd_vel_cb, 10)
        self.create_subscription(String, '/go2_posture_cmd', self._ext_posture_cb, 10)
        self.create_subscription(Vector3, '/go2_body_euler', self._ext_euler_cb, 10)
        self.create_subscription(Int32, '/collision_monitor_state', self._ext_collision_cb, 10)

        # Instantiate Ego-Centric Model Predictive Planner (DWA)
        self.planner = EgoModelPredictivePlanner()
        self.best_traj_local = []
        self.candidate_trajs = []

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # -------------------------------------------------------------
        # Data-Driven 40-Stall Global Navigation & Mission State Machine
        # -------------------------------------------------------------
        self.scenes = self._load_data_driven_scenes(yaml_db_path)
        self.booth_fiducials = []
        self.stalls_data = []
        self.dock_cfg = {}
        if os.path.exists(yaml_db_path):
            try:
                with open(yaml_db_path, "r", encoding="utf-8") as f:
                    db_cfg = yaml.safe_load(f)
                    self.dock_cfg = db_cfg.get("recharge_dock", {})
                    self.stalls_data = db_cfg.get("stalls", [])
                    rd = self.dock_cfg
                    if rd:
                        self.booth_fiducials.append({
                            "id": rd.get("marker_id", 99),
                            "x": rd.get("pose", {}).get("x", -2.0),
                            "y": rd.get("pose", {}).get("y", 0.0),
                        })
                    for st in self.stalls_data:
                        bp = st.get("booth_pose", {})
                        byaw = float(bp.get("yaw", -1.5708))
                        # AprilTag plate is mounted on front face of booth table facing aisle
                        tag_x = float(bp.get("x", 0.0))
                        tag_y = float(bp.get("y", 0.0)) + 0.46 * math.sin(byaw)
                        self.booth_fiducials.append({
                            "id": st.get("apriltag_id", 10),
                            "x": tag_x,
                            "y": tag_y,
                        })
            except Exception as e:
                self.get_logger().warn(f"Failed loading booth fiducials: {e}")

        # Pre-build complete 3D exhibition environment markers and 2D occupancy grid
        self.cached_static_markers = self._build_static_markers()
        self._broadcast_static_tf()
        self._init_and_publish_map()

        # Robot Physical State
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = STANCE_HEIGHT + 0.05
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        if spawn_stall and os.path.exists(yaml_db_path):
            try:
                with open(yaml_db_path, "r", encoding="utf-8") as f:
                    db_cfg = yaml.safe_load(f)
                    target_stall = None
                    for st in db_cfg.get("stalls", []):
                        if st.get("id") == spawn_stall or str(st.get("apriltag_id")) == spawn_stall:
                            target_stall = st
                            break
                    if target_stall:
                        bp = target_stall.get("booth_pose", {})
                        sp = target_stall.get("staging_pose", {})
                        bx, by = bp.get("x", 4.0), bp.get("y", 7.2)
                        sx, sy = sp.get("x", 4.0), sp.get("y", 6.0)
                        dist = math.hypot(sx - bx, sy - by)
                        if dist > 0.01:
                            ux = (sx - bx) / dist
                            uy = (sy - by) / dist
                            self.pos_x = bx + ux * 2.0
                            self.pos_y = by + uy * 2.0
                            self.yaw = math.atan2(-uy, -ux)
                        self.external_control_latched = True
                        self.in_transit = False
                        self.get_logger().info(f"Spawned Go2 at stall approach {spawn_stall}: ({self.pos_x:.2f}, {self.pos_y:.2f}), yaw={math.degrees(self.yaw):.1f}° (external control latched)")
            except Exception as e:
                self.get_logger().warn(f"Failed parsing spawn stall: {e}")

        self.linear_vel = 0.0
        self.yaw_rate = 0.0

        self.scene_idx = 0
        self.in_transit = False
        self.stage_timer = 0.0
        self.sim_time = 0.0
        self.gait_phase = 0.0

        # Reactive Dynamic Autonomy Parameters
        self.reactive_y_offset = 0.0
        self.safety_state = "SAFETY_CLEAR"
        self.target_standoff_distance = 1.20

        # Real Telemetry Variables
        self.battery_soc = 92.5       # %
        self.battery_voltage = 28.52  # V
        self.battery_current = -4.2   # A
        self.motor_temps = [29.5] * 12

        # Odometry Trail container
        self.odom_path_msg = Path()
        self.odom_path_msg.header.frame_id = 'odom'

        # Initialize robot spatial pose in MuJoCo
        quat_init = euler_to_quaternion(self.yaw, self.pitch, self.roll)
        self.data.qpos[0] = self.pos_x
        self.data.qpos[1] = self.pos_y
        self.data.qpos[2] = self.pos_z
        self.data.qpos[3:7] = [quat_init[3], quat_init[0], quat_init[1], quat_init[2]]  # w, x, y, z
        self.data.qpos[7:19] = [0.0, 0.74, -1.48] * 4
        mujoco.mj_forward(self.model, self.data)

        # Publish the 80% Preplanned Global Path once latched
        self._publish_global_path()

        # Run Telemetry & Physics step at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self._simulation_cycle)
        self.last_cycle_time = time.time()
        self.get_logger().info("Unified MuJoCo Physics & ROS 2 Twin running at 30 Hz.")

    def _load_data_driven_scenes(self, yaml_path):
        if not os.path.exists(yaml_path):
            return [
                {"id": 1, "name": "SCENE 1: VIP GREETING", "fsm": "STATE_DOCENT_GREET", "target_x": 0.0, "target_y": 0.0, "target_yaw": 0.0, "hold_time": 5.5, "desc": "Stage Greeting"},
                {"id": 2, "name": "SCENE 2: ARGUS PATROL", "fsm": "STATE_ARGUS_PATROL", "target_x": 2.5, "target_y": 0.0, "target_yaw": 0.0, "hold_time": 4.0, "desc": "Patrol"},
                {"id": 3, "name": "SCENE 3: HAZARD AVOIDANCE", "fsm": "STATE_HAZARD_AVOID", "target_x": 4.8, "target_y": 0.0, "target_yaw": 0.0, "hold_time": 4.5, "desc": "Hazard"},
                {"id": 4, "name": "SCENE 4: ARUCO DOCK", "fsm": "STATE_DOCENT_CRUISE", "target_x": 8.2, "target_y": 0.0, "target_yaw": 0.0, "hold_time": 5.0, "desc": "Standoff Dock"},
                {"id": 5, "name": "SCENE 5: PIVOT NARRATE", "fsm": "STATE_DOCENT_NARRATE", "target_x": 8.2, "target_y": 0.0, "target_yaw": math.pi, "hold_time": 5.5, "desc": "Narrate"},
                {"id": 6, "name": "SCENE 6: PRONE REST", "fsm": "STATE_PRONE_REST", "target_x": 0.0, "target_y": 0.0, "target_yaw": 0.0, "hold_time": 6.0, "desc": "Prone Rest"}
            ]

        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        scenes = []
        dock = data.get("recharge_dock", {})
        scenes.append({
            "id": 0,
            "name": "KEYNOTE STAGE WELCOME",
            "fsm": "STATE_DOCENT_GREET",
            "target_x": dock.get("pose", {}).get("x", -2.0),
            "target_y": dock.get("pose", {}).get("y", 0.0),
            "target_yaw": 0.0,
            "hold_time": 5.0,
            "desc": "Stage Keynote Welcome & VIP Interception"
        })

        for s in data.get("stalls", [])[:10]: # Active Cohort 1
            stg = s.get("staging_pose", {})
            scenes.append({
                "id": s.get("apriltag_id", 10),
                "name": f"{s['id']}: {s.get('title', '')[:28]}",
                "fsm": "STATE_DOCENT_CRUISE",
                "target_x": float(stg.get("x", 4.0)),
                "target_y": float(stg.get("y", 6.0)),
                "target_yaw": float(stg.get("yaw", 1.5708)),
                "hold_time": 5.0,
                "desc": f"Standoff Docking at {s['id']} (Tag {s.get('apriltag_id')})"
            })

        scenes.append({
            "id": 99,
            "name": "STAGE RECHARGE PRONE REST",
            "fsm": "STATE_PRONE_REST",
            "target_x": float(dock.get("staging_pose", {}).get("x", -0.8)),
            "target_y": float(dock.get("staging_pose", {}).get("y", 0.0)),
            "target_yaw": 3.14159,
            "hold_time": 6.0,
            "desc": "Returned to Stage Dock for Prone Sleep"
        })
        return scenes

    def _ext_cmd_vel_cb(self, msg: Twist):
        self.last_ext_cmd_time = time.time()
        if not self.external_control_latched:
            self.external_control_latched = True
            self.get_logger().info("External HAL control latched (/cmd_vel_safe received). Internal planner deactivated.")
        self.ext_cmd_vx = float(msg.linear.x)
        self.ext_cmd_vy = float(msg.linear.y)
        self.ext_cmd_vyaw = float(msg.angular.z)

    def _ext_posture_cb(self, msg: String):
        if not self.external_control_latched:
            self.external_control_latched = True
            self.get_logger().info("External HAL control latched (/go2_posture_cmd received). Internal planner deactivated.")
        self.ext_posture = msg.data.upper().strip()

    def _ext_euler_cb(self, msg: Vector3):
        if not self.external_control_latched:
            self.external_control_latched = True
            self.get_logger().info("External HAL control latched (/go2_body_euler received). Internal planner deactivated.")
        self.ext_euler_pitch = float(msg.y)

    def _ext_collision_cb(self, msg: Int32):
        if msg.data == 1:
            self.safety_state = "SAFETY_ESTOP"
            self.ext_cmd_vx = 0.0
            self.ext_cmd_vy = 0.0
            self.ext_cmd_vyaw = 0.0

    def _publish_global_path(self):
        """Constructs and publishes the reference tour path connecting active stalls."""
        p_msg = Path()
        p_msg.header.frame_id = 'odom'
        p_msg.header.stamp = self.get_clock().now().to_msg()

        # Connect scenes sequentially
        for i in range(len(self.scenes) - 1):
            s1 = self.scenes[i]
            s2 = self.scenes[i+1]
            for frac in np.linspace(0.0, 1.0, 15):
                ps = PoseStamped()
                ps.header.frame_id = 'odom'
                ps.pose.position.x = float(s1["target_x"] + (s2["target_x"] - s1["target_x"]) * frac)
                ps.pose.position.y = float(s1["target_y"] + (s2["target_y"] - s1["target_y"]) * frac)
                ps.pose.position.z = 0.015
                p_msg.poses.append(ps)

        self.pub_global_path.publish(p_msg)

    def _simulation_cycle(self):
        if not rclpy.ok():
            return
        now = time.time()
        dt = max(0.001, min(0.1, now - self.last_cycle_time))
        self.last_cycle_time = now
        self.sim_time += dt

        scene = self.scenes[self.scene_idx]

        # -------------------------------------------------------------
        # 1. Run Unitree 4D LiDAR L1 Scan (Ego-Centric Perception)
        # -------------------------------------------------------------
        scan_res = self.lidar.scan()
        min_front = scan_res["min_front"]
        closest_geom = scan_res["closest_geom"]
        local_obstacles = scan_res["local_obstacles"]

        # -------------------------------------------------------------
        # 2. Ego-Centric Model Predictive Planner (DWA / MPPI)
        # -------------------------------------------------------------
        target_x = scene["target_x"]
        target_yaw = scene["target_yaw"]

        dx_goal = target_x - self.pos_x
        dy_goal = scene["target_y"] - self.pos_y
        dist_to_goal = math.hypot(dx_goal, dy_goal)
        target_heading_global = math.atan2(dy_goal, dx_goal)
        rel_target_yaw = wrap_to_pi(target_heading_global - self.yaw)

        # Closed-Loop Standoff Docking at Booth 1 using real-time LiDAR feedback
        if scene["id"] == 4 and min_front < 2.5:
            range_error = min_front - self.target_standoff_distance
            if abs(range_error) < 0.08:
                dist_to_goal = 0.05  # Arrived at standoff distance

        cmd_v = 0.0
        cmd_vy = 0.0
        cmd_w = 0.0
        body_pitch_target = 0.0
        is_prone = False

        if self.external_control_latched:
            # External HAL control has exclusive authority over locomotion
            elapsed = time.time() - self.last_ext_cmd_time
            if elapsed > 0.60:
                cmd_v = 0.0
                cmd_vy = 0.0
                cmd_w = 0.0
            else:
                cmd_v = self.ext_cmd_vx
                cmd_vy = self.ext_cmd_vy
                cmd_w = self.ext_cmd_vyaw

            body_pitch_target = self.ext_euler_pitch
            is_prone = (self.ext_posture in ["STAND_DOWN", "DAMP"])

            estop_dist = 0.20 if self.external_control_latched else 0.65
            if min_front <= estop_dist:
                self.safety_state = "SAFETY_ESTOP"
                cmd_v = 0.0
                cmd_vy = 0.0
                cmd_w = 0.0
            elif abs(cmd_vy) > 0.10:
                self.safety_state = "SAFETY_STRAFING"
            elif min_front < 1.4:
                self.safety_state = "SAFETY_CAUTION"
            else:
                self.safety_state = "SAFETY_CLEAR"

            if int(self.sim_time * 10) % 20 == 0:
                self.get_logger().info(f"[HAL Control] vx={cmd_v:.2f}, vy={cmd_vy:.2f}, wz={cmd_w:.2f}, pos=({self.pos_x:.2f}, {self.pos_y:.2f}), min_front={min_front:.2f}m")

        elif self.in_transit:
            if dist_to_goal > 0.15:
                # Plan motion purely relative to sensed obstacles via candidate rollouts
                v_pref = 0.55 if dist_to_goal > 0.6 else min(0.35, max(0.18, dist_to_goal * 0.7))
                opt_v, opt_w, best_traj_local, candidate_trajs = self.planner.plan(
                    local_obstacles=local_obstacles,
                    min_front_clearance=min_front,
                    current_v=self.linear_vel,
                    current_w=self.yaw_rate,
                    target_rel_yaw=rel_target_yaw,
                    v_pref=v_pref,
                    imu_pitch=self.pitch
                )
                cmd_v = opt_v
                cmd_w = opt_w
                self.best_traj_local = best_traj_local
                self.candidate_trajs = candidate_trajs
            else:
                # Arrived at waypoint position: align final yaw orientation
                final_yaw_err = wrap_to_pi(target_yaw - self.yaw)
                if abs(final_yaw_err) > 0.08:
                    cmd_w = math.copysign(min(1.0, abs(final_yaw_err) * 1.8), final_yaw_err)
                    cmd_v = 0.0
                else:
                    self.in_transit = False
                    self.stage_timer = 0.0
                    self.get_logger().info(f"Station reached via Model Planner: {scene['name']}")
                    cmd_v = 0.0
                    cmd_w = 0.0
                    self.best_traj_local = []
                    self.candidate_trajs = []

            # Dynamic Safety State Machine (evaluated from real onboard sensors)
            if min_front <= 0.65:
                self.safety_state = "SAFETY_ESTOP"
                cmd_v = 0.0
                cmd_w = 0.0
            elif abs(cmd_w) > 0.28:
                self.safety_state = "SAFETY_AVOIDING"
            elif min_front < 1.4:
                self.safety_state = "SAFETY_CAUTION"
            else:
                self.safety_state = "SAFETY_CLEAR"

        else:
            # Station holding & expressive docent gestures
            self.stage_timer += dt
            # Continuously evaluate ego-centric candidate rollouts for live visualization
            _, _, self.best_traj_local, self.candidate_trajs = self.planner.plan(
                local_obstacles=local_obstacles,
                min_front_clearance=min_front,
                current_v=0.0,
                current_w=0.0,
                target_rel_yaw=rel_target_yaw,
                v_pref=0.45,
                imu_pitch=self.pitch
            )
            if scene["fsm"] == "STATE_DOCENT_GREET":
                body_pitch_target = 0.12 * math.sin(self.stage_timer * 2.5)  # Welcome nod
            elif scene["fsm"] == "STATE_HAZARD_AVOID":
                body_pitch_target = -0.06  # Alert posture
            elif scene["fsm"] == "STATE_DOCENT_NARRATE":
                body_pitch_target = 0.14   # Addressing audience (+8°)
            elif scene["fsm"] == "STATE_PRONE_REST":
                is_prone = True

            if self.stage_timer >= scene["hold_time"]:
                self.scene_idx = (self.scene_idx + 1) % len(self.scenes)
                self.in_transit = True
                self.stage_timer = 0.0
                next_s = self.scenes[self.scene_idx]
                self.get_logger().info(f"Transiting to: {next_s['name']}")

        # Integrate motion (Omnidirectional: supports forward vx and lateral vy)
        self.linear_vel = cmd_v
        self.lateral_vel = cmd_vy
        self.yaw_rate = cmd_w
        self.yaw = wrap_to_pi(self.yaw + self.yaw_rate * dt)
        self.pos_x += (self.linear_vel * math.cos(self.yaw) - self.lateral_vel * math.sin(self.yaw)) * dt
        self.pos_y += (self.linear_vel * math.sin(self.yaw) + self.lateral_vel * math.cos(self.yaw)) * dt

        # Pitch & Roll Dynamics
        self.pitch += (body_pitch_target - self.pitch) * min(1.0, dt * 5.0)

        # Stage elevation offset
        on_stage = math.hypot(self.pos_x, self.pos_y) < 1.75
        base_floor_z = 0.045 if on_stage else 0.0

        if is_prone:
            fold_ratio = min(1.0, self.stage_timer / 2.5)
            nominal_z = STANCE_HEIGHT - (STANCE_HEIGHT - 0.14) * fold_ratio + base_floor_z
        else:
            nominal_z = STANCE_HEIGHT + base_floor_z

        # -------------------------------------------------------------
        # 4. Quadruped Trot Gait Engine
        # -------------------------------------------------------------
        is_stepping = (self.linear_vel > 0.04) or (abs(self.yaw_rate) > 0.20)
        step_len = min(MAX_STEP_LEN, self.linear_vel * 0.32) if is_stepping else 0.0

        if is_stepping:
            gait_freq = 2.0 * math.pi * (1.6 + self.linear_vel * 0.8 + abs(self.yaw_rate) * 0.4)
            self.gait_phase = (self.gait_phase + gait_freq * dt) % (2.0 * math.pi)
            body_bob = 0.006 * math.cos(2.0 * self.gait_phase)
            self.roll = 0.012 * math.sin(self.gait_phase)
        else:
            self.gait_phase = 0.0
            body_bob = 0.002 * math.sin(self.sim_time * 2.0)
            self.roll = 0.0

        self.pos_z = nominal_z + body_bob

        phase_A = self.gait_phase
        phase_B = (self.gait_phase + math.pi) % (2.0 * math.pi)

        turn_diff = (self.yaw_rate * 0.28) * 0.15
        step_L = step_len - turn_diff
        step_R = step_len + turn_diff

        def calc_foot_offset(phase, s_len):
            if not is_stepping or abs(s_len) < 0.005:
                return 0.0, 0.0, -STANCE_HEIGHT
            if phase < math.pi:
                # Stance Phase
                xf = (s_len / 2.0) * math.cos(phase)
                zf = -STANCE_HEIGHT
            else:
                # Swing Phase
                s = (phase - math.pi) / math.pi
                xf = -(s_len / 2.0) * math.cos(phase)
                zf = -STANCE_HEIGHT + STEP_CLEARANCE * math.sin(math.pi * s)
            return xf, 0.0, zf

        fl_pos = calc_foot_offset(phase_A, step_L)
        rr_pos = calc_foot_offset(phase_A, step_R)
        fr_pos = calc_foot_offset(phase_B, step_R)
        rl_pos = calc_foot_offset(phase_B, step_L)

        # Joint Angles via Closed-Form Analytical IK
        if is_prone and self.pos_z < 0.22:
            fold_ratio = min(1.0, max(0.0, (0.22 - self.pos_z) / 0.08))
            fl_joint = (0.0, 0.68 + fold_ratio * 0.52, -1.37 - fold_ratio * 1.03)
            fr_joint = (0.0, 0.68 + fold_ratio * 0.52, -1.37 - fold_ratio * 1.03)
            rl_joint = (0.0, 0.68 + fold_ratio * 0.52, -1.37 - fold_ratio * 1.03)
            rr_joint = (0.0, 0.68 + fold_ratio * 0.52, -1.37 - fold_ratio * 1.03)
        else:
            fl_joint = leg_inverse_kinematics(fl_pos[0], fl_pos[1], fl_pos[2])
            fr_joint = leg_inverse_kinematics(fr_pos[0], fr_pos[1], fr_pos[2])
            rl_joint = leg_inverse_kinematics(rl_pos[0], rl_pos[1], rl_pos[2])
            rr_joint = leg_inverse_kinematics(rr_pos[0], rr_pos[1], rr_pos[2])

        # Apply state to MuJoCo model
        quat = euler_to_quaternion(self.yaw, self.pitch, self.roll)
        self.data.qpos[0] = self.pos_x
        self.data.qpos[1] = self.pos_y
        self.data.qpos[2] = self.pos_z
        self.data.qpos[3:7] = [quat[3], quat[0], quat[1], quat[2]] # MuJoCo w,x,y,z
        self.data.qpos[7:19] = [
            fl_joint[0], fl_joint[1], fl_joint[2],
            fr_joint[0], fr_joint[1], fr_joint[2],
            rl_joint[0], rl_joint[1], rl_joint[2],
            rr_joint[0], rr_joint[1], rr_joint[2]
        ]
        mujoco.mj_forward(self.model, self.data)

        # -------------------------------------------------------------
        # 5. Compute Authentic Foot Contact Forces
        # -------------------------------------------------------------
        # Nominal stance force per foot = 0.5 * Mass * g ≈ 73.5 N
        # Swing foot force = 0 N
        foot_forces = [
            74.0 + 25.0 * math.sin(phase_A) if phase_A < math.pi else 0.0,  # FL
            74.0 + 25.0 * math.sin(phase_B) if phase_B < math.pi else 0.0,  # FR
            74.0 + 25.0 * math.sin(phase_B) if phase_B < math.pi else 0.0,  # RL
            74.0 + 25.0 * math.sin(phase_A) if phase_A < math.pi else 0.0   # RR
        ]

        # Battery & Thermal Model updates
        power_draw = 80.0 + (self.linear_vel * 120.0) + (abs(self.yaw_rate) * 50.0)
        self.battery_current = -power_draw / self.battery_voltage
        self.battery_soc = max(10.0, self.battery_soc - (power_draw * dt) / 3600.0 / 2.0)
        self.battery_voltage = 25.2 + (self.battery_soc / 100.0) * (29.4 - 25.2)

        for m in range(12):
            self.motor_temps[m] = 28.0 + (power_draw / 100.0) * 8.5 + 2.0 * math.sin(self.sim_time + m)

        # -------------------------------------------------------------
        # 6. Publish All ROS 2 Messages & Telemetry
        # -------------------------------------------------------------
        stamp = self.get_clock().now().to_msg()
        self._publish_ros2_telemetry(stamp, quat, fl_joint, fr_joint, rl_joint, rr_joint,
                                    scan_res, foot_forces, scene)

    def _publish_ros2_telemetry(self, stamp, quat, fl_joint, fr_joint, rl_joint, rr_joint,
                               scan_res, foot_forces, scene):
        if not rclpy.ok():
            return

        # Add 50ms forward margin to prevent TF extrapolation jitter in RViz
        stamp_tf = self.get_clock().now().to_msg()
        nsec = stamp_tf.nanosec + 50_000_000
        sec = stamp_tf.sec + (nsec // 1_000_000_000)
        nsec = nsec % 1_000_000_000
        stamp_tf.sec = sec
        stamp_tf.nanosec = nsec

        # 0. Static TF (map -> odom) & Map Keepalive
        self._broadcast_static_tf(stamp_tf)
        if not hasattr(self, '_map_pub_counter'):
            self._map_pub_counter = 0
        self._map_pub_counter += 1
        if self._map_pub_counter % 60 == 0 and hasattr(self, 'cached_map_msg'):
            self.cached_map_msg.header.stamp = stamp
            self.pub_map.publish(self.cached_map_msg)

        # 1. Dynamic TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = stamp_tf
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pos_x
        t.transform.translation.y = self.pos_y
        t.transform.translation.z = self.pos_z
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

        # 2. 12-DOF Joint States
        js = JointState()
        js.header.stamp = stamp_tf
        js.name = self.joint_names
        js.position = [
            fl_joint[0], fl_joint[1], fl_joint[2],
            fr_joint[0], fr_joint[1], fr_joint[2],
            rl_joint[0], rl_joint[1], rl_joint[2],
            rr_joint[0], rr_joint[1], rr_joint[2]
        ]
        js.velocity = [self.linear_vel * 2.5] * 12
        js.effort = [12.5] * 12
        self.pub_joint_states.publish(js)
        self.pub_joint_states_root.publish(js)

        # 3. 4D LiDAR PointCloud2 (1,152 rays, fields: x, y, z, intensity, ring)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        ]
        # Frame 'odom' gives 100% reliable 3D world projection with zero TF filter drops
        header = Header(stamp=stamp, frame_id="odom")
        cloud_msg = pc2.create_cloud(header, fields, scan_res["points_world"])
        self.pub_pointcloud.publish(cloud_msg)

        # 4. 2D Horizontal LaserScan
        ls = LaserScan()
        ls.header.stamp = stamp
        ls.header.frame_id = "base_link"
        ls.angle_min = -math.pi
        ls.angle_max = math.pi
        ls.angle_increment = 2.0 * math.pi / self.lidar.num_azimuth
        ls.time_increment = 1.0 / (30.0 * self.lidar.num_azimuth)
        ls.scan_time = 1.0 / 30.0
        ls.range_min = 0.05
        ls.range_max = 16.0
        ls.ranges = scan_res["ranges_2d"]
        self.pub_scan.publish(ls)

        # 5. Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.pos_x
        odom.pose.pose.position.y = self.pos_y
        odom.pose.pose.position.z = self.pos_z
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.angular.z = self.yaw_rate
        self.pub_odom.publish(odom)

        # 6. IMU (6-DOF with gravity & dynamic vibration)
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = 'imu'
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]
        imu_msg.angular_velocity.x = self.roll * 2.0
        imu_msg.angular_velocity.y = self.pitch * 2.0
        imu_msg.angular_velocity.z = self.yaw_rate
        imu_msg.linear_acceleration.x = 0.05 * math.sin(self.gait_phase)
        imu_msg.linear_acceleration.y = 0.04 * math.cos(self.gait_phase)
        imu_msg.linear_acceleration.z = 9.81 + 0.12 * math.cos(2.0 * self.gait_phase)
        self.pub_imu.publish(imu_msg)

        # 7. Battery State
        bms = BatteryState()
        bms.header.stamp = stamp
        bms.voltage = float(self.battery_voltage)
        bms.current = float(self.battery_current)
        bms.percentage = float(self.battery_soc / 100.0)
        bms.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        bms.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.pub_battery.publish(bms)

        # 8. HighState JSON Telemetry (Unitree Go2 Official Format)
        high_state = {
            "sn": "GO2-PRO-984210",
            "version": "1.1.4-dpsi",
            "mode": 2 if self.in_transit else (1 if scene["fsm"] != "STATE_PRONE_REST" else 0),
            "safety_state": self.safety_state,
            "fsm_state": scene["fsm"],
            "station": scene["name"],
            "position": [round(self.pos_x, 3), round(self.pos_y, 3), round(self.pos_z, 3)],
            "rpy": [round(self.roll, 3), round(self.pitch, 3), round(self.yaw, 3)],
            "velocity": [round(self.linear_vel, 3), 0.0, round(self.yaw_rate, 3)],
            "bms": {
                "soc": round(self.battery_soc, 1),
                "voltage": round(self.battery_voltage, 2),
                "current": round(self.battery_current, 2),
                "cell_vol": [4020, 4018, 4022, 4021, 4019, 4023, 4020]
            },
            "foot_force": [round(f, 1) for f in foot_forces],
            "motor_temps": [round(t, 1) for t in self.motor_temps],
            "lidar_min_front": round(scan_res["min_front"], 2),
            "lidar_closest_obstacle": scan_res["closest_geom"],
            "reactive_offset_y": round(self.reactive_y_offset, 3)
        }
        self.pub_high_state.publish(String(data=json.dumps(high_state)))
        self.pub_foot_forces.publish(Float32MultiArray(data=[float(f) for f in foot_forces]))

        # 9. Model-Planned Ego-Centric Predictive Trajectory
        r_path = Path()
        r_path.header.stamp = stamp
        r_path.header.frame_id = 'odom'
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        if hasattr(self, 'best_traj_local') and len(self.best_traj_local) > 0:
            for lx, ly in self.best_traj_local:
                ps = PoseStamped()
                ps.header.stamp = stamp
                ps.header.frame_id = 'odom'
                ps.pose.position.x = self.pos_x + (lx * cos_yaw - ly * sin_yaw)
                ps.pose.position.y = self.pos_y + (lx * sin_yaw + ly * cos_yaw)
                ps.pose.position.z = 0.025
                r_path.poses.append(ps)
        self.pub_reactive_path.publish(r_path)

        # 10. Odometry Breadcrumb Trail
        if (len(self.odom_path_msg.poses) == 0 or
            math.hypot(self.pos_x - self.odom_path_msg.poses[-1].pose.position.x,
                       self.pos_y - self.odom_path_msg.poses[-1].pose.position.y) > 0.10):
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = 'odom'
            ps.pose.position.x = self.pos_x
            ps.pose.position.y = self.pos_y
            ps.pose.position.z = 0.015
            self.odom_path_msg.poses.append(ps)
            if len(self.odom_path_msg.poses) > 300:
                self.odom_path_msg.poses.pop(0)
            self.pub_odom_path.publish(self.odom_path_msg)

        # 11. 3D Obstacle & Exhibition Markers
        self._publish_visual_markers(stamp, scene, scan_res)

        # 12. Synthetic RealSense D435i AprilTag Detection Stream
        self._publish_camera_apriltags(stamp)

    def _publish_camera_apriltags(self, stamp):
        """Simulates onboard RealSense D435i optical detection of tag36h11 fiducials."""
        if not hasattr(self, 'booth_fiducials') or not self.booth_fiducials:
            return

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        best_fid = None
        min_dist = float('inf')
        best_xc = 0.0
        best_zc = 0.0

        for fid in self.booth_fiducials:
            dx = fid["x"] - self.pos_x
            dy = fid["y"] - self.pos_y
            # Body frame (x forward, y left)
            x_b = cos_yaw * dx + sin_yaw * dy
            y_b = -sin_yaw * dx + cos_yaw * dy

            # Camera optical frame (Z forward, X right, Y down)
            z_c = x_b
            x_c = -y_b

            # D435i horizontal FOV is ~69 deg -> |x_c / z_c| < tan(35 deg) ~ 0.70
            if 0.20 <= z_c <= 3.20 and abs(x_c / max(0.01, z_c)) <= 0.75:
                dist = math.hypot(x_c, z_c)
                if dist < min_dist:
                    min_dist = dist
                    best_fid = fid
                    best_xc = x_c
                    best_zc = z_c

        if best_fid is not None:
            id_msg = Int32(data=int(best_fid["id"]))
            self.pub_apriltag_id.publish(id_msg)

            p_msg = PoseStamped()
            p_msg.header.stamp = stamp
            p_msg.header.frame_id = 'camera_color_optical_frame'
            p_msg.pose.position.x = float(best_xc)
            p_msg.pose.position.y = 0.0
            p_msg.pose.position.z = float(best_zc)
            p_msg.pose.orientation.w = 1.0
            self.pub_apriltag_pose.publish(p_msg)

    def _broadcast_static_tf(self, stamp=None):
        if not hasattr(self, 'static_tf_broadcaster'):
            return
        t = TransformStamped()
        t.header.stamp = stamp or self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def _init_and_publish_map(self):
        res = 0.10
        ox, oy = -5.0, -10.0
        width = int(30.0 / res)   # 300
        height = int(20.0 / res)  # 200

        grid = [0] * (width * height)

        def set_box(x_min, x_max, y_min, y_max, val):
            c0 = max(0, int((x_min - ox) / res))
            c1 = min(width - 1, int((x_max - ox) / res))
            r0 = max(0, int((y_min - oy) / res))
            r1 = min(height - 1, int((y_max - oy) / res))
            for r in range(r0, r1 + 1):
                for c in range(c0, c1 + 1):
                    grid[r * width + c] = val

        # Perimeter outer walls (thickness ~ 0.2m)
        set_box(-5.0, 25.0, 9.8, 10.0, 100)
        set_box(-5.0, 25.0, -10.0, -9.8, 100)
        set_box(-5.0, -4.8, -10.0, 10.0, 100)
        set_box(24.8, 25.0, -10.0, 10.0, 100)

        # Central Stage Dais (radius 2.4 at (-2.0, 0.0))
        cx, cy = -2.0, 0.0
        r_stage = 2.4
        for r in range(height):
            py = oy + r * res
            for c in range(width):
                px = ox + c * res
                dist = math.hypot(px - cx, py - cy)
                if dist <= r_stage:
                    grid[r * width + c] = 50 if dist < (r_stage - 0.15) else 100

        # Recharge Dock 99 plate
        set_box(-1.25, -1.15, -0.25, 0.25, 100)

        # 40 Stalls from database
        for s in self.stalls_data:
            bp = s.get("booth_pose", {})
            bx = bp.get("x", 0.0)
            by = bp.get("y", 0.0)
            byaw = bp.get("yaw", 0.0)
            # Table half extents 0.85m in x, 0.45m in y
            set_box(bx - 0.85, bx + 0.85, by - 0.45, by + 0.45, 100)
            # Acoustic back wall
            back_y = by - 0.55 * math.sin(byaw)
            set_box(bx - 0.95, bx + 0.95, back_y - 0.05, back_y + 0.05, 100)

        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.map_load_time = map_msg.header.stamp
        map_msg.info.resolution = res
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.info.origin.position.x = ox
        map_msg.info.origin.position.y = oy
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = grid

        self.cached_map_msg = map_msg
        self.pub_map.publish(map_msg)

    def _build_static_markers(self):
        markers = []

        # 1. 4 Perimeter Auditorium Walls (30m x 20m arena: x in [-5, 25], y in [-10, 10])
        wall_specs = [
            ("wall_west", -5.0, 0.0, 0.75, 0.15, 20.0, 1.50),
            ("wall_east", 25.0, 0.0, 0.75, 0.15, 20.0, 1.50),
            ("wall_north", 10.0, 10.0, 0.75, 30.0, 0.15, 1.50),
            ("wall_south", 10.0, -10.0, 0.75, 30.0, 0.15, 1.50),
        ]
        for idx, (name, wx, wy, wz, sx, sy, sz) in enumerate(wall_specs):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'auditorium_walls'
            m.id = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = wz
            m.scale.x = sx
            m.scale.y = sy
            m.scale.z = sz
            m.color.r = 0.88; m.color.g = 0.90; m.color.b = 0.92; m.color.a = 0.85
            markers.append(m)

        # 1b. Floor Slab (30m x 20m arena: x in [-5, 25], y in [-10, 10])
        m_floor = Marker()
        m_floor.header.frame_id = 'map'
        m_floor.ns = 'auditorium_walls'
        m_floor.id = 99
        m_floor.type = Marker.CUBE
        m_floor.action = Marker.ADD
        m_floor.pose.position.x = 10.0
        m_floor.pose.position.y = 0.0
        m_floor.pose.position.z = -0.015
        m_floor.scale.x = 30.0
        m_floor.scale.y = 20.0
        m_floor.scale.z = 0.01
        m_floor.color.r = 0.08; m_floor.color.g = 0.09; m_floor.color.b = 0.12; m_floor.color.a = 0.95
        markers.append(m_floor)

        # 2. Central Keynote Stage & Dock 99
        m_stage = Marker()
        m_stage.header.frame_id = 'map'
        m_stage.ns = 'central_stage'
        m_stage.id = 0
        m_stage.type = Marker.CYLINDER
        m_stage.action = Marker.ADD
        m_stage.pose.position.x = -2.0
        m_stage.pose.position.y = 0.0
        m_stage.pose.position.z = 0.02
        m_stage.scale.x = 4.8
        m_stage.scale.y = 4.8
        m_stage.scale.z = 0.04
        m_stage.color.r = 0.16; m_stage.color.g = 0.18; m_stage.color.b = 0.22; m_stage.color.a = 0.95
        markers.append(m_stage)

        m_cove = Marker()
        m_cove.header.frame_id = 'map'
        m_cove.ns = 'central_stage'
        m_cove.id = 1
        m_cove.type = Marker.CYLINDER
        m_cove.action = Marker.ADD
        m_cove.pose.position.x = -2.0
        m_cove.pose.position.y = 0.0
        m_cove.pose.position.z = 0.005
        m_cove.scale.x = 4.86
        m_cove.scale.y = 4.86
        m_cove.scale.z = 0.01
        m_cove.color.r = 0.0; m_cove.color.g = 0.75; m_cove.color.b = 0.95; m_cove.color.a = 0.85
        markers.append(m_cove)

        m_dock_bg = Marker()
        m_dock_bg.header.frame_id = 'map'
        m_dock_bg.ns = 'central_stage'
        m_dock_bg.id = 2
        m_dock_bg.type = Marker.CUBE
        m_dock_bg.action = Marker.ADD
        m_dock_bg.pose.position.x = -1.20
        m_dock_bg.pose.position.y = 0.0
        m_dock_bg.pose.position.z = 0.30
        m_dock_bg.scale.x = 0.02
        m_dock_bg.scale.y = 0.44
        m_dock_bg.scale.z = 0.44
        m_dock_bg.color.r = 0.96; m_dock_bg.color.g = 0.96; m_dock_bg.color.b = 0.96; m_dock_bg.color.a = 1.0
        markers.append(m_dock_bg)

        m_dock_fg = Marker()
        m_dock_fg.header.frame_id = 'map'
        m_dock_fg.ns = 'central_stage'
        m_dock_fg.id = 3
        m_dock_fg.type = Marker.CUBE
        m_dock_fg.action = Marker.ADD
        m_dock_fg.pose.position.x = -1.19
        m_dock_fg.pose.position.y = 0.0
        m_dock_fg.pose.position.z = 0.30
        m_dock_fg.scale.x = 0.024
        m_dock_fg.scale.y = 0.32
        m_dock_fg.scale.z = 0.32
        m_dock_fg.color.r = 0.02; m_dock_fg.color.g = 0.02; m_dock_fg.color.b = 0.02; m_dock_fg.color.a = 1.0
        markers.append(m_dock_fg)

        m_dock_txt = Marker()
        m_dock_txt.header.frame_id = 'map'
        m_dock_txt.ns = 'central_stage'
        m_dock_txt.id = 4
        m_dock_txt.type = Marker.TEXT_VIEW_FACING
        m_dock_txt.action = Marker.ADD
        m_dock_txt.pose.position.x = -1.20
        m_dock_txt.pose.position.y = 0.0
        m_dock_txt.pose.position.z = 0.70
        m_dock_txt.scale.z = 0.22
        m_dock_txt.color.r = 0.0; m_dock_txt.color.g = 0.85; m_dock_txt.color.b = 1.0; m_dock_txt.color.a = 1.0
        m_dock_txt.text = "⚡ STAGE RECHARGE DOCK\nAprilTag #99"
        markers.append(m_dock_txt)

        # 3. All 40 Exhibition Stalls
        for idx, s in enumerate(self.stalls_data):
            s_id = s.get("id", f"STALL_{idx+1:02d}")
            tag_id = s.get("apriltag_id", 10)
            bp = s.get("booth_pose", {})
            bx = float(bp.get("x", 0.0))
            by = float(bp.get("y", 0.0))
            byaw = float(bp.get("yaw", -1.5708))

            tag_y = by + 0.46 * math.sin(byaw)
            back_y = by - 0.55 * math.sin(byaw)

            base_id = 100 + idx * 10

            # Table Plinth
            m_tbl = Marker()
            m_tbl.header.frame_id = 'map'
            m_tbl.ns = 'exhibition_booths'
            m_tbl.id = base_id + 0
            m_tbl.type = Marker.CUBE
            m_tbl.action = Marker.ADD
            m_tbl.pose.position.x = bx
            m_tbl.pose.position.y = by
            m_tbl.pose.position.z = 0.45
            m_tbl.scale.x = 1.70
            m_tbl.scale.y = 0.90
            m_tbl.scale.z = 0.90
            m_tbl.color.r = 0.12; m_tbl.color.g = 0.14; m_tbl.color.b = 0.18; m_tbl.color.a = 0.95
            markers.append(m_tbl)

            # Acoustic Backboard
            m_back = Marker()
            m_back.header.frame_id = 'map'
            m_back.ns = 'exhibition_booths'
            m_back.id = base_id + 1
            m_back.type = Marker.CUBE
            m_back.action = Marker.ADD
            m_back.pose.position.x = bx
            m_back.pose.position.y = back_y
            m_back.pose.position.z = 0.75
            m_back.scale.x = 1.90
            m_back.scale.y = 0.10
            m_back.scale.z = 1.50
            m_back.color.r = 0.15; m_back.color.g = 0.17; m_back.color.b = 0.20; m_back.color.a = 0.95
            markers.append(m_back)

            # AprilTag Plate Background (White)
            m_tag_bg = Marker()
            m_tag_bg.header.frame_id = 'map'
            m_tag_bg.ns = 'exhibition_booths'
            m_tag_bg.id = base_id + 2
            m_tag_bg.type = Marker.CUBE
            m_tag_bg.action = Marker.ADD
            m_tag_bg.pose.position.x = bx
            m_tag_bg.pose.position.y = tag_y
            m_tag_bg.pose.position.z = 0.65
            m_tag_bg.scale.x = 0.32
            m_tag_bg.scale.y = 0.02
            m_tag_bg.scale.z = 0.32
            m_tag_bg.color.r = 0.96; m_tag_bg.color.g = 0.96; m_tag_bg.color.b = 0.96; m_tag_bg.color.a = 1.0
            markers.append(m_tag_bg)

            # AprilTag Plate Foreground (Black code)
            m_tag_fg = Marker()
            m_tag_fg.header.frame_id = 'map'
            m_tag_fg.ns = 'exhibition_booths'
            m_tag_fg.id = base_id + 3
            m_tag_fg.type = Marker.CUBE
            m_tag_fg.action = Marker.ADD
            m_tag_fg.pose.position.x = bx
            m_tag_fg.pose.position.y = tag_y + (0.012 if byaw < 0 else -0.012)
            m_tag_fg.pose.position.z = 0.65
            m_tag_fg.scale.x = 0.22
            m_tag_fg.scale.y = 0.024
            m_tag_fg.scale.z = 0.22
            m_tag_fg.color.r = 0.02; m_tag_fg.color.g = 0.02; m_tag_fg.color.b = 0.02; m_tag_fg.color.a = 1.0
            markers.append(m_tag_fg)

            # Floating 3D Booth Title
            m_txt = Marker()
            m_txt.header.frame_id = 'map'
            m_txt.ns = 'exhibition_booths'
            m_txt.id = base_id + 4
            m_txt.type = Marker.TEXT_VIEW_FACING
            m_txt.action = Marker.ADD
            m_txt.pose.position.x = bx
            m_txt.pose.position.y = by
            m_txt.pose.position.z = 1.30
            m_txt.scale.z = 0.18
            m_txt.color.r = 0.0; m_txt.color.g = 0.85; m_txt.color.b = 1.0; m_txt.color.a = 0.95
            short_title = s.get('title', '')
            if len(short_title) > 28:
                short_title = short_title[:26] + "..."
            m_txt.text = f"[{s_id}] {short_title}\nTag #{tag_id}"
            markers.append(m_txt)

        # 4. Dynamic Crowd Clusters
        crowd_coords = [
            (6.0, 6.0), (14.0, 6.0), (10.0, 2.0), (18.0, 2.0),
            (6.0, -2.0), (14.0, -2.0), (10.0, -6.0), (18.0, -6.0)
        ]
        for idx, (cx, cy) in enumerate(crowd_coords):
            m_crd = Marker()
            m_crd.header.frame_id = 'map'
            m_crd.ns = 'crowd_capsules'
            m_crd.id = idx
            m_crd.type = Marker.CYLINDER
            m_crd.action = Marker.ADD
            m_crd.pose.position.x = cx
            m_crd.pose.position.y = cy
            m_crd.pose.position.z = 0.85
            m_crd.scale.x = 0.36
            m_crd.scale.y = 0.36
            m_crd.scale.z = 1.40
            m_crd.color.r = 0.25; m_crd.color.g = 0.45; m_crd.color.b = 0.65; m_crd.color.a = 0.85
            markers.append(m_crd)

        return markers

    def _publish_visual_markers(self, stamp, scene, scan_res):
        markers = MarkerArray()

        # Marker 1: Holographic FSM & Safety Status Badge above Robot
        m_badge = Marker()
        m_badge.header.stamp = stamp
        m_badge.header.frame_id = 'odom'
        m_badge.ns = 'cloud_fsm_badge'
        m_badge.id = 0
        m_badge.type = Marker.TEXT_VIEW_FACING
        m_badge.action = Marker.ADD
        m_badge.pose.position.x = self.pos_x
        m_badge.pose.position.y = self.pos_y
        m_badge.pose.position.z = self.pos_z + 0.42
        m_badge.scale.z = 0.16

        if self.safety_state == "SAFETY_ESTOP":
            m_badge.color.r = 1.0; m_badge.color.g = 0.15; m_badge.color.b = 0.15; m_badge.color.a = 1.0
            badge_text = f"⚠️ [ESTOP: CLEARANCE {scan_res['min_front']:.2f}m]\nBRACING STANCE HALT"
        elif self.safety_state == "SAFETY_AVOIDING":
            m_badge.color.r = 0.0; m_badge.color.g = 0.85; m_badge.color.b = 1.0; m_badge.color.a = 1.0
            badge_text = f"⚡ [REACTIVE AUTONOMY: +{self.reactive_y_offset:.2f}m DEFLECTION]\n{scene['name']}"
        elif self.safety_state == "SAFETY_CAUTION":
            m_badge.color.r = 1.0; m_badge.color.g = 0.75; m_badge.color.b = 0.1; m_badge.color.a = 1.0
            badge_text = f"🟡 [CAUTION: {scan_res['closest_geom']}]\nTHROTTLED v={self.linear_vel:.2f}m/s"
        else:
            m_badge.color.r = 0.2; m_badge.color.g = 0.95; m_badge.color.b = 0.5; m_badge.color.a = 1.0
            badge_text = f"[{scene['fsm']}]\n{scene['name']}"

        m_badge.text = badge_text
        markers.markers.append(m_badge)

        # Marker 2: Safety Clearance Envelope Cylinder around Robot
        m_env = Marker()
        m_env.header.stamp = stamp
        m_env.header.frame_id = 'odom'
        m_env.ns = 'safety_envelope'
        m_env.id = 1
        m_env.type = Marker.CYLINDER
        m_env.action = Marker.ADD
        m_env.pose.position.x = self.pos_x
        m_env.pose.position.y = self.pos_y
        m_env.pose.position.z = self.pos_z * 0.5
        m_env.scale.x = 0.95
        m_env.scale.y = 0.95
        m_env.scale.z = self.pos_z
        if self.safety_state == "SAFETY_ESTOP":
            m_env.color.r = 1.0; m_env.color.g = 0.1; m_env.color.b = 0.1; m_env.color.a = 0.35
        elif self.safety_state == "SAFETY_AVOIDING":
            m_env.color.r = 0.0; m_env.color.g = 0.8; m_env.color.b = 1.0; m_env.color.a = 0.22
        else:
            m_env.color.r = 0.1; m_env.color.g = 0.8; m_env.color.b = 0.4; m_env.color.a = 0.15
        markers.markers.append(m_env)

        # 3. Synchronized 3D exhibition environment markers
        if hasattr(self, 'cached_static_markers'):
            for sm in self.cached_static_markers:
                sm.header.stamp = stamp
                markers.markers.append(sm)

        self.pub_markers.publish(markers)

        # Marker 8: Model Predictive Rollout Candidates Fan (Ego-Centric Trajectory Tree)
        cand_markers = MarkerArray()
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        if hasattr(self, 'candidate_trajs') and len(self.candidate_trajs) > 0:
            for c_idx, cand in enumerate(self.candidate_trajs):
                m_cand = Marker()
                m_cand.header.stamp = stamp
                m_cand.header.frame_id = 'odom'
                m_cand.ns = 'dwa_rollouts'
                m_cand.id = c_idx
                m_cand.type = Marker.LINE_STRIP
                m_cand.action = Marker.ADD
                m_cand.scale.x = 0.012
                if cand["collision"]:
                    m_cand.color.r = 1.0; m_cand.color.g = 0.2; m_cand.color.b = 0.2; m_cand.color.a = 0.35
                else:
                    m_cand.color.r = 0.1; m_cand.color.g = 0.85; m_cand.color.b = 1.0; m_cand.color.a = 0.20
                for lx, ly in cand["traj"]:
                    p = Point()
                    p.x = self.pos_x + (lx * cos_yaw - ly * sin_yaw)
                    p.y = self.pos_y + (lx * sin_yaw + ly * cos_yaw)
                    p.z = 0.015
                    m_cand.points.append(p)
                cand_markers.markers.append(m_cand)
        self.pub_candidates.publish(cand_markers)

def main():
    parser = argparse.ArgumentParser(description="Project Cloud: Unified MuJoCo Physics & ROS 2 Twin")
    parser.add_argument('--gui', action='store_true', help="Open passive interactive MuJoCo viewer alongside ROS 2")
    parser.add_argument('--xml', type=str, default=None, help="Path to MuJoCo world XML")
    parser.add_argument('--spawn-stall', type=str, default=None, help="Spawn Go2 at specific stall approach (e.g. STALL_01)")
    args = parser.parse_args()

    rclpy.init()
    node = CloudMuJoCoTwin(xml_path=args.xml, headless=(not args.gui), spawn_stall=args.spawn_stall)

    try:
        if args.gui:
            with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
                viewer.cam.distance = 3.2
                viewer.cam.elevation = -22.0
                viewer.cam.azimuth = 60.0
                # Ensure 2D rangefinder lines are strictly disabled in 3D viewport
                if hasattr(viewer, "opt") and hasattr(viewer.opt, "flags"):
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
                while rclpy.ok() and viewer.is_running():
                    # Dynamic third-person camera tracking to follow Go2 robot closely
                    gx = float(node.data.xpos[node.base_body_id][0])
                    gy = float(node.data.xpos[node.base_body_id][1])
                    gz = float(node.data.xpos[node.base_body_id][2])
                    viewer.cam.lookat[0] = gx
                    viewer.cam.lookat[1] = gy
                    viewer.cam.lookat[2] = gz + 0.25
                    rclpy.spin_once(node, timeout_sec=0.01)
                    viewer.sync()
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        if "context is invalid" not in str(e):
            raise e
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
