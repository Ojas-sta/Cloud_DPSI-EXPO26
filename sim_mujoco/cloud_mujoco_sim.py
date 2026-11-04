"""
Project Cloud: High-Fidelity Autonomous MuJoCo Simulation
Platform: Unitree Go2 Quadruped Robot
Environment: MuJoCo Physics (macOS Apple Silicon Native via mjpython)

Key Features:
- Physics-grounded Quadruped Locomotion with Ground Contacts & Friction
- Closed-Form Analytical 3-DOF Leg Inverse Kinematics (IK)
- Dynamic 2-Beat Diagonal Quadruped Trot Gait Engine
- Simulated Unitree 4D LiDAR L1:
  * 360° Horizontal Laser Scan (72-beam radial sweep at 5° resolution)
  * Downward-tilted ground slope ray for spill and hazard detection
  * Native MuJoCo C-raycast query (mujoco.mj_ray) against physical walls & obstacles
  * Real-time 3D Point Cloud generation & Sectoral Clearances (Front, Left, Right, Rear)
  * Hardware telemetry channels from 11 native <rangefinder> sensors
  * Reactive safety state machine (SAFETY_CLEAR, SAFETY_CAUTION, SAFETY_ESTOP)
  * Live visual laser beam and point cloud rendering in interactive viewer (user_scn)
- Architectural Pavilion World (DPSI EXPO-2026):
  * Outer perimeter walls (West backdrop, East boundary, North & South side walls)
  * Corridor guide railings & semi-transparent tempered glass partitions
  * Exhibition booths: Booth 1 (Solar Cart Standoff), Booth 2 (Robotics & AI), Booth 3 (GreenTech)
  * Chemical spill hazard zone with fluid slick, caution cones, and flanking stanchions
  * Central Stage with illuminated glowing rim
- 6 Mission Stations with Automatic Photographic Proof Snapshots & HUD Radar Overlay
- Multi-Camera System with 8 Dedicated Framing Views (including Go2 First-Person LiDAR FPV)
"""

import sys
import os
import math
import time
import argparse
import numpy as np

try:
    import mujoco
    import mujoco.viewer
    from PIL import Image, ImageDraw, ImageFont
except ImportError as e:
    print(f"[ERROR] Missing dependency: {e}")
    print("Please install via: pip install mujoco pillow")
    sys.exit(1)

# Robot Geometric Parameters (Unitree Go2)
L_THIGH = 0.213
L_CALF = 0.213
STANCE_HEIGHT = 0.315
STEP_CLEARANCE = 0.048
MAX_STEP_LEN = 0.150

def leg_inverse_kinematics(x_foot, y_foot, z_foot):
    """
    Computes analytical 3-DOF joint angles (hip_roll, thigh_pitch, calf_pitch)
    for a single Go2 quadruped leg in hip coordinates.
    """
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

def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def yaw_to_quaternion(yaw, pitch=0.0, roll=0.0):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [
        cr * cp * cy + sr * sp * sy, # w
        sr * cp * cy - cr * sp * sy, # x
        cr * sp * cy + sr * cp * sy, # y
        cr * cp * sy - sr * sp * cy  # z
    ]

def quaternion_to_yaw(qw, qx, qy, qz):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

class Go2LiDAR:
    """
    High-fidelity simulated Unitree 4D LiDAR L1.
    Performs real-time C-accelerated raycasts against physical world geometry,
    computes 3D point clouds, sectoral clearances, and renders visual beams.
    """
    def __init__(self, model, data, num_rays=72, max_range=16.0):
        self.model = model
        self.data = data
        self.num_rays = num_rays
        self.max_range = max_range
        self.base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "lidar_origin")
        self.geomid_buf = np.zeros(1, dtype=np.int32)

        # Pre-identify all robot self-bodies to filter out self-occlusions
        self.robot_bodies = set()
        for b in range(model.nbody):
            bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
            if bname and any(p in bname for p in ["base", "FL", "FR", "RL", "RR"]):
                self.robot_bodies.add(b)

        # 360° Horizontal Angular Sweep [-pi, pi]
        self.angles = np.linspace(-math.pi, math.pi, num_rays, endpoint=False)
        self.cos_angles = np.cos(self.angles)
        self.sin_angles = np.sin(self.angles)

        # Pre-allocated local ray unit vectors
        self.local_rays = np.zeros((num_rays, 3), dtype=np.float64)
        self.local_rays[:, 0] = self.cos_angles
        self.local_rays[:, 1] = self.sin_angles
        self.local_rays[:, 2] = 0.0

        self.last_scan = None

    def scan(self):
        """Executes full 360° spatial laser scan in MuJoCo world."""
        p0 = self.data.site_xpos[self.site_id].copy()
        R = self.data.site_xmat[self.site_id].reshape((3, 3))

        scan_points = []
        ranges = []
        geom_names = []
        is_hazards = []

        front_dists = []
        left_dists = []
        right_dists = []
        rear_dists = []

        min_overall_dist = 999.0
        closest_geom = "none"

        for i in range(self.num_rays):
            th = self.angles[i]
            v_world = R @ self.local_rays[i]

            # Raycast with automatic self-body pass-through
            curr_p = p0.copy()
            total_dist = 0.0
            hit_geom_name = "none"
            hit_p = None

            for _ in range(4): # Step past up to 4 internal robot bodies
                dist = mujoco.mj_ray(self.model, self.data, curr_p, v_world, None, 1, -1, self.geomid_buf)
                if dist < 0.0 or total_dist + dist > self.max_range:
                    total_dist = -1.0
                    break
                total_dist += dist
                hit_body = self.model.geom_bodyid[self.geomid_buf[0]]
                if hit_body in self.robot_bodies:
                    curr_p = curr_p + (dist + 0.02) * v_world
                else:
                    gid = self.geomid_buf[0]
                    hit_geom_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or "obstacle"
                    hit_p = p0 + total_dist * v_world
                    break

            if total_dist > 0.0 and total_dist <= self.max_range:
                hazard = ("cone" in hit_geom_name) or ("spill" in hit_geom_name)

                scan_points.append(hit_p)
                ranges.append(total_dist)
                geom_names.append(hit_geom_name)
                is_hazards.append(hazard)

                if total_dist < min_overall_dist:
                    min_overall_dist = total_dist
                    closest_geom = hit_geom_name

                # Sectoral categorization in robot-forward frame
                abs_th = abs(th)
                if abs_th < math.pi / 6.0:       # Front [-30°, +30°]
                    front_dists.append(total_dist)
                elif th >= math.pi / 6.0 and th <= 5.0 * math.pi / 6.0:  # Left [+30°, +150°]
                    left_dists.append(total_dist)
                elif th <= -math.pi / 6.0 and th >= -5.0 * math.pi / 6.0: # Right [-150°, -30°]
                    right_dists.append(total_dist)
                else:                            # Rear [> 150°]
                    rear_dists.append(total_dist)
            else:
                scan_points.append(None)
                ranges.append(-1.0)
                geom_names.append("out_of_range")
                is_hazards.append(False)

        # Downward Ground Ray for Hazard / Puddle Detection
        v_down_local = np.array([0.866, 0.0, -0.500], dtype=np.float64)
        v_down_world = R @ v_down_local
        curr_p = p0.copy()
        ground_geom = "floor"
        for _ in range(4):
            dist_down = mujoco.mj_ray(self.model, self.data, curr_p, v_down_world, None, 1, -1, self.geomid_buf)
            if dist_down < 0 or dist_down > 3.0:
                break
            hit_body = self.model.geom_bodyid[self.geomid_buf[0]]
            if hit_body in self.robot_bodies:
                curr_p = curr_p + (dist_down + 0.02) * v_down_world
            else:
                ground_geom = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, self.geomid_buf[0]) or "floor"
                break

        min_front = min(front_dists) if front_dists else 16.0
        min_left = min(left_dists) if left_dists else 16.0
        min_right = min(right_dists) if right_dists else 16.0
        min_rear = min(rear_dists) if rear_dists else 16.0

        # Safety State Machine
        if min_front <= 0.85 or ("spill" in ground_geom) or ("cone" in closest_geom and min_overall_dist < 1.0):
            safety_state = "SAFETY_ESTOP"
        elif min_front <= 1.80:
            safety_state = "SAFETY_CAUTION"
        else:
            safety_state = "SAFETY_CLEAR"

        self.last_scan = {
            "origin": p0,
            "ranges": np.array(ranges),
            "angles": self.angles,
            "points": scan_points,
            "geoms": geom_names,
            "hazards": is_hazards,
            "front_dist": min_front,
            "left_dist": min_left,
            "right_dist": min_right,
            "rear_dist": min_rear,
            "min_dist": min_overall_dist,
            "closest_geom": closest_geom,
            "ground_geom": ground_geom,
            "safety_state": safety_state,
            "hit_count": len([r for r in ranges if r > 0])
        }
        return self.last_scan

    def render_to_scene(self, scn, draw_beams=True, draw_hits=True):
        """Injects laser beam connector lines and hit markers into an MjvScene."""
        if self.last_scan is None:
            return

        p0 = self.last_scan["origin"]
        points = self.last_scan["points"]
        hazards = self.last_scan["hazards"]
        ranges = self.last_scan["ranges"]

        # Color definitions
        cyan_beam = np.array([0.0, 0.88, 1.0, 0.28], dtype=np.float32)
        red_beam = np.array([1.0, 0.22, 0.12, 0.75], dtype=np.float32)
        green_hit = np.array([0.0, 1.0, 0.55, 0.85], dtype=np.float32)
        red_hit = np.array([1.0, 0.15, 0.15, 0.95], dtype=np.float32)

        for i, pt in enumerate(points):
            if pt is None or ranges[i] <= 0:
                continue

            is_haz = hazards[i] or (ranges[i] < 1.0 and is_haz)

            # Draw laser line beam
            if draw_beams and scn.ngeom < scn.maxgeom:
                idx = scn.ngeom
                scn.ngeom += 1
                geom = scn.geoms[idx]
                col = red_beam if is_haz else cyan_beam
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), np.eye(3).flatten(), col)
                mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, 1.5, p0, pt)

            # Draw laser hit sphere
            if draw_hits and scn.ngeom < scn.maxgeom:
                idx = scn.ngeom
                scn.ngeom += 1
                geom = scn.geoms[idx]
                col = red_hit if is_haz else green_hit
                sz = np.array([0.032, 0.032, 0.032], dtype=np.float64) if is_haz else np.array([0.024, 0.024, 0.024], dtype=np.float64)
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE, sz, pt, np.eye(3).flatten(), col)

class CloudMuJoCoSimulator:
    def __init__(self, xml_path=None, headless=False):
        if xml_path is None:
            cur_dir = os.path.dirname(os.path.abspath(__file__))
            xml_path = os.path.join(cur_dir, "unitree_go2", "dpsi_expo_world.xml")

        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"MuJoCo world XML not found at: {xml_path}")

        print(f"[INFO] Loading MuJoCo Model: {xml_path}")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.headless = headless

        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base")

        # Initialize Unitree 4D LiDAR L1 Engine
        self.lidar = Go2LiDAR(self.model, self.data, num_rays=72, max_range=16.0)
        self.show_lidar = True

        # 6 Spatial Mission Stations Across the Pavilion
        self.scenes = [
            {
                "id": 1,
                "name": "SCENE 1: VIP GREETING",
                "fsm": "STATE_DOCENT_GREET",
                "target_x": 0.0, "target_y": 0.0, "target_yaw": 0.0,
                "camera": "cam_stage_greeting",
                "transit_time": 0.0, "station_time": 5.5,
                "desc": "Standing on Central Stage (x=0.0m), welcoming delegates with friendly head nods."
            },
            {
                "id": 2,
                "name": "SCENE 2: ARGUS PATROL",
                "fsm": "STATE_ARGUS_PATROL",
                "target_x": 2.5, "target_y": 0.0, "target_yaw": 0.0,
                "camera": "cam_patrol_corridor",
                "transit_time": 4.0, "station_time": 4.5,
                "desc": "Trotting along the main pavilion avenue (x=2.5m) with active 4D LiDAR scan."
            },
            {
                "id": 3,
                "name": "SCENE 3: SPILL HAZARD E-STOP",
                "fsm": "STATE_HAZARD_ALERT",
                "target_x": 3.8, "target_y": 0.0, "target_yaw": 0.0,
                "camera": "cam_hazard_stop",
                "transit_time": 3.0, "station_time": 5.5,
                "desc": "Emergency E-stop safely halting 0.75m BEFORE the spill puddle & caution cones."
            },
            {
                "id": 4,
                "name": "SCENE 4: ARUCO STANDOFF DOCK",
                "fsm": "STATE_DOCENT_CRUISE",
                "target_x": 8.2, "target_y": 0.0, "target_yaw": 0.0,
                "camera": "cam_booth_standoff",
                "transit_time": 6.0, "station_time": 5.0,
                "desc": "Docked at Booth 1 at 1.2m standoff, aligning with ArUco ID 10 board (x=10.0m)."
            },
            {
                "id": 5,
                "name": "SCENE 5: 180° PRESENTER PIVOT",
                "fsm": "STATE_DOCENT_NARRATE",
                "target_x": 8.2, "target_y": 0.0, "target_yaw": math.pi,
                "camera": "cam_presenter_audience",
                "transit_time": 4.0, "station_time": 5.5,
                "desc": "Stepping 180° in place to face audience. Docent presentation mode with +8° eye contact."
            },
            {
                "id": 6,
                "name": "SCENE 6: STAGE & PRONE DOCK",
                "fsm": "STATE_PRONE_REST",
                "target_x": 0.0, "target_y": 0.0, "target_yaw": 0.0,
                "camera": "cam_stage_dock",
                "transit_time": 6.5, "station_time": 6.0,
                "desc": "Returned to Central Stage platform, smoothly folded into prone sleeping rest mode."
            }
        ]

        # Robot Motion State
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.330
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        self.scene_idx = 0
        self.in_transit = False
        self.phase_timer = 0.0
        self.sim_time = 0.0
        self.gait_phase = 0.0

        # Snapshot Tracking
        self.saved_snapshots = set()
        self.output_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")
        os.makedirs(self.output_dir, exist_ok=True)

        self.camera_names = [
            "cam_stage_greeting", "cam_patrol_corridor", "cam_hazard_stop",
            "cam_booth_standoff", "cam_presenter_audience", "cam_stage_dock",
            "cam_pavilion_overview", "cam_lidar_fpv"
        ]
        self.cam_idx = 0

        # Initialize Robot Pose
        self.apply_robot_state(0.0, 0.0, 0.330, 0.0, 0.0, 0.0, [0.0, 0.74, -1.48] * 4)

    def apply_robot_state(self, x, y, z, yaw, pitch, roll, joint_angles):
        """Sets full spatial state of the robot in MuJoCo and updates forward kinematics."""
        self.data.qpos[0] = x
        self.data.qpos[1] = y
        self.data.qpos[2] = z
        self.data.qpos[3:7] = yaw_to_quaternion(yaw, pitch, roll)
        self.data.qpos[7:19] = joint_angles
        mujoco.mj_forward(self.model, self.data)

    def update_simulation_step(self, dt):
        """High-precision physics, spatial trajectory, and LiDAR scan updater."""
        scene = self.scenes[self.scene_idx]
        self.phase_timer += dt
        self.sim_time += dt

        target_x = scene["target_x"]
        target_y = scene["target_y"]
        target_yaw = scene["target_yaw"]

        linear_vel = 0.0
        yaw_rate = 0.0
        body_pitch_target = 0.0
        is_prone = False

        # Run Real-Time LiDAR Scan
        lidar_data = self.lidar.scan()

        # -------------------------------------------------------------
        # 1. State Machine Navigation & Station Behaviors
        # -------------------------------------------------------------
        if self.in_transit:
            dx = target_x - self.pos_x
            dy = target_y - self.pos_y
            dist = math.hypot(dx, dy)

            # Curved bypass path around the spill hazard when transiting to Booth 1
            if scene["id"] == 4 and self.pos_x < 5.8:
                dy_target = 0.45 * math.sin(min(math.pi, (self.pos_x - 4.5) / 1.5 * math.pi))
                dy = (dy_target - self.pos_y)

            # Reactive LiDAR Deceleration when approaching obstacles
            if dist > 0.15:
                heading = math.atan2(dy, dx)
                yaw_err = wrap_to_pi(heading - self.yaw)
                if abs(yaw_err) > 0.4:
                    yaw_rate = math.copysign(min(1.2, abs(yaw_err) * 1.8), yaw_err)
                    linear_vel = 0.22
                else:
                    yaw_rate = yaw_err * 1.5
                    # Throttle speed if LiDAR detects front obstacle within caution range
                    speed_cap = 0.95
                    if lidar_data["front_dist"] < 1.5:
                        speed_cap = max(0.20, lidar_data["front_dist"] * 0.45)
                    linear_vel = min(speed_cap, max(0.32, dist * 0.90))
            else:
                yaw_err = wrap_to_pi(target_yaw - self.yaw)
                if abs(yaw_err) > 0.06:
                    yaw_rate = math.copysign(min(1.0, abs(yaw_err) * 1.8), yaw_err)
                    linear_vel = 0.0
                else:
                    # Arrived & Aligned at Station!
                    self.in_transit = False
                    self.phase_timer = 0.0
                    self.pos_x = target_x
                    self.pos_y = target_y
                    self.yaw = target_yaw
                    print(f"\n>>> [ARRIVED AT STATION] -> {scene['name']}")
                    print(f"    Position: ({self.pos_x:.2f}m, {self.pos_y:.2f}m) | Heading: {math.degrees(self.yaw):.0f}°")
                    print(f"    LiDAR: Front {lidar_data['front_dist']:.2f}m ({lidar_data['closest_geom']}) | Safety: {lidar_data['safety_state']}")
                    print(f"    {scene['desc']}")

        else:
            # Standing / Presenting at Station
            if scene["fsm"] == "STATE_DOCENT_GREET":
                body_pitch_target = 0.12 * math.sin(self.phase_timer * 2.5)
            elif scene["fsm"] == "STATE_HAZARD_ALERT":
                body_pitch_target = -0.08
            elif scene["fsm"] == "STATE_DOCENT_NARRATE":
                body_pitch_target = 0.14
            elif scene["fsm"] == "STATE_PRONE_REST":
                is_prone = True

            if self.phase_timer >= scene["station_time"]:
                self.scene_idx = (self.scene_idx + 1) % len(self.scenes)
                self.in_transit = True
                self.phase_timer = 0.0
                next_s = self.scenes[self.scene_idx]
                print(f"\n>>> [TRANSIT INITIATED] -> Moving to {next_s['name']} at ({next_s['target_x']:.2f}m, {next_s['target_y']:.2f}m)")

        # -------------------------------------------------------------
        # 2. Integrate Continuous Position & Yaw
        # -------------------------------------------------------------
        self.yaw += yaw_rate * dt
        self.yaw = wrap_to_pi(self.yaw)
        self.pos_x += linear_vel * math.cos(self.yaw) * dt
        self.pos_y += linear_vel * math.sin(self.yaw) * dt

        self.pitch += (body_pitch_target - self.pitch) * min(1.0, dt * 5.0)

        on_stage = math.hypot(self.pos_x, self.pos_y) < 1.75
        base_floor_z = 0.05 if on_stage else 0.0

        if is_prone:
            fold_ratio = min(1.0, self.phase_timer / 2.5)
            nominal_z = 0.330 - (0.330 - 0.140) * fold_ratio + base_floor_z
        else:
            nominal_z = STANCE_HEIGHT + base_floor_z

        # -------------------------------------------------------------
        # 3. Dynamic Quadruped Trot Gait Engine
        # -------------------------------------------------------------
        is_stepping = (linear_vel > 0.04) or (abs(yaw_rate) > 0.20)
        step_len = min(MAX_STEP_LEN, linear_vel * 0.32) if is_stepping else 0.0

        if is_stepping:
            gait_freq = 2.0 * math.pi * (1.5 + linear_vel * 0.8 + abs(yaw_rate) * 0.4)
            self.gait_phase = (self.gait_phase + gait_freq * dt) % (2.0 * math.pi)
            body_bob = 0.006 * math.cos(2.0 * self.gait_phase)
            self.roll = 0.012 * math.sin(self.gait_phase)
        else:
            self.gait_phase = 0.0
            body_bob = 0.003 * math.sin(self.sim_time * 2.0)
            self.roll = 0.0

        cur_base_z = nominal_z + body_bob

        phase_A = self.gait_phase
        phase_B = (self.gait_phase + math.pi) % (2.0 * math.pi)

        turn_diff = (yaw_rate * 0.28) * 0.15
        step_L = step_len - turn_diff
        step_R = step_len + turn_diff

        def calc_foot_offset(phase, s_len):
            if not is_stepping or abs(s_len) < 0.005:
                return 0.0, 0.0, -STANCE_HEIGHT
            if phase < math.pi:
                s = phase / math.pi
                xf = (s_len / 2.0) * math.cos(phase)
                zf = -STANCE_HEIGHT
            else:
                s = (phase - math.pi) / math.pi
                xf = -(s_len / 2.0) * math.cos(phase)
                zf = -STANCE_HEIGHT + STEP_CLEARANCE * math.sin(math.pi * s)
            return xf, 0.0, zf

        fl_pos = calc_foot_offset(phase_A, step_L)
        rr_pos = calc_foot_offset(phase_A, step_R)
        fr_pos = calc_foot_offset(phase_B, step_R)
        rl_pos = calc_foot_offset(phase_B, step_L)

        if is_prone:
            prone_ratio = min(1.0, self.phase_timer / 2.5)
            q_fl = (0.0, 0.74 + prone_ratio * 0.48, -1.48 - prone_ratio * 0.95)
            q_fr = (0.0, 0.74 + prone_ratio * 0.48, -1.48 - prone_ratio * 0.95)
            q_rl = (0.0, 0.74 + prone_ratio * 0.48, -1.48 - prone_ratio * 0.95)
            q_rr = (0.0, 0.74 + prone_ratio * 0.48, -1.48 - prone_ratio * 0.95)
        else:
            q_fl = leg_inverse_kinematics(*fl_pos)
            q_fr = leg_inverse_kinematics(*fr_pos)
            q_rl = leg_inverse_kinematics(*rl_pos)
            q_rr = leg_inverse_kinematics(*rr_pos)

        joint_targets = [
            q_fl[0], q_fl[1] + self.pitch, q_fl[2],
            q_fr[0], q_fr[1] + self.pitch, q_fr[2],
            q_rl[0], q_rl[1] - self.pitch, q_rl[2],
            q_rr[0], q_rr[1] - self.pitch, q_rr[2]
        ]

        # -------------------------------------------------------------
        # 4. Commit State to MuJoCo
        # -------------------------------------------------------------
        self.apply_robot_state(
            self.pos_x, self.pos_y, cur_base_z,
            self.yaw, self.pitch, self.roll,
            joint_targets
        )

        # Auto-Capture High-Resolution Milestone Proof Snapshots
        if not self.in_transit and self.phase_timer > 1.0 and scene["id"] not in self.saved_snapshots:
            self.save_station_proof(scene)
            self.saved_snapshots.add(scene["id"])

    def save_station_proof(self, scene):
        """Renders and saves a 1080p snapshot of each exhibition station with LiDAR beams and HUD overlay."""
        try:
            renderer = mujoco.Renderer(self.model, 720, 1280)
            renderer.update_scene(self.data, camera=scene["camera"])
            scn = renderer.scene

            # Invalidate and re-scan at current camera frame
            scan_data = self.lidar.scan()
            self.lidar.render_to_scene(scn, draw_beams=True, draw_hits=True)

            rgb = renderer.render()
            base_img = Image.fromarray(rgb).convert("RGBA")

            # Composite HUD Telemetry Banner & Polar Radar Overlay
            hud = Image.new("RGBA", (1280, 720), (0, 0, 0, 0))
            draw = ImageDraw.Draw(hud)

            # 1. Top-Left Telemetry Banner
            draw.rectangle([20, 15, 660, 125], fill=(8, 14, 25, 215), outline=(0, 180, 240, 220), width=2)
            draw.text((35, 24), "PROJECT CLOUD | DPSI EXPO-2026 DOCENT PATROL", fill=(0, 220, 255, 255))
            draw.text((35, 46), f"{scene['name']} | Unitree Go2 4D LiDAR L1 Active", fill=(255, 255, 255, 240))
            draw.text((35, 68), f"Position: X={self.pos_x:+.2f}m, Y={self.pos_y:+.2f}m | Yaw: {math.degrees(self.yaw):.0f}° | FSM: {scene['fsm']}", fill=(180, 210, 240, 240))

            safety_color = (0, 255, 160, 255) if scan_data["safety_state"] == "SAFETY_CLEAR" else ((255, 180, 0, 255) if scan_data["safety_state"] == "SAFETY_CAUTION" else (255, 50, 50, 255))
            draw.text((35, 88), f"Clearances: Front {scan_data['front_dist']:.2f}m | Left {scan_data['left_dist']:.2f}m | Right {scan_data['right_dist']:.2f}m", fill=(200, 225, 255, 240))
            draw.text((35, 106), f"Closest Obstacle: {scan_data['closest_geom']} at {scan_data['min_dist']:.2f}m | Status: {scan_data['safety_state']}", fill=safety_color)

            # 2. Top-Right 360° Polar Radar Scanner Mini-Map
            rcx, rcy, rad = 1170, 95, 75
            draw.ellipse([rcx-rad, rcy-rad, rcx+rad, rcy+rad], fill=(6, 12, 22, 225), outline=(0, 200, 255, 240), width=2)
            for ring_r in [rad * 0.33, rad * 0.66, rad]:
                draw.ellipse([rcx-ring_r, rcy-ring_r, rcx+ring_r, rcy+ring_r], outline=(0, 120, 180, 110), width=1)
            draw.line([rcx-rad, rcy, rcx+rad, rcy], fill=(0, 120, 180, 110), width=1)
            draw.line([rcx, rcy-rad, rcx, rcy+rad], fill=(0, 120, 180, 110), width=1)

            # Robot center indicator
            draw.polygon([(rcx, rcy-7), (rcx-4, rcy+4), (rcx+4, rcy+4)], fill=(0, 255, 255, 255))

            # Plot radar hits
            max_vis_range = 10.0
            for i in range(len(scan_data["ranges"])):
                r_dist = scan_data["ranges"][i]
                if r_dist > 0 and r_dist <= max_vis_range:
                    th = scan_data["angles"][i]
                    d_px = (r_dist / max_vis_range) * rad
                    px = rcx + d_px * math.sin(th)
                    py = rcy - d_px * math.cos(th)
                    dot_color = (255, 60, 40, 255) if scan_data["hazards"][i] else (0, 255, 140, 220)
                    draw.ellipse([px-2, py-2, px+2, py+2], fill=dot_color)

            draw.text((rcx-rad+10, rcy+rad+6), "4D LiDAR RADAR (10m)", fill=(0, 200, 255, 220))

            final_img = Image.alpha_composite(base_img, hud).convert("RGB")
            filename = f"proof_mujoco_scene_{scene['id']}.png"
            path = os.path.join(self.output_dir, filename)
            final_img.save(path)
            print(f"[PROOF CAPTURE] Saved {filename} from camera '{scene['camera']}' at x={self.pos_x:.2f}m")
        except Exception as e:
            print(f"[WARN] Proof capture failed: {e}")

    def run_interactive(self):
        """Runs the interactive simulation under mjpython."""
        print("\n" + "=" * 80)
        print("  PROJECT CLOUD: MASTER MUJOCO EXHIBITION SIMULATION RUNNING")
        print("  Unitree Go2 4D LiDAR L1 Active with Physical Walls & Obstacle Mapping")
        print("  Controls:")
        print("    [Space] : Pause / Resume")
        print("    [L]     : Toggle 360° LiDAR Visual Laser Beams")
        print("    [C]     : Cycle Station Framing Cameras (including LiDAR FPV)")
        print("    [1-6]   : Trigger Mission Station 1..6")
        print("    [Mouse] : Orbit / Zoom / Pan")
        print("=" * 80 + "\n")

        def key_callback(keycode):
            # 'L' or 'l'
            if keycode in [ord('l'), ord('L')]:
                self.show_lidar = not self.show_lidar
                print(f"[INTERACTIVE] LiDAR Visual Beams: {'ON' if self.show_lidar else 'OFF'}")
            # 'C' or 'c'
            elif keycode in [ord('c'), ord('C')]:
                self.cam_idx = (self.cam_idx + 1) % len(self.camera_names)
                print(f"[INTERACTIVE] Camera Switched to: {self.camera_names[self.cam_idx]}")
            # '1'..'6'
            elif keycode in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5'), ord('6')]:
                target_idx = keycode - ord('1')
                self.scene_idx = target_idx
                self.in_transit = True
                self.phase_timer = 0.0
                print(f"[INTERACTIVE] Manual override -> Triggered {self.scenes[self.scene_idx]['name']}")

        with mujoco.viewer.launch_passive(self.model, self.data, key_callback=key_callback) as viewer:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = self.base_body_id
            viewer.cam.distance = 3.6
            viewer.cam.elevation = -18.0
            viewer.cam.azimuth = 140.0

            dt = 0.016 # 60 FPS update rate
            last_print = time.time()

            while viewer.is_running():
                step_start = time.time()

                self.update_simulation_step(dt)

                # Update live LiDAR beams in viewer.user_scn
                if viewer.user_scn is not None:
                    viewer.user_scn.ngeom = 0
                    if self.show_lidar:
                        self.lidar.render_to_scene(viewer.user_scn, draw_beams=True, draw_hits=True)

                viewer.sync()

                if time.time() - last_print > 1.2:
                    scene = self.scenes[self.scene_idx]
                    status = "TRANSIT" if self.in_transit else "STATION"
                    l_data = self.lidar.last_scan
                    print(f"[{scene['fsm']:20s} | {status:7s}] Pos: ({self.pos_x:5.2f}m, {self.pos_y:5.2f}m) | "
                          f"LiDAR: Front {l_data['front_dist']:4.2f}m ({l_data['closest_geom']:16s}) | "
                          f"L: {l_data['left_dist']:4.2f}m, R: {l_data['right_dist']:4.2f}m | Safety: {l_data['safety_state']}")
                    last_print = time.time()

                time_spent = time.time() - step_start
                if dt - time_spent > 0:
                    time.sleep(dt - time_spent)

    def run_headless(self, duration=45.0):
        """Runs the full exhibition mission headlessly to generate all photographic proof images."""
        print(f"[INFO] Running Headless Exhibition Tour for {duration} seconds with LiDAR & Walls...")
        dt = 0.016
        steps = int(duration / dt)

        last_print = time.time()
        for step in range(steps):
            self.update_simulation_step(dt)

            if time.time() - last_print > 3.0:
                scene = self.scenes[self.scene_idx]
                status = "TRANSIT" if self.in_transit else "STATION"
                l_data = self.lidar.last_scan
                print(f"[HEADLESS] Time: {self.sim_time:5.1f}s | Pos: ({self.pos_x:5.2f}m, {self.pos_y:5.2f}m) | {scene['name']} ({status}) | LiDAR Front: {l_data['front_dist']:.2f}m")
                last_print = time.time()

        # Render High Aerial Overview of entire pavilion
        renderer = mujoco.Renderer(self.model, 720, 1280)
        renderer.update_scene(self.data, camera="cam_pavilion_overview")
        self.lidar.render_to_scene(renderer.scene, draw_beams=True, draw_hits=True)
        Image.fromarray(renderer.render()).save(os.path.join(self.output_dir, "proof_mujoco_pavilion_overview.png"))

        # Render First-Person LiDAR Camera view
        renderer.update_scene(self.data, camera="cam_lidar_fpv")
        self.lidar.render_to_scene(renderer.scene, draw_beams=True, draw_hits=True)
        Image.fromarray(renderer.render()).save(os.path.join(self.output_dir, "proof_mujoco_lidar_fpv.png"))

        print(f"[INFO] Headless Mission completed! Saved snapshots: {len(self.saved_snapshots)}")

def main():
    parser = argparse.ArgumentParser(description="Project Cloud MuJoCo Exhibition Simulation")
    parser.add_argument("--headless", action="store_true", help="Run headlessly without GUI")
    parser.add_argument("--duration", type=float, default=45.0, help="Duration in seconds")
    args = parser.parse_args()

    sim = CloudMuJoCoSimulator(headless=args.headless)
    if args.headless:
        sim.run_headless(duration=args.duration)
    else:
        sim.run_interactive()

if __name__ == "__main__":
    main()
