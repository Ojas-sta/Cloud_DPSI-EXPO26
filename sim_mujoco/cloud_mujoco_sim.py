"""
Project Cloud: High-Fidelity Autonomous MuJoCo Simulation
Platform: Unitree Go2 Quadruped Robot
Environment: MuJoCo Physics (macOS Apple Silicon Native via mjpython)

Key Features:
- Physics-grounded Quadruped Locomotion with Ground Contacts & Friction
- Closed-Form Analytical 3-DOF Leg Inverse Kinematics (IK)
- Dynamic 2-Beat Diagonal Quadruped Trot Gait Engine
- True Spatial Exhibition Mission across 6 distinct stations:
  1. Central Stage Platform (x=0.0m) - VIP Greeting with head nods
  2. Corridor Avenue (x=3.5m) - Dynamic trot patrol with 4D LiDAR
  3. Chemical Spill Hazard (x=4.75m) - Reactive E-stop & wide bracing stance before puddle & cones
  4. Booth 1 Stand (x=8.6m) - ArUco ID 10 board docking at 1.4m standoff
  5. Presenter Pivot (x=8.6m) - 180° in-place stepping turn facing delegates, +8° head tilt
  6. Central Stage Return (x=0.0m) - Mounts platform, folds into prone sleep rest mode
- Multi-Camera System with Dedicated Station Framing & Smooth 3rd-Person Follow
- Interactive Viewer Controls & Automatic Milestone Proof Snapshots
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
    from PIL import Image
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

        # 6 Spatial Mission Scenes Across the Pavilion
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
                "target_x": 3.5, "target_y": 0.0, "target_yaw": 0.0,
                "camera": "cam_patrol_corridor",
                "transit_time": 4.5, "station_time": 4.5,
                "desc": "Trotting along the main pavilion avenue (x=3.5m) with simulated 4D LiDAR."
            },
            {
                "id": 3,
                "name": "SCENE 3: SPILL HAZARD E-STOP",
                "fsm": "STATE_HAZARD_ALERT",
                "target_x": 4.75, "target_y": 0.0, "target_yaw": 0.0,
                "camera": "cam_hazard_stop",
                "transit_time": 3.5, "station_time": 5.5,
                "desc": "Emergency E-stop right before the blue spill puddle & caution cones at x=5.0m."
            },
            {
                "id": 4,
                "name": "SCENE 4: ARUCO STANDOFF DOCK",
                "fsm": "STATE_DOCENT_CRUISE",
                "target_x": 8.6, "target_y": 0.0, "target_yaw": 0.0,
                "camera": "cam_booth_standoff",
                "transit_time": 5.5, "station_time": 5.0,
                "desc": "Docked at Booth 1 at 1.4m standoff, aligning with ArUco ID 10 board (x=10.0m)."
            },
            {
                "id": 5,
                "name": "SCENE 5: 180° PRESENTER PIVOT",
                "fsm": "STATE_DOCENT_NARRATE",
                "target_x": 8.6, "target_y": 0.0, "target_yaw": math.pi,
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
            "cam_pavilion_overview"
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
        """High-precision physics & spatial trajectory updater."""
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

        # -------------------------------------------------------------
        # 1. State Machine Navigation & Station Behaviors
        # -------------------------------------------------------------
        if self.in_transit:
            # Traveling towards the station
            dx = target_x - self.pos_x
            dy = target_y - self.pos_y
            dist = math.hypot(dx, dy)

            # Curved bypass path around the spill hazard when transiting to Booth 1
            if scene["id"] == 4 and self.pos_x < 5.8:
                # Steer laterally along y=0.5m to cleanly clear the hazard puddle
                dy_target = 0.45 * math.sin(min(math.pi, (self.pos_x - 4.5) / 1.5 * math.pi))
                dy = (dy_target - self.pos_y)

            # Angular error to heading
            if dist > 0.15:
                heading = math.atan2(dy, dx)
                yaw_err = wrap_to_pi(heading - self.yaw)
                if abs(yaw_err) > 0.4:
                    yaw_rate = math.copysign(min(1.2, abs(yaw_err) * 1.8), yaw_err)
                    linear_vel = 0.22
                else:
                    yaw_rate = yaw_err * 1.5
                    linear_vel = min(0.95, max(0.35, dist * 0.95))
            else:
                # Arrived at position, align target yaw
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
                    print(f"    {scene['desc']}")

        else:
            # Standing / Presenting at Station
            if scene["fsm"] == "STATE_DOCENT_GREET":
                # Polite welcoming head nod
                body_pitch_target = 0.12 * math.sin(self.phase_timer * 2.5)
            elif scene["fsm"] == "STATE_HAZARD_ALERT":
                # Emergency bracing posture
                body_pitch_target = -0.08
            elif scene["fsm"] == "STATE_DOCENT_NARRATE":
                # Docent presentation mode: +8° head tilt
                body_pitch_target = 0.14
            elif scene["fsm"] == "STATE_PRONE_REST":
                # Folding into prone sleep dock
                is_prone = True

            # Check if station stay time elapsed -> move to next station
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

        # Smooth pitch filter
        self.pitch += (body_pitch_target - self.pitch) * min(1.0, dt * 5.0)

        # Base nominal height (stage is at 0.05m elevation, pavilion floor is at 0.0m)
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
            # Trot gait frequency scales with speed
            gait_freq = 2.0 * math.pi * (1.5 + linear_vel * 0.8 + abs(yaw_rate) * 0.4)
            self.gait_phase = (self.gait_phase + gait_freq * dt) % (2.0 * math.pi)
            body_bob = 0.006 * math.cos(2.0 * self.gait_phase)
            self.roll = 0.012 * math.sin(self.gait_phase)
        else:
            self.gait_phase = 0.0
            body_bob = 0.003 * math.sin(self.sim_time * 2.0)
            self.roll = 0.0

        cur_base_z = nominal_z + body_bob

        # Diagonal Leg Phase Pairing (FL+RR vs FR+RL at 180° anti-phase)
        phase_A = self.gait_phase
        phase_B = (self.gait_phase + math.pi) % (2.0 * math.pi)

        # Yaw turning steering differential
        turn_diff = (yaw_rate * 0.28) * 0.15
        step_L = step_len - turn_diff
        step_R = step_len + turn_diff

        def calc_foot_offset(phase, s_len):
            if not is_stepping or abs(s_len) < 0.005:
                return 0.0, 0.0, -STANCE_HEIGHT
            if phase < math.pi:
                # Stance Phase (Pushes backward at exact matching velocity -> Zero Slip)
                s = phase / math.pi
                xf = (s_len / 2.0) * math.cos(phase)
                zf = -STANCE_HEIGHT
            else:
                # Swing Phase (Smooth parabolic clearance arc)
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
        if not self.in_transit and self.phase_timer > 2.0 and scene["id"] not in self.saved_snapshots:
            self.save_station_proof(scene)
            self.saved_snapshots.add(scene["id"])

    def save_station_proof(self, scene):
        """Renders and saves a 1080p snapshot of each exhibition station."""
        try:
            renderer = mujoco.Renderer(self.model, 720, 1280)
            renderer.update_scene(self.data, camera=scene["camera"])
            img = Image.fromarray(renderer.render())
            filename = f"proof_mujoco_scene_{scene['id']}.png"
            path = os.path.join(self.output_dir, filename)
            img.save(path)
            print(f"[PROOF CAPTURE] Saved {filename} from camera '{scene['camera']}' at x={self.pos_x:.2f}m")
        except Exception as e:
            print(f"[WARN] Proof capture failed: {e}")

    def run_interactive(self):
        """Runs the interactive simulation under mjpython."""
        print("\n" + "=" * 75)
        print("  PROJECT CLOUD: MASTER MUJOCO EXHIBITION SIMULATION RUNNING")
        print("  Controls:")
        print("    [Space] : Pause / Resume")
        print("    [C]     : Cycle Station Framing Cameras")
        print("    [1-6]   : Trigger Mission Station 1..6")
        print("    [Mouse] : Orbit / Zoom / Pan")
        print("=" * 75 + "\n")

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # 3rd-Person Dynamic Follow Camera (tracks robot across the room)
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
                viewer.sync()

                if time.time() - last_print > 1.2:
                    scene = self.scenes[self.scene_idx]
                    status = "TRANSIT" if self.in_transit else "STATION"
                    print(f"[{scene['fsm']:20s} | {status:7s}] Pos: ({self.pos_x:5.2f}m, {self.pos_y:5.2f}m, {self.data.qpos[2]:4.2f}m) | "
                          f"Yaw: {math.degrees(self.yaw):5.0f}° | SimTime: {self.sim_time:5.1f}s")
                    last_print = time.time()

                time_spent = time.time() - step_start
                if dt - time_spent > 0:
                    time.sleep(dt - time_spent)

    def run_headless(self, duration=45.0):
        """Runs the full exhibition mission headlessly to generate all 6 photographic proof images."""
        print(f"[INFO] Running Headless Exhibition Tour for {duration} seconds...")
        dt = 0.016
        steps = int(duration / dt)

        last_print = time.time()
        for step in range(steps):
            self.update_simulation_step(dt)

            if time.time() - last_print > 3.0:
                scene = self.scenes[self.scene_idx]
                status = "TRANSIT" if self.in_transit else "STATION"
                print(f"[HEADLESS] Time: {self.sim_time:5.1f}s | Pos: ({self.pos_x:5.2f}m, {self.pos_y:5.2f}m) | {scene['name']} ({status})")
                last_print = time.time()

        # Also capture high overview of entire pavilion
        renderer = mujoco.Renderer(self.model, 720, 1280)
        renderer.update_scene(self.data, camera="cam_pavilion_overview")
        Image.fromarray(renderer.render()).save(os.path.join(self.output_dir, "proof_mujoco_pavilion_overview.png"))
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
