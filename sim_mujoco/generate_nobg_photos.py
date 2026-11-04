"""
Project Cloud: High-Resolution Background-Removed Photo Generator
Platform: Unitree Go2 Quadruped Robot (Google DeepMind MJCF)
Output: 100% Alpha Transparent PNGs with Zero Background Artifacts

Generates 3 full photo suites:
1. Studio Turntable Series (Hero, Front, Side, Rear, Top, Dramatic Low-Angle)
2. Locomotion & Mission Posture Series (Greeting, Dynamic Trot, E-Stop Bracing, Docking, Presenter Pivot, Prone Fold)
3. Station Camera Perspective Series (Scenes 1-6 as viewed by dedicated station cameras)
"""

import sys
import os
import math
import numpy as np
from PIL import Image

try:
    import mujoco
except ImportError as e:
    print(f"[ERROR] Missing dependency: {e}")
    sys.exit(1)

# Robot Geometric Parameters (Unitree Go2)
L_THIGH = 0.213
L_CALF = 0.213
STANCE_HEIGHT = 0.315
STEP_CLEARANCE = 0.048

def leg_inverse_kinematics(x_foot, y_foot, z_foot):
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

def yaw_to_quaternion(yaw, pitch=0.0, roll=0.0):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    ]

def crop_and_pad(rgba_array, padding=35):
    """Crops transparent borders to bounding box with symmetric padding."""
    mask = rgba_array[:, :, 3] > 0
    if not np.any(mask):
        return Image.fromarray(rgba_array)
    rows = np.any(mask, axis=1)
    cols = np.any(mask, axis=0)
    ymin, ymax = np.where(rows)[0][[0, -1]]
    xmin, xmax = np.where(cols)[0][[0, -1]]

    cropped = rgba_array[ymin:ymax+1, xmin:xmax+1]
    ch, cw = cropped.shape[:2]

    out_w = cw + 2 * padding
    out_h = ch + 2 * padding
    canvas = np.zeros((out_h, out_w, 4), dtype=np.uint8)
    canvas[padding:padding+ch, padding:padding+cw] = cropped
    return Image.fromarray(canvas)

class Go2PhotoStudio:
    def __init__(self, xml_path=None):
        if xml_path is None:
            cur_dir = os.path.dirname(os.path.abspath(__file__))
            xml_path = os.path.join(cur_dir, "unitree_go2", "dpsi_expo_world.xml")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base")

        # Set of all robot bodies
        self.robot_bodies = set()
        for b in range(self.model.nbody):
            bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b)
            if bname and any(p in bname for p in ["base", "FL", "FR", "RL", "RR"]):
                self.robot_bodies.add(b)

        # High-res renderer (1080x1080 for square studio, 720x1280 for camera scenes)
        self.renderer_sq = mujoco.Renderer(self.model, 1080, 1080)
        self.renderer_wide = mujoco.Renderer(self.model, 720, 1280)

        # Scene render options (disable rangefinder yellow lines during photo shoot)
        self.opt = mujoco.MjvOption()
        self.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False

        self.assets_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "nobg")
        os.makedirs(self.assets_dir, exist_ok=True)

    def set_robot_pose(self, x=0.0, y=0.0, z=0.33, yaw=0.0, pitch=0.0, roll=0.0, joint_targets=None):
        if joint_targets is None:
            joint_targets = [0.0, 0.74, -1.48] * 4
        self.data.qpos[0] = x
        self.data.qpos[1] = y
        self.data.qpos[2] = z
        self.data.qpos[3:7] = yaw_to_quaternion(yaw, pitch, roll)
        self.data.qpos[7:19] = joint_targets
        mujoco.mj_forward(self.model, self.data)

    def capture_nobg(self, camera, is_wide=False, padding=35):
        """Renders color + segmentation mask, extracts transparent RGBA, and crops."""
        renderer = self.renderer_wide if is_wide else self.renderer_sq

        # 1. Color pass
        renderer.disable_segmentation_rendering()
        renderer.update_scene(self.data, camera=camera, scene_option=self.opt)
        rgb = renderer.render()

        # 2. Segmentation pass
        renderer.enable_segmentation_rendering()
        renderer.update_scene(self.data, camera=camera, scene_option=self.opt)
        seg = renderer.render()

        # 3. Create boolean mask of robot pixels
        geom_ids = seg[:, :, 0]
        mask = np.zeros(geom_ids.shape, dtype=bool)
        for gid in np.unique(geom_ids):
            if gid >= 0 and self.model.geom_bodyid[gid] in self.robot_bodies:
                mask |= (geom_ids == gid)

        rgba = np.zeros((rgb.shape[0], rgb.shape[1], 4), dtype=np.uint8)
        rgba[:, :, :3] = rgb
        rgba[:, :, 3] = np.where(mask, 255, 0).astype(np.uint8)

        return crop_and_pad(rgba, padding=padding)

    def render_all(self):
        saved_files = []
        nominal_joints = [0.0, 0.74, -1.48] * 4

        # =====================================================================
        # 1. STUDIO TURNTABLE SHOTS (Nominal Stand Posture)
        # =====================================================================
        print("\n>>> Generating Suite 1: Studio Turntable Renders...")
        self.set_robot_pose(0.0, 0.0, 0.33, 0.0, 0.0, 0.0, nominal_joints)

        studio_angles = [
            ("go2_nobg_hero_perspective.png", -14.0, 142.0, 1.25, "Hero 3/4 Front Perspective"),
            ("go2_nobg_front_portrait.png", -4.0, 180.0, 1.15, "Direct Front Portrait (Face & LiDAR)"),
            ("go2_nobg_side_profile.png", 0.0, 90.0, 1.30, "Side Profile (Kinematic Linkages)"),
            ("go2_nobg_rear_three_quarter.png", -14.0, 42.0, 1.25, "Rear 3/4 View (Hip Motors & Spine)"),
            ("go2_nobg_top_isometric.png", -52.0, 135.0, 1.35, "Top-Down Isometric View"),
            ("go2_nobg_dramatic_low_angle.png", 10.0, 150.0, 1.05, "Dramatic Ground-Up Hero Shot"),
            ("go2_nobg_three_quarter_right.png", -14.0, 222.0, 1.25, "Right 3/4 Front View")
        ]

        for fname, elev, azim, dist, desc in studio_angles:
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            cam.trackbodyid = self.base_id
            cam.distance = dist
            cam.elevation = elev
            cam.azimuth = azim

            img = self.capture_nobg(cam, is_wide=False, padding=35)
            out_path = os.path.join(self.assets_dir, fname)
            img.save(out_path)
            saved_files.append((fname, out_path, desc))
            print(f"  [SAVED] {fname} ({desc}) -> Size: {img.size}")

        # =====================================================================
        # 2. LOCOMOTION & MISSION POSES (Studio Framing of Dynamic Postures)
        # =====================================================================
        print("\n>>> Generating Suite 2: Locomotion & Mission Posture Renders...")

        # 2A. Greeting Pose (Polite head nod down)
        self.set_robot_pose(0.0, 0.0, 0.33, 0.0, pitch=0.14, roll=0.0, joint_targets=[
            0.0, 0.74 + 0.14, -1.48,
            0.0, 0.74 + 0.14, -1.48,
            0.0, 0.74 - 0.14, -1.48,
            0.0, 0.74 - 0.14, -1.48
        ])
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        cam.trackbodyid = self.base_id
        cam.distance = 1.25
        cam.elevation = -12.0
        cam.azimuth = 145.0
        img = self.capture_nobg(cam, is_wide=False, padding=35)
        p = os.path.join(self.assets_dir, "go2_nobg_pose_greeting.png")
        img.save(p)
        saved_files.append(("go2_nobg_pose_greeting.png", p, "VIP Greeting Welcoming Head Nod"))
        print(f"  [SAVED] go2_nobg_pose_greeting.png -> Size: {img.size}")

        # 2B. Dynamic Diagonal Trot (Mid-stride stride with foot ground clearance)
        fl_q = leg_inverse_kinematics(0.065, 0.0, -STANCE_HEIGHT + STEP_CLEARANCE)
        rr_q = leg_inverse_kinematics(0.065, 0.0, -STANCE_HEIGHT + STEP_CLEARANCE)
        fr_q = leg_inverse_kinematics(-0.065, 0.0, -STANCE_HEIGHT)
        rl_q = leg_inverse_kinematics(-0.065, 0.0, -STANCE_HEIGHT)
        trot_joints = [
            fl_q[0], fl_q[1], fl_q[2],
            fr_q[0], fr_q[1], fr_q[2],
            rl_q[0], rl_q[1], rl_q[2],
            rr_q[0], rr_q[1], rr_q[2]
        ]
        self.set_robot_pose(0.0, 0.0, 0.335, 0.0, pitch=0.0, roll=0.015, joint_targets=trot_joints)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        cam.trackbodyid = self.base_id
        cam.distance = 1.30
        cam.elevation = -10.0
        cam.azimuth = 125.0
        img = self.capture_nobg(cam, is_wide=False, padding=35)
        p = os.path.join(self.assets_dir, "go2_nobg_pose_dynamic_trot.png")
        img.save(p)
        saved_files.append(("go2_nobg_pose_dynamic_trot.png", p, "Dynamic 2-Beat Trot Mid-Stride"))
        print(f"  [SAVED] go2_nobg_pose_dynamic_trot.png -> Size: {img.size}")

        # 2C. Spill Hazard E-Stop Bracing Stance (lowered center of gravity, braced legs)
        brace_fl = leg_inverse_kinematics(0.03, 0.02, -0.285)
        brace_fr = leg_inverse_kinematics(0.03, -0.02, -0.285)
        brace_rl = leg_inverse_kinematics(-0.03, 0.02, -0.285)
        brace_rr = leg_inverse_kinematics(-0.03, -0.02, -0.285)
        brace_joints = [
            brace_fl[0], brace_fl[1], brace_fl[2],
            brace_fr[0], brace_fr[1], brace_fr[2],
            brace_rl[0], brace_rl[1], brace_rl[2],
            brace_rr[0], brace_rr[1], brace_rr[2]
        ]
        self.set_robot_pose(0.0, 0.0, 0.285, 0.0, pitch=-0.08, roll=0.0, joint_targets=brace_joints)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        cam.trackbodyid = self.base_id
        cam.distance = 1.25
        cam.elevation = -12.0
        cam.azimuth = 155.0
        img = self.capture_nobg(cam, is_wide=False, padding=35)
        p = os.path.join(self.assets_dir, "go2_nobg_pose_hazard_estop.png")
        img.save(p)
        saved_files.append(("go2_nobg_pose_hazard_estop.png", p, "Hazard E-Stop Reactive Bracing Stance"))
        print(f"  [SAVED] go2_nobg_pose_hazard_estop.png -> Size: {img.size}")

        # 2D. Docent Presenter Eye-Contact Tilt (+8° head pitch)
        self.set_robot_pose(0.0, 0.0, 0.33, 0.0, pitch=0.14, roll=0.0, joint_targets=nominal_joints)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        cam.trackbodyid = self.base_id
        cam.distance = 1.15
        cam.elevation = -8.0
        cam.azimuth = 175.0
        img = self.capture_nobg(cam, is_wide=False, padding=35)
        p = os.path.join(self.assets_dir, "go2_nobg_pose_presenter_tilt.png")
        img.save(p)
        saved_files.append(("go2_nobg_pose_presenter_tilt.png", p, "Docent Presenter +8° Eye-Contact"))
        print(f"  [SAVED] go2_nobg_pose_presenter_tilt.png -> Size: {img.size}")

        # 2E. Prone Sleep Rest Fold Posture
        prone_q = [0.0, 0.74 + 0.48, -1.48 - 0.95] * 4
        self.set_robot_pose(0.0, 0.0, 0.140, 0.0, pitch=0.0, roll=0.0, joint_targets=prone_q)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        cam.trackbodyid = self.base_id
        cam.distance = 1.10
        cam.elevation = -22.0
        cam.azimuth = 140.0
        img = self.capture_nobg(cam, is_wide=False, padding=35)
        p = os.path.join(self.assets_dir, "go2_nobg_pose_prone_sleep.png")
        img.save(p)
        saved_files.append(("go2_nobg_pose_prone_sleep.png", p, "Folded Prone Sleep Rest Posture"))
        print(f"  [SAVED] go2_nobg_pose_prone_sleep.png -> Size: {img.size}")

        # =====================================================================
        # 3. STATION CAMERAS BACKGROUND-REMOVED (Exact framing from Scenes 1-6)
        # =====================================================================
        print("\n>>> Generating Suite 3: Scene Camera Station Views (Background Removed)...")

        scene_configs = [
            ("go2_nobg_scene1_greeting_cam.png", 0.0, 0.0, 0.33, 0.0, nominal_joints, "cam_stage_greeting", "Scene 1: Central Stage Greeting Camera"),
            ("go2_nobg_scene2_patrol_cam.png", 2.5, 0.0, 0.33, 0.0, trot_joints, "cam_patrol_corridor", "Scene 2: Corridor Patrol Camera"),
            ("go2_nobg_scene3_hazard_cam.png", 3.8, 0.0, 0.285, 0.0, brace_joints, "cam_hazard_stop", "Scene 3: Hazard E-Stop Camera"),
            ("go2_nobg_scene4_booth_cam.png", 8.2, 0.0, 0.33, 0.0, nominal_joints, "cam_booth_standoff", "Scene 4: Booth 1 Standoff Camera"),
            ("go2_nobg_scene5_audience_cam.png", 8.2, 0.0, 0.33, math.pi, nominal_joints, "cam_presenter_audience", "Scene 5: Audience Presentation Camera"),
            ("go2_nobg_scene6_dock_cam.png", 0.0, 0.0, 0.14, 0.0, prone_q, "cam_stage_dock", "Scene 6: Central Stage Dock Camera")
        ]

        for fname, x, y, z, yaw, q, cname, desc in scene_configs:
            self.set_robot_pose(x, y, z, yaw, pitch=0.0, roll=0.0, joint_targets=q)
            img = self.capture_nobg(cname, is_wide=True, padding=35)
            p = os.path.join(self.assets_dir, fname)
            img.save(p)
            saved_files.append((fname, p, desc))
            print(f"  [SAVED] {fname} ({desc}) -> Size: {img.size}")

        print(f"\n[COMPLETE] Successfully generated {len(saved_files)} transparent background-removed photos!")
        return saved_files

if __name__ == "__main__":
    studio = Go2PhotoStudio()
    studio.render_all()
