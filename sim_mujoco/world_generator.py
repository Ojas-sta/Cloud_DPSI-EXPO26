#!/usr/bin/env python3
"""
Project Cloud: Procedural 40-Stall Auditorium MuJoCo World Generator
Reads config/stalls_database.yaml and produces a unified 30m x 20m MuJoCo XML
with 4 aisles (2.4m width), 40 exhibition booths with AprilTag tag36h11 markers,
surface friction zones (Hardwood Stage, Terrazzo Aisles, Carpeted Bays),
and dynamic crowd capsules.
"""

import os
import math
import yaml

def generate_40_stalls_world_xml(yaml_path: str, out_xml_path: str) -> str:
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    venue = data.get("venue", {})
    dock = data.get("recharge_dock", {})
    stalls = data.get("stalls", [])

    xml_lines = [
        '<mujoco model="Project Cloud: DPSI EXPO-2026 40-Stall Exhibition World">',
        '  <include file="go2.xml"/>',
        '',
        '  <visual>',
        '    <headlight diffuse="0.45 0.45 0.48" ambient="0.22 0.22 0.25" specular="0.08 0.08 0.08"/>',
        '    <rgba haze="0.04 0.06 0.08 1"/>',
        '    <global azimuth="-120" elevation="-22" offwidth="1920" offheight="1080"/>',
        '    <quality shadowsize="4096"/>',
        '  </visual>',
        '',
        '  <asset>',
        '    <!-- Skybox & Studio Atmosphere -->',
        '    <texture type="skybox" builtin="gradient" rgb1="0.10 0.13 0.18" rgb2="0.01 0.02 0.04" width="512" height="3072"/>',
        '',
        '    <!-- High-End Dark Polished Terrazzo Exhibition Floor (Aisle Friction: 0.32) -->',
        '    <texture type="2d" name="floor_tex" builtin="checker" mark="none" rgb1="0.10 0.11 0.14" rgb2="0.13 0.15 0.18" width="512" height="512"/>',
        '    <material name="floor_mat" texture="floor_tex" texuniform="true" texrepeat="15 15" reflectance="0.18"/>',
        '',
        '    <!-- Materials -->',
        '    <material name="wall_satin_mat" rgba="0.88 0.90 0.92 1.0" reflectance="0.08"/>',
        '    <material name="wall_acoustic_mat" rgba="0.13 0.15 0.18 1.0" reflectance="0.12"/>',
        '    <material name="glass_structural_mat" rgba="0.22 0.48 0.72 0.28" reflectance="0.40"/>',
        '    <material name="brushed_chrome_mat" rgba="0.78 0.80 0.84 1.0" reflectance="0.65"/>',
        '    <material name="led_cyan_accent" rgba="0.0 0.80 0.95 1.0" emission="0.75"/>',
        '    <material name="led_amber_accent" rgba="1.0 0.55 0.0 1.0" emission="0.80"/>',
        '    <material name="booth_fascia_mat" rgba="0.08 0.10 0.14 1.0" reflectance="0.20"/>',
        '    <material name="stage_slate_mat" rgba="0.16 0.18 0.22 1.0" reflectance="0.12"/>',
        '    <material name="stage_cove_rim_mat" rgba="0.0 0.75 0.95 0.85" emission="0.55"/>',
        '    <material name="booth_plinth_mat" rgba="0.12 0.14 0.18 1.0" reflectance="0.15"/>',
        '    <material name="tag_board_white_mat" rgba="0.96 0.96 0.96 1.0" reflectance="0.08"/>',
        '    <material name="tag_board_black_mat" rgba="0.02 0.02 0.02 1.0" reflectance="0.02"/>',
        '    <material name="spill_slick_mat" rgba="0.22 0.36 0.52 0.70" reflectance="0.65"/>',
        '    <material name="cone_safety_orange_mat" rgba="1.00 0.34 0.02 1.0" emission="0.18"/>',
        '    <material name="cone_prismatic_white_mat" rgba="0.96 0.96 0.96 1.0" reflectance="0.45"/>',
        '    <material name="crowd_jacket_mat" rgba="0.25 0.45 0.65 1.0" reflectance="0.10"/>',
        '  </asset>',
        '',
        '  <worldbody>',
        '    <!-- Studio Lighting Grid -->',
        '    <light pos="10.0 0.0 8.5" dir="0 0 -1" diffuse="0.48 0.48 0.50" specular="0.05 0.05 0.05" directional="true"/>',
        '    <light pos="-2.0 0.0 5.0" dir="0 0 -1" diffuse="0.70 0.72 0.76" cutoff="55" castshadow="true"/>',
        '    <light pos="6.0 6.0 5.0" dir="0 0 -1" diffuse="0.65 0.68 0.74" cutoff="50" castshadow="true"/>',
        '    <light pos="16.0 6.0 5.0" dir="0 0 -1" diffuse="0.65 0.68 0.74" cutoff="50" castshadow="true"/>',
        '    <light pos="6.0 -6.0 5.0" dir="0 0 -1" diffuse="0.65 0.68 0.74" cutoff="50" castshadow="true"/>',
        '    <light pos="16.0 -6.0 5.0" dir="0 0 -1" diffuse="0.65 0.68 0.74" cutoff="50" castshadow="true"/>',
        '',
        '    <!-- 1. Terrazzo Pavilion Floor (30m x 20m) with friction = 0.32 -->',
        '    <geom name="pavilion_floor" size="30 30 0.05" type="plane" material="floor_mat" friction="0.32 0.01 0.002"/>',
        '',
        '    <!-- 2. Central Keynote Stage (Hardwood, friction = 0.22) & Recharge Dock (Marker 99) -->',
        f'    <body name="central_stage" pos="{dock.get("pose", {}).get("x", -2.0)} {dock.get("pose", {}).get("y", 0.0)} 0.020">',
        '      <!-- Polished Hardwood Stage Floor (mu = 0.22, torsion = 0.005) -->',
        '      <geom name="stage_hardwood_base" type="cylinder" size="2.4 0.020" material="stage_slate_mat" friction="0.22 0.005 0.001"/>',
        '      <geom name="stage_cove_ring" type="cylinder" size="2.43 0.004" pos="0 0 -0.015" material="stage_cove_rim_mat" contype="0" conaffinity="0"/>',
        '      <!-- Stage Dock AprilTag 99 Plate (Mounted at West Back of Dais) -->',
        '      <geom name="dock_tag_backing" type="box" size="0.01 0.22 0.22" pos="-1.40 0.0 0.30" material="tag_board_white_mat"/>',
        '      <geom name="dock_tag_code" type="box" size="0.012 0.16 0.16" pos="-1.398 0.0 0.30" material="tag_board_black_mat"/>',
        '      <geom name="dock_tag_accent" type="box" size="0.014 0.04 0.04" pos="-1.396 -0.05 0.35" material="tag_board_white_mat"/>',
        '    </body>',
        '',
        '    <!-- 3. Perimeter Architectural Walls (30m x 20m arena: x in [-5, 25], y in [-10, 10]) -->',
        '    <body name="wall_west" pos="-5.0 0.0 1.5"><geom name="geom_wall_w" type="box" size="0.10 10.0 1.5" material="wall_satin_mat"/></body>',
        '    <body name="wall_east" pos="25.0 0.0 1.5"><geom name="geom_wall_e" type="box" size="0.10 10.0 1.5" material="wall_satin_mat"/></body>',
        '    <body name="wall_north" pos="10.0 10.0 1.5"><geom name="geom_wall_n" type="box" size="15.0 0.10 1.5" material="wall_satin_mat"/></body>',
        '    <body name="wall_south" pos="10.0 -10.0 1.5"><geom name="geom_wall_s" type="box" size="15.0 0.10 1.5" material="wall_satin_mat"/></body>',
        '',
        '    <!-- 4. Exhibition Booths (40 Stalls Generated from stalls_database.yaml) -->'
    ]

    for s in stalls:
        s_id = s["id"]
        tag_id = s.get("apriltag_id", 10)
        b_pose = s.get("booth_pose", {})
        bx = b_pose.get("x", 0.0)
        by = b_pose.get("y", 0.0)
        byaw = b_pose.get("yaw", 0.0)

        # All stalls face the aisle centerlines along the Y axis (byaw = +/- pi/2)
        # Front of booth (AprilTag) faces towards the aisle:
        tag_offset_y = 0.46 * math.sin(byaw)
        tag_offset_x = 0.0
        
        # Acoustic Back Partition sits behind the table, opposite the aisle:
        back_wall_y = -0.55 * math.sin(byaw)
        back_wall_x = 0.0

        xml_lines.extend([
            f'    <!-- {s_id}: {s.get("title", "")} (AprilTag {tag_id}) -->',
            f'    <body name="body_{s_id}" pos="{bx} {by} 0.45">',
            f'      <!-- Carpeted Booth Base (Friction: 0.75) -->',
            f'      <geom name="geom_table_{s_id}" type="box" size="0.85 0.45 0.45" material="booth_plinth_mat" friction="0.75 0.05 0.01"/>',
            f'      <!-- AprilTag tag36h11 Plate (ID {tag_id}) -->',
            f'      <geom name="geom_tag_bg_{s_id}" type="box" size="0.16 0.008 0.16" pos="{tag_offset_x:.3f} {tag_offset_y:.3f} 0.20" material="tag_board_white_mat"/>',
            f'      <geom name="geom_tag_fg_{s_id}" type="box" size="0.11 0.010 0.11" pos="{tag_offset_x:.3f} {tag_offset_y*1.02:.3f} 0.20" material="tag_board_black_mat"/>',
            f'      <!-- Acoustic Back Partition -->',
            f'      <geom name="geom_back_{s_id}" type="box" size="0.95 0.05 0.75" pos="{back_wall_x:.3f} {back_wall_y:.3f} 0.75" material="wall_acoustic_mat"/>',
            f'    </body>'
        ])

    # 5. Dynamic Human Crowd Capsules in Aisles (Students & Evaluators)
    crowd_positions = [
        (6.0, 6.0, "crowd_student_01"),
        (14.0, 6.0, "crowd_student_02"),
        (10.0, 2.0, "crowd_student_03"),
        (18.0, 2.0, "crowd_student_04"),
        (6.0, -2.0, "crowd_student_05"),
        (14.0, -2.0, "crowd_student_06"),
        (10.0, -6.0, "crowd_student_07"),
        (18.0, -6.0, "crowd_student_08"),
    ]
    xml_lines.append('')
    xml_lines.append('    <!-- 5. Dynamic Student Crowds & Evaluators (Moving Obstacles) -->')
    for cx, cy, cname in crowd_positions:
        xml_lines.extend([
            f'    <body name="{cname}" pos="{cx} {cy} 0.85">',
            f'      <geom name="geom_{cname}_torso" type="capsule" size="0.18 0.55" material="crowd_jacket_mat"/>',
            f'      <geom name="geom_{cname}_head" type="sphere" size="0.12" pos="0 0 0.72" material="wall_satin_mat"/>',
            f'    </body>'
        ])

    xml_lines.extend([
        '',
        '    <!-- Dedicated Cameras -->',
        '    <camera name="cam_go2_third_person" pos="-2.2 0.0 1.1" mode="targetbody" target="base"/>',
        '    <camera name="cam_stage_greeting" pos="-4.0 0.0 1.2" mode="targetbody" target="base"/>',
        '    <camera name="cam_aisle_a_overview" pos="2.0 6.0 1.5" mode="targetbody" target="base"/>',
        '    <camera name="cam_pavilion_overview" pos="10.0 -14.0 12.0" xyaxes="1 0 0 0 0.55 0.83"/>',
        '  </worldbody>',
        '</mujoco>'
    ])

    full_xml = "\n".join(xml_lines)
    os.makedirs(os.path.dirname(out_xml_path), exist_ok=True)
    with open(out_xml_path, "w", encoding="utf-8") as f:
        f.write(full_xml)

    return out_xml_path


if __name__ == "__main__":
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    repo_dir = os.path.dirname(cur_dir)
    yaml_in = os.path.join(repo_dir, "config", "stalls_database.yaml")
    xml_out = os.path.join(cur_dir, "unitree_go2", "dpsi_expo_world_40_stalls.xml")
    generate_40_stalls_world_xml(yaml_in, xml_out)
    print(f"Generated 40-stall world: {xml_out}")
