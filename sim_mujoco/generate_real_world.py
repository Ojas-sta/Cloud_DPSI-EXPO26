#!/usr/bin/env python3
"""
Generate Blueprint-Accurate DPSI Exhibition World with:
1. 44 Booth ArUco Markers on Stall Aprons and Tables (IDs 1 to 44)
2. 24 Concourse Floor ArUco Ground Anchors (IDs 100 to 123)
3. Non-glare matte materials (emission="1.0" specular="0")
4. Downward & Forward Camera setup on Unitree Go2 Edu
"""

import os
import json
import math
import yaml
import cv2

def generate_real_world():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_dir = os.path.dirname(script_dir)
    
    # Check layout in repo config first, fallback to external desktop project
    layout_path = os.path.join(repo_dir, "config", "world_layout.json")
    if not os.path.exists(layout_path):
        layout_path = "/Users/roopalisingh/Desktop/projects/dpsi-exhibition/robotics_world/world_layout.json"
    
    with open(layout_path, "r", encoding="utf-8") as f:
        layout = json.load(f)

    # Load floor aruco registry
    floor_reg_path = os.path.join(repo_dir, "config", "floor_aruco_registry.yaml")
    floor_markers = []
    if os.path.exists(floor_reg_path):
        with open(floor_reg_path, "r", encoding="utf-8") as f:
            floor_data = yaml.safe_load(f)
            floor_markers = floor_data.get("markers", [])

    go2_dir = os.path.join(script_dir, "unitree_go2")
    assets_dir = os.path.join(go2_dir, "assets")
    aruco_dir = os.path.join(assets_dir, "aruco")
    os.makedirs(aruco_dir, exist_ok=True)

    # Use DICT_4X4_250 to support both booth IDs (1-44) and floor IDs (100-123)
    dict_4x4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

    # 1. Generate ArUco PNG textures for each booth (IDs 1-44)
    booths = layout.get("booths", [])
    print(f"[*] Generating authentic ArUco textures for {len(booths)} booths...")
    
    booth_aruco_ids = {}
    for idx, b in enumerate(booths):
        bid = b["booth_code"]
        tag_id = idx + 1 # 1 to 44
        booth_aruco_ids[bid] = tag_id
        
        marker_img = cv2.aruco.generateImageMarker(dict_4x4, tag_id, 256, borderBits=1)
        png_path = os.path.join(aruco_dir, f"aruco_{bid}.png")
        cv2.imwrite(png_path, marker_img)

    # 2. Generate ArUco PNG textures for floor markers (IDs 100-123)
    print(f"[*] Generating authentic ArUco textures for {len(floor_markers)} floor highway markers...")
    for fm in floor_markers:
        tag_id = fm["tag_id"]
        marker_img = cv2.aruco.generateImageMarker(dict_4x4, tag_id, 256, borderBits=1)
        png_path = os.path.join(aruco_dir, f"aruco_floor_{tag_id}.png")
        cv2.imwrite(png_path, marker_img)

    print(f"[✓] Created textures in {aruco_dir}")

    # 3. Build MJCF XML lines
    lines = []
    lines.append('<?xml version="1.0" encoding="utf-8"?>')
    lines.append('<mujoco model="Project Cloud: DPSI Expo Real World with ArUco Markers">')
    lines.append('  <include file="go2.xml"/>')
    lines.append('  <compiler angle="radian" coordinate="local" autolimits="true"/>')
    lines.append('  <option gravity="0 0 -9.81" timestep="0.005" integrator="implicitfast"/>')
    lines.append('')
    lines.append('  <visual>')
    lines.append('    <global elevation="-22" azimuth="115"/>')
    lines.append('    <headlight ambient="0.45 0.45 0.45" diffuse="0.85 0.85 0.85" specular="0.25 0.25 0.25"/>')
    lines.append('  </visual>')
    lines.append('')
    lines.append('  <asset>')
    lines.append('    <texture type="skybox" builtin="gradient" rgb1="0.08 0.12 0.22" rgb2="0.02 0.04 0.08" width="512" height="512"/>')
    lines.append('    <texture name="floor_grid" type="2d" builtin="checker" rgb1="0.88 0.90 0.93" rgb2="0.78 0.81 0.85" width="1024" height="1024"/>')
    lines.append('    <material name="floor_mat" texture="floor_grid" texrepeat="30 25" reflectance="0.15" roughness="0.4"/>')
    lines.append('    <material name="wall_mat" rgba="0.25 0.32 0.42 1.0" roughness="0.5"/>')
    lines.append('    <material name="wood_mat" rgba="0.65 0.45 0.28 1.0" roughness="0.4"/>')
    lines.append('    <material name="portal_mat" rgba="0.20 0.25 0.35 1.0" specular="0.8" shininess="0.5"/>')
    lines.append('    <material name="glass_mat" rgba="0.70 0.85 0.95 0.45" roughness="0.1"/>')
    lines.append('    <material name="desk_mat" rgba="0.78 0.60 0.40 1.0" roughness="0.3"/>')
    lines.append('    <material name="plinth_mat" rgba="0.12 0.16 0.24 1.0" roughness="0.5"/>')
    lines.append('    <material name="screen_mat" rgba="0.05 0.08 0.15 1.0" roughness="0.2"/>')
    lines.append('    <material name="marker_stand_mat" rgba="0.15 0.15 0.18 1.0" roughness="0.4"/>')
    lines.append('    <material name="white_mat" rgba="0.95 0.95 0.95 1.0" roughness="0.2"/>')
    lines.append('')
    lines.append('    <!-- Category accent materials -->')
    lines.append('    <material name="mat_physical" rgba="0.96 0.62 0.07 1.0"/>')
    lines.append('    <material name="mat_digital" rgba="0.02 0.52 0.78 1.0"/>')
    lines.append('    <material name="mat_business" rgba="0.02 0.59 0.41 1.0"/>')
    lines.append('    <material name="mat_showntell" rgba="0.49 0.23 0.93 1.0"/>')
    lines.append('    <material name="mat_robotics" rgba="0.86 0.15 0.15 1.0"/>')
    lines.append('    <material name="track_mat" rgba="0.12 0.14 0.18 1.0" roughness="0.8"/>')
    lines.append('    <material name="turf_mat" rgba="0.10 0.45 0.22 1.0" roughness="0.9"/>')
    lines.append('')
    lines.append('    <!-- ArUco Marker Textures & Materials on Stall Tables (IDs 1-44, Anti-Glare Matte) -->')
    for b in booths:
        bid = b["booth_code"]
        lines.append(f'    <texture name="tex_aruco_{bid}" type="2d" file="assets/aruco/aruco_{bid}.png"/>')
        lines.append(f'    <material name="mat_aruco_{bid}" texture="tex_aruco_{bid}" emission="1.0" specular="0" shininess="0"/>')
    lines.append('')
    lines.append('    <!-- Floor ArUco Highway Textures & Materials (IDs 100-123, Non-Reflective Matte) -->')
    for fm in floor_markers:
        tid = fm["tag_id"]
        lines.append(f'    <texture name="tex_aruco_floor_{tid}" type="2d" file="assets/aruco/aruco_floor_{tid}.png"/>')
        lines.append(f'    <material name="mat_aruco_floor_{tid}" texture="tex_aruco_floor_{tid}" emission="1.0" specular="0" shininess="0" roughness="0.9"/>')
    lines.append('  </asset>')
    lines.append('')

    lines.append('  <worldbody>')
    lines.append('    <!-- 1. Ambient & Directional Lighting -->')
    lines.append('    <light pos="0 10 25" dir="0 0 -1" diffuse="0.85 0.85 0.85" specular="0.3 0.3 0.3" castshadow="true"/>')
    lines.append('    <light pos="20 25 20" dir="-0.5 -0.5 -1" diffuse="0.5 0.5 0.5"/>')
    lines.append('    <light pos="-20 -5 18" dir="0.5 0.5 -1" diffuse="0.5 0.5 0.5"/>')
    lines.append('')
    lines.append('    <!-- 2. Global Ground Floor (59.6m x 45.6m) -->')
    lines.append('    <geom name="ground_floor" type="plane" pos="-2.0 12.0 0.0" size="30.0 23.0 0.1" material="floor_mat" friction="0.8 0.1 0.1" contype="1" conaffinity="1"/>')
    lines.append('')

    lines.append('    <!-- 3. Perimeter Enclosure Walls & Partitions -->')
    for w in layout.get("walls", []):
        hx = w["size"][0] / 2.0
        hy = w["size"][1] / 2.0
        hz = w["size"][2] / 2.0
        mat = "wall_mat"
        if "wood" in w.get("material", ""):
            mat = "wood_mat"
        elif "portal" in w.get("material", ""):
            mat = "portal_mat"
        elif "glass" in w.get("material", "") or "balustrade" in w.get("material", ""):
            mat = "glass_mat"
        lines.append(f'    <geom name="{w["name"]}" type="box" pos="{w["pos"][0]} {w["pos"][1]} {w["pos"][2]}" size="{hx:.4f} {hy:.4f} {hz:.4f}" material="{mat}" contype="1" conaffinity="1"/>')
    lines.append('')

    lines.append('    <!-- 4. Portal Framing Columns & Lintels -->')
    for col in layout.get("portal_columns", []):
        hx = col["size"][0] / 2.0
        hy = col["size"][1] / 2.0
        hz = col["size"][2] / 2.0
        lines.append(f'    <geom name="{col["name"]}" type="box" pos="{col["pos"][0]} {col["pos"][1]} {col["pos"][2]}" size="{hx:.4f} {hy:.4f} {hz:.4f}" material="portal_mat" contype="1" conaffinity="1"/>')
    lines.append('')

    lines.append('    <!-- 5. Zone 3 Robotics Arena: Speedway, Ramps & Obstacles -->')
    speedway = layout["robotics_arena"]["arena_1_speedway"]
    lines.append(f'    <geom name="speedway_floor" type="box" pos="{speedway["center"][0]} {speedway["center"][1]} 0.01" size="4.0 4.0 0.02" material="track_mat"/>')
    for bw in speedway.get("boundary_walls", []):
        hx, hy, hz = bw["size"][0]/2.0, bw["size"][1]/2.0, bw["size"][2]/2.0
        lines.append(f'    <geom name="{bw["name"]}" type="box" pos="{bw["pos"][0]} {bw["pos"][1]} {bw["pos"][2]}" size="{hx:.4f} {hy:.4f} {hz:.4f}" material="portal_mat" contype="1" conaffinity="1"/>')
    for r in speedway.get("ramp_obstacle", []):
        hx, hy, hz = r["size"][0]/2.0, r["size"][1]/2.0, r["size"][2]/2.0
        if r["type"] == "bridge":
            lines.append(f'    <geom name="{r["name"]}" type="box" pos="{r["pos"][0]} {r["pos"][1]} {r["pos"][2]}" size="{hx:.4f} {hy:.4f} {hz:.4f}" material="track_mat" contype="1" conaffinity="1"/>')
        elif r["type"] == "ramp_up":
            lines.append(f'    <geom name="{r["name"]}" type="box" pos="{r["pos"][0]} {r["pos"][1]} {r["pos"][2]}" size="{hx:.4f} {hy:.4f} 0.04" euler="0 -0.36 0" material="track_mat" contype="1" conaffinity="1"/>')
        elif r["type"] == "ramp_down":
            lines.append(f'    <geom name="{r["name"]}" type="box" pos="{r["pos"][0]} {r["pos"][1]} {r["pos"][2]}" size="{hx:.4f} {hy:.4f} 0.04" euler="0 0.36 0" material="track_mat" contype="1" conaffinity="1"/>')
    lines.append('')

    lines.append('    <!-- 6. Zone 3 Robotics Arena: Robo Soccer Pitch -->')
    soccer = layout["robotics_arena"]["arena_2_soccer"]
    lines.append(f'    <geom name="soccer_turf" type="box" pos="{soccer["center"][0]} {soccer["center"][1]} 0.01" size="5.0 7.0 0.02" material="turf_mat"/>')
    for bw in soccer.get("boundary_walls", []):
        hx, hy, hz = bw["size"][0]/2.0, bw["size"][1]/2.0, bw["size"][2]/2.0
        lines.append(f'    <geom name="{bw["name"]}" type="box" pos="{bw["pos"][0]} {bw["pos"][1]} {bw["pos"][2]}" size="{hx:.4f} {hy:.4f} {hz:.4f}" material="portal_mat" contype="1" conaffinity="1"/>')
    for g in soccer.get("goals", []):
        hx, hy, hz = g["size"][0]/2.0, g["size"][1]/2.0, g["size"][2]/2.0
        lines.append(f'    <geom name="{g["name"]}" type="box" pos="{g["pos"][0]} {g["pos"][1]} {g["pos"][2]}" size="{hx:.4f} {hy:.4f} {hz:.4f}" rgba="0.9 0.9 0.9 0.6" contype="1" conaffinity="1"/>')
    lines.append('')

    lines.append('    <!-- 7. Concourse Optical Highway Floor ArUco Ground Anchors (IDs 100-123) -->')
    for fm in floor_markers:
        tid = fm["tag_id"]
        fx = fm["pose"]["x"]
        fy = fm["pose"]["y"]
        fz = fm["pose"]["z"]
        fyaw = fm["pose"]["yaw_rad"]
        name = fm["name"]
        lines.append(f'    <!-- Floor Marker #{tid}: {name} ({fm.get("corridor", "")}) -->')
        lines.append(f'    <geom name="geom_floor_tag_{tid}" type="box" pos="{fx:.3f} {fy:.3f} {fz:.4f}" size="0.09 0.09 0.0005" euler="0 0 {fyaw:.4f}" material="mat_aruco_floor_{tid}" contype="0" conaffinity="0"/>')
        lines.append(f'    <site name="site_floor_tag_{tid}" pos="{fx:.3f} {fy:.3f} {fz:.4f}" size="0.015" rgba="0.0 0.9 1.0 0.5"/>')
    lines.append('')

    lines.append('    <!-- 8. 44 Exhibition Stalls with Tables, Screens & Apron/Table ArUco Markers -->')
    cat_mat_map = {
        'Physical': 'mat_physical',
        'Digital': 'mat_digital',
        'Business': 'mat_business',
        'Show & Tell': 'mat_showntell',
        'Robo Race': 'mat_robotics',
        'Robo Soccer': 'mat_robotics'
    }

    for b in booths:
        bid = b["booth_code"]
        px, py, pz = b["pos"]
        yaw = b["yaw"]
        mat = cat_mat_map.get(b["category"], "mat_physical")
        tag_id = booth_aruco_ids[bid]

        lines.append(f'    <body name="stall_{bid}" pos="{px} {py} {pz}" euler="0 0 {yaw}">')
        # Plinth
        lines.append(f'      <geom name="plinth_{bid}" type="box" pos="0 0 0.03" size="1.15 0.85 0.03" material="plinth_mat" contype="1" conaffinity="1"/>')
        # Perimeter category accent
        lines.append(f'      <geom name="accent_{bid}" type="box" pos="0 0 0.065" size="1.11 0.81 0.008" material="{mat}"/>')
        # Desk surface (y = 0.72m)
        lines.append(f'      <geom name="desk_{bid}" type="box" pos="0 0 0.72" size="1.0 0.4 0.025" material="desk_mat" contype="1" conaffinity="1"/>')
        # Desk front apron skirt (Z=0.45m) for low-angle eye-level camera acquisition
        lines.append(f'      <geom name="apron_{bid}" type="box" pos="0 -0.38 0.45" size="0.95 0.015 0.15" material="portal_mat" contype="1" conaffinity="1"/>')
        lines.append(f'      <geom name="aruco_apron_face_{bid}" type="box" pos="0.30 -0.398 0.45" size="0.08 0.001 0.08" material="mat_aruco_{bid}" contype="0" conaffinity="0"/>')
        lines.append(f'      <site name="aruco_apron_site_{bid}" pos="0.30 -0.400 0.45" size="0.015" rgba="0 1 0 0.6"/>')
        # Legs
        lines.append(f'      <geom name="leg_l_{bid}" type="box" pos="-0.92 0 0.36" size="0.02 0.36 0.34" material="portal_mat" contype="1" conaffinity="1"/>')
        lines.append(f'      <geom name="leg_r_{bid}" type="box" pos="0.92 0 0.36" size="0.02 0.36 0.34" material="portal_mat" contype="1" conaffinity="1"/>')
        # Glass partition
        lines.append(f'      <geom name="glass_{bid}" type="box" pos="-1.12 0 0.75" size="0.01 0.77 0.67" material="glass_mat" contype="1" conaffinity="1"/>')
        # Monitor
        lines.append(f'      <geom name="monitor_{bid}" type="box" pos="0 -0.15 1.15" size="0.6 0.03 0.35" material="screen_mat" contype="1" conaffinity="1"/>')
        
        # --- TABLETOP ARUCO MARKER STAND ---
        lines.append(f'      <!-- Tabletop ArUco Marker #{tag_id} -->')
        lines.append(f'      <geom name="aruco_stand_{bid}" type="box" pos="0.50 -0.20 0.755" size="0.10 0.03 0.01" material="marker_stand_mat" contype="0" conaffinity="0"/>')
        lines.append(f'      <geom name="aruco_back_{bid}" type="box" pos="0.50 -0.20 0.855" size="0.09 0.006 0.09" material="white_mat" contype="0" conaffinity="0"/>')
        lines.append(f'      <geom name="aruco_face_{bid}" type="box" pos="0.50 -0.207 0.855" size="0.08 0.001 0.08" material="mat_aruco_{bid}" contype="0" conaffinity="0"/>')
        lines.append(f'      <site name="aruco_site_{bid}" pos="0.50 -0.210 0.855" size="0.015" rgba="0 1 0 0.5"/>')

        # Visitor / Robot Staging Waypoint
        lines.append(f'      <site name="waypoint_{bid}" pos="0 -1.2 0.2" size="0.1" rgba="0 1 0 0.4"/>')
        lines.append('    </body>')

    lines.append('')
    lines.append('    <!-- Cameras -->')
    lines.append('    <!-- Default 3rd Person Follow Camera tracking Go2 base -->')
    lines.append('    <camera name="cam_go2_third_person" mode="targetbody" target="base"/>')
    lines.append('    <camera name="entrance_gate_overview" pos="6.4 -14.0 6.5" euler="0.55 0 0"/>')
    lines.append('    <camera name="isometric_overview" pos="18.0 -22.0 28.0" euler="0.75 0 0.65"/>')
    lines.append('    <camera name="show_and_tell_cam" pos="-18.0 -12.0 12.0" euler="0.65 0 0"/>')
    lines.append('    <camera name="robotics_arena_cam" pos="10.0 10.0 15.0" euler="0.70 0 0"/>')
    lines.append('  </worldbody>')
    lines.append('')

    lines.append('  <!-- Spawn Keyframes: Starting from South Entrance Gate facing North -->')
    lines.append('  <keyframe>')
    lines.append('    <key name="gate_spawn_right" qpos="6.4 -7.5 0.35 0.7071068 0 0 0.7071068 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8"/>')
    lines.append('    <key name="gate_spawn_center" qpos="0.0 -7.5 0.35 0.7071068 0 0 0.7071068 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8"/>')
    lines.append('  </keyframe>')
    lines.append('</mujoco>')

    out_xml_path = os.path.join(go2_dir, "dpsi_expo_real_world.xml")
    with open(out_xml_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print(f"\n[🎉] Blueprint-accurate exhibition world successfully generated at:\n    {out_xml_path}")
    return out_xml_path

if __name__ == "__main__":
    generate_real_world()
