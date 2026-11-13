#!/usr/bin/env python3
"""
Relative ArUco Topological Graph Navigator (Teach-and-Repeat Phase 2)
Venue: DPSI EXPO-2026 Exhibition Pavilion
Platform: Unitree Go2 Edu / Pro Quadruped

Features:
  • Zero pre-drawn blueprint or hardcoded (X, Y) world coordinates needed
  • Loads automatically discovered topological graph ('config/discovered_aruco_graph.yaml')
  • Cold-Start / Kidnapped Robot Localization (snaps pose from ANY visible ArUco tag)
  • Dijkstra shortest-path topological routing through Floor tags (ID>=100) to Booth tags (1..44)
  • Hop-by-hop relative dead-reckoning with drift resetting at each intermediate marker
  • Closed-loop solvePnP visual servoing to 0.95m terminal standoff at booth aprons
"""

import os
import sys
import math
import time
import yaml
import subprocess
import networkx as nx
from typing import Dict, List, Tuple, Any, Optional

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(WORKSPACE_DIR, "config")


class TopologicalArucoNavigator:
    def __init__(
        self,
        graph_yaml_filename: str = "discovered_aruco_graph.yaml",
        use_apriltag: bool = False
    ):
        self.graph_path = os.path.join(CONFIG_DIR, graph_yaml_filename)
        self.use_apriltag = use_apriltag
        self.fiducial_family = "AprilTag (36h11)" if use_apriltag else "ArUco (4X4_250)"
        self.graph = nx.DiGraph()
        self.nodes_data: Dict[int, Dict[str, Any]] = {}
        self.edges_data: List[Dict[str, Any]] = []

        # Navigation & Localization State
        self.is_localized = False
        self.current_anchor_tag: Optional[int] = None
        self.active_route: List[int] = []
        self.active_route_index = 0
        self.target_booth_tag: Optional[int] = None
        self.docked_at_booth = False

        # Load topological graph
        self.load_graph()

        # Load Curricula Fact Sheets (project descriptions without coordinates)
        self.curricula = self.load_curricula()

    def load_graph(self) -> bool:
        """Loads the discovered ArUco topological pose graph from YAML."""
        if not os.path.exists(self.graph_path):
            print(f"  ⚠️ [TopologicalNavigator]: Discovered graph '{self.graph_path}' not found! Run Discovery Mode first.")
            return False

        with open(self.graph_path, "r") as f:
            data = yaml.safe_load(f)

        self.nodes_data = {}
        for str_id, n_data in data.get("nodes", {}).items():
            nid = int(str_id)
            self.nodes_data[nid] = n_data
            self.graph.add_node(nid, **n_data)

        self.edges_data = data.get("edges", [])
        for e in self.edges_data:
            u = int(e["from"])
            v = int(e["to"])
            w = float(e.get("distance_m", 1.0))
            self.graph.add_edge(u, v, weight=w, **e)

        print(f"  ✓ [TopologicalNavigator]: Loaded topological graph from {os.path.basename(self.graph_path)} ({len(self.nodes_data)} nodes, {len(self.edges_data)} edges)")
        return True

    def load_curricula(self) -> Dict[int, Dict[str, Any]]:
        """Loads speech narration scripts mapped by marker_id (NO hardcoded coords)."""
        curricula_map = {}
        cfg_path = os.path.join(CONFIG_DIR, "dpsi_expo_31stalls.yaml")
        if os.path.exists(cfg_path):
            try:
                with open(cfg_path, "r") as f:
                    cfg_data = yaml.safe_load(f)
                for s in cfg_data.get("stalls", []):
                    tid = int(s["tag_id"])
                    curricula_map[tid] = {
                        "stall": s["id"],
                        "title": s["title"],
                        "speech": (
                            f"Honorable evaluators and guests, welcome to Stall {s['id']}: {s['title']}! "
                            f"Evaluated by {s.get('evaluator', 'our faculty panel')}, this student capstone demonstrates "
                            f"cutting-edge mechatronics and engineering innovation here at DPSI EXPO 2026."
                        )
                    }
            except Exception:
                pass
        if not curricula_map:
            curricula_map = {
                1: {
                    "stall": "P01",
                    "title": "Stepwise AI: Biomechanical Gait Analysis",
                    "speech": (
                        "Welcome to Stall P01: Stepwise AI — Biomechanical Gait Analysis! "
                        "Our team deploys wearable 6-axis IMUs and computer vision to analyze ankle pronation, "
                        "detect gait deviations, and quantify rehabilitation progress with clinical precision."
                    )
                },
                6: {
                    "stall": "P06",
                    "title": "Robotic Smart Moving Bin System",
                    "speech": (
                        "Welcome to Stall P06: The Robotic Smart Moving Bin System! "
                        "Engineered by DPSI student innovators, this mobile platform uses planar 2D LiDAR SLAM "
                        "and ultrasonic fill-level telemetry to autonomously navigate concourses and cut maintenance manpower by 65%."
                    )
                },
                2: {
                    "stall": "P02",
                    "title": "Move Smarter: ADAS & Edge Driver Assistance",
                    "speech": (
                        "Welcome to Stall P02: Move Smarter ADAS! "
                        "Featuring millimetric forward radar and edge deep learning for predictive pedestrian collision braking."
                    )
                }
            }
        return curricula_map

    # --------------------------------------------------------------------------
    # COLD-START / KIDNAPPED ROBOT LOCALIZATION
    # --------------------------------------------------------------------------
    def localize_from_visible_tag(
        self,
        detections: List[Dict[str, Any]],
        robot_yaw_deg: float
    ) -> Optional[Tuple[float, float, int]]:
        """
        Kidnapped Robot Recovery:
        Finds any recognized ArUco marker in the camera view and snaps robot pose.
        Returns: (robot_x, robot_y, locked_tag_id) or None.
        """
        if not detections or not self.nodes_data:
            return None

        yaw_rad = math.radians(robot_yaw_deg)

        for det in detections:
            tid = int(det["id"])
            if tid in self.nodes_data:
                tag_node = self.nodes_data[tid]
                tag_x = tag_node["pose"]["x"]
                tag_y = tag_node["pose"]["y"]

                dist = float(det["distance"])
                lat_dx = float(det["dx"])

                # Invert measurement vector to find robot origin in map frame:
                bearing = math.atan2(lat_dx, dist)
                world_bearing = yaw_rad + bearing
                snapped_x = tag_x - dist * math.cos(world_bearing)
                snapped_y = tag_y - dist * math.sin(world_bearing)

                self.is_localized = True
                self.current_anchor_tag = tid
                print(f"  🎯 [TopologicalNavigator]: Cold-Start Pose Snapped from Marker #{tid} -> Robot Pos: ({snapped_x:.2f}m, {snapped_y:.2f}m) | Error Reset to 0.00m")
                return snapped_x, snapped_y, tid

        return None

    # --------------------------------------------------------------------------
    # TOPOLOGICAL DIJKSTRA ROUTING
    # --------------------------------------------------------------------------
    def plan_tag_route(self, start_tag_id: int, target_booth_id: int) -> List[int]:
        """
        Computes the shortest topological hop sequence through Floor tags to target Booth tag.
        Example: [100, 101, 1]
        """
        if start_tag_id not in self.graph:
            print(f"  ⚠️ Start tag #{start_tag_id} not in topological graph!")
            return []
        if target_booth_id not in self.graph:
            print(f"  ⚠️ Target Booth tag #{target_booth_id} not in topological graph!")
            return []

        try:
            path = nx.shortest_path(self.graph, source=start_tag_id, target=target_booth_id, weight="weight")
            self.active_route = path
            self.active_route_index = 0
            self.target_booth_tag = target_booth_id
            self.docked_at_booth = False
            route_str = " -> ".join([f"#{t}" for t in path])
            print(f"  🗺️ [TopologicalNavigator]: Computed Dijkstra Route: {route_str}")
            return path
        except nx.NetworkXNoPath:
            print(f"  ❌ No topological path found between #{start_tag_id} and #{target_booth_id}!")
            return []

    # --------------------------------------------------------------------------
    # HOP-BY-HOP MOTION CONTROLLER
    # --------------------------------------------------------------------------
    def compute_transit_step(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw_deg: float,
        detections: List[Dict[str, Any]],
        dt: float = 0.04
    ) -> Tuple[float, float, float, Dict[str, Any]]:
        """
        Advances the robot along the topological route.
        Returns: (vx, vy, wz, status_dict)
        """
        if not self.active_route or self.active_route_index >= len(self.active_route) - 1:
            return 0.0, 0.0, 0.0, {"status": "ARRIVED_AT_DESTINATION", "docked": True}

        curr_hop_from = self.active_route[self.active_route_index]
        curr_hop_to = self.active_route[self.active_route_index + 1]
        is_final_booth_hop = (self.nodes_data[curr_hop_to]["tag_type"] == "BOOTH")

        target_pose = self.nodes_data[curr_hop_to]["pose"]
        dx_world = target_pose["x"] - robot_x
        dy_world = target_pose["y"] - robot_y
        dist_to_hop = math.hypot(dx_world, dy_world)

        yaw_rad = math.radians(robot_yaw_deg)
        target_yaw_rad = target_pose.get("theta", yaw_rad)

        # Check if next hop tag is visible in camera
        target_det = next((d for d in detections if d["id"] == curr_hop_to), None)

        if target_det is not None:
            # RESET DRIFT: Optical feedback lock!
            measured_dist = target_det["distance"]
            measured_lat = target_det["dx"]

            if is_final_booth_hop:
                # CLOSED-LOOP VISUAL SERVOING TO 0.95m TERMINAL STANDOFF
                standoff_target = 0.95
                dist_err = measured_dist - standoff_target
                lat_err = measured_lat

                if abs(dist_err) <= 0.08 and abs(lat_err) <= 0.10:
                    self.docked_at_booth = True
                    self.active_route_index += 1
                    print(f"  🎯 [TopologicalNavigator]: DOCKED at Stall #{curr_hop_to} (Standoff: {measured_dist:.2f}m, Lat: {lat_err:+.2f}m)!")
                    return 0.0, 0.0, 0.0, {"status": "DOCKED", "docked": True, "target_tag": curr_hop_to}

                # Proportional visual servoing commands
                cmd_vx = min(0.30, max(0.06, dist_err * 0.65)) if dist_err > 0.0 else -0.05
                cmd_wz = -1.2 * (lat_err / max(0.5, measured_dist))
                cmd_vy = -0.4 * lat_err

                return cmd_vx, cmd_vy, cmd_wz, {
                    "status": f"VISUAL SERVOING DOCK (#{curr_hop_to})",
                    "dist": measured_dist,
                    "lat": lat_err,
                    "docked": False
                }

            else:
                # Floor tag intermediate hop arrival check
                if measured_dist <= 0.60:
                    # Hop reached! Advance route index
                    self.current_anchor_tag = curr_hop_to
                    self.active_route_index += 1
                    print(f"  ✓ [TopologicalNavigator]: Reached Intermediate Floor Waypoint #{curr_hop_to}! Resetting drift.")
                    return 0.10, 0.0, 0.0, {"status": f"REACHED_TAG_#{curr_hop_to}", "docked": False}

        # Dead-reckoning transit along learned relative vector between graph nodes
        desired_angle = math.atan2(dy_world, dx_world)
        angle_err = (desired_angle - yaw_rad + math.pi) % (2.0 * math.pi) - math.pi

        # If heading angle error is significant (>25 deg), pivot first
        if abs(angle_err) > math.radians(25.0):
            cmd_wz = math.copysign(min(0.85, abs(angle_err) * 1.5), angle_err)
            cmd_vx = 0.04
        else:
            cmd_wz = angle_err * 1.2
            # Cruise towards waypoint
            cmd_vx = min(0.35, max(0.08, dist_to_hop * 0.70))

        # Check if close enough to intermediate waypoint even if occluded
        if not is_final_booth_hop and dist_to_hop <= 0.35:
            self.current_anchor_tag = curr_hop_to
            self.active_route_index += 1
            print(f"  ✓ [TopologicalNavigator]: Reached Floor Waypoint #{curr_hop_to} via metric odometry.")

        return cmd_vx, 0.0, cmd_wz, {
            "status": f"TRANSIT (#{curr_hop_from} -> #{curr_hop_to})",
            "hop_from": curr_hop_from,
            "hop_to": curr_hop_to,
            "dist_rem": dist_to_hop,
            "docked": False
        }

    def execute_docent_presentation(
        self,
        robot,
        booth_tag_id: int,
        speech_text: str,
        update_visuals_fn
    ):
        """
        Executes the mandatory 5-Step Docent Presentation Sequence:
          Step 1: Deactivate visual servoing dock & halt.
          Step 2: 180° Presenter Pivot to face the audience/judges.
          Step 3: Lock BALANCE_STAND posture and apply +8.0° eye-contact tilt.
          Step 4: Deliver domain-specific speech narration.
          Step 5: Reset head/body pitch to 0.0° balance posture.
        """
        stage_name = f"STALL #{booth_tag_id} PRESENTATION"
        print(f"\n🎭 [Docent Presentation]: Initiating 5-step sequence for Stall #{booth_tag_id}...")

        # Step 1: Deactivate visual servoing & stop
        robot.stop()
        time.sleep(0.3)
        for _ in range(10):
            update_visuals_fn(f"{stage_name}: STEP 1 - DOCK COMPLETE (HALTED)")
            time.sleep(0.04)

        # Step 2: 180° Presenter Pivot to face audience
        print("  • Step 2: Executing 180° Presenter Pivot to face audience...")
        turn_start = time.time()
        turn_dur = 3.2
        while time.time() - turn_start < turn_dur:
            elapsed = time.time() - turn_start
            progress = min(1.0, elapsed / turn_dur)
            wz = 1.0 * (math.pi / turn_dur) * (math.pi / 2.0) * math.sin(progress * math.pi)
            robot.move(0.0, 0.0, wz)
            update_visuals_fn(f"{stage_name}: STEP 2 - 180° PRESENTER PIVOT")
            time.sleep(0.04)

        robot.stop()
        # Heading is now reversed (facing audience West / 180°)
        robot.yaw_deg = (robot.yaw_deg + 180.0) % 360.0
        if robot.yaw_deg > 180.0:
            robot.yaw_deg -= 360.0
        print(f"  ✓ 180° Pivot complete. Facing audience (Yaw = {robot.yaw_deg:.1f}°).")

        # Step 3: Lock BALANCE_STAND & apply +8.0° (+0.14 rad) eye-contact pitch tilt
        print("  • Step 3: Locking BALANCE_STAND & applying +8.0° eye-contact pitch tilt...")
        robot.stand_up()
        robot.set_body_pitch(8.0)
        for _ in range(12):
            update_visuals_fn(f"{stage_name}: STEP 3 - EYE CONTACT TILT (+8.0°)")
            time.sleep(0.04)

        # Step 4: Deliver stall speech narration through Go2 physical speaker
        print(f"\n  🔊 [Docent Speech — Stall #{booth_tag_id}]:\n  \"{speech_text}\"\n")
        audio_file = self.get_audio_file_for_tag(booth_tag_id)
        audio_proc = None

        go2_speaker_script = os.path.join(WORKSPACE_DIR, "Day1_Testes", "play_on_go2_speaker.py")
        webrtc_python = "/Users/roopalisingh/PycharmProjects/TEST/.venv/bin/python"

        if os.path.exists(go2_speaker_script) and os.path.exists(webrtc_python):
            print(f"  🐕 Blasting audio through Unitree Go2 PHYSICAL CHASSIS SPEAKER (Stall #{booth_tag_id})...")
            audio_proc = subprocess.Popen([webrtc_python, go2_speaker_script, "--stall", str(booth_tag_id)])
        elif audio_file:
            print(f"  ▶️ Companion fallback audio track: {os.path.basename(audio_file)}")
            if sys.platform == "darwin":
                audio_proc = subprocess.Popen(["afplay", audio_file])
            else:
                audio_proc = subprocess.Popen(["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", audio_file])
        else:
            if sys.platform == "darwin":
                audio_proc = subprocess.Popen(["say", "-v", "Samantha", speech_text])

        # Animate visualizer while audio plays (or fallback loop)
        t_audio_start = time.time()
        if audio_proc:
            while audio_proc.poll() is None:
                update_visuals_fn(f"{stage_name}: STEP 4 - NARRATION", speech_text=speech_text)
                time.sleep(0.05)
                if time.time() - t_audio_start > 26.0:  # Safety cap at 26 seconds
                    audio_proc.kill()
                    break
        else:
            for _ in range(40):
                update_visuals_fn(f"{stage_name}: STEP 4 - NARRATION", speech_text=speech_text)
                time.sleep(0.04)

        # Step 5: Reset pitch to 0.0° balance posture
        print("  • Step 5: Resetting head tilt to 0.0° balance posture...")
        robot.set_body_pitch(0.0)
        for _ in range(12):
            update_visuals_fn(f"{stage_name}: STEP 5 - PITCH RESET (0.0°)")
            time.sleep(0.04)
        print(f"  ✓ 5-Step Docent Presentation Sequence finished successfully.\n")

    def execute_aisle_reentry(
        self,
        robot,
        update_visuals_fn,
        clear_dist_m: float = 0.65,
        target_yaw_deg: float = 90.0
    ):
        """
        Decision 1 (Milestone 4):
        After the 180° Presenter Pivot at the stall, the robot is facing West (180°),
        directly toward the concourse centerline.
        Walk forward 0.65m clear of the stall table, then execute a -90° right turn
        to face North (+90°) and acquire the next Floor Marker in the camera frustum.
        """
        print(f"\n🚪 [Aisle Re-Entry]: Stepping forward {clear_dist_m:.2f}m clear of stall apron...")
        t_fwd = time.time()
        fwd_dur = clear_dist_m / 0.25  # ~2.6 seconds
        while time.time() - t_fwd < fwd_dur:
            robot.move(vx=0.25, vy=0.0, wz=0.0)
            update_visuals_fn("AISLE RE-ENTRY: WALKING CLEAR OF STALL TABLE (0.65m)")
            time.sleep(0.04)

        robot.stop()
        print(f"  ✓ Clear of stall table. Executing right turn to face North ({target_yaw_deg:.1f}°)...")

        t_turn = time.time()
        turn_dur = 2.0
        while time.time() - t_turn < turn_dur:
            robot.move(0.0, 0.0, -0.785)
            update_visuals_fn("AISLE RE-ENTRY: PIVOTING NORTH TOWARD NEXT DATUM")
            time.sleep(0.04)

        robot.stop()
        robot.yaw_deg = target_yaw_deg
        print(f"  ✓ Re-entered concourse arterial track (Yaw = {robot.yaw_deg:.1f}°). Ready for next hop.\n")

    def get_audio_file_for_tag(self, booth_tag_id: int, voice: str = "v3") -> Optional[str]:
        """Resolves local pre-generated audio file path for given booth tag ID."""
        tag_to_code = {
            1: "p_01", 2: "p_02", 3: "p_03", 4: "p_04", 5: "p_05", 6: "p_06",
            7: "p_07", 8: "p_08", 9: "p_09", 10: "p_10", 11: "p_11", 12: "p_12",
            13: "d_01", 14: "d_02", 15: "d_03", 16: "d_04", 17: "d_05", 18: "d_06",
            19: "d_07", 20: "d_08", 21: "d_09", 22: "d_10", 23: "d_11", 24: "d_12",
            25: "b_01", 26: "b_02", 27: "b_03", 28: "b_04", 29: "b_05", 30: "b_06",
            31: "b_07", 32: "display"
        }
        code = tag_to_code.get(booth_tag_id)
        if not code:
            return None
        candidates = [
            os.path.join(WORKSPACE_DIR, "audio", "fish_audio_25s", voice, f"{code}_narration.mp3"),
            os.path.join(WORKSPACE_DIR, "audio", "fish_audio_25s", "v1", f"{code}_narration.mp3"),
            os.path.join(WORKSPACE_DIR, "audio", "docent_narration", f"{code}_narration.mp3")
        ]
        for c in candidates:
            if os.path.exists(c):
                return c
        return None

    def get_speech_for_booth(self, booth_tag_id: int) -> str:
        """Retrieves domain speech text for stall."""
        info = self.curricula.get(booth_tag_id)
        if info:
            return info["speech"]
        return f"Welcome to Exhibition Stall #{booth_tag_id}! Powered by autonomous topological ArUco navigation."
