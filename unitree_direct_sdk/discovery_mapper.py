#!/usr/bin/env python3
"""
Pre-Exhibition Discovery Mapper & Topological ArUco Graph Builder
Venue: DPSI EXPO-2026 Exhibition Pavilion
Platform: Unitree Go2 Edu / Pro Quadruped

Features:
  • Autonomous 1-pass topological mapping (Discovery Mode)
  • Extracts 6-DOF ArUco poses (Floor tags ID>=100, Booth tags 1..44)
  • Automatically links sequential Floor tags and associates Booth tags as leaf nodes
  • Maintains a background 2D occupancy grid from 360° LiDAR scans
  • Exports 'config/discovered_aruco_graph.yaml' and 'config/discovered_occupancy_map.pgm'
"""

import os
import time
import math
import yaml
import numpy as np
import networkx as nx
from typing import Dict, List, Tuple, Any, Optional

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(WORKSPACE_DIR, "config")


class DiscoveryMapper:
    def __init__(
        self,
        grid_resolution_m: float = 0.10,
        grid_size_cells: int = 200,
        grid_origin_xy: Tuple[float, float] = (-5.0, 5.0)
    ):
        self.resolution = grid_resolution_m
        self.grid_size = grid_size_cells
        self.origin_x, self.origin_y = grid_origin_xy

        # 2D Occupancy Grid: 128 = unknown, 254 = free, 0 = occupied
        self.occupancy_grid = np.full((self.grid_size, self.grid_size), 128, dtype=np.uint8)

        # Topological Pose Graph (networkx.DiGraph)
        self.graph = nx.DiGraph()

        # Observation tracking
        self.discovered_nodes: Dict[int, Dict[str, Any]] = {}
        self.discovered_edges: List[Dict[str, Any]] = []
        self.last_observed_floor_tag: Optional[int] = None
        self.trajectory_history: List[Tuple[float, float, float]] = []

        # Booth Stall Name Mappings (Curricula metadata)
        self.booth_stall_labels = {
            1: "P01 (Stepwise AI)",
            2: "P02 (Move Smarter: ADAS)",
            6: "P06 (Robotic Smart Moving Bin)",
            10: "BOOTH_01 (Solar Tracking Cart)",
            11: "BOOTH_02 (Agri Drone)"
        }

        # Status
        self.total_observations = 0
        self.mapping_active = True
        self.last_update_time = time.time()

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """Converts world coordinate (meters) to grid coordinate (px, py)."""
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def update_robot_odometry(self, x: float, y: float, yaw_deg: float):
        """Records robot path trajectory and marks free corridor space."""
        self.trajectory_history.append((x, y, yaw_deg))
        gx, gy = self.world_to_grid(x, y)
        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
            # Mark robot path as free space (radius 0.6m)
            r_cells = int(0.60 / self.resolution)
            for dy in range(-r_cells, r_cells + 1):
                for dx in range(-r_cells, r_cells + 1):
                    if dx * dx + dy * dy <= r_cells * r_cells:
                        nx_c, ny_c = gx + dx, gy + dy
                        if 0 <= nx_c < self.grid_size and 0 <= ny_c < self.grid_size:
                            self.occupancy_grid[ny_c, nx_c] = 254

    def update_lidar_scan(self, robot_x: float, robot_y: float, robot_yaw_deg: float, clearance_left: float, clearance_right: float):
        """Updates occupancy grid walls based on 360° LiDAR concourse clearances."""
        yaw_rad = math.radians(robot_yaw_deg)
        left_angle = yaw_rad + math.pi / 2.0
        right_angle = yaw_rad - math.pi / 2.0

        # Left wall obstacle point
        lx = robot_x + clearance_left * math.cos(left_angle)
        ly = robot_y + clearance_left * math.sin(left_angle)
        lgx, lgy = self.world_to_grid(lx, ly)
        if 0 <= lgx < self.grid_size and 0 <= lgy < self.grid_size:
            self.occupancy_grid[lgy, lgx] = 0  # Occupied

        # Right wall obstacle point
        rx = robot_x + clearance_right * math.cos(right_angle)
        ry = robot_y + clearance_right * math.sin(right_angle)
        rgx, rgy = self.world_to_grid(rx, ry)
        if 0 <= rgx < self.grid_size and 0 <= rgy < self.grid_size:
            self.occupancy_grid[rgy, rgx] = 0  # Occupied

    def update_aruco_detections(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw_deg: float,
        detections: List[Dict[str, Any]]
    ):
        """Processes real/emulated OpenCV ArUco detections and updates the pose graph."""
        if not detections:
            return

        yaw_rad = math.radians(robot_yaw_deg)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)

        for det in detections:
            tag_id = int(det["id"])
            dist = float(det["distance"])
            lat_dx = float(det["dx"])

            # Reject noisy / extreme detections (>4.5m)
            if dist > 4.5 or dist < 0.2:
                continue

            self.total_observations += 1
            self.last_update_time = time.time()

            # Estimate marker world position from robot body pose:
            # Forward offset along robot heading ~ sqrt(dist^2 - lat_dx^2)
            # Lateral offset ~ lat_dx (+ to the right, - to the left)
            # Robot world coordinates:
            # x_w = x_r + fwd * cos(yaw) + lat * sin(yaw)
            # y_w = y_r + fwd * sin(yaw) - lat * cos(yaw)
            fwd_comp = math.sqrt(max(0.0, dist**2 - lat_dx**2))
            cos_yaw = math.cos(yaw_rad)
            sin_yaw = math.sin(yaw_rad)
            marker_world_x = robot_x + fwd_comp * cos_yaw + lat_dx * sin_yaw
            marker_world_y = robot_y + fwd_comp * sin_yaw - lat_dx * cos_yaw

            tag_type = "FLOOR" if tag_id >= 100 else "BOOTH"

            if tag_id not in self.discovered_nodes:
                # Newly discovered ArUco node!
                node_data = {
                    "tag_id": tag_id,
                    "tag_type": tag_type,
                    "pose": {
                        "x": round(float(marker_world_x), 3),
                        "y": round(float(marker_world_y), 3),
                        "theta": round(float(yaw_rad), 4)
                    },
                    "observations": 1,
                    "first_seen_time": round(time.time(), 2),
                    "label": self.booth_stall_labels.get(tag_id, f"Tag_{tag_id}")
                }
                self.discovered_nodes[tag_id] = node_data
                self.graph.add_node(tag_id, **node_data)
                print(f"  🌟 [DiscoveryMapper]: Discovered NEW {tag_type} Marker #{tag_id} ({node_data['label']}) at ({node_data['pose']['x']:.2f}m, {node_data['pose']['y']:.2f}m)")

                if tag_type == "FLOOR":
                    # Link with previously discovered Floor tag if within reasonable concourse distance (< 4.5m)
                    if self.last_observed_floor_tag is not None and self.last_observed_floor_tag != tag_id:
                        prev_id = self.last_observed_floor_tag
                        prev_pose = self.discovered_nodes[prev_id]["pose"]
                        dx = node_data["pose"]["x"] - prev_pose["x"]
                        dy = node_data["pose"]["y"] - prev_pose["y"]
                        edge_dist = math.hypot(dx, dy)
                        dtheta = node_data["pose"]["theta"] - prev_pose["theta"]

                        # Add bidirectional topological edge between sequential floor tags
                        self._add_edge(prev_id, tag_id, dx, dy, dtheta, edge_dist)
                        self._add_edge(tag_id, prev_id, -dx, -dy, -dtheta, edge_dist)
                    self.last_observed_floor_tag = tag_id

                elif tag_type == "BOOTH":
                    # Attach Booth tag as a leaf node to the closest Floor staging tag
                    closest_floor_tag = self._find_closest_floor_tag(node_data["pose"]["x"], node_data["pose"]["y"])
                    if closest_floor_tag is not None:
                        f_pose = self.discovered_nodes[closest_floor_tag]["pose"]
                        dx_world = node_data["pose"]["x"] - f_pose["x"]
                        dy_world = node_data["pose"]["y"] - f_pose["y"]
                        edge_dist = math.hypot(dx_world, dy_world)
                        f_theta = f_pose["theta"]
                        # Target docking heading to face the booth from the concourse is along the branch vector
                        target_docking_heading = math.atan2(dy_world, dx_world)
                        node_data["pose"]["theta"] = round(target_docking_heading, 4)
                        dtheta = (target_docking_heading - f_theta + math.pi) % (2.0 * math.pi) - math.pi

                        cos_f = math.cos(f_theta)
                        sin_f = math.sin(f_theta)
                        local_fwd = dx_world * cos_f + dy_world * sin_f
                        local_lat = dx_world * sin_f - dy_world * cos_f

                        node_data["parent_floor_tag"] = closest_floor_tag
                        node_data["relative_to_parent"] = {
                            "dx_world": round(dx_world, 3),
                            "dy_world": round(dy_world, 3),
                            "distance_m": round(edge_dist, 3),
                            "target_heading_rad": round(target_docking_heading, 4),
                            "heading_offset_deg": round(math.degrees(dtheta), 1),
                            "local_fwd": round(local_fwd, 3),
                            "local_lat": round(local_lat, 3)
                        }
                        node_data["standoff_distance_m"] = 0.95

                        self._add_edge(closest_floor_tag, tag_id, dx_world, dy_world, dtheta, edge_dist)
                        self._add_edge(tag_id, closest_floor_tag, -dx_world, -dy_world, -dtheta, edge_dist)
                        print(f"    🔗 Associated Booth Tag #{tag_id} -> Staging Floor Tag #{closest_floor_tag} (Dist: {edge_dist:.2f}m)")

            else:
                # Existing node: Exponential Moving Average (EMA) pose refinement
                node = self.discovered_nodes[tag_id]
                node["observations"] += 1
                alpha = 0.15  # Smoothing factor
                node["pose"]["x"] = round((1.0 - alpha) * node["pose"]["x"] + alpha * marker_world_x, 3)
                node["pose"]["y"] = round((1.0 - alpha) * node["pose"]["y"] + alpha * marker_world_y, 3)
                self.graph.nodes[tag_id].update(node)

                if tag_type == "FLOOR":
                    # Update floor sequence
                    if self.last_observed_floor_tag != tag_id and self.last_observed_floor_tag is not None:
                        prev_id = self.last_observed_floor_tag
                        if not self.graph.has_edge(prev_id, tag_id):
                            prev_pose = self.discovered_nodes[prev_id]["pose"]
                            dx = node["pose"]["x"] - prev_pose["x"]
                            dy = node["pose"]["y"] - prev_pose["y"]
                            edge_dist = math.hypot(dx, dy)
                            dtheta = node["pose"]["theta"] - prev_pose["theta"]
                            self._add_edge(prev_id, tag_id, dx, dy, dtheta, edge_dist)
                            self._add_edge(tag_id, prev_id, -dx, -dy, -dtheta, edge_dist)
                    self.last_observed_floor_tag = tag_id

        # Re-verify and maintain optimal Floor staging associations for all Booth leaves
        self._update_booth_associations()

    def _update_booth_associations(self):
        """
        Refinement 2: Ensures all Booth leaf nodes are optimally anchored to their
        closest Floor staging tag, storing metric transforms in both discovery world frame
        and parent Floor tag local frame (capturing exact -90°/right-angle branching).
        """
        floor_nodes = [nid for nid, d in self.discovered_nodes.items() if d["tag_type"] == "FLOOR"]
        if not floor_nodes:
            return

        for bid, bdata in list(self.discovered_nodes.items()):
            if bdata["tag_type"] == "BOOTH":
                closest_f = min(floor_nodes, key=lambda fid: math.hypot(
                    self.discovered_nodes[fid]["pose"]["x"] - bdata["pose"]["x"],
                    self.discovered_nodes[fid]["pose"]["y"] - bdata["pose"]["y"]
                ))
                prev_parent = bdata.get("parent_floor_tag")
                if prev_parent != closest_f and prev_parent is not None and self.graph.has_edge(prev_parent, bid):
                    self.graph.remove_edge(prev_parent, bid)
                    self.graph.remove_edge(bid, prev_parent)
                    self.discovered_edges = [
                        e for e in self.discovered_edges
                        if not ((e["from"] == prev_parent and e["to"] == bid) or (e["from"] == bid and e["to"] == prev_parent))
                    ]
                
                bdata["parent_floor_tag"] = closest_f
                f_pose = self.discovered_nodes[closest_f]["pose"]
                dx_world = bdata["pose"]["x"] - f_pose["x"]
                dy_world = bdata["pose"]["y"] - f_pose["y"]
                edge_dist = math.hypot(dx_world, dy_world)
                
                # Compute target docking heading to face booth from staging floor tag
                f_theta = f_pose["theta"]
                target_docking_heading = math.atan2(dy_world, dx_world)
                bdata["pose"]["theta"] = round(target_docking_heading, 4)
                dtheta = (target_docking_heading - f_theta + math.pi) % (2.0 * math.pi) - math.pi

                # Transform into Floor tag's local frame (+Y = corridor forward, +X = corridor right)
                cos_f = math.cos(f_theta)
                sin_f = math.sin(f_theta)
                local_fwd = dx_world * cos_f + dy_world * sin_f
                local_lat = dx_world * sin_f - dy_world * cos_f

                bdata["relative_to_parent"] = {
                    "dx_world": round(dx_world, 3),
                    "dy_world": round(dy_world, 3),
                    "distance_m": round(edge_dist, 3),
                    "target_heading_rad": round(target_docking_heading, 4),
                    "heading_offset_deg": round(math.degrees(dtheta), 1),
                    "local_fwd": round(local_fwd, 3),
                    "local_lat": round(local_lat, 3)
                }
                bdata["standoff_distance_m"] = 0.95
                if not self.graph.has_edge(closest_f, bid):
                    self._add_edge(closest_f, bid, dx_world, dy_world, dtheta, edge_dist)
                    self._add_edge(bid, closest_f, -dx_world, -dy_world, -dtheta, edge_dist)

    def _find_closest_floor_tag(self, bx: float, by: float) -> Optional[int]:
        """Finds closest discovered Floor tag to a Booth tag."""
        floor_nodes = [nid for nid, data in self.discovered_nodes.items() if data["tag_type"] == "FLOOR"]
        if not floor_nodes:
            return None
        return min(floor_nodes, key=lambda fid: math.hypot(
            self.discovered_nodes[fid]["pose"]["x"] - bx,
            self.discovered_nodes[fid]["pose"]["y"] - by
        ))

    def _add_edge(self, u: int, v: int, dx: float, dy: float, dtheta: float, dist: float):
        """Adds directed topological edge with relative metric transform."""
        edge_data = {
            "from": u,
            "to": v,
            "dx": round(dx, 3),
            "dy": round(dy, 3),
            "dtheta": round(dtheta, 4),
            "distance_m": round(dist, 3)
        }
        self.graph.add_edge(u, v, weight=dist, **edge_data)
        # Record in edges list if not already present
        if not any(e["from"] == u and e["to"] == v for e in self.discovered_edges):
            self.discovered_edges.append(edge_data)

    def save_discovered_map(
        self,
        yaml_filename: str = "discovered_aruco_graph.yaml",
        pgm_filename: str = "discovered_occupancy_map.pgm"
    ) -> Tuple[str, str]:
        """Serializes the topological graph to YAML and saves the 2D occupancy grid."""
        os.makedirs(CONFIG_DIR, exist_ok=True)
        yaml_path = os.path.join(CONFIG_DIR, yaml_filename)
        pgm_path = os.path.join(CONFIG_DIR, pgm_filename)

        # 1. Export Topological Graph YAML
        num_floors = sum(1 for n in self.discovered_nodes.values() if n["tag_type"] == "FLOOR")
        num_booths = sum(1 for n in self.discovered_nodes.values() if n["tag_type"] == "BOOTH")

        graph_export = {
            "metadata": {
                "system": "Project Cloud: Topological ArUco SLAM & Teach-and-Repeat Graph",
                "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "num_floor_tags": num_floors,
                "num_booth_tags": num_booths,
                "total_nodes": len(self.discovered_nodes),
                "total_edges": len(self.discovered_edges)
            },
            "nodes": {str(k): v for k, v in sorted(self.discovered_nodes.items())},
            "edges": self.discovered_edges
        }

        with open(yaml_path, "w") as f:
            yaml.dump(graph_export, f, sort_keys=False, default_flow_style=False)
        print(f"  💾 [DiscoveryMapper]: Saved topological ArUco graph -> {yaml_path}")

        # 2. Export 2D Occupancy Grid PGM & Companion YAML
        with open(pgm_path, "wb") as f:
            # PGM Header (P5 = binary grayscale)
            header = f"P5\n{self.grid_size} {self.grid_size}\n255\n".encode("ascii")
            f.write(header)
            # Invert vertically for standard image format
            flipped = np.flipud(self.occupancy_grid)
            f.write(flipped.tobytes())

        # Occupancy grid metadata YAML
        pgm_yaml_path = pgm_path.replace(".pgm", ".yaml")
        pgm_meta = {
            "image": pgm_filename,
            "resolution": self.resolution,
            "origin": [self.origin_x, self.origin_y, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196
        }
        with open(pgm_yaml_path, "w") as f:
            yaml.dump(pgm_meta, f, sort_keys=False)
        print(f"  💾 [DiscoveryMapper]: Saved background occupancy grid -> {pgm_path}")

        return yaml_path, pgm_path

    def get_hud_info(self) -> Dict[str, Any]:
        """Provides real-time discovery status metrics for HUD overlay."""
        num_floors = sum(1 for n in self.discovered_nodes.values() if n["tag_type"] == "FLOOR")
        num_booths = sum(1 for n in self.discovered_nodes.values() if n["tag_type"] == "BOOTH")
        return {
            "mode": "DISCOVERY MAPPER",
            "floor_tags_count": num_floors,
            "booth_tags_count": num_booths,
            "total_edges": len(self.discovered_edges),
            "observations": self.total_observations,
            "last_floor_tag": self.last_observed_floor_tag,
            "nodes": list(self.discovered_nodes.keys())
        }
