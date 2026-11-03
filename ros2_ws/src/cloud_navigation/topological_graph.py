"""
Project Cloud: Directed Topological Landmark Graph & Path Planner
Parses waypoints.yaml, calculates transit vectors, standoff poses (1.8m),
and provides deterministic routing across exhibition booths and makerspace cabinets.
Reference: PROJECT.md Section 7.2 & Appendix A
"""

import math
import os
import yaml
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass

@dataclass
class Edge:
    source_id: str
    target_id: str
    dx: float
    dy: float
    heading_deg: float
    nominal_speed: float
    clearance_radius_m: float

@dataclass
class Node:
    id: str
    label: str
    marker_id: int
    x: float
    y: float
    z: float
    yaw_deg: float
    action: str
    narration_key: str
    outbound_edges: List[Edge]
    cabinet_id: Optional[int] = None

class TopologicalGraph:
    """
    Topological spatial memory and route engine for Project Cloud.
    """
    def __init__(self, config_path: Optional[str] = None):
        if config_path is None:
            # Default to repo config path
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            config_path = os.path.join(base_dir, "config", "waypoints.yaml")

        self.config_path = config_path
        self.nodes: Dict[str, Node] = {}
        self.marker_to_node: Dict[int, Node] = {}
        self.cabinet_to_node: Dict[int, Node] = {}
        self.default_standoff_m = 1.80
        self._load_yaml()

    def _load_yaml(self):
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Topological graph config not found at: {self.config_path}")

        with open(self.config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        self.default_standoff_m = data.get("default_standoff_distance_meters", 1.80)

        # Parse exhibition nodes
        for item in data.get("nodes", []):
            edges = []
            for e in item.get("outbound_edges", []):
                vec = e.get("transit_vector", {})
                edges.append(Edge(
                    source_id=item["id"],
                    target_id=e["target"],
                    dx=vec.get("dx", 0.0),
                    dy=vec.get("dy", 0.0),
                    heading_deg=vec.get("heading_deg", 0.0),
                    nominal_speed=vec.get("nominal_speed", 0.35),
                    clearance_radius_m=e.get("clearance_radius_m", 1.2)
                ))

            pose_data = item.get("pose", {})
            node = Node(
                id=item["id"],
                label=item.get("label", ""),
                marker_id=item.get("marker_id", -1),
                x=pose_data.get("x", 0.0),
                y=pose_data.get("y", 0.0),
                z=pose_data.get("z", 0.0),
                yaw_deg=pose_data.get("yaw", 0.0),
                action=item.get("action", "NONE"),
                narration_key=item.get("narration_key", ""),
                outbound_edges=edges
            )
            self.nodes[node.id] = node
            if node.marker_id >= 0:
                self.marker_to_node[node.marker_id] = node

        # Parse makerspace cabinet nodes
        for item in data.get("makerspace_nodes", []):
            pose_data = item.get("pose", {})
            node = Node(
                id=item["id"],
                label=item.get("label", ""),
                marker_id=item.get("marker_id", -1),
                x=pose_data.get("x", 0.0),
                y=pose_data.get("y", 0.0),
                z=pose_data.get("z", 0.0),
                yaw_deg=pose_data.get("yaw", 0.0),
                action="ARGUS_INSPECT_CABINET",
                narration_key="makerspace_inventory",
                outbound_edges=[],
                cabinet_id=item.get("cabinet_id")
            )
            self.nodes[node.id] = node
            if node.marker_id >= 0:
                self.marker_to_node[node.marker_id] = node
            if node.cabinet_id is not None:
                self.cabinet_to_node[node.cabinet_id] = node

    def get_node(self, node_id: str) -> Optional[Node]:
        return self.nodes.get(node_id)

    def get_node_by_marker(self, marker_id: int) -> Optional[Node]:
        return self.marker_to_node.get(marker_id)

    def get_node_by_cabinet(self, cabinet_id: int) -> Optional[Node]:
        return self.cabinet_to_node.get(cabinet_id)

    def get_standoff_pose(self, node_id: str, standoff_m: Optional[float] = None) -> Tuple[float, float, float]:
        """
        Computes the target world pose (x, y, yaw_rad) for the robot to observe
        the node's marker from the canonical standoff distance (1.8m).
        """
        node = self.nodes.get(node_id)
        if not node:
            raise ValueError(f"Unknown node ID: {node_id}")

        standoff = standoff_m if standoff_m is not None else self.default_standoff_m
        yaw_rad = math.radians(node.yaw_deg)

        # Marker faces outward; robot should stand standoff distance along marker normal
        # Marker normal is pointing in direction of yaw_deg
        # Robot stands at marker - normal * standoff, looking at marker (heading = yaw_deg + 180)
        standoff_x = node.x - standoff * math.cos(yaw_rad)
        standoff_y = node.y - standoff * math.sin(yaw_rad)
        approach_yaw = (yaw_rad) % (2.0 * math.pi)

        return standoff_x, standoff_y, approach_yaw

    def get_tour_sequence(self) -> List[str]:
        """Returns canonical VIP exhibition tour node ID sequence."""
        return ["NODE_00", "NODE_10", "NODE_11", "NODE_12", "NODE_13", "NODE_30", "NODE_99"]

    def get_next_tour_node(self, current_node_id: str) -> Optional[str]:
        seq = self.get_tour_sequence()
        if current_node_id in seq:
            idx = seq.index(current_node_id)
            if idx + 1 < len(seq):
                return seq[idx + 1]
        return None
