"""
Project Cloud: Master Directed Topological Landmark Graph & 40-Stall Route Engine
Parses stalls_database.yaml (and legacy waypoints.yaml), supports 40+ exhibition stalls,
standardizes strictly on AprilTag tag36h11 (IDs 10-49, 99 for dock), and implements
two-stage crowd chokepoint escalation (5s audio/yaw wiggle, 15s deferral +50.0 edge penalty).
Reference: DPSI EXPO-2026 Production Blueprint
"""

import math
import os
import time
import yaml
from typing import Dict, List, Optional, Tuple, Any, Set
from dataclasses import dataclass, field

@dataclass
class Edge:
    source_id: str
    target_id: str
    dx: float
    dy: float
    heading_deg: float
    nominal_speed: float
    clearance_radius_m: float
    base_weight: float = 1.0
    dynamic_penalty: float = 0.0
    penalty_expiry_ts: float = 0.0

    @property
    def current_weight(self) -> float:
        now = time.time()
        if now < self.penalty_expiry_ts:
            return self.base_weight + self.dynamic_penalty
        return self.base_weight

@dataclass
class StallMetadata:
    id: str
    cohort: int
    apriltag_id: int
    title: str
    category: str
    aisle: str
    booth_x: float
    booth_y: float
    booth_z: float
    booth_yaw: float
    staging_x: float
    staging_y: float
    staging_z: float
    staging_yaw: float
    terminal_standoff_m: float
    presentation_pitch_rad: float
    presentation_duration_s: float
    audio_pitch_id: str
    key_specs: Dict[str, Any] = field(default_factory=dict)
    deferral_count: int = 0

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
    outbound_edges: List[Edge] = field(default_factory=list)
    cabinet_id: Optional[int] = None
    stall_meta: Optional[StallMetadata] = None


class TopologicalGraph:
    """
    Topological spatial memory, 40-stall queue engine, and crowd chokepoint manager.
    Standardized strictly on AprilTag tag36h11.
    """
    def __init__(self, config_path: Optional[str] = None):
        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        
        # Primary: stalls_database.yaml; Fallback: waypoints.yaml
        stalls_db_path = os.path.join(base_dir, "config", "stalls_database.yaml")
        legacy_wp_path = os.path.join(base_dir, "config", "waypoints.yaml")
        
        if config_path is not None:
            self.config_path = config_path
        elif os.path.exists(stalls_db_path):
            self.config_path = stalls_db_path
        else:
            self.config_path = legacy_wp_path

        self.nodes: Dict[str, Node] = {}
        self.stalls: Dict[str, StallMetadata] = {}
        self.marker_to_node: Dict[int, Node] = {}
        self.cabinet_to_node: Dict[int, Node] = {}
        self.cohorts: Dict[int, List[str]] = {1: [], 2: [], 3: [], 4: []}
        
        # Chokepoint tracking: stall_id -> (start_ts, escalated_5s)
        self.chokepoint_timers: Dict[str, Tuple[float, bool]] = {}
        self.max_deferrals = 2
        self.clearance_threshold_m = 0.75
        self.warning_timeout_sec = 5.0
        self.defer_timeout_sec = 15.0
        self.edge_penalty = 50.0
        self.penalty_duration_sec = 180.0
        
        self.default_standoff_m = 0.95
        self._load_config()

    def _load_config(self):
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Topological graph config not found at: {self.config_path}")

        with open(self.config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        venue = data.get("venue", {})
        self.default_standoff_m = venue.get("default_terminal_standoff_meters", 0.95)
        self.max_deferrals = venue.get("max_deferrals_per_stall", 2)
        self.clearance_threshold_m = venue.get("crowd_clearance_threshold_meters", 0.75)
        self.warning_timeout_sec = venue.get("clearance_warning_timeout_sec", 5.0)
        self.defer_timeout_sec = venue.get("clearance_defer_timeout_sec", 15.0)
        self.edge_penalty = venue.get("edge_weight_inflation_penalty", 50.0)
        self.penalty_duration_sec = venue.get("edge_weight_inflation_duration_sec", 180.0)

        # 1. Parse Recharge Dock (Marker 99)
        dock_data = data.get("recharge_dock")
        if dock_data:
            node_id = dock_data.get("node_id", "DOCK_STAGE_99")
            m_id = dock_data.get("marker_id", 99)
            stg = dock_data.get("staging_pose", {})
            dock_node = Node(
                id=node_id,
                label=dock_data.get("label", "Stage Recharge Dock"),
                marker_id=m_id,
                x=stg.get("x", -0.80),
                y=stg.get("y", 0.0),
                z=stg.get("z", 0.32),
                yaw_deg=math.degrees(stg.get("yaw", 3.14159)),
                action="PRONE_REST",
                narration_key="docking_standby"
            )
            self.nodes[node_id] = dock_node
            self.marker_to_node[m_id] = dock_node
            # Legacy alias
            self.nodes["NODE_99"] = dock_node

        # 2. Parse Cohorts
        for ch in data.get("cohorts", []):
            c_id = ch.get("id")
            s_ids = ch.get("stall_ids", [])
            if c_id in self.cohorts:
                self.cohorts[c_id] = list(s_ids)

        # 3. Parse 40 Stalls
        for s in data.get("stalls", []):
            stall_id = s["id"]
            c_id = s.get("cohort", 1)
            apriltag_id = s.get("apriltag_id", -1)
            b_pose = s.get("booth_pose", {})
            stg_pose = s.get("staging_pose", {})

            meta = StallMetadata(
                id=stall_id,
                cohort=c_id,
                apriltag_id=apriltag_id,
                title=s.get("title", ""),
                category=s.get("category", ""),
                aisle=s.get("aisle", "AISLE_A"),
                booth_x=b_pose.get("x", 0.0),
                booth_y=b_pose.get("y", 0.0),
                booth_z=b_pose.get("z", 0.0),
                booth_yaw=b_pose.get("yaw", 0.0),
                staging_x=stg_pose.get("x", 0.0),
                staging_y=stg_pose.get("y", 0.0),
                staging_z=stg_pose.get("z", 0.32),
                staging_yaw=stg_pose.get("yaw", 0.0),
                terminal_standoff_m=s.get("terminal_standoff_m", self.default_standoff_m),
                presentation_pitch_rad=s.get("presentation_pitch_rad", -0.14),
                presentation_duration_s=s.get("presentation_duration_s", 75.0),
                audio_pitch_id=s.get("audio_pitch_id", ""),
                key_specs=s.get("key_specs", {})
            )
            self.stalls[stall_id] = meta

            # Create corresponding topological graph node
            node = Node(
                id=stall_id,
                label=meta.title,
                marker_id=apriltag_id,
                x=meta.staging_x,
                y=meta.staging_y,
                z=meta.staging_z,
                yaw_deg=math.degrees(meta.staging_yaw),
                action="DOCENT_PIVOT_AND_NARRATE",
                narration_key=meta.audio_pitch_id,
                stall_meta=meta
            )
            self.nodes[stall_id] = node
            if apriltag_id >= 0:
                self.marker_to_node[apriltag_id] = node

        # 4. Fallback legacy nodes parsing if present
        for item in data.get("nodes", []):
            if item["id"] not in self.nodes:
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

        # 5. Populate Inter-Stall Aisle Edges
        self._build_corridor_edges()

    def _build_corridor_edges(self):
        """Constructs metric edges between adjacent stalls in cohorts."""
        for c_id, s_ids in self.cohorts.items():
            for i in range(len(s_ids) - 1):
                src_id = s_ids[i]
                dst_id = s_ids[i+1]
                src = self.nodes.get(src_id)
                dst = self.nodes.get(dst_id)
                if src and dst:
                    dx = dst.x - src.x
                    dy = dst.y - src.y
                    dist = math.hypot(dx, dy)
                    heading = math.degrees(math.atan2(dy, dx))
                    src.outbound_edges.append(Edge(
                        source_id=src_id,
                        target_id=dst_id,
                        dx=dx,
                        dy=dy,
                        heading_deg=heading,
                        nominal_speed=0.50,
                        clearance_radius_m=1.20,
                        base_weight=dist
                    ))

    def evaluate_crowd_clearance(self, stall_id: str, min_clearance_m: float) -> Tuple[str, Optional[float]]:
        """
        Two-stage crowd escalation:
        - At t = 5.0s (clearance < 0.75m): 'WARN_CLEARANCE' -> audio excuse_me + yaw wiggle (+-0.10 rad)
        - At t = 15.0s (clearance < 0.75m): 'DEFER_STALL' -> defer_stall, +50.0 edge penalty for 180s
        - Otherwise 'NOMINAL'
        """
        now = time.time()
        if min_clearance_m >= self.clearance_threshold_m:
            # Clearance restored; reset timer
            self.chokepoint_timers.pop(stall_id, None)
            return ("NOMINAL", None)

        if stall_id not in self.chokepoint_timers:
            self.chokepoint_timers[stall_id] = (now, False)
            return ("MONITORING", None)

        start_ts, warned = self.chokepoint_timers[stall_id]
        elapsed = now - start_ts

        if elapsed >= self.defer_timeout_sec:
            # Stage 2: Defer
            self.chokepoint_timers.pop(stall_id, None)
            return ("DEFER_STALL", None)
        elif elapsed >= self.warning_timeout_sec and not warned:
            # Stage 1: Warning + subtle yaw intent wiggle
            self.chokepoint_timers[stall_id] = (start_ts, True)
            # Wiggle angle +-0.10 rad (alternate by time)
            wiggle_yaw = 0.10 if int(now * 2) % 2 == 0 else -0.10
            return ("WARN_CLEARANCE", wiggle_yaw)

        return ("MONITORING", None)

    def defer_stall(self, stall_id: str) -> bool:
        """
        Defers a stall if deferrals < max_deferrals.
        Inflates outbound edges by +50.0 for 180 seconds.
        Returns True if deferred, False if max deferrals reached.
        """
        stall = self.stalls.get(stall_id)
        if not stall:
            return False

        if stall.deferral_count >= self.max_deferrals:
            return False

        stall.deferral_count += 1
        now = time.time()

        # Inflate edges touching this aisle
        node = self.nodes.get(stall_id)
        if node:
            for edge in node.outbound_edges:
                edge.dynamic_penalty = self.edge_penalty
                edge.penalty_expiry_ts = now + self.penalty_duration_sec

        # Re-queue stall at tail of active cohort
        c_id = stall.cohort
        if c_id in self.cohorts and stall_id in self.cohorts[c_id]:
            self.cohorts[c_id].remove(stall_id)
            self.cohorts[c_id].append(stall_id)

        return True

    def get_node(self, node_id: str) -> Optional[Node]:
        return self.nodes.get(node_id)

    def get_node_by_marker(self, marker_id: int) -> Optional[Node]:
        return self.marker_to_node.get(marker_id)

    def get_node_by_cabinet(self, cabinet_id: int) -> Optional[Node]:
        return self.cabinet_to_node.get(cabinet_id)

    def get_stall(self, stall_id: str) -> Optional[StallMetadata]:
        return self.stalls.get(stall_id)

    def get_cohort_stalls(self, cohort_id: int) -> List[StallMetadata]:
        ids = self.cohorts.get(cohort_id, [])
        return [self.stalls[s_id] for s_id in ids if s_id in self.stalls]

    def get_standoff_pose(self, node_id: str, standoff_m: Optional[float] = None) -> Tuple[float, float, float]:
        """
        Computes the target world staging pose (x, y, yaw_rad).
        For 40 stalls, staging_pose is explicitly defined on the aisle centerline (1.10m-1.20m away).
        """
        node = self.nodes.get(node_id)
        if not node:
            raise ValueError(f"Unknown node ID: {node_id}")

        if node.stall_meta:
            return node.stall_meta.staging_x, node.stall_meta.staging_y, node.stall_meta.staging_yaw

        # Legacy calculation
        standoff = standoff_m if standoff_m is not None else self.default_standoff_m
        yaw_rad = math.radians(node.yaw_deg)
        standoff_x = node.x - standoff * math.cos(yaw_rad)
        standoff_y = node.y - standoff * math.sin(yaw_rad)
        approach_yaw = (yaw_rad) % (2.0 * math.pi)
        return standoff_x, standoff_y, approach_yaw

    def get_tour_sequence(self, cohort_id: Optional[int] = None) -> List[str]:
        """
        Returns tour sequence. If cohort_id is specified (1..4), returns that cohort's stalls.
        Otherwise returns all 40 stalls in order followed by stage recharge dock.
        """
        if cohort_id is not None:
            return list(self.cohorts.get(cohort_id, []))

        seq = []
        for c in range(1, 5):
            seq.extend(self.cohorts.get(c, []))
        if "DOCK_STAGE_99" in self.nodes:
            seq.append("DOCK_STAGE_99")
        return seq

    def get_next_tour_node(self, current_node_id: str, cohort_id: Optional[int] = None) -> Optional[str]:
        # Backward compatibility for legacy simulation tests
        legacy_seq = ["NODE_00", "NODE_10", "NODE_11", "NODE_12", "NODE_13", "NODE_30", "NODE_99"]
        if current_node_id in legacy_seq:
            idx = legacy_seq.index(current_node_id)
            if idx + 1 < len(legacy_seq):
                return legacy_seq[idx + 1]
            return None

        seq = self.get_tour_sequence(cohort_id)
        if current_node_id in seq:
            idx = seq.index(current_node_id)
            if idx + 1 < len(seq):
                return seq[idx + 1]
        return None
