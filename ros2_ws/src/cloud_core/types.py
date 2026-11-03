"""
Project Cloud: Core Data Models & Message Definitions
Compatible with native ROS 2 and standalone Python runtime.
Reference: PROJECT.md Section 7.8
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
import time

@dataclass
class TimeStamp:
    sec: int = 0
    nanosec: int = 0

    @classmethod
    def now(cls):
        t = time.time()
        s = int(t)
        ns = int((t - s) * 1e9)
        return cls(sec=s, nanosec=ns)

@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

@dataclass
class Pose:
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)

@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class Twist:
    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)

@dataclass
class PoseStamped:
    stamp: TimeStamp = field(default_factory=TimeStamp.now)
    frame_id: str = "camera_color_optical_frame"
    pose: Pose = field(default_factory=Pose)

@dataclass
class SpillAlert:
    stamp: TimeStamp = field(default_factory=TimeStamp.now)
    hazard_type: str = "LIQUID_SPILL"
    confidence: float = 0.95
    estimated_area_sq_m: float = 0.42
    location: Point = field(default_factory=Point)
    severity: str = "CRITICAL"
    image_base64_jpeg: str = ""

@dataclass
class LEDCommand:
    MODE_OFF: int = 0
    MODE_SOLID: int = 1
    MODE_BREATHE: int = 2
    MODE_CHASE: int = 3
    MODE_STROBE: int = 4
    MODE_CASCADE: int = 5

    mode: int = 1
    red: int = 0
    green: int = 113
    blue: int = 227
    frequency_hz: float = 0.5
    duration_sec: float = 0.0

@dataclass
class InventoryItem:
    item_id: int = 0
    sku_code: str = ""
    canonical_name: str = ""
    aliases_json: str = "[]"
    stock_qty: int = 0
    unit: str = "pcs"
    cabinet_id: int = 1
    drawer_code: str = "Drawer A"
    shelf_tier: int = 1
    anchor_tag_id: int = 1
    reorder_threshold: int = 2
    custody_bench: Optional[int] = None

@dataclass
class RobotStateReport:
    stamp: TimeStamp = field(default_factory=TimeStamp.now)
    active_persona: str = "THE_DOCENT"
    fsm_state: str = "STATE_IDLE_STANDBY"
    active_landmark_id: str = "NODE_00"
    battery_percentage: float = 94.5
    battery_voltage: float = 24.8
    current_pose: Pose = field(default_factory=Pose)
    current_twist: Twist = field(default_factory=Twist)
    emergency_stop_active: bool = False
    push_button_ready: bool = True

@dataclass
class QueryInventoryRequest:
    spoken_query: str = ""

@dataclass
class QueryInventoryResponse:
    success: bool = False
    message: str = ""
    item: Optional[InventoryItem] = None
    match_confidence: float = 0.0
    speech_response: str = ""

@dataclass
class TriggerTourRequest:
    tour_profile: str = "VIP_EXECUTIVE"
    starting_node: str = "NODE_00"

@dataclass
class TriggerTourResponse:
    accepted: bool = False
    message: str = ""
    initial_state: str = "STATE_DOCENT_GREET"
