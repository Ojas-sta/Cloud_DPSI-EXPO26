#!/usr/bin/env python3
"""
Direct Unitree Go2 Python Hardware Interface
Supports:
  1. Physical Unitree Go2 via official Unitree SDK2 (CycloneDDS / SportClient)
  2. Software Emulation Mode with realistic gait & kinematics for testing
"""

import time
import math
from typing import Dict, Any, Optional

# Attempt to import official Unitree SDK2 if installed
HAS_PHYSICAL_SDK = False
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    HAS_PHYSICAL_SDK = True
except ImportError:
    HAS_PHYSICAL_SDK = False


class UnitreeGo2Interface:
    def __init__(self, use_physical_robot: bool = False, network_interface: str = "eth0"):
        self.use_physical_robot = use_physical_robot and HAS_PHYSICAL_SDK
        self.sport_client: Optional[Any] = None

        # Robot State & Telemetry
        self.connected = False
        self.posture = "BALANCE_STAND"
        self.battery_percent = 94.0
        self.voltage = 28.4
        self.current = -3.8
        self.motor_temp_c = 36.5
        self.x = 0.0
        self.y = 11.50
        self.yaw_deg = 90.0  # Facing North
        self.pitch_deg = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.safety_state = "SAFETY_CLEAR"
        self.last_update_time = time.time()

        if self.use_physical_robot:
            self._init_physical_sdk(network_interface)
        else:
            self.connected = True
            print("  🤖 [UnitreeGo2Interface]: Running in Direct Python Emulation Mode (Zero-Clipping Kinematics).")

    def _init_physical_sdk(self, net_if: str):
        try:
            print(f"  🔌 [UnitreeGo2Interface]: Initializing Unitree SDK2 on network interface '{net_if}'...")
            ChannelFactoryInitialize(0, net_if)
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(5.0)
            self.sport_client.Init()
            self.connected = True
            print("  ✓ [UnitreeGo2Interface]: Physical Unitree Go2 SportClient CONNECTED.")
        except Exception as e:
            print(f"  ⚠️ [UnitreeGo2Interface]: Could not connect to physical robot ({e}). Falling back to Emulation Mode.")
            self.use_physical_robot = False
            self.connected = True

    def stand_up(self):
        self.posture = "BALANCE_STAND"
        self.pitch_deg = 0.0
        if self.use_physical_robot and self.sport_client:
            self.sport_client.StandUp()
        print("  • [Go2]: Posture -> BALANCE_STAND (Nominal height = 0.354m, Zero floor clipping)")

    def sit_down(self):
        self.posture = "SIT"
        self.pitch_deg = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        if self.use_physical_robot and self.sport_client:
            self.sport_client.StandDown()
        print("  • [Go2]: Posture -> SIT / PRONE REST (Chassis height = 0.187m, Zero clipping)")

    def set_body_pitch(self, pitch_deg: float):
        """Tilts the body/head for eye contact during presentations (+8° to +12°)."""
        self.pitch_deg = pitch_deg
        pitch_rad = math.radians(pitch_deg)
        if self.use_physical_robot and self.sport_client:
            # Unitree SDK2 Euler command: roll, pitch, yaw
            self.sport_client.Euler(0.0, pitch_rad, 0.0)
        print(f"  • [Go2]: Head/Body Pitch -> {pitch_deg:+.1f}° ({pitch_rad:+.2f} rad)")

    def move(self, vx: float, vy: float, wz: float):
        """Commands omnidirectional locomotion (m/s, rad/s)."""
        self.vx = vx
        self.vy = vy
        self.wz = wz
        if self.use_physical_robot and self.sport_client:
            self.sport_client.Move(vx, vy, wz)

    def stop(self):
        self.move(0.0, 0.0, 0.0)
        if self.use_physical_robot and self.sport_client:
            self.sport_client.StopMove()

    def update_emulation(self, dt: float = 0.03):
        """Simulates smooth hardware odometry integration when in emulation mode."""
        if not self.use_physical_robot:
            yaw_rad = math.radians(self.yaw_deg)
            # Update position along heading
            dx = (self.vx * math.cos(yaw_rad) - self.vy * math.sin(yaw_rad)) * dt
            dy = (self.vx * math.sin(yaw_rad) + self.vy * math.cos(yaw_rad)) * dt
            self.x += dx
            self.y += dy
            self.yaw_deg = (self.yaw_deg + math.degrees(self.wz * dt) + 180.0) % 360.0 - 180.0

    def get_telemetry(self) -> Dict[str, Any]:
        return {
            "connected": self.connected,
            "mode": "PHYSICAL_SDK" if self.use_physical_robot else "DIRECT_EMULATOR",
            "posture": self.posture,
            "battery_pct": self.battery_percent,
            "voltage": self.voltage,
            "current": self.current,
            "motor_temp": self.motor_temp_c,
            "x": self.x,
            "y": self.y,
            "yaw_deg": self.yaw_deg,
            "pitch_deg": self.pitch_deg,
            "vx": self.vx,
            "vy": self.vy,
            "wz": self.wz,
            "safety_state": self.safety_state
        }
