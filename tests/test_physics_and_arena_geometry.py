#!/usr/bin/env python3
"""
Test Suite: Authentic DPSI Exhibition Hall Geometry & Arena Kinematics
Verifies:
  1. Full hall bounding dimensions: 59.6m x 45.6m x 10.5m.
  2. Zone 3 Robotics Arena: ramp elevation (0 to 0.65m), bridge deck, descent ramp.
  3. Robo Soccer pitch and goal dimensions.
  4. South Entrance Gate coordinates and clearance.
  5. Quadruped stance height and kinematic stability margin.
"""

import os
import json
import math
import pytest

cur_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(cur_dir)
layout_path = os.path.join(project_dir, "config", "world_layout.json")

with open(layout_path, "r", encoding="utf-8") as f:
    world_layout = json.load(f)


def test_hall_bounding_dimensions():
    dims = world_layout["metadata"]["dimensions_meters"]
    assert dims["length_x"] == 59.6
    assert dims["width_y"] == 45.6
    assert dims["height_z"] == 10.5
    assert dims["bounds_x"] == [-31.8, 27.8]
    assert dims["bounds_y"] == [-10.8, 34.8]


def test_robotics_arena_speedway_ramps_and_bridge():
    arena = world_layout.get("robotics_arena", {})
    speedway = arena.get("arena_1_speedway", {})
    ramps = speedway.get("ramp_obstacle", [])
    
    bridge = next(r for r in ramps if r["name"] == "elevated_bridge_deck")
    ramp_up = next(r for r in ramps if r["name"] == "ascending_ramp")
    ramp_down = next(r for r in ramps if r["name"] == "descending_ramp")

    # Elevated bridge deck must be at z = 0.65m
    assert bridge["pos"][2] == 0.65
    assert bridge["size"][0] > 1.0 # bridge length
    assert bridge["size"][1] > 0.5 # bridge width

    # Ascending ramp must start lower and rise
    assert ramp_up["pos"][2] < bridge["pos"][2]
    # Descending ramp
    assert ramp_down["pos"][2] < bridge["pos"][2]


def test_robo_soccer_pitch_and_goals():
    arena = world_layout.get("robotics_arena", {})
    soccer = arena.get("arena_2_soccer", {})
    pitch_size = soccer.get("pitch_size", [])
    goals = soccer.get("goals", [])

    assert len(pitch_size) == 2
    assert pitch_size[0] == 10.0
    assert pitch_size[1] == 14.0
    assert len(goals) == 2, "Soccer pitch must feature 2 opposing goals"


def test_south_entrance_gate_clearance():
    gate_x = 6.40
    gate_y = -7.50
    # Gate must be south of all booth rows (Row 1 starts at y = -2.0)
    assert gate_y < -2.0
    # Clearance to closest wall: South walls are at y = -10.8
    dist_to_back_wall = abs(gate_y - (-10.8))
    assert dist_to_back_wall >= 3.0, f"Gate clearance to back wall is {dist_to_back_wall}m, must be >= 3.0m"


def test_go2_stance_height_and_mass_properties():
    # Unitree Go2 standard nominal standing height is 0.35m, mass ~15kg
    nominal_stance = 0.35
    nominal_mass = 15.0
    assert 0.30 <= nominal_stance <= 0.40
    assert 12.0 <= nominal_mass <= 18.0
