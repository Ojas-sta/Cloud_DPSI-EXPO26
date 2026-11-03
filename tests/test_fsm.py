"""
Project Cloud: Finite State Machine Test Suite
Validates state transitions, priority hazard preemption, and tactile progression.
"""

import sys
import os
import pytest

repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src_dir = os.path.join(repo_dir, "ros2_ws", "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from cloud_fsm import CloudSupervisoryFSM, SafetyMonitorNode, point_in_polygon
from cloud_core.types import SpillAlert, Point, TriggerTourRequest

def test_point_in_polygon():
    poly = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    assert point_in_polygon(5.0, 5.0, poly)
    assert not point_in_polygon(15.0, 5.0, poly)
    assert not point_in_polygon(-1.0, 5.0, poly)

def test_fsm_initial_state():
    fsm = CloudSupervisoryFSM()
    assert fsm.state == CloudSupervisoryFSM.STATE_IDLE_STANDBY
    fsm.destroy_node()

def test_fsm_tour_progression_lifecycle():
    fsm = CloudSupervisoryFSM()

    # Trigger VIP tour
    resp = fsm._handle_trigger_tour(TriggerTourRequest())
    assert resp.accepted
    assert fsm.state == CloudSupervisoryFSM.STATE_DOCENT_GREET

    # Push green button to proceed
    fsm._on_console_button(True)
    assert fsm.state == CloudSupervisoryFSM.STATE_DOCENT_CRUISE
    assert fsm.target_node_id == "NODE_10"

    # Optical follower locks onto marker and signals arrival
    fsm._on_navigation_arrived(True)
    assert fsm.state == CloudSupervisoryFSM.STATE_DOCENT_PIVOT

    # 180-degree Presenter Pivot completes
    fsm._on_pivot_completed(True)
    assert fsm.state == CloudSupervisoryFSM.STATE_DOCENT_NARRATE
    assert fsm.current_node_id == "NODE_10"

    # Push button to advance to Booth 2
    fsm._on_console_button(True)
    assert fsm.state == CloudSupervisoryFSM.STATE_DOCENT_CRUISE
    assert fsm.target_node_id == "NODE_11"

    fsm.destroy_node()

def test_fsm_priority_hazard_preemption():
    fsm = CloudSupervisoryFSM()

    # Start cruise
    fsm.transition_to(CloudSupervisoryFSM.STATE_DOCENT_CRUISE)

    # Trigger critical spill alert
    alert = SpillAlert(
        hazard_type="CHEMICAL_SPILL",
        confidence=0.95,
        estimated_area_sq_m=0.45,
        location=Point(x=0.1, y=1.5, z=0.0),
        severity="CRITICAL"
    )
    fsm._on_spill_alert(alert)

    # Verify preemption
    assert fsm.state == CloudSupervisoryFSM.STATE_HAZARD_ALERT
    assert fsm.previous_state == CloudSupervisoryFSM.STATE_DOCENT_CRUISE

    # Clear hazard
    fsm.clear_hazard()
    assert fsm.state == CloudSupervisoryFSM.STATE_DOCENT_CRUISE

    fsm.destroy_node()

def test_safety_geofence_monitoring():
    safety = SafetyMonitorNode()
    # Test safe location (exhibition center)
    is_safe, zone = safety.is_safe_position(5.0, 5.0)
    assert is_safe
    assert zone is None

    # Test keepout zone breach (Chemical Etching Zone: x in [12, 16], y in [10, 14])
    is_safe, zone = safety.is_safe_position(13.0, 11.0)
    assert not is_safe
    assert "CHEMICAL" in zone

    safety.destroy_node()
