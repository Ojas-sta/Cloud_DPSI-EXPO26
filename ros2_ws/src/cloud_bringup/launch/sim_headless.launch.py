"""
Project Cloud: Headless SIL Simulation Launch Runner
Initializes all core nodes (Robot SIL, Tracker, Spill Detector, Follower,
Pivot, Inventory, RAG, Safety Monitor, Webhooks, FSM) in a unified test harness.
"""

import sys
import os
import time

# Ensure ros2_ws/src is in sys.path
src_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from cloud_hardware_bridge import MockGo2Robot
from cloud_perception import ArucoTrackerNode, FloorSpillDetectorNode
from cloud_navigation import OpticalFollowerNode, PresenterPivotNode, TopologicalGraph
from cloud_inventory import InventoryServiceNode
from cloud_cognition import SpeechPipelineNode, PersonaManager, GroundedRAGEngine
from cloud_fsm import CloudSupervisoryFSM, SafetyMonitorNode
from cloud_webhooks import DiscordDispatcherNode

class SimulationHarness:
    def __init__(self):
        print("=" * 70)
        print("  PROJECT CLOUD // DPSI EXPO '26 // FULL SIL SIMULATION HARNESS")
        print("=" * 70)
        self.robot = MockGo2Robot()
        self.tracker = ArucoTrackerNode()
        self.spill_detector = FloorSpillDetectorNode()
        self.follower = OpticalFollowerNode()
        self.pivot = PresenterPivotNode()
        self.inventory = InventoryServiceNode()
        self.safety = SafetyMonitorNode()
        self.webhooks = DiscordDispatcherNode()
        self.fsm = CloudSupervisoryFSM()
        print("All 9 autonomous subsystem nodes successfully instantiated!")

    def shutdown(self):
        print("\nShutting down simulation nodes...")
        self.robot.destroy_node()
        self.pivot.destroy_node()
        self.fsm.destroy_node()
        print("Clean shutdown complete.")

if __name__ == "__main__":
    harness = SimulationHarness()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        harness.shutdown()
