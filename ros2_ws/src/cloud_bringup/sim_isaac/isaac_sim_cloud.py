"""
Project Cloud: NVIDIA Isaac Sim (Omniverse) Digital Twin Runner
Target: NVIDIA Isaac Sim 2023.1+ on Ubuntu 22.04 LTS with RTX GPU.
"""

import sys
import os

def run_isaac_sim():
    print("=" * 72)
    print("  PROJECT CLOUD // NVIDIA ISAAC SIM OMNIVERSE DIGITAL TWIN")
    print("=" * 72)

    try:
        from omni.isaac.kit import SimulationApp
    except ImportError:
        print("[NOTICE] NVIDIA Isaac Sim / Omniverse is not installed on this host.")
        print("         Isaac Sim requires Ubuntu 22.04 LTS and an NVIDIA RTX GPU.")
        print("         To run on an RTX workstation:")
        print("           ~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh isaac_sim_cloud.py")
        return

    simulation_app = SimulationApp({"headless": False})
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.robots import Robot
    import omni.graph.core as og

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Load Unitree Go2 USD Asset from Isaac Sim Assets Library
    go2_usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1/Isaac/Robots/Unitree/go2.usd"
    add_reference_to_stage(usd_path=go2_usd_path, prim_path="/World/Go2")
    robot = world.scene.add(Robot(prim_path="/World/Go2", name="unitree_go2"))

    world.reset()
    print("[ISAAC SIM READY] Unitree Go2 loaded into physics stage.")

    # Main Simulation Loop
    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()

if __name__ == "__main__":
    run_isaac_sim()
