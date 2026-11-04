#!/bin/zsh
# Launches the native MuJoCo interactive 3D viewer with the Unitree Go2 in the full DPSI Exhibition Summit world
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
WORLD_XML="$SCRIPT_DIR/sim_mujoco/unitree_go2/dpsi_summit_go2_world.xml"

echo "============================================================================"
echo "  🚀 LAUNCHING NATIVE MUJOCO 3D SIMULATION WINDOW"
echo "  • Environment: DPSI Summit 2026 (44 Stalls, Speedway, Soccer Pitch)"
echo "  • Robot: Unitree Go2 Edu Quadruped (12 DOF)"
echo "============================================================================"

/Users/roopalisingh/miniconda3/bin/python "$SCRIPT_DIR/sim_mujoco/launch_viewer.py" "$WORLD_XML"
