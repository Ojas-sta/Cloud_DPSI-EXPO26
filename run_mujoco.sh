#!/bin/bash
# Convenient root launcher for Project Cloud MuJoCo Simulation
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
exec "$SCRIPT_DIR/sim_mujoco/launch_mujoco.sh" "$@"
