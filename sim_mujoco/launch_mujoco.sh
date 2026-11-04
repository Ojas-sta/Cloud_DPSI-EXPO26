#!/bin/bash
# ==============================================================================
# Project Cloud: DPSI EXPO-2026 Master MuJoCo Physics Simulation
# Platform: Unitree Go2 Quadruped Robot
# Environment: Native MuJoCo 3.17 on macOS Apple Silicon
# ==============================================================================

set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/.." && pwd)

echo "============================================================================"
echo "  🚀 LAUNCHING PROJECT CLOUD IN MUJOCO PHYSICS (DPSI EXPO-2026)"
echo "  Platform: Official Unitree Go2 Quadruped (Google DeepMind Model)"
echo "  Locomotion: Dynamic Trot Gait & 6-Scene Exhibition Mission"
echo "============================================================================"

# Kill any leftover background simulators (clean slate)
pkill -f "cloud_rviz_publisher" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true

MJPYTHON="/Users/roopalisingh/miniconda3/bin/mjpython"

if [ ! -f "$MJPYTHON" ]; then
    echo "[WARN] mjpython not found at $MJPYTHON, attempting to find on PATH..."
    MJPYTHON=$(which mjpython)
fi

if [[ "$*" == *"--ros2"* ]] || [[ "$*" == *"--rviz"* ]]; then
    echo "[INFO] Forwarding to Unified MuJoCo Physics & RViz2 Digital Twin..."
    exec "$REPO_DIR/run_cloud_sim.sh" --gui
fi

exec "$MJPYTHON" "$SCRIPT_DIR/cloud_mujoco_sim.py" "$@"
