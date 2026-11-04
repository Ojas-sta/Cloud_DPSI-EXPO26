#!/bin/zsh
# ==============================================================================
# Project Cloud: One-Click RViz2 Launcher with Official Unitree Go2 URDF
# Platform: Unitree Go2 Quadruped Robot
# Environment: ROS 2 Jazzy via Pixi Robostack on macOS
# ==============================================================================

set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/../../.." && pwd)
URDF_PATH="$REPO_DIR/ros2_ws/src/go2_description/urdf/go2_rviz.urdf"

if [ ! -f "$URDF_PATH" ]; then
    echo "[ERROR] URDF file not found at: $URDF_PATH"
    exit 1
fi

echo "============================================================================"
echo "  🚀 LAUNCHING PROJECT CLOUD IN RVIZ2 (OFFICIAL UNITREE GO2 MODEL)"
echo "============================================================================"

# Source ROS 2 Jazzy environment
PIXI_ENV="/Users/roopalisingh/robostack/.pixi/envs/jazzy"
if [ -f "$PIXI_ENV/setup.zsh" ]; then
    source "$PIXI_ENV/setup.zsh"
fi

export AMENT_PREFIX_PATH="$REPO_DIR/ros2_ws/install/go2_description:$AMENT_PREFIX_PATH"

# Forward to the unified MuJoCo + ROS 2 launcher
exec "$REPO_DIR/run_cloud_sim.sh" "$@"
