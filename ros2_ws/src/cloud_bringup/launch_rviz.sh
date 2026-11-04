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

# Clean up any leftover background publishers
pkill -f "cloud_rviz_publisher" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true

# 1. Start Robot State Publisher with the Official Go2 URDF
echo "[INFO] Starting robot_state_publisher with URDF: $URDF_PATH"
ros2 run robot_state_publisher robot_state_publisher "$URDF_PATH" &
RSP_PID=$!

sleep 1

# 2. Start the Live Exhibition Telemetry, Quadruped IK Gait & Mission Publisher
echo "[INFO] Starting Cloud Go2 Quadruped Gait Controller..."
python3 "$SCRIPT_DIR/cloud_rviz_publisher.py" &
PUB_PID=$!

# Trap signals to cleanly kill background processes on exit
cleanup() {
  echo "\n[INFO] Shutting down Cloud simulation nodes (RSP: $RSP_PID, PUB: $PUB_PID)..."
  kill $RSP_PID $PUB_PID 2>/dev/null || true
  exit 0
}
trap cleanup SIGINT SIGTERM EXIT

sleep 1

# 3. Launch RViz2 with our pre-configured Exhibition Pavilion workspace
echo "[INFO] Launching RViz2 GUI with /cloud_exhibition.rviz..."
rviz2 -d "$SCRIPT_DIR/cloud_exhibition.rviz"
