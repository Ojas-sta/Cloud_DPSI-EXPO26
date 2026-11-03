#!/bin/zsh
# Project Cloud: One-Click RViz2 Launcher with Official Unitree Go2 URDF
# Environment: ROS 2 Jazzy via Pixi Robostack

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/../../../.." && pwd)

echo "============================================================================"
echo "  🚀 LAUNCHING PROJECT CLOUD IN RVIZ2 (OFFICIAL UNITREE GO2 MODEL)"
echo "============================================================================"

# Source ROS 2 Jazzy environment
source /Users/roopalisingh/robostack/.pixi/envs/jazzy/setup.zsh

export AMENT_PREFIX_PATH="$REPO_DIR/ros2_ws/install/go2_description:$AMENT_PREFIX_PATH"

# 1. Start the Live Exhibition Telemetry & 12-DOF Joint Publisher
echo "[INFO] Starting Cloud RViz simulation publisher..."
python3 "$SCRIPT_DIR/cloud_rviz_publisher.py" &
PUB_PID=$!

# Trap signals to cleanly kill publisher on exit
cleanup() {
  echo "\n[INFO] Shutting down Cloud RViz publisher (PID $PUB_PID)..."
  kill $PUB_PID 2>/dev/null
  exit 0
}
trap cleanup SIGINT SIGTERM EXIT

sleep 1

# 2. Launch RViz2 with our pre-configured Exhibition Pavilion workspace
echo "[INFO] Launching RViz2 GUI with /cloud_exhibition.rviz..."
rviz2 -d "$SCRIPT_DIR/cloud_exhibition.rviz"
