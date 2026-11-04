#!/bin/zsh
# ==============================================================================
# Project Cloud: DPSI EXPO-2026 Unified Digital Twin & Telemetry Cockpit
# Platform: Unitree Go2 Edu Quadruped
# Physics Engine: MuJoCo 3.x
# Visualization & Telemetry Hub: RViz2 (ROS 2 Jazzy via Pixi Robostack)
# ==============================================================================

set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
URDF_PATH="$SCRIPT_DIR/ros2_ws/src/go2_description/urdf/go2_rviz.urdf"
RVIZ_CONFIG="$SCRIPT_DIR/ros2_ws/src/cloud_bringup/cloud_master.rviz"
TWIN_SCRIPT="$SCRIPT_DIR/sim_mujoco/cloud_mujoco_ros2_twin.py"

echo "============================================================================"
echo "  🚀 PROJECT CLOUD: UNITREE GO2 DIGITAL TWIN (MUJOCO + RVIZ2)"
echo "============================================================================"
echo "  • Physics Engine: MuJoCo 3.x (Ground Contacts, Trot Gait, 12-DOF IK)"
echo "  • LiDAR: Unitree 4D LiDAR L1 (16 Rings x 72 Azimuth = 1,152 Beams)"
echo "  • Autonomy: 80% Preplanned Global Route + 20% Reactive Dynamic Avoidance"
echo "  • Telemetry: Authentic Go2 HighState, BMS, IMU, Joint States, 3D PointCloud"
echo "  • Visualization Hub: RViz2 (Textured DAE Meshes, Rainbow Elevation PC2)"
echo "============================================================================"

# 1. Source ROS 2 Jazzy environment
PIXI_ENV="/Users/roopalisingh/robostack/.pixi/envs/jazzy"
if [ -f "$PIXI_ENV/setup.zsh" ]; then
    source "$PIXI_ENV/setup.zsh"
fi

# Clean CycloneDDS transport for macOS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export AMENT_PREFIX_PATH="$SCRIPT_DIR/ros2_ws/install/go2_description:$AMENT_PREFIX_PATH"

# 2. Cleanup any previous runs
pkill -f "cloud_mujoco_ros2_twin" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true

# Check arguments
TWIN_ARGS=""
if [[ "$*" == *"--gui"* ]] || [[ "$*" == *"--twin-gui"* ]]; then
    TWIN_ARGS="--gui"
    echo "[INFO] Running MuJoCo in interactive passive GUI mode alongside RViz2."
else
    echo "[INFO] Running MuJoCo physics engine in headless background mode."
fi

# 3. Start robot_state_publisher with the official Go2 URDF
echo "[INFO] Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher "$URDF_PATH" &
RSP_PID=$!

sleep 1

# 4. Start Unified MuJoCo Physics & ROS 2 Twin Node
echo "[INFO] Starting Unified MuJoCo Physics & 4D LiDAR Twin Node..."
# On macOS, MuJoCo native GUI viewer requires mjpython for Cocoa event loop
MJPYTHON_BIN="$PIXI_ENV/bin/mjpython"
if [ ! -f "$MJPYTHON_BIN" ]; then
    MJPYTHON_BIN="python3"
fi

"$MJPYTHON_BIN" "$TWIN_SCRIPT" $TWIN_ARGS &
TWIN_PID=$!

# Trap signals for clean shutdown
cleanup() {
    echo "\n[INFO] Shutting down Project Cloud simulation nodes (RSP: $RSP_PID, TWIN: $TWIN_PID)..."
    kill $TWIN_PID $RSP_PID 2>/dev/null || true
    pkill -f "cloud_mujoco_ros2_twin" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

sleep 2

# 5. Launch RViz2 Master Visualization Cockpit
echo "[INFO] Launching Master RViz2 Cockpit: $RVIZ_CONFIG"
rviz2 -d "$RVIZ_CONFIG"
