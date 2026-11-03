#!/usr/bin/env bash
# ==============================================================================
# Project Cloud: DPSI EXPO-2026 Autonomous Tour Gazebo Simulation
# Platform: Unitree Go2 Edu Quadruped
# Environment: Gazebo Harmonic / ROS 2 Jazzy (Robostack Pixi on macOS)
# ==============================================================================

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
REPO_DIR="$( cd "$DIR/../../.." >/dev/null 2>&1 && pwd )"
PIXI_ENV="/Users/roopalisingh/robostack/.pixi/envs/jazzy"

echo "=========================================================="
echo " Starting Project Cloud Simulation in Gazebo Harmonic"
echo " Robot: Official Unitree Go2 Quadruped"
echo " World: DPSI EXPO Pavilion (Exhibition Booths, Hazard, Stage)"
echo "=========================================================="

if [ -f "$PIXI_ENV/setup.zsh" ]; then
    source "$PIXI_ENV/setup.zsh"
elif [ -f "$PIXI_ENV/setup.bash" ]; then
    source "$PIXI_ENV/setup.bash"
fi

export SDF_PATH="$REPO_DIR/ros2_ws/src/cloud_bringup/sim_gazebo/models:$SDF_PATH"
export GZ_SIM_RESOURCE_PATH="$REPO_DIR/ros2_ws/src/cloud_bringup/sim_gazebo/models:$GZ_SIM_RESOURCE_PATH"
WORLD_PATH="$REPO_DIR/ros2_ws/src/cloud_bringup/sim_gazebo/cloud_gazebo.world"

echo "Loaded Models from: $SDF_PATH"
echo "Launching Gazebo Sim: $WORLD_PATH"

gz sim -r "$WORLD_PATH"
