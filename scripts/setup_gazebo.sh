#!/bin/bash
# setup_gazebo.sh — copies simulation files into PX4-Autopilot directory
set -e
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PX4_DIR="$HOME/PX4-Autopilot"

echo "=== Drone Gazebo Setup ==="
[ ! -d "$PX4_DIR" ] && echo "ERROR: $PX4_DIR not found" && exit 1

cp "$REPO_DIR/simulation/gazebo/worlds/aruco_landing.sdf" \
   "$PX4_DIR/Tools/simulation/gz/worlds/aruco_landing.sdf"
echo "✅ World: aruco_landing.sdf"

mkdir -p "$PX4_DIR/Tools/simulation/gz/models/aruco_17/materials/textures"
cp "$REPO_DIR/simulation/gazebo/models/aruco_17/model.sdf"    \
   "$PX4_DIR/Tools/simulation/gz/models/aruco_17/model.sdf"
cp "$REPO_DIR/simulation/gazebo/models/aruco_17/model.config" \
   "$PX4_DIR/Tools/simulation/gz/models/aruco_17/model.config"
echo "✅ Model: aruco_17"

python3 "$REPO_DIR/scripts/generate_aruco.py"

echo ""
echo "=== Setup complete! ==="
echo "Launch: PX4_GZ_WORLD=aruco_landing make px4_sitl gz_x500_lidar_cam_down"
