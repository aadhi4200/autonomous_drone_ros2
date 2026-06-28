#!/bin/bash
# build.sh — build the drone_ws workspace
set -e
cd "$(dirname "$0")/../.."   # go to drone_ws root
echo "=== Building drone_ws ==="
source /opt/ros/humble/setup.bash
colcon build --symlink-install
echo "✅ Build complete. Run: source install/setup.bash"
