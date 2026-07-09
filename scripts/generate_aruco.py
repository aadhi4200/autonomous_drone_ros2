#!/usr/bin/env python3
"""
generate_aruco.py
Generates ArUco ID=17 PNG texture for Gazebo model.
Run once before launching SITL (after the workspace has been built/sourced):
  python3 scripts/generate_aruco.py
"""
import os
import sys

_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(
    _repo_root, "src", "autonomous_drone_ros2", "drone_interfaces"))

try:
    from drone_interfaces.aruco_marker import generate_marker_png
except ImportError as e:
    raise SystemExit(
        "Could not import drone_interfaces.aruco_marker — build the workspace "
        "first (colcon build) or run from the repo root.") from e

import shutil

MARKER_ID = 17


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(script_dir)
    texture_dir = os.path.join(repo_root, "simulation", "gazebo", "models",
                                "aruco_17", "materials", "textures")
    output_path = os.path.join(texture_dir, "aruco_17.png")

    generate_marker_png(MARKER_ID, output_path)
    print(f"✅ ArUco ID={MARKER_ID} saved: {output_path}")

    px4_dir = os.path.expanduser(
        "~/PX4-Autopilot/Tools/simulation/gz/models/aruco_17/materials/textures")
    if os.path.exists(os.path.dirname(os.path.dirname(px4_dir))):
        os.makedirs(px4_dir, exist_ok=True)
        shutil.copy2(output_path, os.path.join(px4_dir, "aruco_17.png"))
        print(f"✅ Copied to PX4: {px4_dir}")


if __name__ == "__main__":
    main()
