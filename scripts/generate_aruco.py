#!/usr/bin/env python3
"""
generate_aruco.py
Generates ArUco ID=17 PNG texture for Gazebo model.
Run once before launching SITL:
  python3 scripts/generate_aruco.py
"""
import cv2, cv2.aruco as aruco, os, shutil

MARKER_ID   = 17
MARKER_SIZE = 1000
BORDER_PX   = 100

def main():
    script_dir  = os.path.dirname(os.path.abspath(__file__))
    repo_root   = os.path.dirname(script_dir)
    texture_dir = os.path.join(repo_root, "simulation", "gazebo", "models",
                               "aruco_17", "materials", "textures")
    output_path = os.path.join(texture_dir, "aruco_17.png")
    os.makedirs(texture_dir, exist_ok=True)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    marker     = aruco.generateImageMarker(aruco_dict, MARKER_ID, MARKER_SIZE)
    bordered   = cv2.copyMakeBorder(marker, BORDER_PX, BORDER_PX, BORDER_PX, BORDER_PX,
                                    cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(output_path, bordered)
    print(f"✅ ArUco ID={MARKER_ID} saved: {output_path}")

    px4_dir = os.path.expanduser(
        "~/PX4-Autopilot/Tools/simulation/gz/models/aruco_17/materials/textures")
    if os.path.exists(os.path.dirname(os.path.dirname(px4_dir))):
        os.makedirs(px4_dir, exist_ok=True)
        shutil.copy2(output_path, os.path.join(px4_dir, "aruco_17.png"))
        print(f"✅ Copied to PX4: {px4_dir}")

if __name__ == "__main__": main()
