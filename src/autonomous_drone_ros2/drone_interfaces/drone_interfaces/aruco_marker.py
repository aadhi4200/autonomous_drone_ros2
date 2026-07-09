"""
aruco_marker.py | Package: drone_interfaces
Shared ArUco marker generation (PNG texture + Gazebo model.sdf/model.config),
used by both scripts/generate_aruco.py (offline, ID=17, build-time) and the
runtime /markers/generate backend endpoint (Feature 1). Kept in one place so
the two never drift into generating incompatible marker geometry.
"""
import os
import shutil

import cv2
import cv2.aruco as aruco

MARKER_SIZE = 1000
BORDER_PX = 100
PAD_SIZE_M = 2.0

MODEL_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>{size} {size} 0.001</size></box></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>model://{model_name}/materials/textures/aruco_{marker_id}.png</albedo_map></metal></pbr>
        </material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{size} {size} 0.001</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

MODEL_CONFIG_TEMPLATE = """<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>ArUco marker ID={marker_id} (DICT_6X6_250) — runtime-generated landing target</description>
</model>
"""


def generate_marker_png(marker_id: int, out_path: str) -> str:
    """Renders a bordered DICT_6X6_250 marker PNG to out_path."""
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    if hasattr(aruco, "generateImageMarker"):
        marker = aruco.generateImageMarker(aruco_dict, marker_id, MARKER_SIZE)
    else:
        # OpenCV < 4.7 (e.g. the apt-packaged 4.5.x) only has the old name.
        marker = aruco.drawMarker(aruco_dict, marker_id, MARKER_SIZE)
    bordered = cv2.copyMakeBorder(
        marker, BORDER_PX, BORDER_PX, BORDER_PX, BORDER_PX,
        cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(out_path, bordered)
    return out_path


def write_pad_model(marker_id: int, models_root: str, model_name: str | None = None) -> dict:
    """Writes aruco_<id>/{model.sdf, model.config, materials/textures/aruco_<id>.png}
    under models_root. model_name defaults to 'aruco_<id>' (the model *type*);
    pass a distinct name (e.g. 'aruco_pad_B') if you want a per-waypoint spawn
    identity distinct from the marker id itself.
    Returns {"model_dir", "texture_path", "sdf_path"}.
    """
    model_name = model_name or f"aruco_{marker_id}"
    model_dir = os.path.join(models_root, model_name)
    texture_dir = os.path.join(model_dir, "materials", "textures")
    texture_path = os.path.join(texture_dir, f"aruco_{marker_id}.png")

    generate_marker_png(marker_id, texture_path)

    sdf_path = os.path.join(model_dir, "model.sdf")
    with open(sdf_path, "w") as f:
        f.write(MODEL_SDF_TEMPLATE.format(model_name=model_name, marker_id=marker_id, size=PAD_SIZE_M))

    config_path = os.path.join(model_dir, "model.config")
    with open(config_path, "w") as f:
        f.write(MODEL_CONFIG_TEMPLATE.format(model_name=model_name, marker_id=marker_id))

    return {"model_dir": model_dir, "texture_path": texture_path, "sdf_path": sdf_path}


def write_pad_model_everywhere(marker_id: int, repo_models_root: str, model_name: str | None = None) -> dict:
    """Writes the pad model into the repo's own simulation/gazebo/models AND
    into ~/PX4-Autopilot/Tools/simulation/gz/models — mirroring the existing
    generate_aruco.py behavior, since GZ_SIM_RESOURCE_PATH at SITL runtime
    resolves model:// URIs against the PX4-Autopilot copy, not the repo copy.
    """
    result = write_pad_model(marker_id, repo_models_root, model_name)

    px4_models_root = os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/models")
    if os.path.isdir(os.path.dirname(px4_models_root)):
        px4_result = write_pad_model(marker_id, px4_models_root, model_name)
        result["px4_sdf_path"] = px4_result["sdf_path"]

    return result
