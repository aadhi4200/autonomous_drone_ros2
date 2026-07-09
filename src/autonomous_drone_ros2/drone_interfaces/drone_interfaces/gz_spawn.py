"""
gz_spawn.py | Package: drone_interfaces
Runtime model spawn into an already-running Gazebo world, via the `gz service`
CLI (subprocess) rather than a ROS service — this avoids depending on
ros_gz_sim's SpawnEntity being exposed as a ROS service in whatever
ros_gz_sim version is installed (spec calls this out as version-dependent).
"""
import subprocess


def find_world_name(default: str = "aruco_landing", timeout_s: float = 3.0) -> str:
    """Looks for an active '/world/<name>/create' service and returns <name>.
    Falls back to `default` if gz isn't reachable or no world is up yet."""
    try:
        out = subprocess.run(
            ["gz", "service", "-l"],
            capture_output=True, text=True, timeout=timeout_s,
        ).stdout
    except Exception:
        return default

    for line in out.splitlines():
        line = line.strip()
        if line.startswith("/world/") and line.endswith("/create"):
            return line.split("/")[2]
    return default


def spawn_model(world_name: str, model_name: str, sdf_path: str,
                 x: float, y: float, z: float, timeout_s: float = 3.0) -> tuple[bool, str]:
    """Spawns the model described by sdf_path into the running world at
    (x, y, z). Returns (success, message)."""
    req = (
        f'sdf_filename: "{sdf_path}", name: "{model_name}", '
        f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}'
    )
    try:
        proc = subprocess.run(
            [
                "gz", "service",
                "-s", f"/world/{world_name}/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", str(int(timeout_s * 1000)),
                "--req", req,
            ],
            capture_output=True, text=True, timeout=timeout_s + 2,
        )
    except Exception as e:
        return False, f"gz service call failed: {e}"

    if proc.returncode != 0:
        return False, proc.stderr.strip() or proc.stdout.strip() or "gz service returned non-zero"
    return True, proc.stdout.strip()
