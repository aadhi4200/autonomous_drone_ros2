"""px4_stack.py — shared PX4 SITL + Gazebo process construction, used by both
`px4_sitl.launch.py` (standalone, for running this stage on its own — see
CLAUDE.md-adjacent request to keep stages separately runnable/visible rather
than only reachable through one combined launch file) and
`full_stack.launch.py` (the combined one-command bring-up). Factored out so
neither file duplicates the process definitions.

PX4 boot mechanism notes (verified live 2026-07-10 against this exact
checkout, v1.18.0-alpha1-495-g692d624670): the `make px4_sitl gz_<model>`
convenience target documented in README.md/launch_full_sim.sh does NOT work
on this PX4 version — per-model ninja target generation was removed
upstream. This is the two-process replacement:
  - `make px4_sitl_default` once (builds the plain px4 binary; safe to
    rerun, no-ops if already built) — NOT done here, this module only
    constructs the *run* processes, not the build step.
  - Gazebo as its own process (`gz sim -r <world.sdf>`) — PX4 waits for it
    internally ("Waiting for Gazebo world..." / "Gazebo world is ready" in
    its own log), so no extra readiness gate is needed between the two.
  - PX4 itself via `bin/px4 -d` (`-d` = daemon mode, don't start the pxh>
    shell) with `PX4_SYS_AUTOSTART=<airframe id>` selecting the vehicle
    (replaces the old `gz_<model>` target name).
  - `-d` is the actual fix for the pxh> shell's stdin-EOF busy-loop —
    confirmed live: `< /dev/null` alone did NOT stop it when tested against
    the raw binary (filled a log to 800+MB in under a minute). Both are
    kept together for safety.
"""
import os

from launch.actions import ExecuteProcess

PX4_DIR = os.environ.get('PX4_DIR', os.path.expanduser('~/PX4-Autopilot'))
PX4_ROOTFS_DIR = os.path.join(PX4_DIR, 'build', 'px4_sitl_default', 'rootfs')
PX4_BIN = os.path.join(PX4_DIR, 'build', 'px4_sitl_default', 'bin', 'px4')

# Airframe ID for this project's vehicle (ROMFS/px4fmu_common/init.d-posix/
# airframes/4018_gz_x500_lidar_cam_down) — replaces the old `gz_<model>`
# make-target name, which no longer exists on this PX4 version.
PX4_SYS_AUTOSTART = os.environ.get('PX4_SYS_AUTOSTART', '4018')

# Gazebo model/world resource store. Defaults to a pre-populated clone of
# github.com/PX4/PX4-gazebo-models (base assets: ground plane, sun, the x500
# vehicle family) — verified present in this environment. Falls back to the
# stock simulation-gazebo script's own auto-download location if not found.
GZ_MODEL_STORE = os.environ.get(
    'GZ_MODEL_STORE',
    os.path.expanduser('~/PX4-gazebo-models')
    if os.path.isdir(os.path.expanduser('~/PX4-gazebo-models'))
    else os.path.expanduser('~/.simulation-gazebo'))

# This project's own world/model files (aruco_landing.sdf, the aruco_<id>
# marker models Feature 1 generates at runtime) — kept separate from
# GZ_MODEL_STORE and merged onto GZ_SIM_RESOURCE_PATH, rather than copied
# into the model store, so Feature 1's marker-spawn code (which writes
# here) doesn't need to know about the store at all.
DRONE_WS = os.environ.get('DRONE_WS', os.path.expanduser('~/drone_ws2'))
PROJECT_SIM_DIR = os.path.join(DRONE_WS, 'src', 'autonomous_drone_ros2', 'simulation', 'gazebo')

SYNCED_HOME_FILE = os.environ.get(
    'SYNCED_HOME_FILE', os.path.join(DRONE_WS, '.last_synced_home'))
PX4_HOME_ALT_DEFAULT = os.environ.get('PX4_HOME_ALT_DEFAULT', '0')


def synced_home_env() -> dict:
    """Mirrors launch_full_sim.sh's home-file parsing exactly — same file,
    same first-comma/last-comma split, same fallback behavior (SITL spawns
    at the world default origin and the backend's home_position_match gate
    stays red until the operator syncs and relaunches, per CLAUDE.md
    section 3.3/10.1).
    """
    if not os.path.isfile(SYNCED_HOME_FILE):
        return {}
    raw = open(SYNCED_HOME_FILE).read().strip()
    if ',' not in raw:
        return {}
    lat = raw.split(',', 1)[0]
    lon = raw.rsplit(',', 1)[1]
    return {'PX4_HOME_LAT': lat, 'PX4_HOME_LON': lon, 'PX4_HOME_ALT': PX4_HOME_ALT_DEFAULT}


# Feature 1's runtime marker-spawn code (aruco_marker.py's
# write_pad_model_everywhere()) writes generated aruco_<id> models here, not
# just into PROJECT_SIM_DIR/models — mirroring the pre-PX4-version-drift
# setup_gazebo.sh convention (which also copied the static aruco_17 model
# here). Must stay on GZ_SIM_RESOURCE_PATH or newly-generated markers won't
# resolve in Gazebo even though the file write itself succeeds.
PX4_TOOLS_GZ_MODELS = os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz', 'models')


def make_gazebo_process(gz_world, headless) -> ExecuteProcess:
    """gz_world/headless: LaunchConfiguration substitutions (or plain
    strings) for the world name and '0'/'1' headless flag."""
    return ExecuteProcess(
        cmd=['bash', '-c', 'gz sim -r "$WORLD_SDF" $( [ "$GZ_HEADLESS" = "1" ] && echo -s )'],
        additional_env={
            'WORLD_SDF': [os.path.join(PROJECT_SIM_DIR, 'worlds') + os.sep, gz_world, '.sdf'],
            'GZ_HEADLESS': headless,
            'GZ_SIM_RESOURCE_PATH': f"{os.path.join(GZ_MODEL_STORE, 'models')}:"
                                     f"{os.path.join(PROJECT_SIM_DIR, 'models')}:"
                                     f"{PX4_TOOLS_GZ_MODELS}",
            'GZ_SIM_SERVER_CONFIG_PATH': os.path.join(GZ_MODEL_STORE, 'server.config'),
            **({'PX4_GZ_SIM_RENDER_ENGINE': os.environ['PX4_GZ_SIM_RENDER_ENGINE']}
               if os.environ.get('PX4_GZ_SIM_RENDER_ENGINE') else {}),
        },
        output='screen',
    )


def make_px4_process(gz_world) -> ExecuteProcess:
    """gz_world: LaunchConfiguration substitution (or plain string) — must
    match whatever make_gazebo_process() was given, so PX4 connects to the
    same running world rather than waiting on a mismatched name."""
    return ExecuteProcess(
        cmd=['bash', '-c', f'"{PX4_BIN}" -d < /dev/null'],
        cwd=PX4_ROOTFS_DIR,
        additional_env={
            'PX4_SYS_AUTOSTART': PX4_SYS_AUTOSTART,
            'PX4_GZ_WORLD': gz_world,
            **synced_home_env(),
        },
        output='screen',
    )
